/*
 * * Author: V.G.
 * Title: Controllo scambi con azionamento servo SG90
 * Hardware simile al controllo segnali con in più:
 * regolazione fine corsa manuale
 * frequenza del pwm 50hz, com richiesto dalle specifiche SERVO
 * Versione: in sviluppo multi protocollo MQTT/MODBUS
 * Facilmente generalizzabile fino a 16 servo.
 * 9_10_20: aggiunta funzione per scancellare le impostazioni wifi, funziona  SOLO attraverso MODBUS
 * 9_1_22: revisione generale
 * 5_3_22: lettura interruttori pf8574 per controllo manuale
 * 
 * Hardware: modulo CUSTOM MCU ESP8266+MODULO PWM PCA+ROTARY ENCODER
 * Librerie Software: 
 * 
 * Slave Modbus TCP/IP, libreria
 * https://github.com/emelianov/modbus-esp8266
 * 
 * Encoder:
 * https://github.com/PaulStoffregen/Encoder
 * 
 * MQTT client:
 * https://github.com/marvinroger/async-mqtt-client
 * 
 * PCF8547:
 * https://www.mischianti.org/2019/07/22/pcf8575-i2c-16-bit-digital-i-o-expander
 * 
 * PCA9685:
 * 
 * Service browser (MDNS):
 * avahi-discover
 * avahi-browse -a
 * 
 * Modbus tester: qmodMaster
 * Abilitato servizio OTA tramite libreria ESP8266HTTPUpdateServer
 * http://192.168.1.12:8080/update
 * 
 */

#define ControllerID "tc01"
 
//Macro di base per il debugging
#define runEvery(t) for (static uint16_t _lasttime;\
                         (uint16_t)((uint16_t)millis() - _lasttime) >= (t);\
                         _lasttime += (t))
/*************** MQTT ****************/
#define MQTT_HOST IPAddress(192, 168, 1, 21)
#define MQTT_PORT 1883                      
#define mqttTopicChannel "trains/"
#define mqttControllerID  ControllerID "/"
//Parametri MQTT
#define mqttTopicWill mqttTopicChannel mqttControllerID  "will"
#define mqttTopicStatus mqttTopicChannel mqttControllerID  "status"
#define mqttTopicTurnoutReceive mqttTopicChannel mqttControllerID "track/turnout/"
#define mqttTopicTurnoutSend mqttTopicChannel mqttControllerID "track/turnout/status/"
/***************** MQTT *********************/

#include "deviatoio.h"
#include "switch.h"
#include "led.h"
//#include <Adafruit_PWMServoDriver.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>
#include <ModbusIP_ESP8266.h>
#include <AsyncMqttClient.h>
#include <Ticker.h>
#include <ArduinoJson.h>
#include <FS.h>
#include "WiFiManager.h" 
#define ENCODER_DO_NOT_USE_INTERRUPTS
#include <Encoder.h>
#include "PCF8574.h"

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

#define PCA9685_ADDRESS 0x40
#define PCF8547_ADDRESS 0x20
//Macro per far lampeggiare un LED
#define toggleLed(MIOLED) digitalRead(MIOLED)?digitalWrite(MIOLED, LOW):digitalWrite(MIOLED, HIGH)
//#define DEBUG_PRINT
#define N_DEVIATOI  3
//Encoder rotativo
#define ENC_SW            0 //GPIO0,   D3
#define ENC_A             12 //GPIO12, D6
#define ENC_B             13 //GPIO13, D7
//Led di segnalazione

#define LED2              2
#define LED1              14
#define LED3              15

#define LED_NORMALE       LED2 //GPIO02 LED BLU sul modulo ESP8266  LED2
#define LED_ROVESCIO      LED1 //GPIO14                             LED1
#define LED_PROG          LED3 //GPIO15 attivo alto                 LED3

void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
}

/********************** IO ************/
switchEvent progButton(ENC_SW);
led pLed(LED_PROG);
led nLed(LED_NORMALE, true);
led rLed(LED_ROVESCIO, true);
/********************** IO ************/

/******************* HTTP OTA ******************/ 
ESP8266WebServer httpServer(8080);
ESP8266HTTPUpdateServer httpUpdater;
/******************* HTTP OTA ******************/ 

ModbusIP mbslave;
enum{
  //Il primo registro ha indirizzo 0, 16 bit di lunghezza
  POSITIONS, //comando posizione 0: normale, 1 rovescia
  NORMALPOS, //Controllo posizione normale: 0 NORMALE, 1: NON NORMALE
  REVERSEPOS, //Controllo posizione rovescia: 0 NORMALE, 1: NON NORMALE
  VERSION,
  RESET, 
  DUMP1,
  DUMP2,
  HOLDING_REGS_SIZE   //misura il numero dei registri
};

//vettore di puntatori ad oggetti
deviatoio *deviatoioPointerVector[16];

//di DEFAULT, ma qui sono scambiati
//GPIO 4 is SDA
//GPIO 5 is SCL
//Default address 0x40
Adafruit_PWMServoDriver servoDriver;
Encoder knobProg(ENC_A, ENC_B);
//                  SDA, SCL
PCF8574 pcf8574(0x20,5,  4);

//Macro per creare un evento temporizzato.
#define execEverytime(t)  for(static unsigned long _lasttime; millis() - _lasttime >= (t); _lasttime += (t))

unsigned long looptime,timeMarker;
int stato = 0, servoIndex = 0;
int refPos, refFc, angle;
int settingAngle;

/****************************SETUP SETUP SETUP********************************/
void setup(){
  Serial.begin(115200);
  Serial.flush();
  delay(100);
  Serial.println(ControllerID);
  Serial.println("Starting program....");

  /******************************** LOCALE ******************************************/
  //Create obiects on the heap
  for (int i=0; i < N_DEVIATOI; i++){
    deviatoioPointerVector[i] = new deviatoio(i, &servoDriver);
  }

//Fai prima l'inizializzazione di questa libreria che funziona...
  pcf8574.pinMode(P0, INPUT_PULLUP); pcf8574.pinMode(P1, INPUT_PULLUP);
  pcf8574.pinMode(P2, INPUT_PULLUP); pcf8574.pinMode(P3, INPUT_PULLUP);
  pcf8574.pinMode(P4, INPUT_PULLUP); pcf8574.pinMode(P5, INPUT_PULLUP);
  pcf8574.pinMode(P6, INPUT_PULLUP); pcf8574.pinMode(P7, INPUT_PULLUP);
  Serial.print("Inizializzazione pcf8574...");
  if (pcf8574.begin()){
    Serial.println("OK...");
  }else{
    Serial.println("Errore generico di comunicazione NON FUNZIONA");
  }
  /******************************** LOCALE ******************************************/

  //Controlla che ci sia comunque un file di configurazione fc
  checkConfig();
  //In caso affermativo lo carica
  loadConfig();
  printPositionLimits();
  
  //ATTENZIONE IDIOTA.....la scheda PCB prototipo che hai usato per le prove ha SCL ed SDA scambiate
  //Rispetto al default.....sicche devi foare override
  //SDA,SCL
  //Wire.begin(5,4);
  Serial.println("Inizializzazione servo....");
  servoDriver.begin();
  servoDriver.setOutputMode(true); //Output TOTEM-POLE
  servoDriver.setPWMFreq(50);      //Servo signal
  //pinMode(LED_AUX, OUTPUT); digitalWrite(LED_AUX, LOW);
  Serial.println(servoDriver.readPrescale());
  Serial.println(servoDriver.getOscillatorFrequency());
  /********************************* WIFI MANAGER ********************************/
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  //reset settings - for testing
  //wifiManager.resetSettings();
  wifiManager.setConfigPortalTimeout(60);
  //set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  wifiManager.setAPCallback(configModeCallback);
  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //and goes into a blocking loop awaiting configuration
  if(!wifiManager.autoConnect(ControllerID)) {
    Serial.println("failed to connect and hit timeout");
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(1000);
  } 
  /******************************** WIFI MANAGER ************************************/


   Serial.println("");
   Serial.println("WiFi connected: "); 
   Serial.println(WiFi.SSID());
   Serial.println("IP address: ");
   Serial.println(WiFi.localIP());
   delay(1000);
   
/******************Inizializzazione servizio MDNS *******/
  if (!MDNS.begin(ControllerID)) {
    Serial.println("Error setting up MDNS responder!");
    while(1) { 
      delay(1000);
    }
  }
  MDNS.addService("modbusSlave", "tcp", 502); 
  MDNS.addService("httpOTA", "tcp", 8080);
/******************Inizializzazione servizio MDNS *******/  
/******************* HTTP OTA ******************/  
  httpUpdater.setup(&httpServer);
  httpServer.begin();
/******************* HTTP OTA ******************/
/******************* MODBUS *******************/
//Inizializzazione registri modbus
  mbslave.begin();
  /*
  mbslave.addHreg(POSITIONS, 0x00);
  mbslave.addHreg(NORMALPOS, 0x00);   
  mbslave.addHreg(REVERSEPOS, 0x00);
  mbslave.addHreg(VERSION,   0x00);
  mbslave.addHreg(RESET,   0x00);
  mbslave.addHreg(DUMP1,   0x00); 
  mbslave.addHreg(DUMP2,   0x00);
  */
  //ALternativa tutti insieme !!!!!
  mbslave.addHreg(0, 0x00, HOLDING_REGS_SIZE);
    
  mbslave.Hreg(VERSION, 0xabcd );
  mbslave.Hreg(RESET, 0x0);
/******************* MODBUS *******************/          
  delay(2000);
  Serial.println("Modbus slave started  and initialized..... ");
//MQTT
  mqttClient.setWill(mqttTopicWill, 1,false,"on");
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  connectToMqtt();
  Serial.println(mqttTopicTurnoutReceive);
  Serial.println(mqttTopicTurnoutSend);
  timeMarker = millis();
  pLed.on();
  delay(500);
  nLed.on();
  delay(500);
  rLed.on();
  delay(500);
  pLed.off();
  nLed.off();
  rLed.off(); 
  pLed.blink();
  stato = 0;
}

/******************************* FIRST LEVEL MAIN LOOP LOOP LOOP *********************************/
void loop(){
  pLed.loop(); nLed.loop(); rLed.loop();
  progButton.loop(); //Gestione pulsante programmazione
  //FSM per la scelta della modalità
  //e' complicato.....
  switch (stato){
    //Stato di attesa iniziale con timeout
    //5 secondi di tempo per entrare in modalità programmazione
    case 0:
      if(millis() - timeMarker > 5000) {
        stato = 1; //running
        Serial.println("Running....");
        mqttClient.publish(mqttTopicStatus, 2, true, "run");  
        mqttClient.publish(mqttTopicStatus, 2, true, "idle");
        nLed.off(); rLed.off();
        pLed.on();
      }
      if(progButton.test() ){
        stato = 3; //si comincia da programmazione FC NORMALE del PRIMO della lista
        knobProg.write(0);
        servoIndex = 0;
        deviatoioPointerVector[servoIndex]->setReversePositionLimit(90);
        deviatoioPointerVector[servoIndex]->setNormalPositionLimit(90);
        refFc = deviatoioPointerVector[servoIndex]->readNormalPositionLimit();
        Serial.println("Inizio Programmazione...");
        nLed.off(); rLed.off(); pLed.off();
        mqttClient.publish(mqttTopicStatus, 2, true, "prog");  
      }
      break;
    //stato di running normale in telecomando
    case 1:
      main_loop(); //loop secondario
      if(pcf8574.digitalRead(P0)==LOW) {Serial.println("Modalita controllo locale");stato = 5;}
      break;
      
      //taratura FC ROVESCIO
      //La posizione è controllata dal cursore encoder
    case 2:
      if(progButton.test()){
        deviatoioPointerVector[servoIndex]->setReversePositionLimit(settingAngle);
        deviatoioPointerVector[servoIndex]->disable();
        stato = 4;
      }
      if(millis() - timeMarker > 5000){ //Alterno le due posizioni automaticamente
        deviatoioPointerVector[servoIndex]->setReversePositionLimit(settingAngle);
        Serial.print("rovescio servo: "); Serial.print(servoIndex); Serial.print(" fc:  "); Serial.println(deviatoioPointerVector[servoIndex]->readReversePositionLimit());
        stato = 3;
        timeMarker = millis();
        knobProg.write(0);
        refFc = deviatoioPointerVector[servoIndex]->readNormalPositionLimit();
        rLed.off();
        nLed.blink();
      }
      settingAngle = refFc+knobProg.read();
      if(settingAngle > 180) settingAngle = 180;
      if(settingAngle < 0) settingAngle = 0;
      deviatoioPointerVector[servoIndex]->setToangle(settingAngle);
      break; 
      //taratura FC NORMALE, regolazione angolo
    case 3:
      if(progButton.test()){ //disabilito il servo, ho finito con la programmazione di questo servo
        deviatoioPointerVector[servoIndex]->setNormalPositionLimit(settingAngle);
        deviatoioPointerVector[servoIndex]->disable();
        stato = 4;
      }
      if(millis() - timeMarker > 5000){
        deviatoioPointerVector[servoIndex]->setNormalPositionLimit(settingAngle);
        Serial.print("normale servo: "); Serial.print(servoIndex); Serial.print(" fc:  "); Serial.println(deviatoioPointerVector[servoIndex]->readNormalPositionLimit());
        stato = 2;
        timeMarker = millis();
        knobProg.write(0);
        refFc = deviatoioPointerVector[servoIndex]->readReversePositionLimit();
        nLed.off();
        rLed.blink();
      }
      settingAngle = refFc+knobProg.read();
      //runEvery(500){Serial.println(settingAngle);}
      if(settingAngle > 180) settingAngle = 180;
      if(settingAngle < 0) settingAngle = 0;
      deviatoioPointerVector[servoIndex]->setToangle(settingAngle);
      //runEvery(500){Serial.println(settingAngle);}
      break;
      //Ho finito con un servo, passo al successivo della lista
    case 4:
      //Controllo se sono arrivato all'ultimo servo
      if (servoIndex >= N_DEVIATOI-1) {
        stato = 1;
        saveConfig();
        Serial.println("Running....");
        mqttClient.publish(mqttTopicStatus, 2, true, "run");
        rLed.off(); nLed.off();
        pLed.on(); 
      }
      //Altrimenti procedo con il prossimo
      else {
        servoIndex++;
        deviatoioPointerVector[servoIndex]->setReversePositionLimit(90);
        deviatoioPointerVector[servoIndex]->setNormalPositionLimit(90);
        refFc = deviatoioPointerVector[servoIndex]->readNormalPositionLimit();
        stato = 3;
        rLed.off(); nLed.off(); pLed.off();
      }
      break;
     case 5:
       controlloLocale_loop();
       break;           
  }
}

/************************* SECOND LEVEL MAIN LOOP LOOP **********************/
void main_loop(){
  //testEncoder();
  pLed.on();
  for (int i=0; i < N_DEVIATOI; i++){
    deviatoioPointerVector[i]->motore(); //funzione necesaria per far avanzare i timers interni del movimento lento
  }
  httpServer.handleClient(); /******************* HTTP OTA ******************/
  MDNS.update();
  mbslave.task();
  checkModbusCommandEvent();
  checkFeedbackEvent();
  if(WiFi.status() != WL_CONNECTED){
    Serial.println("Disconnected !");
    ESP.reset();
    delay(1000);   
  }
  /*
  looptime=millis();
  //sub-loop 100ms
  execEverytime(100){ 
    #ifdef DEBUG_PRINT
    String message = String(looptime)+"  Input: "+String(mbslave.Hreg(VERSION));
    Serial.println(message);
    Serial.println(servoDriver.readPrescale());
    Serial.println(servoDriver.getOscillatorFrequency()); 
    #endif
    checkModbus();
  }
  */
}
