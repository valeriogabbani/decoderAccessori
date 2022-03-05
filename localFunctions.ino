#define N_DEVIATOI  3

/*
 * Scansiona periodica modbus e comando Servo.
 */
 void checkModbus(){
  int feedbackN = 0;
  int feedbackR = 0;
  //Copia lo stato sui deviatoi
  for(int i=0; i < N_DEVIATOI; i++){
    if((mbslave.Hreg(POSITIONS) & (1 << i)) == 0) 
      deviatoioPointerVector[i]->motorCommand = false;
    else
      deviatoioPointerVector[i]->motorCommand = true;
    //feedbackN += deviatoioPointerVector[i]->normalPosition << i;
    //feedbackR += deviatoioPointerVector[i]->reversePosition << i;
  }
  //mbslave.Hreg(NORMALPOS, feedbackN );
  //mbslave.Hreg(REVERSEPOS, feedbackR );
}

/*
 * Legge la posizione degli aghi e SE qualcosa è cambiato
 * Commissiona le varizioni su modbus e su MQTT
 */
void checkFeedbackEvent(){
  static uint16_t lastfeedbackN = 0;
  static uint16_t lastfeedbackR = 0;
  uint16_t feedbackR = 0;
  uint16_t feedbackN = 0;
  char buffer[128];
  
  for(int i=0; i < N_DEVIATOI; i++){
    feedbackN += deviatoioPointerVector[i]->normalPosition << i;
    feedbackR += deviatoioPointerVector[i]->reversePosition << i;    
  }
  if (feedbackN != lastfeedbackN){
    lastfeedbackN = feedbackN;
    mbslave.Hreg(NORMALPOS, feedbackN );
    itoa(feedbackN,buffer,10);
    //mqttClient.publish(mqttTopicStatusN, 2, true, buffer);
  }
  if (feedbackR != lastfeedbackR){
    lastfeedbackR = feedbackR;
    mbslave.Hreg(REVERSEPOS, feedbackR );
    itoa(feedbackR,buffer,10);
    //mqttClient.publish(mqttTopicStatusR, 2, true, buffer);
  }
  
}

/*
 * Rileva evento modbus
 * Si accorge quando lo stato del registro comando è cambiato.
 * Aggiorna lo stato dei motori
 */

 void checkModbusCommandEvent(){
  static uint16_t lastCommand;
  if(lastCommand != mbslave.Hreg(POSITIONS)){
    lastCommand = mbslave.Hreg(POSITIONS);
    checkModbus();
  }
  if(mbslave.Hreg(RESET) == 0x8888){
      WiFi.disconnect(false);
      ESP.reset();
  }
 }

/*
 * Copia la configurazione dei FC dal SPIFFS alla memoria RAM
 */
bool loadConfig() {
  File configFile = SPIFFS.open("/config.json", "r");
  if (!configFile) {
    Serial.println("Failed to open config file");
    return false;
  }

  size_t size = configFile.size();
  if (size > 1024) {
    Serial.println("Config file size is too large");
    return false;
  }

  Serial.print("Allocating buffer: "); Serial.println(size);
  // Allocate a buffer to store contents of the file.
  std::unique_ptr<char[]> buf(new char[size]);
  StaticJsonDocument<200> jsondoc;

  // We don't use String here because ArduinoJson library requires the input
  // buffer to be mutable. If you don't use ArduinoJson, you may as well
  // use configFile.readString instead.
  Serial.print("Coping file in buffer: "); Serial.println(size);
  configFile.readBytes(buf.get(), size);

  Serial.println("Allocate 200B of memory for json document.... ");
  auto error = deserializeJson(jsondoc, buf.get());
  if (error) {
    Serial.println("Failed to parse config file");
    return false;
  }
  Serial.println("Memory allocated and deserialized....... ");
  //Inizilizza i fincersa in memoria dei deviatoi
  //Perciascuno
  for (int i=0; i < N_DEVIATOI; i++){
    String key = "d"+String(i)+"n";
    String val;
    Serial.println("Parametri letti dal file:");
    if (jsondoc.containsKey(key)){
      deviatoioPointerVector[i]->setNormalPositionLimit(atoi(jsondoc[key]));
      Serial.print(key); Serial.print(":"); Serial.println(atoi(jsondoc[key]));
    }
    else{
      deviatoioPointerVector[i]->setNormalPositionLimit(90);
    }
    
    key = "d"+String(i)+"r";
    if (jsondoc.containsKey(key)){
      deviatoioPointerVector[i]->setReversePositionLimit(atoi(jsondoc[key]));
      Serial.print(key); Serial.print(":"); Serial.println(atoi(jsondoc[key]));
    }
    else{
      deviatoioPointerVector[i]->setReversePositionLimit(90);
    }
    //Posiziona il deviatoio nella posizione NORMALE:
    //Non è corretto metterlo qui devi farlo nella macchina a stati del motore.
    //deviatoioPointerVector[i]->setToangle(deviatoioPointerVector[i]->readNormalPositionLimit());
  }//for deviatoi
  configFile.close();
  return true;
}

/*
 * Stampa il valore dei fc in memoria RAM
 */
void printPositionLimits(){
  Serial.println("Valori fc in memoria:");
  for (int i=0; i < N_DEVIATOI; i++){
    Serial.print("Deviatoio: "); Serial.print(i);
    Serial.print(" Normale: "); Serial.print(deviatoioPointerVector[i]->readNormalPositionLimit());
    Serial.print(" Rovescio: "); Serial.println(deviatoioPointerVector[i]->readReversePositionLimit());   
    
  }
}

/*
 * Salva la configurazione del FC in RAM nella FLASH SPIFFS
 */
void saveConfig(){
  File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
    Serial.println("createConfigFile: Unable to open config file (for some reason)...");
  }
  Serial.println("createConfigFile: Create config file.......");
  StaticJsonDocument<200> jsondoc;
  jsondoc["Valerio"] = "Gabbani";
  for (int i=0; i < N_DEVIATOI; i++){
    String key = "d"+String(i)+"n";
    Serial.print("saveConfigFile: "); Serial.println(key);
    jsondoc[key] = String(deviatoioPointerVector[i]->readNormalPositionLimit());
    key = "d"+String(i)+"r";
    Serial.print("saveConfigFile: "); Serial.println(key); 
    jsondoc[key] = String(deviatoioPointerVector[i]->readReversePositionLimit());    
  }
  serializeJson(jsondoc, configFile);
  configFile.close();
  Serial.println("createConfigFile: file created");
}


/*
 * Crea un file di configurazione se non ce ne già uno un SPIFFS
 */
 void createConfigFile(){
  File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
    Serial.println("createConfigFile: Unable to open config file (for some reason)...");
  }
  Serial.println("createConfigFile: Create config file.......");
  StaticJsonDocument<200> jsondoc;
  
  jsondoc["Valerio"] = "Gabbani";
  for (int i=0; i < N_DEVIATOI; i++){
    String key = "d"+String(i)+"n";
    Serial.print("createConfigFile: "); Serial.println(key);
    jsondoc[key] = "90";
    key = "d"+String(i)+"r";
    Serial.print("createConfigFile: "); Serial.println(key); 
    jsondoc[key] = "90";    
  }
  serializeJson(jsondoc, configFile);
  configFile.close();
  Serial.println("createConfigFile: file created");
 }
/*
 * 
 * Verifica la presenza e consistenza di un file di configurazione in SPIFFS
 */
 void checkConfig(){
  //Check configuration
  Serial.println("Mounting FS...");
  if (!SPIFFS.begin()) {
    Serial.println("Failed to mount file system, format");
    SPIFFS.format();
    ESP.restart();
  }
  Serial.println("checkConfig: Opening Config file for check");
  
  if (!SPIFFS.exists("/config.json")){
    Serial.println("checkConfig: No config file.....create one");
    createConfigFile();
    delay(2000);
    ESP.restart();  
  }

  File configFile = SPIFFS.open("/config.json", "r");
  if (!configFile) {
    Serial.println("checkConfig: Unable to open config file (for some reason)...");
    while(true);
  }
  Serial.println("checkConfig: Opening Config file exist....proceed with check");
  
  size_t size = configFile.size();
  if (size > 1024) {
    Serial.println("checkConfig: Config file size is too large");
  }

  Serial.print("checkConfig: Allocating buffer: "); Serial.println(size);
  // Allocate a buffer to store contents of the file.
  std::unique_ptr<char[]> buf(new char[size]);
  configFile.readBytes(buf.get(), size);
  
  StaticJsonDocument<200> jsondoc;
  auto error = deserializeJson(jsondoc, buf.get());
  if (error) {
    Serial.print("checkConfig: Failed to parse config file error "); Serial.println(error.c_str());
  }
  if (!jsondoc.containsKey("Valerio")){
    Serial.println("checkConfig: Config file not valid, delete");
    configFile.close();
    SPIFFS.remove("/config.json");
    ESP.restart();
  }
  else {
    Serial.println("checkConfig: Config file is valid....");
    configFile.close(); 
  }
 }

/*
 * Testa il funzionamento dell'encoder
 */

 void testEncoder(){
  long newPos = knobProg.read();
  static long oldPos;
  if (newPos != oldPos) {
    oldPos = newPos;
    Serial.println(oldPos);
  }
 }
