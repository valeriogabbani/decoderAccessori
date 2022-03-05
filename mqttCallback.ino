/*
 * Funzioni callback libreria MQTT client
 */
void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

/*
 * Elabora i mesaggi ricevuti dal broker, qualunque TOPIC
 * Devi filtrare per TOPIC e MESSAGE
 */
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.println("Publish received.");
  Serial.print("  topic: ");
  Serial.println(topic);
  Serial.print("  qos: ");
  Serial.println(properties.qos);
  Serial.print("  dup: ");
  Serial.println(properties.dup);
  Serial.print("  retain: ");
  Serial.println(properties.retain);
  Serial.print("  len: ");
  Serial.println(len);
  Serial.print("  index: ");
  Serial.println(index);
  Serial.print("  total: ");
  Serial.println(total);

  //Impostazione stato dei deviatoi
  if(strcmp(topic, mqttTopicTurnoutReceive) == 0){
     uint16_t posizioni;
     posizioni = atoi(payload);
    if(posizioni <= 0xffff and posizioni >= 0){
      Serial.println(posizioni);
      mbslave.Hreg(POSITIONS, posizioni ); 
    }
  }
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

/*
 * Dopo la connessione:
 * Specifica i topics a cui ti vuoi collegare 
 * per ricevere messaggi
 */
void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  uint16_t packetIdSub = mqttClient.subscribe(mqttTopicTurnoutReceive, 1);
  Serial.print("Subscribing at QoS 1, packetId: ");
  Serial.println(packetIdSub);
  mqttClient.publish(mqttTopicWill, 2, true, "alive");
  char buffer[128];
  itoa(mbslave.Hreg(NORMALPOS),buffer,10);
  //mqttClient.publish(mqttTopicStatusN, 2, true, buffer);
  itoa(mbslave.Hreg(REVERSEPOS),buffer,10);
  //mqttClient.publish(mqttTopicStatusR, 2, true, buffer);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}
