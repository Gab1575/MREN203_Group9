#include "wifiCom.h"



void wifiCom::startup() { 
  // Attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    
    delay(5000);
  }
  
  Serial.println("Connected to WiFi!");
  Udp.begin(localPort);
}

void wifiCom::sendData(double sensorValue[]) {
  String dataString = "Sensor values: ";
  for (int i = 0; i < 2; i++) {
    dataString += String(sensorValue[i]);
    if (i < 1) {
      dataString += ", ";
    }
  }

  Udp.beginPacket(laptopIP, localPort);
  Udp.print(dataString);
  Udp.endPacket();

  Serial.println("Packet sent: " + dataString);
}