#ifndef WIFICOM_H
#define WIFICOM_H

#include <Arduino.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>

#define WIFI_NAME "MREN203"
#define WIFI_PASS "MREN203wifi"
#define WIFI_IP_ADDRESS 192,168,50,64
#define WIFI_PORT 2390

class wifiCom {
public:
	void startup();
	void sendData(double sensorValue[]);

private:
	const char* ssid = WIFI_NAME;
	const char* pass = WIFI_PASS;
	int status = WL_IDLE_STATUS;
	IPAddress laptopIP = IPAddress(WIFI_IP_ADDRESS);
	unsigned int localPort = WIFI_PORT;      // Must match the Python script
	WiFiUDP Udp;
};

#endif // WIFICOM_H
