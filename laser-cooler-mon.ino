#define MQTT_MAX_PACKET_SIZE 256

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <OneWire.h>

// Define these in the wifi_creds.hpp file
//const char ssid[] = "YOUR SSID"
//const char wifi_passwd[] = "YOUR WIFI PASSWORD"
#include "wifi_creds.h"

#define MQTT_TOPIC "cook/laser/cooler"

#define ONE_WIRE_PIN 2
OneWire  ds(ONE_WIRE_PIN);
#define N_SENSORS 2
byte sensorAddr[N_SENSORS][8] = {

  {0x28, 0xFF, 0xD4, 0xA9, 0xA1, 0x15, 0x04, 0x1C}, // (coldOut)
  {0x28, 0xFF, 0x01, 0xD7, 0xA1, 0x15, 0x04, 0x2D}, // (hotReturn)
};
char * sensorNames[N_SENSORS] = {
  "coldOut",
  "hotReturn",
};

WiFiClient client;
static const char* k_mqttbroker = "172.16.0.161";
PubSubClient g_pubSubClient(client);

void reconnect() {
  // Loop until we're reconnected
  while (!g_pubSubClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (g_pubSubClient.connect("Laser Exhaust Monitor")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(g_pubSubClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void DoWifiConnect()
{
  WiFi.begin(ssid, wifi_passwd);
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup(void){
  Serial.begin(115200);

  DoWifiConnect();

  g_pubSubClient.setServer(k_mqttbroker,1883);

  reconnect();
}


void loop(void)
{
  delay(5000);

  ArduinoJson::StaticJsonBuffer<256> jsonBuf;
  ArduinoJson::JsonObject& obj = jsonBuf.createObject();

  // Instruct all ds18b20 devices to take a measurement
  ds.reset();
  ds.skip();
  ds.write(0x44, 1); // Start conversion
  delay(800); // Sensors take ~750ms

  obj["uptime"] = millis();

  // Read each ds18b20 device individually
  float temp[N_SENSORS];
  for (int i=0; i<N_SENSORS; i++) {
    Serial.print("Temperature sensor ");
    Serial.print(i);
    Serial.print(": ");
    if (readTemperature(sensorAddr[i], &temp[i])) {
      Serial.print(temp[i]);
      Serial.println();
      obj[sensorNames[i]] = temp[i];
    }
    delay(100);
  }

  reconnect();

  String str;
  obj.printTo(str);
  Serial.println(str);

  bool ret = g_pubSubClient.publish(MQTT_TOPIC, str.c_str());
  if (!ret)
  {
    Serial.println("Failed to publish");
  }
}


// Reads data from sensor addr `a` and puts the temperature in temp (in celsius)
// Returns 1 on success, 0 on failure
int readTemperature(byte a[8], float *temp) {
  ds.reset();
  ds.select(a);
  ds.write(0xBE); // Read Scratchpad

  byte data[9];
  for (int i=0; i<9; i++) {
    data[i] = ds.read();
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  byte calculatedCRC = OneWire::crc8(data, 8);
  if (calculatedCRC != data[8]) {
    Serial.print("CRC Failed: ");
    Serial.print(calculatedCRC, HEX);
    Serial.print(" != ");
    Serial.println(data[8], HEX);
    return 0;
  }

  uint8_t reg_msb = data[1];
  uint8_t reg_lsb = data[0];
  uint16_t TReading = reg_msb << 8 | reg_lsb;

  int SignBit, Whole, Fract;
  SignBit = TReading & 0x8000;  // test most sig bit
  if (SignBit) {
    // negative
    TReading = (TReading ^ 0xffff) + 1; // 2's comp
  }

  Whole = TReading >> 4;  // separate off the whole and fractional portions
  Fract = (TReading & 0xf) * 100 / 16;

  *temp = Whole + (TReading & 0xf) / 16.;
  if (SignBit) {
    *temp *= -1;
  }

  return 1;
}
