#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#define DEVICE_NAME "esp-now"

// define PINs used - GPIO mode
#define LED 2 // build-in LED

#ifndef STASSID
#define STASSID "ssid"
#define STAPSK "password"
#endif

// WiFi definitions
const char *ssid = STASSID;
const char *password = STAPSK;
WiFiClient espClient;

// MQTT definitions
void callback(char *topic, byte *payload, unsigned int length)
{
  // do something with the message

  String topicStr = topic;
  String recv_payload = String((char *)payload);

  Serial.println("mqtt_callback - message arrived - topic [" + topicStr +
                 "] payload [" + recv_payload + "]");
}
PubSubClient client("192.168.0.212", 1883, callback, espClient);

void mqtt_connect()
{
  Serial.print("connecting to broker...");
  while (!client.connect(DEVICE_NAME, "esp-now/status", 1, 1, "offline"))
  {
    Serial.print(".");
    delay(1000);
  }
}

// ESP-NOW data structure
typedef struct struct_message
{
  float outdoortemp;
  float outdoorhumidity;
  float indoortemp;
  float indoorhumidity;
  float pressure;
  float lux;
} struct_message;
struct_message myData;

// callback function that will be executed when ESP-NOW data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  char macStr[18];
  String sMac;
  Serial.print("Last Packet received from ");
  // make printable format of MAC address supplied
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  macStr[17] = '\0';
  sMac = String(macStr);
  sMac.toUpperCase();
  Serial.println(sMac);

  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.printf("indoor temp: %02f indoor humidity: %02f outdoor temp: %02f outdoor humidity: %02f pressure: %02f, lux: %02f \n", myData.indoortemp, myData.indoorhumidity, myData.outdoortemp, myData.outdoorhumidity, myData.pressure, myData.lux);

  StaticJsonDocument<200> JSONbuffer;
  JSONbuffer["device"] = DEVICE_NAME;
  JSONbuffer["indoor temperature"] = myData.indoortemp;
  JSONbuffer["indoor humidity"] = myData.indoorhumidity;
  JSONbuffer["outdoor temperature"] = myData.outdoortemp;
  JSONbuffer["outdoor humidity"] = myData.outdoorhumidity;
  JSONbuffer["pressure"] = myData.pressure;
  JSONbuffer["lux"] = myData.lux;
  char JSONmessageBuffer[200];
  serializeJson(JSONbuffer, JSONmessageBuffer);
  Serial.println("sending values over MQTT:");
  if (client.publish("esp-now", JSONmessageBuffer) == true)
  {
    Serial.println("Success sending message");
  }
  else
  {
    Serial.println("Error sending message");
  }
}

void setup()
{
  // Initialize Serial Monitor
  Serial.begin(115200);

  WiFi.mode(WIFI_AP_STA);
  Serial.println(WiFi.macAddress());
  WiFi.begin(ssid, password);
  // check wi-fi is connected to wi-fi network
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("Ready");
  Serial.println("IP address/MAC address: ");
  Serial.println(WiFi.localIP());
  Serial.println(WiFi.macAddress());
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // use buildin LED to show activity
  pinMode(LED, OUTPUT);
  // Register for Recv callback to get the data is received
  esp_now_register_recv_cb(OnDataRecv);
}

void loop()
{
  // put your main code here, to run repeatedly:
  client.loop();

  if (!client.connected())
  {
    mqtt_connect();
  }
}
