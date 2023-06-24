#include <Arduino.h>
#include <spi.h>
#include <LoRa.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#ifndef STASSID
#define STASSID "ssid"
#define STAPSK "password"
#endif

// LoRa pin definitions
#define SCK 18  // GPIO18 -- SX127x's SCK
#define MISO 19 // GPIO19 -- SX127x's MISO
#define MOSI 23 // GPIO23 -- SX127x's MOSI
#define SS 5    // GPIO5 -- SX127x's CS
#define RST 14  // GPIO14 -- SX127x's RESET
#define DIO0 26 // GPIO26 -- SX127x's IRQ (Interrupt Request)

const char *ssid = STASSID;
const char *password = STAPSK;
WiFiClient espClient;

IPAddress local_IP(192, 168, 0, 180);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);

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
  Serial.print("connecting to MQTT broker...");
  while (!client.connect("greenhouse_lora", "/home/greenhouse/status", 1, 1, "offline"))
  {
    Serial.print(".");
    delay(1000);
  }
}

void setup_wifi()
{
  if (!WiFi.config(local_IP, gateway, subnet))
  {
    Serial.println("STA Failed to configure");
  }
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  delay(200);
  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println("Connection Failed! Rebooting...");
    ESP.restart();
  }
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void lora_init()
{
  LoRa.setPins(SS, RST, DIO0);
  if (!LoRa.begin(433E6))
  {
    Serial.println("Starting LoRa failed!");
    while (1)
      ;
  }
}

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

union datatosend
{
  struct_message myData;
  byte byteArray[sizeof(struct_message)]; /* you can use other variable types if you want. Like: a 32bit integer if you have 4 8bit variables in your struct */
};

datatosend buffer;

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;
  lora_init();
  setup_wifi();
  mqtt_connect();
}

void loop()
{
  int packetSize = LoRa.parsePacket();
  if (packetSize)
  {
    // received a packet
    Serial.print("Received packet '");
    LoRa.readBytes((uint8_t *)&buffer.byteArray, sizeof(buffer.byteArray));

    // print RSSI of packet
    Serial.print("' with RSSI ");
    int rssi = LoRa.packetRssi();
    Serial.println(rssi);
    Serial.printf("indoor temp: %02f indoor humidity: %02f outdoor temp: %02f outdoor humidity: %02f pressure: %02f, lux: %02f \n", buffer.myData.indoortemp, buffer.myData.indoorhumidity, buffer.myData.outdoortemp, buffer.myData.outdoorhumidity, buffer.myData.pressure, buffer.myData.lux);

    StaticJsonDocument<200> JSONbuffer;
    JSONbuffer["device"] = "Greenhouse_lora";
    JSONbuffer["indoor temp"] = buffer.myData.indoortemp;
    JSONbuffer["indoor humidity"] = buffer.myData.indoorhumidity;
    JSONbuffer["outdoor temp"] = buffer.myData.outdoortemp;
    JSONbuffer["outdoor humidity"] = buffer.myData.outdoorhumidity;
    JSONbuffer["pressure"] = buffer.myData.pressure;
    JSONbuffer["lux"] = buffer.myData.lux;
    JSONbuffer["RSSI"] = rssi;   
    char JSONmessageBuffer[200];
    serializeJson(JSONbuffer, JSONmessageBuffer);
    Serial.println("sending values over MQTT:");
    if (client.publish("/home/greenhouse/values", JSONmessageBuffer) == true)
    {
      Serial.println("Success sending message");
    }
    else
    {
      Serial.println("Error sending message");
    }
    client.loop();
    Serial.println("-------------");
  }
  if (!client.connected())
  {
    mqtt_connect();
  }
  client.loop();
}
