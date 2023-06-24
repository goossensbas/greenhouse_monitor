#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include <esp_now.h>
#include <WiFi.h>

// Pin definitions
#define PIN_SDA1 17
#define PIN_SCL1 16
#define PIN_SDA2 22
#define PIN_SCL2 21

TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

// Timer definitions
#define SLEEP_SEC 4

// WIFI settings
#define WIFI_CHANNEL 0

// BME280 definitions
Adafruit_BME280 bme_indoor;
Adafruit_BME280 bme_outdoor;

// Lux meter BH1750 definitions
BH1750 luxmeter;

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

esp_now_peer_info_t peerInfo;
uint8_t broadcastAddress[] = {0x34, 0x94, 0x54, 0x25, 0xC0, 0xD0};

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  Serial.println("Going to sleep...");
  esp_sleep_enable_timer_wakeup(SLEEP_SEC * 1000000);
  esp_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
  esp_deep_sleep_start();
}

void I2C_init()
{
  I2Cone.begin(PIN_SDA1, PIN_SCL1, 100000);
  I2Ctwo.begin(PIN_SDA2, PIN_SCL2, 100000);
  // Init BME280_indoor
  if (!bme_indoor.begin(0x76, &I2Cone))
  {
    Serial.println("Could not find a valid BME280 indoor sensor, check wiring!");
  }
  if (!bme_outdoor.begin(0x76, &I2Ctwo))
  {
    Serial.println("Could not find a valid BME280 outdoor sensor, check wiring!");
  }
  // Init BH1750
  if (!luxmeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE, 0x23, &I2Cone))
  {
    Serial.println("could not find a valid BH1750 sensor, check wiring!");
  }
}

void setup()
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  // WiFi(ESP_IF_WIFI_STA, WIFI_PROTOCOL_LR);
  Serial.println(WiFi.macAddress());

  I2C_init();

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = WIFI_CHANNEL;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }

  float lux = luxmeter.readLightLevel();

  if (lux < 0)
  {
    Serial.println(F("Error condition detected"));
  }
  else
  {
    if (lux > 40000.0)
    {
      // reduce measurement time - needed in direct sun light
      if (luxmeter.setMTreg(32))
      {
        Serial.println(F("Setting MTReg to low value for high light environment"));
      }
      else
      {
        Serial.println(F("Error setting MTReg to low value for high light environment"));
      }
    }
    else
    {
      if (lux > 10.0)
      {
        // typical light environment
        if (luxmeter.setMTreg(69))
        {
          Serial.println(F("Setting MTReg to default value for normal light environment"));
        }
        else
        {
          Serial.println(F("Error setting MTReg to default value for normal light environment"));
        }
      }
      else
      {
        if (lux <= 10.0)
        {
          // very low light environment
          if (luxmeter.setMTreg(138))
          {
            Serial.println(F("Setting MTReg to high value for low light environment"));
          }
          else
          {
            Serial.println(F("Error setting MTReg to high value for low light environment"));
          }
        }
      }
    }
  }

  myData.indoortemp = bme_indoor.readTemperature();
  myData.indoorhumidity = bme_indoor.readHumidity();
  myData.outdoortemp = bme_outdoor.readTemperature();
  myData.outdoorhumidity = bme_outdoor.readHumidity();
  myData.pressure = bme_outdoor.readPressure();
  myData.lux = lux;
  Serial.printf("indoor temp: %02f indoor humidity: %02f outdoor temp: %02f outdoor humidity: %02f pressure: %02f, lux: %02f \n", myData.indoortemp, myData.indoorhumidity, myData.outdoortemp, myData.outdoorhumidity, myData.pressure, myData.lux);
  // Send message via ESP-NOW

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
  int retries = 0;
  int max_try = 5;
  if (result == ESP_OK)
  {
    Serial.println("Sent with success");
  }
  else
  {
    Serial.println("Error sending the data.");
  }
}

void loop()
{
  // put your main code here, to run repeatedly:
}