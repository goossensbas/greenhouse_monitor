#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>

// LoRa pin definitions
#define SCK 18  // GPIO18 -- SX127x's SCK
#define MISO 19 // GPIO19 -- SX127x's MISO
#define MOSI 23 // GPIO23 -- SX127x's MOSI
#define SS 5    // GPIO5 -- SX127x's CS
#define RST 14  // GPIO14 -- SX127x's RESET
#define DIO0 26 // GPIO26 -- SX127x's IRQ (Interrupt Request)

// I2C Pin definitions
#define PIN_SDA1 17
#define PIN_SCL1 16
#define PIN_SDA2 22
#define PIN_SCL2 21

TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

// BME280 definitions
Adafruit_BME280 bme_indoor;
Adafruit_BME280 bme_outdoor;

// Lux meter BH1750 definitions
BH1750 luxmeter;

// Timer definitions
#define SLEEP_SEC 4

// Define data struct
typedef struct struct_message
{
  float outdoortemp;
  float outdoorhumidity;
  float indoortemp;
  float indoorhumidity;
  float pressure;
  float lux;
} struct_message;

// convert to array of char
union datatosend
{
  struct_message myData;
  byte byteArray[sizeof(struct_message)]; /* you can use other variable types if you want. Like: a 32bit integer if you have 4 8bit variables in your struct */
};

// Initiate instance
datatosend buffer;

float read_lux()
{

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
  return lux;
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
  Serial.println("LoRa Sender");
  LoRa.setPins(SS, RST, DIO0);
  if (!LoRa.begin(433E6))
  {
    Serial.println("Starting LoRa failed!");
    while (1)
      ;
  }
  I2C_init();
  buffer.myData.indoortemp = bme_indoor.readTemperature();
  buffer.myData.indoorhumidity = bme_indoor.readHumidity();
  buffer.myData.outdoortemp = bme_outdoor.readTemperature();
  buffer.myData.outdoorhumidity = bme_outdoor.readHumidity();
  buffer.myData.pressure = bme_outdoor.readPressure();
  buffer.myData.lux = read_lux();
  Serial.printf("indoor temp: %02f indoor humidity: %02f outdoor temp: %02f outdoor humidity: %02f pressure: %02f, lux: %02f \n", buffer.myData.indoortemp, buffer.myData.indoorhumidity, buffer.myData.outdoortemp, buffer.myData.outdoorhumidity, buffer.myData.pressure, buffer.myData.lux);
  Serial.print(F("[SX1278] Transmitting packet ... "));

  Serial.print("Sending packet: ");
  LoRa.beginPacket();
  LoRa.write(buffer.byteArray, sizeof(buffer));
  int state = LoRa.endPacket();

  if (state == 1)
  {
    // the packet was successfully transmitted
    Serial.println(F(" success!"));
    Serial.println("Going to sleep...");
    esp_sleep_enable_timer_wakeup(SLEEP_SEC * 1000000);
    esp_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
    esp_deep_sleep_start();
  }
  else
  {
    // some other error occurred
    Serial.print(F("failed"));
    delay(1000);
  }



}

void loop()
{

}
