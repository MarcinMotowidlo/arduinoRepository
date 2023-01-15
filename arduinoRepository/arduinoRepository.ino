#include <OneWire.h>
#include <DS18B20.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <string.h>

// sensor Port
#define ONEWIRE_PIN 2
#define LCD_DISPLAY_ADDRESS 0x27
#define HUMIDITY_SENSOR_PORT A0

LiquidCrystal_I2C lcd(LCD_DISPLAY_ADDRESS, 16, 2);

bool debugFeaturesEnabled = true;
int sensorReadIntervalMs = 1000;
byte temperatureSensorAddress[8] = { 0x28, 0x65, 0xB3, 0x72, 0x2, 0x0, 0x0, 0xD7 };

// initialize onewireBus
OneWire onewire(ONEWIRE_PIN);
// initialize sensor
DS18B20 sensors(&onewire);

/**
* Initialize
* - Serial port
* - temperature sensor on 1Wire bus
**/
void setup() {
  Serial.begin(9600);
  Serial.write("Serial port initialized");

  setupLcd();
  setupSensors();

  // debug features:
  if (debugFeaturesEnabled) {
    identifyOneWireDevices();
  }
}

void loop() {
  if (sensors.available()) {

    char lcdBuffer[16];      // LCD line 1 buffer
    char tempBuffer[4];      // temp value buffer
    char humidityBuffer[4];  // humidity value buffer

    // read temperature:
    float temperature = sensors.readTemperature(temperatureSensorAddress);
    Serial.print("Temperature ['C]: ");
    Serial.println(temperature);
    
    // read humidity:
    int humiditySensor = analogRead(HUMIDITY_SENSOR_PORT);
    float humidityVoltage = humiditySensor * (5.0 / 1023.0);
    float humidityPercentage = ((5 - humidityVoltage) / 5) * 100;
    Serial.print("Humidity [%]: ");
    Serial.println(humidityPercentage);

    // display results:
    dtostrf(temperature, 4, 2, tempBuffer);
    snprintf(lcdBuffer, 16, "Temp: %s 'C", tempBuffer);
    writeLcd(0, lcdBuffer);

    dtostrf(humidityPercentage, 4, 2, humidityBuffer);
    snprintf(lcdBuffer, 16, "Humi: %s %%", humidityBuffer);
    writeLcd(1, lcdBuffer);

    sensors.request(temperatureSensorAddress);
    delay(sensorReadIntervalMs);
  }
}

/**
* Help method to display 8 byte address on serial port
**/
void printAddress(byte address[8]) {
  // print address starting with Ox
  for (byte i = 0; i < 8; i++) {
    Serial.print(F("0x"));
    Serial.print(address[i], HEX);

    if (i < 7)
      Serial.print(", ");
  }
  Serial.println();
}

/**
* Help method to identify all devices connected to 1Wire bus
**/
byte* identifyOneWireDevices() {
  byte address[8];

  onewire.reset_search();
  while (onewire.search(address)) {
    if (address[0] != 0x28) {
      continue;
    }
    if (debugFeaturesEnabled) {
      printAddress(address);
    }
    return address;
  }
}

void setupLcd() {
  lcd.begin();
  lcd.backlight();
}

void setupSensors() {
  sensors.begin();
  sensors.request(temperatureSensorAddress);
}

void writeLcd(int line, String text) {
  lcd.setCursor(0, line);
  lcd.print(text);
}
