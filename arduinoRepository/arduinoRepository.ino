#include <OneWire.h>
#include <DS18B20.h>

// sensor Port
#define ONEWIRE_PIN 2

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
  sensors.begin();
  sensors.request(temperatureSensorAddress);

  // debug features:
  if (debugFeaturesEnabled) {
    identifyOneWireDevices();
  }
}

void loop() {
  if (sensors.available()) {
    float temperature = sensors.readTemperature(temperatureSensorAddress);

    Serial.print(temperature);
    Serial.println(" 'C");

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
