#include <LSM6DS3.h>
#include <ArduinoBLE.h>
#include <HX711.h>

// -------------------config---------------------------------
// calibrate scale to newton!
// for calibration set the factor to 1. Put a weight of 1kg on the pedal of the installed crank
// Divide the weight that you get back by 10 and this is the the new calibration factor.
// 170mm=0.17m
#define factor 1200
#define crankmeter 0.17
#define samples 50
#define rate 100   //time between samples
#define maxrpm 500
#define dataPin P0_4
#define clockPin P0_5
// -------------------------------------------------------------

HX711 force;

LSM6DS3 IMU(I2C_MODE, 0x6A);

BLEService cyclingPowerService("1818");
BLEService batteryService("180F");
BLEService debugService("87dad091-0a03-40c2-a9c8-632ab09e17df");

BLECharacteristic cyclingPowerMeasurementChar("2A63", BLERead | BLENotify, 8);
BLECharacteristic cyclingPowerFeatureChar("2A65", BLERead, 4);
BLECharacteristic sensorLocatChar("2A5D", BLERead, 1);

BLEByteCharacteristic tareChar("87dad091-0a03-40c2-a9c8-632ab09e17d1", BLERead | BLEWrite);

BLEUnsignedCharCharacteristic batteryLevelChar("2A19", BLERead | BLENotify);

unsigned char bleBuffer[8];
unsigned char slBuffer[1];
unsigned char fBuffer[4];

short power;
unsigned short flags = 0x00;

byte sensorlocation = 0x05;

float rpm = 0;
int newtons = 0;
int watt = 0;
uint32_t wattAvg = 0;

int previousMillis = 0;
int temp = 0;
uint32_t rpmAvg = 0;
int rpmTemp = 0;

bool noRotation = false;

void setup() {
  Serial.begin(9600);

  pinMode (P0_26, OUTPUT);
  pinMode (P0_30, OUTPUT);
  pinMode (P0_6, OUTPUT);
  pinMode (P0_14, OUTPUT);

  digitalWrite(P0_14, LOW);
  digitalWrite(P0_26, HIGH);
  digitalWrite(P0_30, HIGH);
  digitalWrite(P0_6, LOW);

  delay(4000);

  if (!BLE.begin()) {
    while (1);
  }

  if (IMU.begin() != 0) {
    while(1);
  }

  force.begin(dataPin, clockPin);
  force.set_scale(factor);
  force.tare(30);

  digitalWrite(P0_6, HIGH);
  digitalWrite(P0_30, LOW);
  delay(500);
  digitalWrite(P0_30, HIGH);

  BLE.setLocalName("OpenPowermeter");
  BLE.setAdvertisedService(cyclingPowerService);
  cyclingPowerService.addCharacteristic(cyclingPowerMeasurementChar);
  cyclingPowerService.addCharacteristic(cyclingPowerFeatureChar);
  cyclingPowerService.addCharacteristic(sensorLocatChar);
  batteryService.addCharacteristic(batteryLevelChar);
  debugService.addCharacteristic(tareChar);
  BLE.addService(cyclingPowerService);
  BLE.addService(batteryService);
  BLE.addService(debugService);

  slBuffer[0] = sensorlocation & 0xff;
  sensorLocatChar.writeValue(slBuffer, 1);

  fBuffer[0] = 0x00;
  fBuffer[1] = 0x00;
  fBuffer[2] = 0x00;
  fBuffer[3] = 0x08;
  cyclingPowerFeatureChar.writeValue(fBuffer, 4);

  bleBuffer[0] = flags & 0xff;
  bleBuffer[1] = (flags >> 8) & 0xff;
  bleBuffer[2] = power & 0xff;
  bleBuffer[3] = (power >> 8) & 0xff;
  cyclingPowerMeasurementChar.writeValue(bleBuffer, 4);

  batteryLevelChar.writeValue(50);

  tareChar.setEventHandler(BLEWritten, bleTare);

  BLE.advertise();
}

void loop() {
  BLEDevice central = BLE.central();
  if(central){
    noRotation = false;
    for (int i = 1; i <= samples; i++) {
      temp = force.get_units(1);
      rpmTemp = IMU.readFloatGyroZ()/6;
      if(temp < 0){
        temp = temp * -1;
      }
      if(rpmTemp < 1){
        noRotation = true;
        rpmTemp = 0;
      }
      rpmAvg = rpmAvg + rpmTemp;
      wattAvg = wattAvg + temp;
      previousMillis = millis();
      while(millis() < previousMillis + rate){
        BLE.central();
      }
    }
    newtons = wattAvg / samples;
    rpm = rpmAvg / samples;
    wattAvg = 0;
    rpmAvg = 0;

    if(noRotation){
      rpm = 0;
    }

    power = newtons * crankmeter * rpm * 0.21;

    bleBuffer[0] = flags & 0xff;
    bleBuffer[1] = (flags >> 8) & 0xff;
    bleBuffer[2] = power & 0xff;
    bleBuffer[3] = (power >> 8) & 0xff;

    cyclingPowerMeasurementChar.writeValue(bleBuffer, 4);
  }
}

void bleTare(BLEDevice central, BLECharacteristic characteristic){
  digitalWrite(P0_6, LOW);
  force.tare(30);
  digitalWrite(P0_6, HIGH);
}