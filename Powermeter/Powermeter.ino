#include <LSM6DS3.h>
#include <ArduinoBLE.h>
#include <HX711.h>

// -------------------config---------------------------------
// calibrate scale to newton!
// for calibration set the factor to 1. Put a weight of 1kg on the pedal of the installed crank
// Divide the weight that you get back by 10 and this is the the new calibration factor.
// 170mm=0.17m
#define factor 605
#define offset 0
#define crankmeter 0.17
#define samples 450
#define dataPin P0_4
#define clockPin P0_5
#define SerialDebug
// -------------------------------------------------------------

HX711 force;

LSM6DS3 IMU(I2C_MODE, 0x6A);

BLEService cyclingPowerService("1818");
BLEService batteryService("180F");

BLECharacteristic cyclingPowerMeasurementChar("2A63", BLERead | BLENotify, 8);
BLECharacteristic cyclingPowerFeatureChar("2A65", BLERead, 4);
BLECharacteristic sensorLocatChar("2A5D", BLERead, 1);

BLEUnsignedCharCharacteristic batteryLevelChar("2A19", BLERead | BLENotify);

unsigned char powerMeasurementBuffer[8];
unsigned char powerFeatureBuffer[4];

unsigned short power = 0;
unsigned short flags = 0x00;

float rpm = 0;
float newtons = 0;
float forceAvg = 0;

float forceTemp = 0;
float rpmAvg = 0;
float rpmTemp = 0;

void setup() {
  #ifdef SerialDebug
    Serial.begin(9600);
  #endif

  pinMode (P0_14, OUTPUT);
  pinMode (P1_10, OUTPUT);

  //set voltage divider pin low
  digitalWrite(P0_14, LOW);
  //deactivate microphone to save power
  digitalWrite(P1_10, LOW);

  if (!BLE.begin()) {
    while (1);
  }

  if (IMU.begin() != 0) {
    while(1);
  }

  //activate interrupt pin in IMU
  IMU.writeRegister(LSM6DS3_ACC_GYRO_INT1_CTRL, LSM6DS3_ACC_GYRO_INT1_SIGN_MOT_ENABLED);

  force.begin(dataPin, clockPin);
  force.set_scale(factor);
  force.set_offset(offset);

  BLE.setLocalName("OpenPowermeter");
  BLE.setAdvertisedService(cyclingPowerService);

  cyclingPowerService.addCharacteristic(cyclingPowerMeasurementChar);
  cyclingPowerService.addCharacteristic(cyclingPowerFeatureChar);
  cyclingPowerService.addCharacteristic(sensorLocatChar);
  batteryService.addCharacteristic(batteryLevelChar);

  BLE.addService(cyclingPowerService);
  BLE.addService(batteryService);

  sensorLocatChar.writeValue(byte(0x05));

  powerFeatureBuffer[0] = 0x00;
  powerFeatureBuffer[1] = 0x00;
  powerFeatureBuffer[2] = 0x00;
  powerFeatureBuffer[3] = 0x08;
  cyclingPowerFeatureChar.writeValue(powerFeatureBuffer, 4);

  powerMeasurementBuffer[0] = flags & 0xff;
  powerMeasurementBuffer[1] = (flags >> 8) & 0xff;
  powerMeasurementBuffer[2] = power & 0xff;
  powerMeasurementBuffer[3] = (power >> 8) & 0xff;
  cyclingPowerMeasurementChar.writeValue(powerMeasurementBuffer, 4);

  batteryLevelChar.writeValue(50);

  BLE.advertise();
}

void loop() {
  BLEDevice central = BLE.central();

  while(central){
    central = BLE.central();
    for(int i = 1; i <= samples; i++) {
      forceTemp = force.get_units() * -1;
      rpmTemp = IMU.readFloatGyroZ()/6;
      if(forceTemp < 0){
        forceTemp = 0;
      }
      rpmAvg = rpmAvg + rpmTemp;
      forceAvg = forceAvg + forceTemp;
      
      BLE.central();
    }
    newtons = forceAvg / samples;
    rpm = (rpmAvg / samples) * -1;

    if (rpm < 0){
      rpm = 0;
    }

    forceAvg = 0;
    rpmAvg = 0;

    power = newtons * crankmeter * rpm * 0.21;

    #ifdef SerialDebug
      Serial.print(newtons);
      Serial.print(",");
      Serial.print(rpm);
      Serial.print(",");
      Serial.println(power);
      Serial.println(analogRead(P0_31));
    #endif

    powerMeasurementBuffer[0] = flags & 0xff;
    powerMeasurementBuffer[1] = (flags >> 8) & 0xff;
    powerMeasurementBuffer[2] = power & 0xff;
    powerMeasurementBuffer[3] = (power >> 8) & 0xff;

    cyclingPowerMeasurementChar.writeValue(powerMeasurementBuffer, 4);
  }

  force.power_down();

  while(central == false){
    central = BLE.central();
  }

  force.power_up();
}