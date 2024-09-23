#include <Arduino_LSM6DS3.h>
#include <arduino.h>
#include <HX711.h>
#include <bluefruit.h>

// -------------------config---------------------------------
// calibrate scale to newton!
// for calibration set the factor to 1. Put a weight of 1kg on the pedal of the installed crank
// Divide the weight that you get back by 10 and this is the the new calibration factor.
// 170mm=0.17m
#define factor 605
#define offset -862800
#define crankmeter 0.17
#define samples 450
#define dataPin PIN_WIRE1_SDA
#define clockPin PIN_WIRE1_SCL
#define SerialDebug
// -------------------------------------------------------------

HX711 force;

BLEService cyclingPowerService = BLEService("1818");
BLEDis bledis;
BLEBas blebas; 

BLECharacteristic cyclingPowerMeasurementChar = BLECharacteristic("2A63");
BLECharacteristic cyclingPowerFeatureChar = BLECharacteristic("2A65");
BLECharacteristic sensorLocatChar = BLECharacteristic("2A5D");


unsigned char powerMeasurementBuffer[8];
unsigned char powerFeatureBuffer[4];
unsigned char sensorLocatBuffer[1];

unsigned short power = 0;
unsigned short flags = 0x20;

float rpm = 0;
float newtons = 0;
float forceAvg = 0;

float forceTemp = 0;
float rpmAvg = 0;
float rpmTemp = 0;

unsigned short rotations = 0;
unsigned long crankTime = 0;

float gyroX, gyroY;

void setup() {
  #ifdef SerialDebug
    Serial.begin(9600);
  #endif

  pinMode (VBAT_ENABLE, OUTPUT);
  pinMode (PIN_PDM_PWR, OUTPUT);
  pinMode (PIN_LSM6DS3TR_C_INT1, INPUT);

  //set voltage divider pin low
  digitalWrite(VBAT_ENABLE, LOW);
  //deactivate microphone to save power
  digitalWrite(PIN_PDM_PWR, LOW);

  force.begin(dataPin, clockPin);
  force.set_scale(factor);
  force.set_offset(offset);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");

    while (1);
  }

  setupBLE();
}

void loop() {


  while(true){

    for(int i = 1; i <= samples; i++) {
      forceTemp = force.get_units() * -1;

      IMU.readGyroscope(gyroX, gyroY, rpmTemp);

      if(forceTemp > 0){
        forceAvg = forceAvg + forceTemp;
      }
      rpmAvg = rpmAvg + rpmTemp;
      
    }

    newtons = forceAvg / samples;
    rpm = (rpmAvg / samples) * -1;

    if (rpm < 0){
      rpm = 0;
    }

    forceAvg = 0;
    rpmAvg = 0;

    power = newtons * crankmeter * rpm * 0.21;

    if(millis() < crankTime){
      crankTime = 0;
    }

    if(rpm > 5){
      while(millis() > (crankTime + (60000 / rpm))){
        crankTime += (60000 / rpm);
        rotations++;
      }
    }

    #ifdef SerialDebug
      Serial.print(newtons);
      Serial.print(",");
      Serial.print(rpm);
      Serial.print(",");
      Serial.println(power);
      Serial.println(analogRead(PIN_LSM6DS3TR_C_INT1));
      //Serial.println(force.get_units(3));
      Serial.println(digitalRead(PIN_LSM6DS3TR_C_INT1));
    #endif

    powerMeasurementBuffer[0] = flags & 0xff;
    powerMeasurementBuffer[1] = (flags >> 8) & 0xff;
    powerMeasurementBuffer[2] = power & 0xff;
    powerMeasurementBuffer[3] = (power >> 8) & 0xff;
    powerMeasurementBuffer[4] = rotations & 0xff;
    powerMeasurementBuffer[5] = (rotations >> 8) & 0xff;
    powerMeasurementBuffer[6] = short(crankTime*1.024) & 0xff;
    powerMeasurementBuffer[7] = (short(crankTime*1.024) >> 8) & 0xff;

    cyclingPowerMeasurementChar.notify(powerMeasurementBuffer, 8);
  }

}

void setupBLE(){
  Bluefruit.begin();

  bledis.setManufacturer("openSource");
  bledis.setModel("OpenPowermeter");
  bledis.begin();

  blebas.begin();
  blebas.write(50);  

  cyclingPowerService.begin();

  sensorLocatBuffer[0] = 0x05;
  sensorLocatChar.setProperties(CHR_PROPS_READ);
  sensorLocatChar.setFixedLen(1);
  sensorLocatChar.begin();
  sensorLocatChar.write(sensorLocatBuffer,1);

  powerFeatureBuffer[0] = 0x00;
  powerFeatureBuffer[1] = 0x00;
  powerFeatureBuffer[2] = 0x00;
  powerFeatureBuffer[3] = 0x08;
  cyclingPowerFeatureChar.setProperties(CHR_PROPS_READ);
  cyclingPowerFeatureChar.setFixedLen(4);
  cyclingPowerFeatureChar.begin();
  cyclingPowerFeatureChar.write(powerFeatureBuffer, 4);

  powerMeasurementBuffer[0] = flags & 0xff;
  powerMeasurementBuffer[1] = (flags >> 8) & 0xff;
  powerMeasurementBuffer[2] = power & 0xff;
  powerMeasurementBuffer[3] = (power >> 8) & 0xff;
  powerMeasurementBuffer[4] = rotations & 0xff;
  powerMeasurementBuffer[5] = (rotations >> 8) & 0xff;
  powerMeasurementBuffer[6] = short(crankTime*1.024) & 0xff;
  powerMeasurementBuffer[7] = (short(crankTime*1.024) >> 8) & 0xff;
  cyclingPowerMeasurementChar.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  cyclingPowerMeasurementChar.setFixedLen(8);
  cyclingPowerMeasurementChar.begin();
  cyclingPowerMeasurementChar.write(powerMeasurementBuffer, 8);



  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(cyclingPowerService);
  Bluefruit.Advertising.addName();

  Bluefruit.autoConnLed(false);
  
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);   
  Bluefruit.Advertising.setFastTimeout(30);      
  Bluefruit.Advertising.start(0);                 
}