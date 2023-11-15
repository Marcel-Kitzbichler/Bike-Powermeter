#include <LSM6DS3.h>
#include <HX711.h>

HX711 force;

LSM6DS3 IMU(I2C_MODE, 0x6A);

// -------------------config---------------------------------
#define factor 1200
#define rate 20   //time between samples
#define dataPin P0_4
#define clockPin P0_5
// -------------------------------------------------------------

void setup(){
  Serial.begin(9600);

  pinMode (P0_14, OUTPUT);
  digitalWrite(P0_14, LOW);

  if (IMU.begin() != 0) {
    while(1);
  }

  force.begin(dataPin, clockPin);
  force.set_scale(factor);
  force.tare(30);
}
void loop(){
  Serial.print(force.get_units(1));
  Serial.print(",");
  Serial.println(IMU.readFloatGyroZ()/6);
  delay(rate);
}