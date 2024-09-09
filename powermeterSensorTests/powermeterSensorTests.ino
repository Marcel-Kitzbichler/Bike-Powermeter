#include <LSM6DS3.h>
#include <HX711.h>

HX711 force;

LSM6DS3 IMU(I2C_MODE, 0x6A);

// -------------------config---------------------------------
#define factor 1200
#define offset 0
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
  force.set_offset(offset);
}
void loop(){
  Serial.print("Raw Loadcell Reading without factor and offset: ");
  Serial.println(force.read_average(20));
  Serial.print("Loadcell reading with factor and offset: ");
  Serial.println(force.get_units(20));
  Serial.print("Gyroscope reading in rpm: ");
  Serial.println(IMU.readFloatGyroZ()/6);
  delay(rate);
}