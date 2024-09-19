#include <LSM6DS3.h>
#include <HX711.h>

HX711 force;

LSM6DS3 IMU(I2C_MODE, 0x6A);

// -------------------config---------------------------------
#define factor 605
#define offset -864000
#define dataPin PIN_WIRE1_SDA
#define clockPin PIN_WIRE1_SCL
// -------------------------------------------------------------

void setup(){
  Serial.begin(9600);

  pinMode (VBAT_ENABLE, OUTPUT);
  digitalWrite(VBAT_ENABLE, LOW);

  if (IMU.begin() != 0) {
    while(1);
  }

  force.begin(dataPin, clockPin);
  force.set_scale(factor);
  force.set_offset(offset);
}
void loop(){
  //Serial.print("Raw Loadcell Reading without factor and offset: ");
  //Serial.println(force.read_average(20));
  //Serial.print("Loadcell reading with factor and offset: ");
  Serial.println(force.get_units(20));
  //Serial.print("Gyroscope reading in rpm: ");
  //Serial.println(IMU.readFloatGyroZ()/6);
}