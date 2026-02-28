// Basic demo for accelerometer/gyro readings from Adafruit LSM6DS3TR-C

#include <Adafruit_LSM6DS3TRC.h>
#include "Wire.h"
#include <SoftwareSerial.h>

// For SPI mode, we need a CS pin
#define LSM_CS 10
// For software-SPI mode we need SCK/MOSI/MISO pins
#define LSM_SCK 13
#define LSM_MISO 12
#define LSM_MOSI 11

#define TCAADDR 0x70

const uint8_t IMU_PORTS[3] = {1, 2, 7};
Adafruit_LSM6DS3TRC imu[3];


SoftwareSerial Serial1(0, 1); // RX, TX pins


void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void setup(void) {

  Wire.begin();
  delay(100);

  Serial.begin(9600);
  Serial1.begin(9600);

  while (!Serial) delay(10);

  for (uint8_t t=0; t<8; t++) {
      tcaselect(t);
      Serial.print("TCA Port #"); Serial.println(t);

      Serial.println("Init 3 IMUs...");
  }
  for (int k = 0; k < 3; k++) {
    tcaselect(IMU_PORTS[k]);
    delay(5);

    Serial.print("IMU "); Serial.print(k);
    Serial.print(" on TCA port "); Serial.print(IMU_PORTS[k]);
    Serial.print(": ");

    if (!imu[k].begin_I2C()) {
      Serial.println("FAILED");
      while (1) delay(10);
    }
    Serial.println("OK");

    // Configure each IMU (do it per-sensor)
    imu[k].configInt1(false, false, true);
    imu[k].configInt2(false, true, false);
  }

  Serial.println("All IMUs initialized.");
}


void loop() {

  Serial1.print("1");

  /*sensors_event_t accel, gyro, temp;

  for (int k = 0; k < 3; k++) {
    tcaselect(IMU_PORTS[k]);
    delay(2);

    imu[k].getEvent(&accel, &gyro, &temp);

    Serial.print("IMU "); Serial.print(k);
    Serial.print(" (port "); Serial.print(IMU_PORTS[k]); Serial.println(")");

    Serial.print("  Temp: "); Serial.print(temp.temperature); Serial.println(" C");

    Serial.print("  Accel: ");
    Serial.print(accel.acceleration.x); Serial.print(", ");
    Serial.print(accel.acceleration.y); Serial.print(", ");
    Serial.print(accel.acceleration.z); Serial.println(" m/s^2");

    Serial.print("  Gyro:  ");
    Serial.print(gyro.gyro.x); Serial.print(", ");
    Serial.print(gyro.gyro.y); Serial.print(", ");
    Serial.print(gyro.gyro.z); Serial.println(" rad/s");

    Serial.println();
    delay(50);
  }


  delay(100);

  */
  
  //  // serial plotter friendly format

  //  Serial.print(temp.temperature);
  //  Serial.print(",");

  //  Serial.print(accel.acceleration.x);
  //  Serial.print(","); Serial.print(accel.acceleration.y);
  //  Serial.print(","); Serial.print(accel.acceleration.z);
  //  Serial.print(",");

  // Serial.print(gyro.gyro.x);
  // Serial.print(","); Serial.print(gyro.gyro.y);
  // Serial.print(","); Serial.print(gyro.gyro.z);
  // Serial.println();
  //  delayMicroseconds(10000);
}