#include <Keyboard.h>
#include <microDS18B20.h>
#include "max6675.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "Wire.h"

#define PIN_TEMP    22
#define PIN_MPU_INT 23
#define CLK_TMP     1
#define CS_TMP      2
#define DO_TMP      4

MicroDS18B20<PIN_TEMP> sensor;
MAX6675 thermocouple(CLK_TMP, CS_TMP, DO_TMP);
Adafruit_MPU6050 mpu;

void Read_MPU_ISR() {
  Serial.println("INTERRUPT");
}

void reportData(float t1, float t2) {
  Keyboard.print(t1);
  Keyboard.press(KEY_TAB);
  Keyboard.releaseAll();
  Keyboard.print(t2);
  Keyboard.press(KEY_KP_ENTER);
  Keyboard.releaseAll();
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  
  mpu.begin();
  // Initialize MPU6050 and configure it for motion detection interrupt
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setMotionDetectionThreshold(10); // Set your desired motion detection threshold
  mpu.setMotionDetectionDuration(5); // Set the duration for motion detection
  mpu.setInterruptPinLatch(true);
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  attachInterrupt(digitalPinToInterrupt(PIN_MPU_INT), Read_MPU_ISR, RISING); // Trigger interrupt on rising edge
}

void loop() {
  // put your main code here, to run repeatedly:
  sensor.requestTemp();
  float temperatureLocal = sensor.getTemp();
  float temperatureRemote = thermocouple.readCelsius();
  reportData(temperatureLocal, temperatureRemote);

  delay(1000);
}
