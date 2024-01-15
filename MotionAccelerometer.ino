#include <Keyboard.h>
#include <microDS18B20.h>
#include "max6675.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "Wire.h"

#define PIN_TEMP    22
#define PIN_MPU_INT 23
#define PIN_SWITCH  24
#define CLK_TMP     1
#define CS_TMP      2
#define DO_TMP      4

#define REPORT_ON_SERIAL
#define REPORT_ON_KEYBOARD
#define TRIGGER_ON_SWITCH

#define TRIGGER_COOLDOWN_MS 2000

MicroDS18B20<PIN_TEMP> sensor;
MAX6675 thermocouple(CLK_TMP, CS_TMP, DO_TMP);
Adafruit_MPU6050 mpu;

volatile bool triggered = false;
volatile bool onCooldown = false;
long lastTriggerTime;

void ISR_MPU() {
  #ifdef TRIGGER_ON_MOTION
    Serial.println("INTERRUPT-MPU");
    if (!onCooldown) {
      triggered = true;
    }
  #endif
}

void ISR_SWITCH() {
  #ifdef TRIGGER_ON_SWITCH
    Serial.println("INTERRUPT-SWITCH");
    if (!onCooldown) {
      triggered = true;
    }
  #endif 
}

void reportData(float t1, float t2) {
  #ifdef REPORT_ON_SERIAL
    Serial.print(t1); Serial.print("\t"); Serial.println(t2);
  #endif

  #ifdef REPORT_ON_KEYBOARD
    Keyboard.print(t1);
    Keyboard.press(KEY_TAB);
    Keyboard.releaseAll();
    Keyboard.print(t2);
    Keyboard.press(KEY_KP_ENTER);
    Keyboard.releaseAll();  
  #endif
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  
  
  #ifdef TRIGGER_ON_MOTION
    // Configure accelerometer for motion interrupt
    mpu.begin();  
    mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setMotionDetectionThreshold(10);
    mpu.setMotionDetectionDuration(5);
    mpu.setInterruptPinLatch(true);
    mpu.setInterruptPinPolarity(true);
    mpu.setMotionInterrupt(true);
    attachInterrupt(digitalPinToInterrupt(PIN_MPU_INT), ISR_MPU, RISING); 
  #endif

  #ifdef TRIGGER_ON_SWITCH
    // Configure switch for interrupt
    pinMode(PIN_SWITCH, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_SWITCH), ISR_SWITCH, FALLING);
  #endif
}

void loop() {
  // put your main code here, to run repeatedly:
  sensor.requestTemp();
  if (triggered) {
    lastTriggerTime = millis();
    onCooldown = true;
    float temperatureLocal = sensor.getTemp();
    float temperatureRemote = thermocouple.readCelsius();
    reportData(temperatureLocal, temperatureRemote);
  }

  if (onCooldown) {
    if ((millis() - lastTriggerTime) > TRIGGER_COOLDOWN_MS) {
      onCooldown = false;
    }
  }
}
