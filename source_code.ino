/*
 * LiftBot: Self-Guided Robot car with Accelerometer for Packet Stability
 * Components Used:
    - 1 Arduino Nano
    - 1 Accelerometer Gyroscope Sensor, MPU6050 Module
    - 1 Servo Motor SG90 ( 180° Degrees )
    - 2 IR Sensors
    - 1 L298 Motor Shield Mini
    - Battery Support
*/

#include <Arduino_FreeRTOS.h>
#include <Wire.h>
#include <Servo.h>
#include <MPU6050.h>

MPU6050 mpu;
Servo servoMotor;

int16_t ax, ay, az;
int16_t gx, gy, gz;

int16_t ServoPos = 90;

// Motors' Pins
int in1 = 2;
int in2 = 3;
int in3 = 4;
int in4 = 5;

// Infra red senors' Pins
int right = 6;
int left = 7;

// Servo Motor's pin
int servoPin = 8;

TaskHandle_t gyroHandler;
TaskHandle_t mvtHandler;

void gyroTask(void *pvParameters);
void mvtTask(void *pvParameters);

void setup() {
  Wire.begin();
  Serial.begin(9600);

  pinMode(right, INPUT);
  pinMode(left, INPUT);
  
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  mpu.initialize();

  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

  servoMotor.attach(servoPin);
  servoMotor.write(90);

  xTaskCreate(gyroTask, "GyroTask", 128, NULL, 1, &gyroHandler);
  xTaskCreate(mvtTask, "MvtTask", 128, NULL, 2, &mvtHandler);
}

void loop() { // Nothing here because we are using Threads (RTOS)
}

void gyroTask(void *pvParameters) {
  while (true) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Adjust the servo position based on gyroscopic data
    ax = map(ax, -17000, 17000, 0, 180);  // Adjust based on Gyro_x reading
    Serial.println(ax);
    servoMotor.write(ax);

    delay(250);
  }
}

void mvtTask(void *pvParameters) {
  while (true) {

    // Line detected by both sensor
    if (digitalRead(left) == LOW && digitalRead(right) == LOW) {
      // Forward
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
    }

    // Line detected by left sensor
    else if (digitalRead(left) == LOW && !digitalRead(right) == LOW) {
      // Left
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
    }

    // Line detected by right sensor
    else if (!digitalRead(left) == LOW && digitalRead(right) == LOW) {
      // Right
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
    }

    // Line detected by none
    else if (!digitalRead(left) == LOW && !digitalRead(right) == LOW) {
      //stop
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
    }
  }
}
