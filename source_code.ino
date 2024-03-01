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

int16_t ax, ay, az, gx, gy, gz;

const int MPU_addr = 0x68;

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

const double R_angle = 0.0067;  // Measurement noise covariance
const double Q_angle = 0.001; // Process noise covariance
const double dt = 0.015;      // Time step (15ms in my configuration)

double angle = 90;
double P[2][2] = {{1, 0}, {0, 1}}; // Initial error covariance matrix
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

  mpu.CalibrateGyro();

  servoMotor.attach(servoPin);
  servoMotor.write(angle);

  xTaskCreate(gyroTask, "GyroTask", 128, NULL, 1, &gyroHandler);
  xTaskCreate(mvtTask, "MvtTask", 128, NULL, 2, &mvtHandler);
}

void loop() {  // Nothing here because we are using Threads (RTOS)
}

void gyroTask(void *pvParameters) {
  while (true) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    double gyroRate = gx / 131.0; 

    angle += dt * gyroRate;

    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_angle * dt;

    ax = map(ax, -18000, 18000, 0, 180);
    double y = ax; 
    double S = P[0][0] + R_angle;
    double K[2];  // Kalman gain
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    angle += K[0] * (y - angle);
    P[0][0] -= K[0] * P[0][0];
    P[0][1] -= K[0] * P[0][1];
    P[1][0] -= K[1] * P[0][0];
    P[1][1] -= K[1] * P[0][1];

    // Update servo position
    servoMotor.write(angle);


    // For Debugging
    // Serial.print("Angle: ");
    // Serial.println(angle);

    delay(15);
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
