//....................................MPU6050 define........................................
#include <ESP.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BluetoothSerial.h>
#include <ESP32Servo.h>

//........................................Servo define....................................

// Use the ESP32Servo library for ESP32
#define servo_control_pin 15  // Define the pin for the servo signal
Servo servo;                  // Create a servo object to control the servo


BluetoothSerial SerialBT;
Adafruit_MPU6050 mpu;

//.........................................Define motor control pins......................
#define IN1 4
#define IN2 16
#define IN3 17
#define IN4 5
#define ENA 2   // PWM for left motor
#define ENB 18  // PWM for right motor

//....................................  Communication functions and structure ..............................
int servo_pos=90;

void setup() {
  Serial.begin(115200);
  SerialBT.begin();
  Wire.begin();


 // Attach the servo to the defined pin
  servo.attach(servo_control_pin);
  servo.write(servo_pos);
  delay(1000);

  //................................................ MPU6050 setup ....................................
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1)
      ;
  }

  Serial.println("MPU6050 connected!");


  // Configure MPU6050 settings
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);     // Sensitivity ±500°/s
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);  // Noise filtering


  //........................................... Set motor pins as outputs .................................
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
}

////......................PID...........................

float kp = 100, ki = 4, kd = 2;  // best kp=100, ki=4 and kd=2


float feedback = 0;
float error, previous_error = 0;
float P, I = 0, D;
float PID_out;
unsigned long time1;
unsigned long time2;

void PID_Forward(int ref) {

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  time2 = millis();
  float dt = (time2 - time1) / 1000.0;

  float gyroZ = g.gyro.z * 180 / PI;
  //if (gyroZ > 0.05)                  // in feedback small value has significance
  feedback += dt * gyroZ;

  error = ref - feedback;
  P = kp * error;
  I += error * dt;
  float derivative = (error - previous_error) / dt;
  D = kd * derivative;
  PID_out = P + (ki * I) + D;
  int motor_speed_change = constrain(PID_out, -105, 105);
  int motor_speed = 150;
  int motorR = motor_speed + motor_speed_change;
  int motorL = motor_speed - motor_speed_change;
  // Serial.print("R = ");
  // Serial.print(motorR);
  // Serial.print("   L = ");
  // Serial.println(motorL);
  analogWrite(ENA, motorR);
  analogWrite(ENB, motorL);

  previous_error = error;
  time1 = time2;
}

void PID_Backward(int ref) {

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  time2 = millis();

  float dt = (time2 - time1) / 1000.0;

  float gyroZ = g.gyro.z * 180 / PI;
  //if (gyroZ > 0.05)                  // in feedback small value has significance
  feedback += dt * gyroZ;

  error = ref - feedback;
  P = kp * error;
  I += error * dt;
  float derivative = (error - previous_error) / dt;
  D = kd * derivative;
  PID_out = P + (ki * I) + D;
  int motor_speed_change = constrain(PID_out, -105, 105);
  int motor_speed = 150;
  int motorR = motor_speed - motor_speed_change;
  int motorL = motor_speed + motor_speed_change;
  // Serial.print("R = ");
  // Serial.print(motorR);
  // Serial.print("   L = ");
  // Serial.println(motorL);
  analogWrite(ENA, motorR);
  analogWrite(ENB, motorL);

  previous_error = error;
  time1 = time2;
}

///////........................................ Main Loop function ......................................
void loop() {

  char ch;
  if (SerialBT.available()) {
    ch = SerialBT.read();
    if (ch == 'F') {
      time1 = millis();
      PID_Forward(0);
      forward();
      delay(20);
    } else if (ch == 'B') {
      time1 = millis();
      PID_Backward(0);
      backward();
      delay(20);
    }
    else if(ch=='L')
    {
      servo_pos++;
      servo_pos = min(180,servo_pos);
      servo.write(servo_pos);
      delay(30);
    }
    else if(ch=='R')
    {
      servo_pos--;
      servo_pos = max(0,servo_pos);
      servo.write(servo_pos);
      delay(30);
    }
     else
      motor_stop();
  }
}
