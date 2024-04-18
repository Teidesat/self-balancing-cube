#include <Arduino.h>
#include <Wire.h>

#include "variables_esp32.hpp"

void writeTo(byte device, byte address, byte value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

void beep() {
    digitalWrite(BUZZER, HIGH);
    delay(70);
    digitalWrite(BUZZER, LOW);
    delay(80);
}

void angle_calc() {
  Wire.beginTransmission(MPU6050);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 6, true);  
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B);                  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 6, true); 
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  // add mpu6050 offset values
  GyZ -= GyZ_offset;
  GyY -= GyY_offset;
  GyX -= GyX_offset;

  robot_angleX += GyZ * loop_time / 1000 / 65.536;   
  Acc_angleX = atan2(AcY, -AcX) * 57.2958;               // angle from acc. values * 57.2958 (deg/rad)
  robot_angleX = robot_angleX * Gyro_amount + Acc_angleX * (1.0 - Gyro_amount);

  robot_angleY += GyY * loop_time / 1000 / 65.536; 
  Acc_angleY = -atan2(AcZ, -AcX) * 57.2958;              // angle from acc. values * 57.2958 (deg/rad)
  robot_angleY = robot_angleY * Gyro_amount + Acc_angleY * (1.0 - Gyro_amount);

  angleX = robot_angleX;
  angleY = robot_angleY;
  
  if(abs(angleX - offsets.X2) < 2 && abs(angleY - offsets.Y2) < 0.6) {
    balancing_point = 2;
    if(!vertical) {beep();}
    vertical = true;
  } 
  else if(abs(angleX - offsets.X3) < 2 && abs(angleY - offsets.Y3) < 0.6) {
    balancing_point = 3;
    if(!vertical) {beep();}
    vertical = true;
  } 
  else if(abs(angleX - offsets.X4) < 0.6 && abs(angleY - offsets.Y4) < 2) {
    balancing_point = 4;
    if (!vertical) {beep();}
    vertical = true;
  } 
  else if(abs(angleX - offsets.X1) < 0.4 && abs(angleY - offsets.Y1) < 0.4) {
    balancing_point = 1;
    if(!vertical) {beep();}
    vertical = true;
  } 
}

void angle_setup() {
  Wire.begin();
  delay (100);
  writeTo(MPU6050, PWR_MGMT_1, 0);
  writeTo(MPU6050, ACCEL_CONFIG, accSens << 3); // Specifying output scaling of accelerometer
  writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3); // Specifying output scaling of gyroscope
  delay (100);

  for(int i = 0; i < 1024; i++) {
    angle_calc();
    GyZ_offset_sum += GyZ;
    delay(3);
  }
  GyZ_offset = GyZ_offset_sum >> 10;
  Serial.print("GyZ offset value = "); Serial.println(GyZ_offset);
  beep();
  
  for(int i = 0; i < 1024; i++) {
    angle_calc();
    GyY_offset_sum += GyY;
    delay(3);
  }
  GyY_offset = GyY_offset_sum >> 10;
  Serial.print("GyY offset value = "); Serial.println(GyY_offset);
  beep();
  
  for(int i = 0; i < 1024; i++) {
    angle_calc();
    GyX_offset_sum += GyX;
    delay(3);
  }
  GyX_offset = GyX_offset_sum >> 10;
  Serial.print("GyX offset value = "); Serial.println(GyX_offset);
  beep();
  beep();
}

void pwmSet(uint8_t channel, uint32_t value) {
  ledcWrite(channel, value);
}

void Motor1_control(int sp) {
  if(sp < 0) {
    digitalWrite(DIR1, LOW);
    sp = -sp;
  } 
  else {digitalWrite(DIR1, HIGH);}
  pwmSet(DIR1, sp > 255 ? 255 : 255 - sp);
}

void Motor2_control(int sp) {
  if(sp < 0) {
    digitalWrite(DIR2, LOW);
    sp = -sp;
  } 
  else {digitalWrite(DIR2, HIGH);}
  pwmSet(DIR2, sp > 255 ? 255 : 255 - sp);
}

void Motor3_control(int sp) {
  if(sp < 0) {
    digitalWrite(DIR3, LOW);
    sp = -sp;
  } 
  else {digitalWrite(DIR3, HIGH);}
  pwmSet(DIR3, sp > 255 ? 255 : 255 - sp);
}

void XY_to_threeWay(float pwm_X, float pwm_Y) {
  int16_t m1 = round(0.5 * pwm_X - 0.75 * pwm_Y);  
  int16_t m2 = round(0.5 * pwm_X + 0.75 * pwm_Y);
  int16_t m3 = -pwm_X;  
  m1 = constrain(m1, -255, 255);
  m2 = constrain(m2, -255, 255);
  m3 = constrain(m3, -255, 255);
  Motor1_control(m1);
  Motor2_control(m2);
  Motor3_control(m3);
}

void setupMotors() {
  //set up brake pin
  pinMode(BRAKE, OUTPUT);
  digitalWrite(BRAKE, HIGH);

  // set up motors
  pinMode(DIR1, OUTPUT);
  Motor1_control(0);
  pinMode(DIR2, OUTPUT);
  Motor2_control(0);
  pinMode(DIR3, OUTPUT);
  Motor3_control(0);

  // set up motor1
  pinMode(DIR1, OUTPUT);
  ledcSetup(PWM1_CH, BASE_FREQ, TIMER_BIT);
  ledcAttachPin(PWM1, PWM1_CH);
  Motor1_control(0);
  
  // set up motor2
  pinMode(DIR2, OUTPUT);
  ledcSetup(PWM2_CH, BASE_FREQ, TIMER_BIT);
  ledcAttachPin(PWM2, PWM2_CH);
  Motor2_control(0);
  
  // set up motor3
  pinMode(DIR3, OUTPUT);
  ledcSetup(PWM3_CH, BASE_FREQ, TIMER_BIT);
  ledcAttachPin(PWM3, PWM3_CH);
  Motor3_control(0);

  calibrated = true;
  offsets.ID1 = 99;
  offsets.ID2 = 99;
  offsets.ID3 = 99;
}

void battVoltage(double voltage) {
  if(voltage > 8 && voltage <= 9.5) {digitalWrite(BUZZER, HIGH);} 
  else {digitalWrite(BUZZER, LOW);}
}