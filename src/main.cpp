//#include "code_esp32/functions_esp32.hpp"
#include "code_esp8266/functions_esp8266.hpp"

void setup() {
  Serial.begin(SERIAL_SPEED);
  
  setupMotors();

  pinMode(BUZZER, OUTPUT);
  delay(2000);
  digitalWrite(BUZZER, HIGH);
  delay(70);
  digitalWrite(BUZZER, LOW);
}

void loop() {
  currentT = millis();

  if(currentT - previousT_1 >= loop_time) {
    angle_calc();
    if(balancing_point == 1) {
      angleX -= offsets.X1;
      angleY -= offsets.Y1;
      if(abs(angleX) > 8 || abs(angleY) > 8) {vertical = false;}
    } 
    else if(balancing_point == 2) {
      angleX -= offsets.X2;
      angleY -= offsets.Y2;
      if(abs(angleY) > 5) {vertical = false;}
    } 
    else if(balancing_point == 3) {
      angleX -= offsets.X3;
      angleY -= offsets.Y3;
      if(abs(angleY) > 5) {vertical = false;}
    } 
    else if(balancing_point == 4) {
      angleX -= offsets.X4;
      angleY -= offsets.Y4;
      if(abs(angleX) > 5) {vertical = false;}
    }
    
    if(abs(angleX) < 8 || abs(angleY) < 8) {Gyro_amount = 0.996;} // fast restore angle
    else {Gyro_amount = 0.1;}

    if(vertical && calibrated && !calibrating) {    
      digitalWrite(BRAKE, HIGH);
      gyroZ = GyZ / 131.0; // Convert to deg/s
      gyroY = GyY / 131.0; // Convert to deg/s
      gyroX = GyX / 131.0; // Convert to deg/s

      gyroYfilt = alpha * gyroY + (1 - alpha) * gyroYfilt;
      gyroZfilt = alpha * gyroZ + (1 - alpha) * gyroZfilt;
      
      int pwm_X = constrain(K1 * angleX + K2 * gyroZfilt + K3 * motor_speed_X, -255, 255);
      int pwm_Y = constrain(K1 * angleY + K2 * gyroYfilt + K3 * motor_speed_Y, -255, 255);
      motor_speed_X += pwm_X; 
      motor_speed_Y += pwm_Y;
      
      if(balancing_point == 1) {XY_to_threeWay(-pwm_X, -pwm_Y);} 
      else if(balancing_point == 2) {Motor1_control(pwm_Y);} 
      else if(balancing_point == 3) {Motor2_control(-pwm_Y);} 
      else if(balancing_point == 4) {Motor3_control(pwm_X);}
    } 
    else {
      XY_to_threeWay(0, 0);
      digitalWrite(BRAKE, LOW);
      motor_speed_X = 0;
      motor_speed_Y = 0;
    }
    previousT_1 = currentT;
  }
  
  if(currentT - previousT_2 >= 2000) {    
    battVoltage((double)analogRead(VBAT) / 207); 
    previousT_2 = currentT;
  }
}