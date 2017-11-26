// MPU-6050 Short Example Sketch
// By Arduino User JohnChi
// August 17, 2014
// Public Domain

#include<Wire.h>

//Light define
#define LED_FRONT_1 2
#define LED_FRONT_2 3
#define LED_BACK_1 4
#define LED_BACK_2 5

//Motors define
#define MOTOR_L_E 9   //ENA Right motor enable
#define MOTOR_L_F 7   //IN1 Left motor forward
#define MOTOR_L_B 8   //IN2 Left motor backforward
#define MOTOR_R_E 10  //ENB Right motor enable
#define MOTOR_R_B 6   //IN3 Right motor backforward
#define MOTOR_R_F 11  //IN4 Right motor forward

const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

void setup(){
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);

  pinMode(LED_FRONT_1, OUTPUT);
  pinMode(LED_FRONT_2, OUTPUT);
  pinMode(LED_BACK_1, OUTPUT);
  pinMode(LED_BACK_2, OUTPUT);

  pinMode(MOTOR_L_E, OUTPUT);
  pinMode(MOTOR_L_F, OUTPUT);
  pinMode(MOTOR_L_B, OUTPUT);
  pinMode(MOTOR_R_E, OUTPUT);
  pinMode(MOTOR_R_B, OUTPUT);
  pinMode(MOTOR_R_F, OUTPUT);
  digitalWrite(MOTOR_L_E, HIGH);
  digitalWrite(MOTOR_R_E, HIGH);
  
}

void loop(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(GyX);
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.println(GyZ);
//  delay(333);

/*
  digitalWrite(MOTOR_L_F, HIGH);
  analogWrite(MOTOR_L_E, abs(AcX/39));
*/

  /*
  if(AcX<-1000){
  digitalWrite(LED_FRONT_1, HIGH);
  digitalWrite(LED_FRONT_2, HIGH);
  digitalWrite(MOTOR_L_B, LOW);
  digitalWrite(MOTOR_R_B, LOW);
  digitalWrite(MOTOR_L_F, HIGH);
  digitalWrite(MOTOR_R_F, HIGH);
  } else if (AcX>200){
  digitalWrite(LED_BACK_1, HIGH);
  digitalWrite(LED_BACK_2, HIGH);
  digitalWrite(MOTOR_L_F, LOW);
  digitalWrite(MOTOR_R_F, LOW);
  digitalWrite(MOTOR_L_B, HIGH);
  digitalWrite(MOTOR_R_B, HIGH);
  } else {
    
  digitalWrite(LED_FRONT_1, LOW);
  digitalWrite(LED_FRONT_2, LOW);
  digitalWrite(LED_BACK_1, LOW);
  digitalWrite(LED_BACK_2, LOW);

  digitalWrite(MOTOR_L_B, LOW);
  digitalWrite(MOTOR_R_B, LOW);
  digitalWrite(MOTOR_L_F, LOW);
  digitalWrite(MOTOR_R_F, LOW);
  }
  */
  
}
