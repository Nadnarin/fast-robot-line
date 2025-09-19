#include <Arduino.h>

// Motor A
#define AIN1 27
#define AIN2 14
#define PWMA 25

// Motor B
#define BIN1 26
#define BIN2 33
#define PWMB 32

// STANDBY
#define STBY 4

void setup() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
}

void loop() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 100); 
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMB, 100);

  delay(1000);

  // หยุดมอเตอร์
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);

  delay(1000);

  // // มอเตอร์ A เดินหน้า
  // digitalWrite(AIN1, LOW);
  // digitalWrite(AIN2, HIGH);
  // analogWrite(PWMA, 100); 

  // // มอเตอร์ B ถอยหลัง
  // digitalWrite(BIN1, HIGH);
  // digitalWrite(BIN2, LOW);
  // analogWrite(PWMB, 100);

  // delay(1000);

  // // หยุดมอเตอร์
  // analogWrite(PWMA, 0);
  // analogWrite(PWMB, 0);

  // delay(1000);
}
