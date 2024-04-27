// time to shine
#include <Pololu3piPlus32U4.h>
// #include <bits/stdc++.h
#include <Pololu3piPlus32U4BumpSensors.h>
#include <Pololu3piPlus32U4Buzzer.h>
using namespace Pololu3piPlus32U4;

Buzzer buzzer;   
Motors motors;  
BumpSensors bumpSensors;


const int ECHO_PIN = 2;   
const int TRIG_PIN = 3;


const float MAX_DISTANCE = 100.0;

const float DISTANCE_FACTOR = MAX_DISTANCE / 100;
const float STOP_DISTANCE = 20;


const float MOTOR_BASE_SPEED = 55;
const int MOTOR_MIN_SPEED = 10;
const float MOTOR_FACTOR = MOTOR_BASE_SPEED / 100;  

const float L_MOTOR_FACTOR = 1.0;
const float R_MOTOR_FACTOR = 1.0;
const float L_MOTOR_FACTOR_THRESHOLD = 80;
const float R_MOTOR_FACTOR_THRESHOLD = 80;


unsigned long usCm;
unsigned long usPm; 
const unsigned long US_PERIOD = 50; 


unsigned long motorCm;
unsigned long motorPm;
const unsigned long MOTOR_PERIOD = 20; 

float distance = 0;

void predict(){
  std::map<int, float> mp; // angle and distance
  unsigned long time = millis();
  digitalWrite(TRIG_PIN, LOW);                                    
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);                                    
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration1 = pulseIn(ECHO_PIN, HIGH, 30000);
  float distance1 = duration1 * 0.0343 / 2;
  Serial.println(distance1);
  motors.setSpeeds(35, -35);
  int angle = 0;
  while(millis() - time <= 500){
    Serial.println(distance1);
    delay(100);
    digitalWrite(TRIG_PIN, LOW);                                    
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);                                    
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    duration1 = pulseIn(ECHO_PIN, HIGH, 30000);
    distance1 = duration1 * 0.0343 / 2;
    map.insert({distance, angle});
    angle += 20;
  }
  printMap();

}

void setup() {                     
  Serial.begin(9600);
  pinMode(ECHO_PIN, INPUT); 
  pinMode(TRIG_PIN, OUTPUT);
  bumpSensors.calibrate();
  delay(100); 
}

void loop() {  
  bumpSensors.read();
  bool leftPressed = bumpSensors.leftIsPressed();
  bool rightPressed = bumpSensors.rightIsPressed();
  
  if (leftPressed || rightPressed) {
    buzzer.play(F("c32c32")); 
    predict();
      float leftSpeed = -55;
      float rightSpeed = -0;
      motors.setSpeeds(-(leftSpeed-1), -rightSpeed);
      motors.setSpeeds(+(leftSpeed-1), +rightSpeed);

      leftSpeed = -0;
      rightSpeed = -55;
      motors.setSpeeds(-leftSpeed, -rightSpeed+5);  
  }
  usReadCm();
  setMotors();  
  delay(50);
}
void usReadCm() {          
  usCm = millis();                                                    
  if (usCm > usPm + US_PERIOD) {                                
    digitalWrite(TRIG_PIN, LOW);                                    
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);                                    
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW)
    long duration = pulseIn(ECHO_PIN, HIGH, 30000);
    distance = duration * 0.0343 / 2;

    if (distance > MAX_DISTANCE) distance = MAX_DISTANCE; 
    if (distance == 0) distance = MAX_DISTANCE;

    usPm = usCm;

    delay(10);
  }
}


unsigned long millisElapsed = 0;

void setMotors() {  
  motorCm = millis();
  if (motorCm > motorPm + MOTOR_PERIOD) { 
    float leftSpeed = MOTOR_BASE_SPEED;
    float rightSpeed = MOTOR_BASE_SPEED; 

    if (leftSpeed < MOTOR_MIN_SPEED){ leftSpeed = MOTOR_MIN_SPEED; Serial.print("YES"); }
    if (rightSpeed < MOTOR_MIN_SPEED){ rightSpeed = MOTOR_MIN_SPEED; Serial.print("YES");}

    if (leftSpeed <= L_MOTOR_FACTOR_THRESHOLD) {
      leftSpeed *= L_MOTOR_FACTOR;
    }

    if (rightSpeed <= R_MOTOR_FACTOR_THRESHOLD) {
      rightSpeed *= R_MOTOR_FACTOR;
    }
    if (distance <= STOP_DISTANCE) {
      millisElapsed = 0;
      predict();
      leftSpeed = -55;
      rightSpeed = -0;
      motors.setSpeeds(-(leftSpeed-1), -rightSpeed);
      motors.setSpeeds(+(leftSpeed-1), +rightSpeed);
      leftSpeed = -0;
      rightSpeed = -55;
    }
    
    motors.setSpeeds(-leftSpeed, -rightSpeed);   
    motorPm = motorCm;
    // Serial.print(distance);
    // Serial.print("     => ");
    // Serial.println(millis());
  }
}

