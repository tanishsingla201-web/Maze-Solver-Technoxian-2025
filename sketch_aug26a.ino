// Motor Pins

//IN1,IN2=LEFT
//IN3,IN4=RIGHT
#include <Wire.h>

#define MPU_ADDR 0x68
#define PWR_MGMT_1  0x6B
#define GYRO_CONFIG 0x1B
#define GYRO_ZOUT_H 0x47


// === Initialize MPU9250 ===
void initMPU() {
  Wire.begin();
  
  // Wake up
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission();

  // Gyro ±250 dps
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_CONFIG);
  Wire.write(0x00);
  Wire.endTransmission();
}
const int buttonPin = 9;
unsigned long lastButtonPress = 0;


// Integration timing
float deltat = 0.0f;
unsigned long lastUpdate = 0;
bool motorsEnabled = false;

int trigf=10,echof=11,trigr=12,echor=13,trigl=14,echol=15;
int in1 = 2;
int in2 = 3;
int in3 = 4;
int in4 = 7;

int en1 = 5;
int en2 = 6;
int stdby = 8;
// PID gains (start with these, tuned later)
float Kp = 1.8, Ki = 0.00, Kd = 1.0;

float error = 0 ;
float integral = 0;
float lastError = 0;
unsigned long lastTime = 0;
const unsigned long loopMs = 50; // 50 ms control loop

const int baseSpeed = 100; // starting base PWM (0-255)
const int maxSpeed = 150 ;
const int integralLimit = 200; // tune later

float yaw = 0.0f;
float yawZero = 0.0f;    // reference so relative yaw starts at 0
const float ANGLE_TOL = 3.0f;  // degrees tolerance for stopping the turn

const int corridorThresh = 22;       // "wall is present" threshold (cm)
const int stopFront   = 7;     


float readGyroZ() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_ZOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);

  int16_t gz_raw = Wire.read() << 8 | Wire.read();
  return gz_raw / 131.0;  // dps
}

// === Turn by target angle ===
void turnByAngle(float targetAngleDeg) {
  float yaw = 0;
  lastTime = millis();

  // Decide direction
  if (targetAngleDeg > 0) {
    // Turning right
    digitalWrite(in1,HIGH);   // your motor code here
    digitalWrite(in2,LOW);   
    digitalWrite(in3,LOW);   
    digitalWrite(in4,HIGH);
    analogWrite(en1, 50); 
    analogWrite(en2, 50);  
  } 
        
  else {
    // Turning left
    digitalWrite(in1,LOW);   // your motor code here
    digitalWrite(in2,HIGH);   
    digitalWrite(in3,HIGH);   
    digitalWrite(in4,LOW); 
    analogWrite(en1, 50); 
    analogWrite(en2, 50);
  }

  while (abs(yaw) < abs(targetAngleDeg)) {
    float gz_dps = readGyroZ();

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;
    yaw += gz_dps * dt;

    Serial.print("Yaw: ");
    Serial.println(yaw);
  }

  // Stop motors after turn
  stop();

  Serial.print("Turn complete: ");
  Serial.println(yaw);
}


void setup() {
  Serial.begin(115200);

  pinMode(trigf,OUTPUT);
  pinMode(trigl,OUTPUT);
  pinMode(trigr,OUTPUT);
  pinMode(echof,INPUT);
  pinMode(echol,INPUT);
  pinMode(echor,INPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(stdby,OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP) ;

  pinMode(en1, OUTPUT);
  pinMode(en2, OUTPUT);

  analogWrite(en1, baseSpeed);
  analogWrite(en2, baseSpeed);
  digitalWrite(stdby, HIGH);

  Wire.begin();
  initMPU();
  Serial.println("MPU Initialized!");

}

void stop(){
  digitalWrite(in1,LOW);
  digitalWrite(in2,LOW);
  digitalWrite(in3,LOW);
  digitalWrite(in4,LOW);
  delay(1000);
}

void turnRight() {
  
  turnByAngle(-90);
  delay(2000);
  
  
}

void turnBack() {
  
  turnByAngle(180);
  delay(2000);
  
}

void turnLeft() {
  
  turnByAngle(90);
  delay(2000);
}

// void skipACell(){
 
//   digitalWrite(in1,HIGH);
//   digitalWrite(in2,LOW);
//   digitalWrite(in3,LOW);
//   digitalWrite(in4,HIGH);
//   delay(900);
// }

void toggleMotors() {
  static bool lastState = HIGH;
  bool currentState = digitalRead(buttonPin);

  if (lastState == HIGH && currentState == LOW) {  // Button pressed
    if (millis() - lastButtonPress > 250) {       // Debounce 250ms
      motorsEnabled = !motorsEnabled;
      digitalWrite(stdby, motorsEnabled ? HIGH : LOW);
      lastButtonPress = millis();
    }
  }
  lastState = currentState;
}



float getDist(int trig,int echo){
  float distance = 0;
  for(int i=0;i<3;i++){
  digitalWrite(trig, LOW); 
  delayMicroseconds(2);
  digitalWrite(trig, HIGH); 
  delayMicroseconds(10); 
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH, 20000); 
  distance += (duration*0.034)/2.0;
  }
  
  return distance/3.0;  // cm
}

void PID(){

  // unsigned long now = millis();
  // if (now - lastTime < loopMs) return;
  // float dt = (now - lastTime) / 1000.0; // seconds
  // lastTime = now;

  float frontDist = getDist(trigf, echof);
  float leftDist  = getDist(trigl, echol);   // cm
  float rightDist = getDist(trigr, echor);  // cm

  // float error = getDist(trigl, echol) - getDist(trigr, echor);
  if (leftDist < corridorThresh && rightDist < corridorThresh) {
    error = rightDist - leftDist;
  }
  else {
    // No walls → go straight
    error = 0;
  }



  // Integral with anti-windup
  // integral += error * dt;

  integral += error;
  if (integral > integralLimit) integral = integralLimit;
  if (integral < -integralLimit) integral = -integralLimit;

  // Derivative (optionally low-pass)
  // float derivative = (error - lastError) / dt;

  float derivative = (error - lastError);
  // derivative = derivativeFilter(derivative); // optional

  float correction = Kp * error + Ki * integral + Kd * derivative;

  lastError = error;

  // Map correction to motor speeds (differential)
  int leftPWM  = constrain((int)(baseSpeed + correction), 0, maxSpeed);
  int rightPWM = constrain((int)(baseSpeed - correction), 0, maxSpeed);

  // // Compensate motor deadband if needed:
  // leftPWM  = applyDeadband(leftPWM);
  // rightPWM = applyDeadband(rightPWM);

  analogWrite(en1,  leftPWM);
  analogWrite(en2, rightPWM);

  Serial.print("err:"); Serial.print(error);
  Serial.print(" corr:"); Serial.print(correction);
  Serial.print(" LPWM:"); Serial.print(leftPWM);
  Serial.print(" RPWM:"); Serial.print(rightPWM);
  Serial.print(" Front: "); Serial.println(frontDist);
  Serial.print(" left:"); Serial.println(leftDist);
  Serial.print(" right:"); Serial.print(rightDist);
}

void moveForward() {
   digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);
}

void goForward(){
  //think of a way to keep it from being called again and again in  the loop...(it will get called every time we exit the while loop of leftPriority..)
  while(getDist(trigr,echor)<25 || getDist(trigl,echol)<25){
    moveForward();
  }
  // stop();
}

// void turnRight(){
//    //sirf ghumayega
//   digitalWrite(in1,LOW);
//   digitalWrite(in2,HIGH);
//   digitalWrite(in3,LOW);
//   digitalWrite(in4,LOW);
//   delay(180);
// }

// void turnLeft(){
//   //sirf ghumayega
//   digitalWrite(in1,LOW);
//   digitalWrite(in2,LOW);
//   digitalWrite(in3,HIGH);
//   digitalWrite(in4,LOW);
//   delay(180);
// }


void loop(){
  toggleMotors();
  if (!motorsEnabled) return;

  goForward();
  PID();
  if(getDist(trigl,echol)>25){
    stop();
    turnLeft();
    return ;
  }
  else if(getDist(trigr, echor)>25)
  {
    stop();
    turnRight();
    return ;
  }
}