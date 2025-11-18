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


int trigf=10,echof=11,trigl=12,echol=13,trigr=14,echor=15;
// float durationf,distancef,durationl,distancel,durationr,distancer;

int in1=3,in2=2,in3=4,in4=7,en1=5,en2=6,stdby=8;
int status=0;
int s=1;
int b=-1;
int row=16 , col=0;

// PID gains (start with these, tuned later)
float Kp = 1.2;
float Ki = 0.0;    // start at 0
float Kd = 0.00;

float integral = 0;
float lastError = 0;
unsigned long lastTime = 0;
const unsigned long loopMs = 50; // 50 ms control loop

const int baseSpeed = 100; // starting base PWM (0-255)
const int integralLimit = 200; // tune later
const int maxSpeed= 150 ;

float yaw = 0.0f;
float yawZero = 0.0f;    // reference so relative yaw starts at 0
const float ANGLE_TOL = 3.0f;  // degrees tolerance for stopping the turn

int matrix[16][16];

enum Axis{Y,X,Y_prime,X_prime};
Axis axis;


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
    digitalWrite(in1,LOW);   // your motor code here
    digitalWrite(in2,HIGH);   
    digitalWrite(in3,LOW);   
    digitalWrite(in4,HIGH);   
  } else {
    // Turning left
    digitalWrite(in1,HIGH);   // your motor code here
    digitalWrite(in2,LOW);   
    digitalWrite(in3,HIGH);   
    digitalWrite(in4,LOW); 
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

  Serial.begin(9600);

  pinMode(trigf,OUTPUT);
  pinMode(trigl,OUTPUT);
  pinMode(trigr,OUTPUT);
  pinMode(echof,INPUT);
  pinMode(echol,INPUT);
  pinMode(echor,INPUT);
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  pinMode(in3,OUTPUT);
  pinMode(in4,OUTPUT);
  pinMode(en1,OUTPUT);
  pinMode(en2,OUTPUT);
  pinMode(stdby,OUTPUT);

  analogWrite(en1,255);           //speed baad mein control karni hai, chala ke dekhne ke baad...
  analogWrite(en2,255);

  for (int i=0;i<16;i++){
    for (int j=0;j<16;j++){
      matrix[i][j]=0;
    }
  }

  digitalWrite(stdby,HIGH);
}

void turnRight() {
  
  turnByAngle(90);
  delay(2000);

}

void turnBack() {
  
  turnByAngle(180);
  delay(2000);

}

void turnLeft() {
  
  turnByAngle(-90);
  delay(2000);

}


// float filteredLeftDistance() {
//   static float prev = 0;
//   float raw = getDist(trigl,echol);
//   prev = 0.7 * prev + 0.3 * raw; // exponential smoothing
//   return prev;
// }

// float filteredRightDistance() {
//   static float prev = 0;
//   float raw = getDist(trigr,echor);
//   prev = 0.7 * prev + 0.3 * raw; 
//   return prev;
// }

// int applyDeadband(int pwm) {
//   if (pwm == 0) return 0; // fully stop
//   const int deadband = 60; // adjust experimentally
//   if (pwm > 0 && pwm < deadband) pwm = deadband;
//   return pwm;
// }


void PID(){

  unsigned long now = millis();
  if (now - lastTime < loopMs) return;
  float dt = (now - lastTime) / 1000.0; // seconds
  lastTime = now;

  // float leftDist  = filteredLeftDistance();   // cm
  // float rightDist = filteredRightDistance();  // cm

  float error = getDist(trigl,echol) - getDist(trigr,echor);

  // Integral with anti-windup
  integral += error * dt;
  if (integral > integralLimit) integral = integralLimit;
  if (integral < -integralLimit) integral = -integralLimit;

  // Derivative (optionally low-pass)
  float derivative = (error - lastError) / dt;
  // derivative = derivativeFilter(derivative); // optional

  float correction = Kp * error + Ki * integral + Kd * derivative;

  lastError = error;

  // Map correction to motor speeds (differential)
  int leftPWM  = constrain((int)(baseSpeed - correction), 0, maxSpeed);
  int rightPWM = constrain((int)(baseSpeed + correction), 0, maxSpeed);

  // Compensate motor deadband if needed:
  // leftPWM  = applyDeadband(leftPWM);
  // rightPWM = applyDeadband(rightPWM);

  analogWrite(en1,  leftPWM);
  analogWrite(en2, rightPWM);

  Serial.print("err:"); Serial.print(error);
  Serial.print(" corr:"); Serial.print(correction);
  Serial.print(" L:"); Serial.print(leftPWM);
  Serial.print(" R:"); Serial.println(rightPWM);

}


float getDist(int trig,int echo){
  float distance=0;
  for(int i=0;i<3;i++){
  digitalWrite(trig, LOW); 
  delayMicroseconds(2);
  digitalWrite(trig, HIGH); 
  delayMicroseconds(10); 
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH); 
  distance += (duration*0.034)/2.0;
  }
  return distance/3.0;  // cm
}

void skipACell(){
  //to make it move JUST ONE CELL ahead...
       //bilkul thoda sa delay, to just make it move forward by one block, so that turn karne ke immediately baad ek aur turn detect kar ke ghoomta na reh jaye.
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  digitalWrite(in3,LOW);
  digitalWrite(in4,HIGH);
  delay(800);

  switch(axis){
      case 0:                        //0=Y,1=X,2=Y',3=X'(enum)
      row=row-1;
      break;

      case 1:
      col=col+1;
      break;

      case 2:
      row=row+1;
      break;

      case 3:
      col=col-1;
      break;
    }
    matrix[row][col]=s;
      s++;
  }

void moveFront(){
  

  unsigned long start = millis();
while (millis() - start < 900) {
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  digitalWrite(in3,LOW);
  digitalWrite(in4,HIGH);

  PID();  // keep correcting during movement
}

  switch(axis){
      case 0:                        //0=Y,1=X,2=Y',3=X'(enum)
      row=row-1;
      break;

      case 1:
      col=col+1;
      break;

      case 2:
      row=row+1;
      break;

      case 3:
      col=col-1;
      break;
    }
    matrix[row][col]=s;
      s++;
  }     

void stop(){
  //can add speed control here....and call it in move front...(so that speed is reduced on the basis of distance left between front obstacle and bot.. but that won't help much in our case, cuz we need to slow down at turns, rather than at a point when we can't go forward any further....)
  digitalWrite(in1,LOW);
  digitalWrite(in2,LOW);
  digitalWrite(in3,LOW);
  digitalWrite(in4,LOW);
  delay(1000);
}

void goForward(){
  //think of a way to keep it from being called again and again in  the loop...(it will get called every time we exit the while loop of leftPriority..)
  while(getDist(trigr,echor)<25||getDist(trigl,echol)<25){
    moveFront();
  }
  status=1;
}

void rightPriority(){
  moveFront();
  if(getDist(trigr,echor)>25){
    stop();
    analogWrite(en1,100);
    analogWrite(en2,100);
    turnRight();
    skipACell();
  }
  else if(getDist(trigf,echof)>25){
    moveFront();
  }
  else if(getDist(trigl,echol)>25){
    stop();
    analogWrite(en1,100);
    analogWrite(en2,100);
    turnLeft();
    skipACell();
  }
}

void leftPriority(){
  moveFront();
  if(getDist(trigl,echol)>25){
    stop();
    analogWrite(en1,100);
    analogWrite(en2,100);
    turnLeft();
    skipACell();
  }
  else if(getDist(trigf,echof)>25){
    moveFront();
  }
  else if(getDist(trigr,echor)>25){
    stop();
    analogWrite(en1,100);
    analogWrite(en2,100);
    turnRight();
    skipACell();
  }
}

void moveBack(){
  //to make it move JUST ONE CELL ahead...the digitalWrite part of this code will remain same as moveFront...
  //bilkul thoda sa delay, to just make it move forward by one block, so that turn karne ke immediately baad ek aur turn detect kar ke ghoomta na reh jaye.

   unsigned long start = millis();
  while (millis() - start < 900) {
   digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);

  PID();  // keep correcting during movement
}
       
  switch(axis){
      case 0:
      row=row-1;
      break;

      case 1:
      col=col+1;
      break;

      case 2:
      row=row+1;
      break;
      
      case 3:
      col=col-1;
      break;
    }
    matrix[row][col]=b;
      b--;
  }     


bool leftClear(){
  int r=row,c=col;
  switch(axis){
    case 2:
    if (matrix[r][c+1]==0){
      return true;
    }else return false;

    case 1:
    if(matrix[r-1][c]==0){
      return true;
    }else return false;

    case 3:
    if (matrix[r+1][c]==0){
      return true;
    }  else return false;

    case 0:
    if(matrix[r][c-1]==0){
      return true;
    }else return false;
    
  }
}


bool rightClear(){
  int r=row,c=col;
  switch(axis){
    case 2:
    if (matrix[r][c-1]==0){
      return true;
    }else return false;

    case 1:
    if(matrix[r+1][c]==0){
      return true;
    }else return false;

    case 3:
    if (matrix[r-1][c]==0){
      return true;
    }  else return false;

    case 0:
    if(matrix[r][c+1]==0){
      return true;
    }else return false;

  }
}

bool frontClear(){
  int r=row,c=col;
  switch(axis){
    case 2:
    if (matrix[r+1][c]==0){
      return true;
    }else return false;

    case 1:
    if(matrix[r][c+1]==0){
      return true;
    }else return false;

    case 3:
    if (matrix[r][c-1]==0){
      return true;
    }else return false;

    case 0:
    if(matrix[r-1][c]==0){
      return true;
    }else return false;

  }
}

bool leftVisited(){
  int r=row,c=col;
  switch(axis){
    case 2:
    if (matrix[r][c+1]>0){
      return true;
    }else return false;

    case 1:
    if(matrix[r-1][c]>0){
      return true;
    }else return false;

    case 3:
    if (matrix[r+1][c]>0){
      return true;
    }else return false;
    

    case 0:
    if(matrix[r][c-1]>0){
      return true;
    }else return false;
  }
}

bool rightVisited(){
  int r=row,c=col;
  switch(axis){
    case 2:
    if (matrix[r][c-1]>0){
      return true;
    }else return false;

    case 1:
    if(matrix[r+1][c]>0){
      return true;
    }else return false;

    case 3:
    if (matrix[r-1][c]>0){
      return true;
    }else return false;
    

    case 0:
    if(matrix[r][c+1]>0){
      return true;
    }else return false;
  }
}

bool frontVisited(){
  int r=row,c=col;
  switch(axis){
    case 2:
    if (matrix[r+1][c]>0){
      return true;
    }else return false;

    case 1:
    if(matrix[r][c+1]>0){
      return true;
    }else return false;

    case 3:
    if (matrix[r][c-1]>0){
      return true;
    }else return false;
    
    case 0:
    if(matrix[r-1][c]>0){
      return true;
    }else return false;
  }
}

void backtrack() {
  int r=row, c=col;
  int lastBlockedCell=s;
  turnBack();
  moveBack();
  while(!leftClear()||!rightClear()||!frontClear()){
  switch(axis) {
    case 0:
    if(getDist(trigr,echor)>25&&matrix[r][c+1]==lastBlockedCell-1){
      turnRight();
      moveBack();
    }
    else if(getDist(trigl,echol)>25&&matrix[r][c-1]==lastBlockedCell-1){
      turnLeft();
      moveBack();
    }
    else if(getDist(trigf,echof)>25&&matrix[r-1][c]==lastBlockedCell-1){
      moveBack();
    }else{
      if(getDist(trigr,echor)>25&&rightVisited()){
        turnRight();
        moveBack();
      }else if(getDist(trigl,echol)>25&&leftVisited()){
        turnLeft();
        moveBack();
      }else{
      moveBack();
      }
    }
    break;

    case 1:
    if(getDist(trigr,echor)>25&&matrix[r+1][c]==lastBlockedCell-1){
      turnRight();
      moveBack();
    }
    else if(getDist(trigl,echol)>25&&matrix[r-1][c]==lastBlockedCell-1){
      turnLeft();
      moveBack();
    }
    else if(getDist(trigf,echof)>25&&matrix[r][c+1]==lastBlockedCell-1){
      moveBack();
    }else{
      if(getDist(trigr,echor)>25&&rightVisited()){
        turnRight();
        moveBack();
      }else if(getDist(trigl,echol)>25&&leftVisited()){
        turnLeft();
        moveBack();
      }else{
      moveBack();
      }
    }
    break;

    case 2:
    if(getDist(trigr,echor)>25&&matrix[r][c-1]==lastBlockedCell-1){
      turnRight();
      moveBack();
    }
    else if(getDist(trigl,echol)>25&&matrix[r][c+1]==lastBlockedCell-1){
      turnLeft();
      moveBack();
    }
    else if(getDist(trigf,echof)>25&&matrix[r+1][c]==lastBlockedCell-1){
      moveBack();
    }else{
      if(getDist(trigr,echor)>25&&rightVisited()){
        turnRight();
        moveBack();
      }else if(getDist(trigl,echol)>25&&leftVisited()){
        turnLeft();
        moveBack();
      }else{
      moveBack();
      }
    }
    break;

    case 3:
    if(getDist(trigr,echor)>25&&matrix[r-1][c]==lastBlockedCell-1){
      turnRight();
      moveBack();
    }
    else if(getDist(trigl,echol)>25&&matrix[r+1][c]==lastBlockedCell-1){
      turnLeft();
      moveBack();
    }
    else if(getDist(trigf,echof)>25&&matrix[r][c-1]==lastBlockedCell-1){
      moveBack();
    }else{
      if(getDist(trigr,echor)>25&&rightVisited()){
        turnRight();
        moveBack();
      }else if(getDist(trigl,echol)>25&&leftVisited()){
        turnLeft();
        moveBack();
      }else{
      moveBack();
      }
    }
    break;
    }

  }    //while se bahar jiske bhi clear hone ki vajah se nikla hoga.. us cell par chala jayega..
  if(leftClear()){
    turnLeft();
    moveFront();
  }
  else if(rightClear()){
    turnRight();
    moveFront();
  }
  else if(frontClear()){
    moveFront();
  }

}
  

bool blocked() {
  if((getDist(trigr,echor)<25 && getDist(trigl,echol)<25 && getDist(trigf,echof)<25)/*||(!leftClear() && !rightClear() && !frontClear())*/||(getDist(trigr,echor)<25 && getDist(trigl,echol)<25 && !frontClear())||(getDist(trigl,echol)<25 &&getDist(trigf,echof)<25 && !rightClear())||(getDist(trigr,echor)<25 && getDist(trigf,echof)<25 && !leftClear())){
    return true;
  }else return false;
}

void loop() {

  if(status==0){
    axis=0;
    matrix[row][col]=s;
    s++;
    goForward();
  }
  while(!blocked()) {
    rightPriority();
  }
  
  backtrack();

  while(!blocked()) {
    leftPriority();
  }
  
  backtrack();


  sfsdfsdfsd
}



