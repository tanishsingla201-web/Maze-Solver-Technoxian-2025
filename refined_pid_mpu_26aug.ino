// Motor Pins

// in1,in2 = LEFT
// in3,in4 = RIGHT
#include <Wire.h>

#define MPU_ADDR    0x68
#define PWR_MGMT_1  0x6B
#define GYRO_CONFIG 0x1B
#define GYRO_ZOUT_H 0x47

// === Ultrasonic pins ===
int trigf = 10, echof = 11;
int trigr = 12, echor = 13;
int trigl = 14, echol = 15;  // A0/A1 as digital OK on Uno

// === Motor pins (L293D / TB6612 style) ===
int in1 = 2;
int in2 = 3;
int in3 = 4;
int in4 = 7;
int PWMA = 5;   // PWM - left
int PWMB = 6;   // PWM - right
int stdby = 8; // for TB6612; on L293D you can ignore but keep HIGH

const int baseSpeed   = 100; // fwd speed
const int maxSpeed    = 150; // PID clamp
const int turnPWM     = 50; // turning speed (both wheels opposite)

float error = 0 ;

// === Button ===
const int buttonPin = 9;
unsigned long lastButtonPress = 0;
bool motorsEnabled = false;

// === PID gains (wall following) ===
float Kp = 1.4, Ki = 0.0, Kd = 0.0;
float integral = 0;
float lastError = 0;
const int integralLimit = 200; // no dt -> keep small

// === Control loop timing (not strictly used now) ===
unsigned long lastTime = 0;

// === Maze params ===
const int sideOpenThresh = 25;  // cm to consider cell opening
const int stopFront      = 7;  // cm hard stop to avoid collision
const int corridorThresh = 22;
// === Yaw integration ===
float yawZero = 0.0f;           // not used but kept for future
const float ANGLE_TOL = 3.0f;   // deg tolerance
float gzBias = 0.0f;            // gyro bias (dps)

// -------------------- MPU helpers --------------------
static inline int16_t readGyroZRaw() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_ZOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);
  int16_t gz_raw = (Wire.read() << 8) | Wire.read();
  return gz_raw;
}

float readGyroZ() {
  // Sensitivity: 131 LSB/(°/s) at ±250 dps
  return (readGyroZRaw() / 131.0f) - gzBias; // dps after bias
}

void initMPU() {
  Wire.begin();

  // Wake up device
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

void calibrateGyroZ(uint16_t samples = 500) {
  delay(300); // let it settle
  long sum = 0;
  for (uint16_t i = 0; i < samples; i++) {
    sum += readGyroZRaw();
    delay(2);
  }
  float avgRaw = sum / (float)samples;
  gzBias = avgRaw / 131.0f; // dps
}

// -------------------- Ultrasound --------------------
float getDist(int trig,int echo){
  float distance = 0;
  for(int i=0;i<5;i++){
  digitalWrite(trig, LOW); 
  delayMicroseconds(2);
  digitalWrite(trig, HIGH); 
  delayMicroseconds(10); 
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH, 20000); 
  distance += (duration*0.034)/2.0;
  }
  
  return distance/5.0;  // cm
}

float getDistF(int trig,int echo){
  float distance = 0;
  for(int i=0;i<5;i++){
  digitalWrite(trig, LOW); 
  delayMicroseconds(2);
  digitalWrite(trig, HIGH); 
  delayMicroseconds(10); 
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH, 20000); 
  distance += (duration*0.034)/2.0;
  }
  float approxDistance = distance/5.0;
  if (approxDistance == 0) {
    return 1000.0; 
  }
  
  return approxDistance; // cm
}
// -------------------- Motion primitives --------------------
void stop() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  delay(1000); // you wanted a 1s settle before turning
}

void setFwdDir() {
  digitalWrite(in1, HIGH); // left fwd
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH); // right fwd
  digitalWrite(in4, LOW);

  analogWrite(PWMA, baseSpeed);
  analogWrite(PWMB, baseSpeed);
}

void setInPlaceRight() {
  // left fwd, right back -> sharper, symmetric turn
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void setInPlaceLeft() {
  // left back, right fwd
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

// void moveForward() {
//   setFwdDir();
  
//}

void driveMotors(int left, int right) {
  // Left Motor
  if (left >= 0) { digitalWrite(in1, HIGH); digitalWrite(in2, LOW); }
  else           { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); }
  analogWrite(PWMA, abs(left));

  // Right Motor
  if (right >= 0) { digitalWrite(in3, HIGH); digitalWrite(in4, LOW); }
  else            { digitalWrite(in3, LOW);  digitalWrite(in4, HIGH); }
  analogWrite(PWMB, abs(right));
}

// -------------------- PID (wall following) --------------------
void PID() {
  digitalWrite(in1, HIGH); // left fwd
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH); // right fwd
  digitalWrite(in4, LOW);

  float frontDist = getDistF(trigf, echof);
  float leftDist  = getDist(trigl, echol);
  float rightDist = getDist(trigr, echor);

  // follow corridor center: positive error => steer right faster
  if (leftDist < corridorThresh && rightDist < corridorThresh) {
    // Corridor → balance between walls
    // Use distR - distL so positive error => steer RIGHT
    error = rightDist - leftDist;
  }

  else {
    // No walls → go straight
    error = 0;
  }


  // Integral w/o dt (Ki tuned for that); anti-windup
  integral += error;
  if (integral >  integralLimit) integral =  integralLimit;
  if (integral < -integralLimit) integral = -integralLimit;

  float derivative = (error - lastError);
  float correction = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;

  int leftSpeed  = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

  leftSpeed  = constrain(leftSpeed,  -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);
  driveMotors(leftSpeed, rightSpeed);


  // Optional: slow down if obstacle ahead
  // if (frontDist < stopFront) {
  //   analogWrite(PWMA, 0);
  //   analogWrite(PWMB, 0);
  // }

  // Debug (keep modest at 9600)
  Serial.print("err:"); Serial.print(error);
  Serial.print(" corr:"); Serial.print(correction);
  Serial.print(" LPWM:"); Serial.print(leftSpeed);
  Serial.print(" RPWM:"); Serial.print(rightSpeed);
  Serial.print(" F:"); Serial.print(frontDist);
  Serial.print(" L:"); Serial.print(leftDist);
  Serial.print(" R:"); Serial.println(rightDist);
}

// -------------------- Gyro-accurate turn --------------------
void turnByAngle(float targetAngleDeg) {
  // positive = right turn, negative = left turn
  unsigned long last = millis();
  float yaw = 0;

  // set direction based on sign & set turn speed
  if (targetAngleDeg > 0) setInPlaceRight();
  else                    setInPlaceLeft();
  analogWrite(PWMA, turnPWM);
  analogWrite(PWMB, turnPWM);

  const unsigned long failSafeMs = 4000; // 4s guard for 180°
  unsigned long start = millis();

  while (true) {
    unsigned long now = millis();
    float dt = (now - last) / 1000.0f;
    last = now;

    float gz_dps = readGyroZ();
    yaw += gz_dps * dt; // integrate

    float remaining = targetAngleDeg - yaw;
    if (fabs(remaining) <= ANGLE_TOL) break;

    if (now - start > failSafeMs) break; // safety exit

    Serial.print("Yaw: ");
    Serial.println(yaw);
  }

  stop(); // full stop at the end of the turn

  Serial.print("Turn complete: ");
  Serial.println(yaw);

}

// void turnRight() { turnByAngle(-90.0f); }
// void turnLeft()  { turnByAngle(+90.0f); }
// void turnBack()  { turnByAngle(180.0f); }

// -------------------- Button toggle --------------------
void toggleMotors() {
  static bool lastState = HIGH;
  bool currentState = digitalRead(buttonPin);
  if (lastState == HIGH && currentState == LOW) {
    if (millis() - lastButtonPress > 250) { // debounce
      motorsEnabled = !motorsEnabled;
      digitalWrite(stdby, motorsEnabled ? HIGH : LOW);
      lastButtonPress = millis();
    }
  }
  lastState = currentState;
}

// -------------------- Arduino setup/loop --------------------
void setup() {
  Serial.begin(115200);

  pinMode(trigf, OUTPUT);
  pinMode(trigl, OUTPUT);
  pinMode(trigr, OUTPUT);
  pinMode(echof, INPUT);
  pinMode(echol, INPUT);
  pinMode(echor, INPUT);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(stdby, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  digitalWrite(stdby, HIGH);
  analogWrite(PWMA, baseSpeed);
  analogWrite(PWMB, baseSpeed);

  Wire.begin();
  initMPU();
  calibrateGyroZ(600); // IMPORTANT: keep bot still during boot
  Serial.println("MPU initialized & gyro calibrated.");
}

void loop() {
  toggleMotors();
  if (!motorsEnabled) return;

  // Read openings once to avoid duplicate sonar hits
  float left  = getDist(trigl, echol);
  float right = getDist(trigr, echor);
  float front = getDistF(trigf, echof);

  // Safety: if front is blocked, turn (left-priority)
  if (front < stopFront) {
    stop();
    if (left > sideOpenThresh)      
    { turnByAngle(-90.0f);
      setFwdDir();
      delay(2000);

       }
    else if (right > sideOpenThresh){ 
      turnByAngle(90.0f);
      setFwdDir();
      delay(2000);
        }
    else{ 
      turnByAngle(180.0f);
      setFwdDir();
      delay(2000);
        }
  }

  else{
    PID();
    
  }

  // Corridor logic: go forward, but if a side is open, stop and turn
  // if (left > sideOpenThresh) {
  //   stop();
  //   turnLeft();
   
  // } else if (right > sideOpenThresh) {
  //   stop();
  //   turnRight();
  // }

  // Default: drive forward with PID correction for centering
}