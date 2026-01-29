// === Ultrasonic pins ===
int trigf = 12 , echof =13;
int trigr = 14, echor = 15;
int trigl = 10, echol = 11;  // A0/A1 as digital OK on Uno

// === Motor pins (L293D / TB6612 style) ===
int AIN1 = 2;
int AIN2 = 3;
int BIN1 = 4;
int BIN2 = 7;
int PWMA = 5;   // PWM - left
int PWMB = 6;   // PWM - right
int stdby = 8;

const int baseSpeed   = 80; // fwd speed
const int maxSpeed    = 150; // PID clamp

// === Maze parameter ===
const int stopFront      = 8;  // cm hard stop to avoid collision
const int corridorThresh = 17;

const unsigned long PID_PERIOD_MS = 75;

float getDistance(int trig, int echo) {
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH, 20000);  // 20ms timeout
  if (duration == 0) return -1;              // no echo → far away
  return duration * 0.034 / 2;                 // cm
}
// === PID gains ===
float Kp = 1.4, Ki = 0.0, Kd = 1.0;
unsigned long lastPID = 0;
float integ = 0, lastErr = 0;

void PID_step() {
  float leftDist  = getDistance(trigl, echol);
  delay(5);
  float rightDist = getDistance(trigr, echor);
  delay(5);
  float frontDist = getDistance(trigf, echof);
  delay(5);

  float err = 0;
  static float lastLeft = corridorThresh/2;
  static float lastRight = corridorThresh/2;
  //------ cases for error calculation------
  // when both wall present 
  if (leftDist < corridorThresh && rightDist < corridorThresh) {
      err = rightDist - leftDist;
      lastLeft  = leftDist;
      lastRight = rightDist;
  }
  // left wall missing
  else if (leftDist > corridorThresh && rightDist < corridorThresh) {
      err = rightDist - lastLeft;
      lastRight = rightDist;
  }
  // right wall missing
  else if (leftDist < corridorThresh && rightDist > corridorThresh) {
      err = lastRight - leftDist;
      lastLeft = leftDist;
  }
  else {
      //bot go straight
      err = 0;
  }
  // -- correction --
  integ = integ + err;
  integ = constrain(integ, -100.0f, 100.0f);
  float deriv = (err - lastErr);
  float corr  = (Kp*err) + (Ki*integ) + (Kd*deriv);
  lastErr = err;

  int leftSpeed  = baseSpeed + (int)corr;
  int rightSpeed = baseSpeed - (int)corr;
  constrain(leftSpeed, 0, 255);
  constrain(rightSpeed, 0, 255);
  // bot moves with new speed 
  analogWrite(PWMA, leftSpeed);
  analogWrite(PWMB, rightSpeed);

  Serial.print("err:"); Serial.println(err); 
  Serial.print(" corr:"); Serial.println(corr); 
  Serial.print(" LPWM:"); Serial.println(leftSpeed); 
  Serial.print(" RPWM:"); Serial.println(rightSpeed); 
  Serial.print(" F:"); Serial.println(frontDist);
  Serial.print(" L:"); Serial.println(leftDist);
  Serial.print(" R:"); Serial.println(rightDist);
}

// -------------------- Arduino setup --------------------
void setup() {
  Serial.begin(9600);

  pinMode(trigf, OUTPUT);
  pinMode(trigl, OUTPUT);
  pinMode(trigr, OUTPUT);
  pinMode(echof, INPUT);
  pinMode(echol, INPUT);
  pinMode(echor, INPUT);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(stdby, OUTPUT);
  //pinMode(buttonPin, INPUT_PULLUP);
  // initially motor starts goes front 
  digitalWrite(stdby, HIGH);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMA, baseSpeed);
  analogWrite(PWMB, baseSpeed);
}

void loop() {
  // button function is missing 

  //  float front = getDistance(trigf, echof);
  // if (front < stopFront) {  // motor stops. 
  // digitalWrite(AIN1, LOW);
  // digitalWrite(AIN2, LOW);
  // digitalWrite(BIN1, LOW);
  // digitalWrite(BIN2, LOW);
  // analogWrite(PWMA, 0);
  // analogWrite(PWMB, 0);
  // delay(1000);
  // }
  PID_step();
  // else{
  //   PID_step();
  // }  
} // code end 
