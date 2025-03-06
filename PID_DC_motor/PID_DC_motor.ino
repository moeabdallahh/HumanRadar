 #define ENCA PF8 //2 // YELLOW
#define ENCB PF7 //3 // WHITE
#define PWM PD15 //5 //PWM
#define IN2 PE5//6 //INB
#define IN1 PE3 //7 //INA

volatile float posi = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
long prevT = 0;
float eprev = 0;
float eintegral = 0;

void setup() {
  Serial.begin(9600);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  
  Serial.println("target pos");
}

void loop() {

  // set target position
  //float target = 400; //number must be between 30 and -30
  float target = -30*sin(prevT/1e6 + 90); //250*sin(prevT/1e6);

  // PID constants
  float kp = 5;
  float kd = 2;
  float ki = 0;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/(1.0e6); //calculats: seconds = microseconds / 1e6 
  prevT = currT;

  // Read the position
  float pos = 0; 
  noInterrupts(); // disable interrupts temporarily while reading
  pos = posi;
  interrupts(); // turn interrupts back on
  
  // error
  float e = pos - target;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }

  // signal the motor
  setMotor(dir,pwr,PWM,IN1,IN2);


  // store previous error
  eprev = e;

  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();
}

void setMotor(int dir, float pwmVal, float pwm, float in1, float in2)
{
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

void readEncoder(){
  float b = digitalRead(ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}


//=====================================================

//#define ENCA PF8 //2 // YELLOW
//#define ENCB PF7 //3 // WHITE
//#define PWM PD15 //5 //PWM
//#define IN2 PE5//6 //INB
//#define IN1 PE3 //7 //INA
//
//volatile float posi = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
//long prevT = 0;
//float eprev = 0;
//float eintegral = 0;
//
//// Define variables for EMA filter
//float alpha = 1.005; // Smoothing factor, adjust as needed
//float filteredPos = 0;
//
//void setup() {
//  Serial.begin(9600);
//  pinMode(ENCA, INPUT);
//  pinMode(ENCB, INPUT);
//  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
//  
//  pinMode(PWM, OUTPUT);
//  pinMode(IN1, OUTPUT);
//  pinMode(IN2, OUTPUT);
//  
//  Serial.println("target pos");
//}
//
//void loop() {
//
//  // set target position
//  //float target = 10; //number must be between 30 and -30
//  float target = -30*sin(prevT/1e6 + 90); //250*sin(prevT/1e6);
//
//  // PID constants
//  float kp = 5;
//  float kd = .05;
//  float ki = 0;
//
//  // time difference
//  long currT = micros();
//  float deltaT = ((float) (currT - prevT))/(1.0e6); //calculats: seconds = microseconds / 1e6 
//  prevT = currT;
//
//  // Read the position
//  float pos = 0; 
//  noInterrupts(); // disable interrupts temporarily while reading
//  pos = posi;
//  interrupts(); // turn interrupts back on
//  
//  // Apply low-pass filter (EMA)
//  filteredPos = (alpha * pos) + ((1 - alpha) * filteredPos);
//
//  // error
//  float e = filteredPos - target;
//
//  // derivative
//  float dedt = (e - eprev) / deltaT;
//
//  // integral
//  eintegral = eintegral + e * deltaT;
//
//  // control signal
//  float u = kp * e + kd * dedt + ki * eintegral;
//
//  // motor power
//  float pwr = fabs(u);
//  if (pwr > 255) {
//    pwr = 255;
//  }
//
//  // motor direction
//  int dir = 1;
//  if (u < 0) {
//    dir = -1;
//  }
//
//  // signal the motor
//  setMotor(dir, pwr, PWM, IN1, IN2);
//
//
//  // store previous error
//  eprev = e;
//
//  Serial.print(target);
//  Serial.print(" ");
//  Serial.print(filteredPos); // Output the filtered position instead of the raw position
//  Serial.println();
//}
//
//void setMotor(int dir, float pwmVal, float pwm, float in1, float in2)
//{
//  analogWrite(pwm, pwmVal);
//  if (dir == 1) {
//    digitalWrite(in1, HIGH);
//    digitalWrite(in2, LOW);
//  }
//  else if (dir == -1) {
//    digitalWrite(in1, LOW);
//    digitalWrite(in2, HIGH);
//  }
//  else {
//    digitalWrite(in1, LOW);
//    digitalWrite(in2, LOW);
//  }  
//}
//
//void readEncoder() {
//  float b = digitalRead(ENCB);
//  if (b > 0) {
//    posi++;
//  }
//  else {
//    posi--;
//  }
//}
