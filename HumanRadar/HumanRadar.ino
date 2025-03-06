#include <SparkFun_AK975X_Arduino_Library.h>
#include <Wire.h>


//GLOABAL VARIABLES FOR ALL PROJECT WILL BE SEPRARTED BY SENSOR OF USE CASE
//PID controller and interface pins
#define ENCA PF8 //2 // YELLOW
#define ENCB PF7 //3 // WHITE
#define PWM PD15 //5 //PWM
#define IN2 PE5//6 //INB
#define IN1 PE3 //7 //INA

volatile int posi = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;
float alpha = 1.005; // Smoothing factor, adjust as needed
float filteredPos = 0;
float target_g;
float kp;
float ki;
float kd;
long currT;

//Human Presesnce Sensor
AK975X movementSensor;
int ir1, ir2, ir3, ir4;


//Ultrasonic Sensor
const int triggerPin = PE2; //Pin definitions on STM32F722ZE;
const int echoPin = PE4;
float pulse_width, distance;

//Buzzer
int buzzerPin = PG2;

void setup() 
{
  Serial.begin(9600);

  
  //PID Setup 
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);

  //HPS Setup
  Wire.begin();
  void setMode(uint8_t mode = AK975X_MODE_0);
  if (movementSensor.begin() == false)
  {
    Serial.println("Device not found. Check wiring.");
    while (1);
  }
  
  //Ultrasonic Setup
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(triggerPin, LOW);

}




void loop() 
{
  if (movementSensor.available())
  {
    ir1 = movementSensor.getIR1();
    ir2 = movementSensor.getIR2();
    ir3 = movementSensor.getIR3();
    ir4 = movementSensor.getIR4();
  }
  
  movementSensor.refresh(); //Read dummy register after new data is read
  
  target_g = -25*sin(prevT/1e6 + 90);

  kp = 5;
  kd = .05;
  ki = 0;

//PID HERE
//===================================
  PID();
//=========================================

  long elapsedtime;
  if(ir1 && ir2 && ir3 && ir4 >= 1500)
  {
    elapsedtime = micros();
    setMotor(0, 0, PWM, IN1, IN2);
    delay(2000);
    Ultrasonic();
    Buzzer(true);
    currT = currT - elapsedtime;
  }
  else
  {
    Buzzer(false);
  }
//PRINTING THE VALUE OF THE IR
//    Serial.print("]\t2:LFT[");
//    Serial.print(ir2);
//    Serial.print("]\t4:RGH[");
//    Serial.print(ir4);
//    Serial.print("]\t3:UP[");
//    Serial.print(ir3);
//    Serial.print("  :DWN[");
//    Serial.print(ir1);
//    Serial.println();

//PRINTING THE VALUE OF THE DISTANCE
//  Serial.print("Distance = ");
//  Serial.println(distance);
//  Serial.println(" m");

//PRINTING THE PERFORMANCE OF THE PID
//  Serial.print(target_g);
//  Serial.print(" ");
//  Serial.print(filteredPos); // Output the filtered position instead of the raw position
//  Serial.println();
}

//FUNCTIONS
//==================================
void setMotor(int dir, float pwmVal, float pwm, float in1, float in2)
{
  analogWrite(pwm, pwmVal);
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }  
}

void readEncoder() {
  float b = digitalRead(ENCB);
  if (b > 0) {
    posi++;
  }
  else {
    posi--;
  }
}

void Buzzer(bool state)
{
  if (state) {
    digitalWrite(buzzerPin, HIGH);
    if(distance > 2.01) 
    {
      tone(buzzerPin, 2000); //tone(x,Y)x=pin on board, Y = frequency tone for buzzer output
    }
    else if(distance < 2 && distance > 1.01) 
    {
      tone(buzzerPin, 1500); //tone(x,Y)x=pin on board, Y = frequency tone for buzzer output
    }
    else if(distance < 1 && distance > .76) 
    {
      tone(buzzerPin, 1100); //tone(x,Y)x=pin on board, Y = frequency tone for buzzer output
    }
    else if(distance < .75 && distance > .51)
    {
      tone(buzzerPin, 800);
    }
    else if(distance < .50 && distance >.26)
    {
      tone(buzzerPin, 600);
    }
    else if(distance < .25 && distance > .3)
    {
      tone(buzzerPin, 100);
    }
  } 
  else 
  {
    digitalWrite(buzzerPin, LOW);
    noTone(buzzerPin); // Stop any ongoing tone
  }
}
void PID()
{
  kp = 5;
  kd = .05;
  ki = 0;
  
  // time difference
  currT = micros();
  float deltaT = ((float) (currT - prevT))/(1.0e6); //calculats: seconds = microseconds / 1e6 
  prevT = currT;

  // Read the position
  float pos = 0; 
  noInterrupts(); // disable interrupts temporarily while reading
  pos = posi;
  interrupts(); // turn interrupts back on
  
  // Apply low-pass filter (EMA)
  filteredPos = (alpha * pos) + ((1 - alpha) * filteredPos);

  // error
  float e = filteredPos - target_g;

  // derivative
  float dedt = (e - eprev) / deltaT;

  // integral
  eintegral = eintegral + e * deltaT;

  // control signal
  float u = kp * e + kd * dedt + ki * eintegral;

  // motor power
  float pwr = fabs(u);
  if (pwr > 255) {
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if (u < 0) {
    dir = -1;
  }

  // signal the motor
  setMotor(dir, pwr, PWM, IN1, IN2);

  // store previous error
  eprev = e;
}

void Ultrasonic(){
    digitalWrite(triggerPin, HIGH); //sends/reads trigger
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW); //sends/reads echo
    pulse_width = pulseIn(echoPin, HIGH); //finds the time between trigger and echo pulses
    distance = (pulse_width*.000343)/2; //calculating distance the speed of sound output in meters
}
