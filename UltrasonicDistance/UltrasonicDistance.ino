const int triggerPin = PE2; //Pin definitions on STM32F722ZE;
const int echoPin = PE4;
int buzzerPin = PG2;

float pulse_width, distance;

void setup() 
{
  // put your setup code here, to run once:
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
}

void loop() 
{

  //Buzzer
  digitalWrite(triggerPin, HIGH); //sends/reads trigger
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW); //sends/reads echo
  pulse_width = pulseIn(echoPin, HIGH); //finds the time between trigger and echo pulses
  distance = (pulse_width*.000343)/2; //calculating distance the speed of sound output in meters
  Serial.print("Distance = ");
  Serial.println(distance);
  Serial.println(" m");
  delay(500);




  if(distance > 1) 
  {
    tone(buzzerPin, 440); //tone(x,Y)x=pin on board, Y = frequency tone for buzzer output
  }
  if(distance < .75 && distance > .51)
  {
    tone(buzzerPin, 800);
  }
  if(distance < .50 && distance >.26)
  {
    tone(buzzerPin, 600);
  }
  if(distance < .25 && distance > .3)
  {
    tone(buzzerPin, 100);
  }
}
