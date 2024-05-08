#include<Servo.h>
Servo servo1;
Servo servo2;
int servopin=9;
int servopin2=3;
const int pin7 = 7;
const int pin6 = 6;
const int pin5 = 5;
const int pin4 = 4;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Set the pins as input
  pinMode(pin7, INPUT);
  pinMode(pin6, INPUT);
  pinMode(pin5, INPUT);
  pinMode(pin4, INPUT);
  pinMode(servopin, OUTPUT);
  digitalWrite(servopin,LOW);
  servo1.attach(servopin);

  pinMode(servopin2, OUTPUT);
  digitalWrite(servopin2,LOW);
  servo2.attach(servopin2);
//
      servo2.write(70);
    delay(300);
     servo1.write(68);
    delay(300);
}
int mone = 90;
int mtwo = 35;
void loop() {
  // Read the state of each pin
  int state7 = digitalRead(pin7);
  int state6 = digitalRead(pin6);
  int state5 = digitalRead(pin5);
  int state4 = digitalRead(pin4);//bl
    Serial.println("state7");
  Serial.println(state7);
      Serial.println("state6");
   Serial.println(state6);
         Serial.println("state5");
    Serial.println(state5);
             Serial.println("state4");
     Serial.println(state4);
//
//
//
//
////    
////    
 if(state4==HIGH)
  {
//BR
    servo2.write(40);
    delay(100);
     servo1.write(75 );
    delay(100);
  }
//
  if(state6==HIGH)
 {
////TR
   servo2.write(40);
   delay(100);
   servo1.write(120);
   delay(100);
  }
//
//
 if(state5==HIGH)
 {
////BL
   servo2.write(40);
   delay(100);
   servo1.write(55);
   delay(100);
 }
 if(state7==HIGH)
 {
////TL
   servo2.write(40);
   delay(100);
   servo1.write(45);
   delay(100);
 }

 }