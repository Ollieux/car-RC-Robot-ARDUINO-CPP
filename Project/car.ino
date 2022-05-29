#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <OneWire.h>
#include <DallasTemperature.h>


#define rightBack A3
#define rightFront A2
#define leftBack A5
#define leftFront A4

#define enableLeft 5
#define enableRight 6

#define cePin 8
#define csnPin 7

#define trigPin A0 
#define echoPin A1 

#define servoPin 9

#define temprPin 2

RF24 radio(cePin, csnPin);

Servo towerServo; 

OneWire oneWire(temprPin);
DallasTemperature sensors(&oneWire);

int xValue;
int yValue;
int autoMode;

float sentData[1];
//bool autoMode;
//struct package{
//  int X = 0;
//  int Y = 0;
//  bool Mode = LOW;
//};
//
//typedef struct package Package;
//Package data;

int datagram[3];

int leftMotorSpeed = 0;
int rightMotorSpeed = 0;

byte address[][6] = {"00008", "00009"};



void setup(){
  Serial.begin(9600);
  radio.begin();
  // myRadio.setChannel(115);  //
  // myRadio.setPALevel(RF24_PA_MAX);
  // myRadio.setDataRate( RF24_250KBPS ) ;  //
  radio.openWritingPipe(address[0]);
  radio.openReadingPipe(1, address[1]);
  radio.setPALevel(RF24_PA_MAX);
  towerServo.attach(servoPin);
  sensors.begin();

  delay(1000);

  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);

  pinMode(enableLeft, OUTPUT);
  pinMode(enableRight, OUTPUT);
  pinMode(leftFront, OUTPUT);
  pinMode(leftBack, OUTPUT);
  pinMode(rightFront, OUTPUT);
  pinMode(rightBack, OUTPUT);

  digitalWrite(leftFront, LOW);
  digitalWrite(leftBack, LOW);
  digitalWrite(rightFront, LOW);
  digitalWrite(rightBack, LOW); 
}

void loop(){
  delay(10);
  sensors.requestTemperatures();
  sentData[0] = sensors.getTempCByIndex(0);
  Serial.println("temperatura");
  Serial.println(sentData[0]);
  radio.startListening();
  if (radio.available()){
    while (radio.available()){
      radio.read(datagram, sizeof(datagram));
    }
      xValue = datagram[0];
      yValue = datagram[1];
      autoMode = datagram[2];
      Serial.println(xValue);
      Serial.println(yValue);
      Serial.println(autoMode);
    //}
    //autoMode = data.Mode;
  }

  if (autoMode == 1){
    delay(25);
    obstacleAvoidance();
//    delay(100);
//    radio.stopListening();
    
  }

  else{
    remoteControl();
  }
}

long ultrasonicRead(){
  long duration, distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration / 2) / 29.1;
  return distance;
}

void moveForward(){


  digitalWrite(leftBack, HIGH);
  digitalWrite(leftFront, LOW);
  digitalWrite(rightBack, HIGH);
  digitalWrite(rightFront, LOW);  
}

void moveBackward(){
  digitalWrite(leftBack, LOW);
  digitalWrite(leftFront, HIGH);
  digitalWrite(rightBack, LOW);
  digitalWrite(rightFront, HIGH);  
}

void turnRight(){
  digitalWrite(leftBack, HIGH);
  digitalWrite(leftFront, LOW);
  digitalWrite(rightBack, LOW);
  digitalWrite(rightFront, HIGH);  
}

void turnLeft(){
  digitalWrite(leftBack, LOW);
  digitalWrite(leftFront, HIGH);
  digitalWrite(rightBack, HIGH);
  digitalWrite(rightFront, LOW);
}

void Stop(){
  digitalWrite(leftBack, LOW);
  digitalWrite(leftFront, LOW);
  digitalWrite(rightBack, LOW);
  digitalWrite(rightFront, LOW);  
}

void obstacleAvoidance(){

  digitalWrite(enableLeft, HIGH);
  digitalWrite(enableRight, HIGH);
  
  long currDistance = ultrasonicRead();
  delay(100);
  if(currDistance <= 40){
    Stop();
    delay(100);
    radio.stopListening();
    // delay(50);
    radio.write(sentData, sizeof(sentData));
    delay(500);
    towerServo.write(30);
    delay(300);
    long leftDistance = ultrasonicRead();
    delay(500);
    towerServo.write(150);
    delay(300);
    long rightDistance = ultrasonicRead();
    delay(500);
    towerServo.write(90);
    
    if (leftDistance <= rightDistance){
      turnRight();
      delay(200);
    }
    else{
      turnLeft();
      delay(200);
    }
    
  }
  else{
    towerServo.write(90);
    moveForward();
  }
}

void remoteControl(){
  //Serial.println(xValue);
  //Serial.println();
  //Serial.println(yValue);

  if (yValue < 470){
    moveBackward();
    rightMotorSpeed = map(yValue, 470, 0, 0, 255);
    leftMotorSpeed = map(yValue, 470, 0, 0, 255); 
  }

   else if (yValue > 550){
    moveForward();
    rightMotorSpeed = map(yValue, 550, 1023, 0, 255);
    leftMotorSpeed = map(yValue, 550, 1023, 0, 255); 
  }

  else{
    rightMotorSpeed = 0;
    leftMotorSpeed = 0;
  }

  if (xValue < 470){
    int xValueScaled = map(xValue, 470, 0, 0, 255); 
    rightMotorSpeed = rightMotorSpeed - xValueScaled; 
    leftMotorSpeed = leftMotorSpeed + xValueScaled;
    
    if (leftMotorSpeed > 255){
      leftMotorSpeed = 255;
    }
    
    if (rightMotorSpeed < 0) {
      rightMotorSpeed = 0;
    }
  }
  
  else if (xValue > 550){
    int xValueScaled = map(xValue, 550, 1023, 0, 255); 
    rightMotorSpeed = rightMotorSpeed + xValueScaled; 
    leftMotorSpeed = leftMotorSpeed - xValueScaled;

    if (rightMotorSpeed > 255){
      rightMotorSpeed = 255;
    }
    
    if (leftMotorSpeed < 0) {
      leftMotorSpeed = 0;
    }
  }

  if (rightMotorSpeed < 70){
    rightMotorSpeed = 0;
  }

  if (leftMotorSpeed < 70){
    leftMotorSpeed = 0;
  }

  analogWrite(enableLeft, leftMotorSpeed);
  analogWrite(enableRight, rightMotorSpeed);
}
