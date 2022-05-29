#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


#define buttonPin 3
#define buzzerPin 2 


bool prevButton = true;
bool currButton = true;

bool buzzerState = LOW;
int xValue;
int yValue;
int autoMode = 0;
float sentData[1];
float termoRead = 0;


RF24 radio (8, 7);
byte address[][6] = {"00008" , "00009"};
//struct package
//{
//  int X = 0;
//  int Y = 0;
//  bool Mode = false;
//  };
//
//typedef struct package Package;
//Package data;

int datagram[3];

void setup() {
  Serial.begin(9600);
  pinMode(buttonPin, INPUT_PULLUP);
  delay(100);
  radio.begin();  
  radio.openWritingPipe( address[1]); 
  radio.openReadingPipe(1, address[0]);
  radio.setPALevel(RF24_PA_LOW); 
  delay(100);
  currButton = digitalRead(buttonPin);
  
}

void loop() {
  delay(10);
  radio.stopListening();
//  data.X = analogRead(A4);
//  data.Y = analogRead(A3);
  datagram[0] = analogRead(A4);
  datagram[1] = analogRead(A3);
//  Serial.println(data.X);
//  Serial.println(data.Y);
  Serial.println(datagram[0]);
  Serial.println(datagram[1]);
  Serial.println();
  Serial.println(digitalRead(3));
  Serial.println(currButton);
  Serial.println(prevButton);
  Serial.println();

  prevButton = currButton;
  currButton = digitalRead(buttonPin);
   if (prevButton == HIGH && currButton == LOW){
      //autoMode = !autoMode;
      //data.Mode = autoMode;
      if (autoMode == 0){
        autoMode = 1;
      }
      else{
        autoMode = 0;
      }
   }
   else{
    //data.Mode = autoMode;
    autoMode = autoMode;
   }
  datagram[2] = autoMode;
  delay(50);
  radio.write(datagram, sizeof(datagram));
    
  delay(10);
  radio.startListening();
  // while(!radio.available()); //
  if (radio.available()){
    //while (radio.available()){
      radio.read(sentData, sizeof(sentData));
      //Serial.println("-------------");
     // Serial.println(sentData[0]);
      
    //}
    termoRead = sentData[0];
  }
  else{
    termoRead = 1;
  }
  Serial.println(termoRead);
    
  if (termoRead >= 10 && buzzerState == LOW){
    buzzerState = HIGH;
    digitalWrite(buzzerPin, buzzerState);
  }

  else if (termoRead >= 30 && buzzerState == HIGH){
    
  }

  else{
    buzzerState = LOW;
    digitalWrite(buzzerPin, buzzerState);
  }
  
} 

  
