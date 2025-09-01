#include <Arduino.h>
#include <LiquidCrystal.h>

const int rs = 30, en = 31, d4 = 32, d5 = 33, d6 = 34, d7 = 35;

const int S2 = 13; 
const int S3 = 12;
const int S4 = 11;
const int S5 = 10;

const int LED1 = 3;
const int LED2 = 2;

int contrast=75;
int stateCount = 0;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);  

void setup() {
  // put your setup code here, to run once:
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);

  pinMode(S2, INPUT_PULLUP);
  pinMode(S3, INPUT_PULLUP);
  pinMode(S4, INPUT_PULLUP);
  pinMode(S5, INPUT_PULLUP);

  analogWrite(6, contrast);
  lcd.begin(16, 2);
}

void loop() {
  // put your main code here, to run repeatedly:
  int state1 = digitalRead(S2);
  int state2 = digitalRead(S3);
  int state3 = digitalRead(S4);
  int state4 = digitalRead(S5);

  int potValue = analogRead(A0); // Read potentiometer value (0-1023)
  // int sensorValue = map(potValue,0,1023,1,5);

  switch (stateCount){
    case 0: //both switches off
    lcd.clear();
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    if(state1 == HIGH)
      stateCount=1;
    else if(state2==HIGH)
      stateCount=2;
    else if(state3==HIGH)
      stateCount=3;
    else if(state4==HIGH)
      stateCount=4;
    break;

    case 1: //one light on, switch 2 on
    digitalWrite(LED1, HIGH);

    if(state1==LOW)
      stateCount=0;//take us to case 0
    else if((state2==HIGH)&&(state1==LOW)) //half of this is kind of redundant, but that's ok
      stateCount=2; //take us to case 2
    else if((state3==HIGH)&&(state1==LOW))
      stateCount=3;
    else if((state4==HIGH)&&(state1==LOW))
      stateCount=4;
    break;

    case 2:
    digitalWrite(LED1, HIGH);
    delay(500);
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, HIGH);
    delay(500);
    digitalWrite(LED2, LOW);
    if(state2==LOW)
      stateCount=0; //take us to case 0, all sawitches off
    else if(state1==HIGH)
      stateCount=1;//take us to state 1
    else if(state3==HIGH)
      stateCount=3;
    else if(state4==HIGH)
      stateCount=4;
    break;

    case 3:
    lcd.print(" John Marcial ");
    delay(100);
    if(state3==LOW)
      stateCount=0; //take us to case 0, all sawitches off
    else if(state1==HIGH)
      stateCount=1;//take us to state 1
    else if(state2==HIGH)
      stateCount=2;
    else if(state4==HIGH)
      stateCount=4;
    break;

    case 4:
    lcd.setCursor(0, 0); // Set cursor to first row
    lcd.print("Pot Value: ");
    lcd.setCursor(0, 1);
    lcd.print(potValue); // Display label
    delay(100); // Update every 100ms

    if(state4==LOW)
      stateCount=0; //take us to case 0, all sawitches off
    else if(state1==HIGH)
      stateCount=1;//take us to state 1
    else if(state2==HIGH)
      stateCount=2;
    else if(state3==HIGH)
      stateCount=3;
    break;

    default:
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    break;
  }
}
