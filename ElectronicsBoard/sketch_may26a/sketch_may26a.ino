#include <SPI.h>
#include <SD.h>

// Traffic Lights - LED Outputs
#define ledRed A0
#define ledYellow A1
#define ledGreen A2

// DC Motor & Motor Module - L298N
#include <L298N.h>

// Pin definition
const unsigned int IN1 = 5;
const unsigned int IN2 = 6;
const unsigned int EN = 9;

// Create one motor instance
L298N motor(EN, IN1, IN2);

// Servo
#include <Servo.h>
Servo myservo;

//Potentiometer
#define pot A3

// Piezo Buzzer
#define piezoPin 4

// Sonar - HC-SR04
#define echoPin 8 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin A4 //attach pin D3 Arduino to pin Trig of HC-SR04

// Line Sensor
#define lineSensorPin 3

// Crash Sensor / Button
#define crashSensor 7

// PIR
#define PIRSensor 2

// Real Time Clock (RTC)
#include "RTClib.h"
RTC_Millis rtc;     // Software Real Time Clock (RTC)
DateTime rightNow;  // used to store the current time.

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);           // Open serial communications and wait for port to open:
  while (!Serial) {
    delay(1);                   // wait for serial port to connect. Needed for native USB port only
  }
  // Traffic Lights - LED Outputs
  pinMode(ledRed, OUTPUT);
  pinMode(ledYellow, OUTPUT);
  pinMode(ledGreen, OUTPUT);

  // DC Motor & Motor Module - L298N
  motor.setSpeed(70);

  // Servo
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object

  //Potentiometer
  pinMode(pot, INPUT);

  // Piezo Buzzer
  pinMode(piezoPin, OUTPUT);

  // Sonar - HC-SR04
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT

  // Line Sensor
  pinMode(lineSensorPin, INPUT);

  // Crash Sensor / Button
  pinMode(crashSensor, INPUT);


  // SD Card initialisation
  Serial.print("Initializing SD card...");
  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    while (1);
  }
  // Real Time Clock (RTC)
  rtc.begin(DateTime(F(__DATE__), F(__TIME__)));
  Serial.println("initialization done.");
  logEvent("System Initialisation...");


}

void loop() {
  //engineOn(); // Commented out because of potentially dodgy hardware
  carHorn();
  reverseAlarm();
  lineWheels();
  delay(100);
}

void logEvent(String dataToLog) {
  /*
     Log entries to a file on an SD card.
  */
  // Get the updated/current time
  DateTime rightNow = rtc.now();

  // Open the log file
  File logFile = SD.open("events.csv", FILE_WRITE);
  if (!logFile) {
    Serial.print("Couldn't create log file");
    abort();
  }

  // Log the event with the date, time and data
  logFile.print(rightNow.year(), DEC);
  logFile.print(",");
  logFile.print(rightNow.month(), DEC);
  logFile.print(",");
  logFile.print(rightNow.day(), DEC);
  logFile.print(",");
  logFile.print(rightNow.hour(), DEC);
  logFile.print(",");
  logFile.print(rightNow.minute(), DEC);
  logFile.print(",");
  logFile.print(rightNow.second(), DEC);
  logFile.print(",");
  logFile.print(dataToLog);

  // End the line with a return character.
  logFile.println();
  logFile.close();
  Serial.print("Event Logged: ");
  Serial.print(rightNow.year(), DEC);
  Serial.print(",");
  Serial.print(rightNow.month(), DEC);
  Serial.print(",");
  Serial.print(rightNow.day(), DEC);
  Serial.print(",");
  Serial.print(rightNow.hour(), DEC);
  Serial.print(",");
  Serial.print(rightNow.minute(), DEC);
  Serial.print(",");
  Serial.print(rightNow.second(), DEC);
  Serial.print(",");
  Serial.println(dataToLog);

  trafficLights();
  motorDC();
  servo();
  potentiometer();
  piezoBuzzer();
  sonar();
  lineSensor();
  button();

}

void trafficLights() {

}

void motorDC() {

}

void servo() {

}

void potentiometer() {

}

void piezoBuzzer() {

}

void sonar() {

}

void lineSensor() {

}

void button() {

}

void reverseAlarm() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  int distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  Serial.print("Distance; ");
  Serial.print(distance);
  Serial.println(" cm");
  if (distance >= 50) {
    digitalWrite(ledGreen, HIGH);
    digitalWrite(ledYellow, LOW);
    digitalWrite(ledRed, LOW);
  } else {
    if (distance >= 30) {
      digitalWrite(ledYellow, HIGH);
      digitalWrite(ledGreen, LOW);
    digitalWrite(ledRed, LOW);
    } else {
      digitalWrite(ledRed, HIGH);
      digitalWrite(ledYellow, LOW);
    digitalWrite(ledGreen, LOW);
    }
  }
}

void lineWheels() {
  int lineSensorValue = digitalRead(lineSensorPin);
  // Servo position values range from 0-180
  int servoPos = 100;
  if (lineSensorValue == 1) {
    Serial.println("lineSensor");
    myservo.write(servoPos);
  } else {
    myservo.write(0);
  }
}

void carHorn() {
  int crashSensorValue = digitalRead(crashSensor);
  if (crashSensorValue == LOW) {
    digitalWrite(piezoPin, HIGH);
  } else {
    digitalWrite(piezoPin, LOW);
  }
}

void engineOn() {
  // Code not working - Something weird with PIR
  int PIRSensorValue = digitalRead(PIRSensor);
  Serial.println(PIRSensorValue);
  // Servo position values range from 0-180
  if (PIRSensorValue == 1) {
    digitalWrite(ledRed, HIGH);
  } else {
    digitalWrite(ledRed, LOW);
  }
}
