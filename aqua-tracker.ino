#include <NewPing.h>
#include <TimedAction.h>
#include <GravityTDS.h>

#define trigPin 27
#define echoPin 28
#define MAX_DISTANCE 500

#define tdsPin 26

void updateSonar();
void updateTds();

NewPing sonar(trigPin, echoPin, MAX_DISTANCE);

TimedAction sonarAction = TimedAction(100, updateSonar);
TimedAction tdsAction = TimedAction(1000, updateTds);

GravityTDS gravityTds;

byte dataPin = 18;
byte latchPin = 20;
byte clockPin = 21;

const byte comPin[] = { 17, 16, 15, 14 };
const byte rgbPins[] = { 13, 12, 11 };

const byte num[] = { 0xc0, 0xf9, 0xa4, 0xb0, 0x99, 0x92, 0x82, 0xf8,
                     0x80, 0x90, 0x88, 0x83, 0xc6, 0xa1, 0x86, 0x8e };

uint16_t distance = 0;

float temperature = 25, tdsValue = 0;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);

  for (int i = 0; i < 4; i++) {
    pinMode(comPin[i], OUTPUT);
  }

  gravityTds.setPin(tdsPin);
  gravityTds.setAref(5.0);       // reference voltage on ADC, default 5.0V
  gravityTds.setAdcRange(1024);  // 1024 for 10bit ADC;4096 for 12bit ADC
  gravityTds.begin();            // initialization
}

void loop() {
  sonarAction.check();
  tdsAction.check();

  uint16_t digits = distance;
  uint16_t n;

  while (digits > 0) {
    n = n * 10 + digits % 10;
    digits /= 10;
  }

  if (n == 0) {
    electDigitalDisplay(0);
    writeData(num[0]);
    delay(5);
    writeData(0xff);

    return;
  }

  for (int i = 0; n > 0; i++) {
    electDigitalDisplay(i);
    writeData(num[n % 10]);
    delay(5);
    writeData(0xff);

    n /= 10;
  }

  // update rgb led
}

void updateSonar() {
  distance = (uint16_t)sonar.ping_cm();
}

void updateTds() {
  gravityTds.setTemperature(temperature);  // set the temperature and execute temperature compensation
  gravityTds.update();
  tdsValue = gravityTds.getTdsValue();
}

void electDigitalDisplay(byte com) {
  for (int i = 0; i < 4; i++) {
    digitalWrite(comPin[i], LOW);
  }
  digitalWrite(comPin[com], HIGH);
}

void writeData(int value) {
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, value);
  digitalWrite(latchPin, HIGH);
}

