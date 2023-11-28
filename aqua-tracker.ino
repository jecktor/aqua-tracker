#include <NewPing.h>
#include <TimedAction.h>

#define trigPin 27
#define echoPin 28
#define MAX_DISTANCE 500

#define tdsPin 26
#define VREF 5.0   // analog reference voltage(Volt) of the ADC
#define SCOUNT 30  // sum of sample point

void updateSonar();
void updateTds();

NewPing sonar(trigPin, echoPin, MAX_DISTANCE);

TimedAction sonarAction = TimedAction(100, updateSonar);
TimedAction tdsAction = TimedAction(1000, updateTds);

byte dataPin = 18;
byte latchPin = 20;
byte clockPin = 21;

const byte comPin[] = { 17, 16, 15, 14 };
const byte rgbPins[] = { 11, 12, 13 };

const byte num[] = { 0xc0, 0xf9, 0xa4, 0xb0, 0x99, 0x92, 0x82, 0xf8,
                     0x80, 0x90, 0x88, 0x83, 0xc6, 0xa1, 0x86, 0x8e };

uint16_t distance = 0;

int analogBuffer[SCOUNT];  // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, tdsValue = 0, temperature = 25;

void setup() {
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);

  for (int i = 0; i < 4; i++) {
    pinMode(comPin[i], OUTPUT);
  }

  pinMode(tdsPin, INPUT);
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
}

void updateSonar() {
  distance = (uint16_t)sonar.ping_cm();
}

void updateTds() {
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U)  //every 40 milliseconds,read the analog value from the ADC
  {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(tdsPin);  //read the analog value and store into the buffer
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT)
      analogBufferIndex = 0;
  }
  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 800U) {
    printTimepoint = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
    averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0;                                                                                                   // read the analog value more stable by the median filtering algorithm, and convert to voltage value
    float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);                                                                                                                //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationVolatge = averageVoltage / compensationCoefficient;                                                                                                             //temperature compensation
    tdsValue = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5;  //convert voltage value to tds value
    Serial.print("TDS Value:");
    Serial.print(tdsValue, 0);
    Serial.println("ppm");
  }
  // Update rgb led
  if (tdsValue <= 170) {
    analogWrite(rgbPins[0], 255);
    analogWrite(rgbPins[1], 0);
    analogWrite(rgbPins[2], 0);
  } else if (tdsValue <= 400) {
    analogWrite(rgbPins[0], 0);
    analogWrite(rgbPins[1], 255);
    analogWrite(rgbPins[2], 0);
  } else if (tdsValue > 400) {
    analogWrite(rgbPins[0], 255);
    analogWrite(rgbPins[1], 255);
    analogWrite(rgbPins[2], 0);
  }
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

int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
  else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  return bTemp;
}
