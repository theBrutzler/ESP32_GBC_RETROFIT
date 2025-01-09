/*
  Analog Input

  Demonstrates analog input by reading an analog sensor on analog pin 0 and
  turning on and off a light emitting diode(LED) connected to digital pin 13.
  The amount of time the LED will be on and off depends on the value obtained
  by analogRead().

  The circuit:
  - potentiometer
    center pin of the potentiometer to the analog input 0
    one side pin (either one) to ground
    the other side pin to +5V
  - LED
    anode (long leg) attached to digital output 13 through 220 ohm resistor
    cathode (short leg) attached to ground

  - Note: because most Arduinos have a built-in LED attached to pin 13 on the
    board, the LED is optional.

  created by David Cuartielles
  modified 30 Aug 2011
  By Tom Igoe

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/AnalogInput
*/

int DISPLAY_BLK = 39;   // select the input pin for the potentiometer
int LEFT_RIGHT_Pin = 6;   // select the input pin for the potentiometer
int UP_DOWN_Pin = 7;   // select the input pin for the potentiometer
int VBATPin = 4;   // select the input pin for the potentiometer
int sensorValue1 = 0;  // variable to store the value coming from the sensor
int sensorValue2 = 0;  // variable to store the value coming from the sensor
int VBATValu = 0;  // variable to store the value coming from the sensor
float VBATValu2 = 0;  // variable to store the value coming from the sensor
float VBATPerc = 0;  // variable to store the value coming from the sensor

void setup() {
  Serial.begin(115200);
  pinMode(DISPLAY_BLK, OUTPUT);
  digitalWrite(DISPLAY_BLK, LOW);
}

void loop() {
  // read the value from the sensor:
  sensorValue1 = analogRead(LEFT_RIGHT_Pin);
  sensorValue2 = analogRead(UP_DOWN_Pin);
  VBATValu = analogRead(VBATPin);
  VBATPerc = (((VBATValu) * 2.7f - 2000.f) / (2800.f - 2000.f) * 100.f);
  VBATValu2 = ((VBATValu) * 2.7f * 0.001f);
  
  Serial.print("Value:");
  Serial.print(VBATValu2);
  Serial.print(",");  
  Serial.print("Perc:");
  Serial.print(VBATPerc);
  Serial.print(",");
  Serial.print("UP_DOWN:");
  Serial.print(sensorValue2);
  Serial.print(",");
  Serial.print("LEFT_RIGHT:");
  Serial.println(sensorValue1);
  delay(20);
}
