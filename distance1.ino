/*
  This code reads the Analog Voltage output from the
  LV-MaxSonar sensors
  If you wish for code with averaging, please see
  playground.arduino.cc/Main/MaxSonar
  Please note that we s1; do not recommend using averaging with our sensors.
  Mode and Median filters are recommended.
*/
#include <LiquidCrystal_I2C.h>
#include <SharpIR.h>

// Define the number of sensors in the array as a constant
const byte nbSensors = 2;
uint16_t distArray[nbSensors];
uint16_t sensArray[nbSensors];



LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 16 chars and 2 line display


const byte anPin1 = 2;
const byte anCurr = 1;
const byte anVolt = 4;
const int dPin1 = 7;
const int brightPin1 = 5;
const int LEDPin1 = 7;
const int LEDPin2 = 8;
const int LEDPin3 = 11;
const int LEDPin4 = 4;
const int switchPin1 = 5;
const int switchPin2 = 6;

const int numReadings = 30;
float currReadings[numReadings];      // the readings from the analog input
float voltReadings[numReadings];      // the readings from the analog input
int readIndex = 0;                  // the index of the current reading
float currTotal = 0;                  // the running total
float currAverage = 0;                // the average
float voltTotal = 0;                  // the running total
float voltAverage = 0;                // the average
bool fan = false;
bool aux = false;
//SharpIR IRsensor0( SharpIR::GP2Y0A02YK0F, A3 );
//SharpIR IRsensor1( SharpIR::GP2Y0A21YK0F, A4 );

SharpIR sensorArray[] = {
  SharpIR(SharpIR::GP2Y0A02YK0F, A3), // First sensor using pin A1
  SharpIR(SharpIR::GP2Y0A21YK0F, A4), // Second sensor using pin A2
  // Add as many sensors as required
};

long distance1;
int s1;
int count = 0;

void setup() {
  Serial.begin(9600);  // sets the serial port to 9600
  pinMode(dPin1, INPUT_PULLUP);
  pinMode(switchPin1, INPUT_PULLUP);
  pinMode(switchPin2, INPUT_PULLUP);
  pinMode(brightPin1, OUTPUT);
  pinMode(LEDPin1, OUTPUT);
  pinMode(LEDPin2, OUTPUT);
  pinMode(LEDPin3, OUTPUT);
  pinMode(LEDPin4, OUTPUT);
  lcd.init();                      // initialize the lcd
  // Print a message to the LCD.
  lcd.backlight();
  for (int thisReading = 0; thisReading < numReadings; thisReading++){
    currReadings[thisReading] = 0;
    voltReadings[thisReading] = 0;
}
}

void read_sensors() {
  /*
    Scale factor is (Vcc/512) per inch. A 5V supply yields ~9.8mV/in
    Arduino analog pin goes from 0 to 1024, so the value has to be divided by 2 to get the actual inches
  */
  //distance1 = analogRead(anPin1) * 2.54 / 2;
  //current = analogRead(anPin2);
   /* for (byte i = 0; i < nbSensors; i++) {
    distArray[i] = sensorArray[i].getDistance();
  }*/
  s1 = digitalRead(dPin1);
  fan = digitalRead(switchPin1);
  aux = digitalRead(switchPin2);
  currTotal = currTotal - currReadings[readIndex];
  voltTotal = voltTotal - voltReadings[readIndex];
  // read from the sensor:
  currReadings[readIndex] = analogRead(anCurr);
  voltReadings[readIndex] = analogRead(anVolt);
  // add the reading to the total:
  currTotal = currTotal + currReadings[readIndex];
  voltTotal = voltTotal + voltReadings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  currAverage = currTotal/numReadings;   //Smoothing algorithm (http://www.arduino.cc/en/Tutorial/Smoothing)    
  voltAverage = voltTotal/numReadings; 
    
}

void print_all() {
  analogWrite(brightPin1, 100);
  digitalWrite(LEDPin1, fan);
  digitalWrite(LEDPin2, aux);
  lcd.setCursor(3, 0);
  lcd.print(" ");
  lcd.print(voltAverage *28.19/1024);
  lcd.print(" v");
//  lcd.setCursor(3, 1);
//  for (byte i = 0; i < nbSensors; i++) {
//    lcd.print(distArray[i]);
//    lcd.print(" ; ");
//  }
//  
  lcd.setCursor(3, 2);
  lcd.print((currAverage-510)*5/1024/0.04-0.04);
  lcd.setCursor(3, 3);
  //lcd.print(s1);
  lcd.display();
}

void loop() {
  read_sensors();
  count = count + 1 ;
  if (count > 10) {
    print_all();
    count = 0;
  }
  
  delay(50);
}
