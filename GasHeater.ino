#include <Servo.h>
#include <Keypad.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Thread.h>

// which analog pin to connect
#define THERMISTORPIN A0
// resistance at 25 degrees C
#define THERMISTORNOMINAL 31000
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 20
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950
//#define BCOEFFICIENT 3000
// the value of the 'other' resistor
#define SERIESRESISTOR 97000

uint16_t samples[NUMSAMPLES];

#define lcd1Light DD3
LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
unsigned long lastActionTime;
int lcdSleepDelay = 10000;
int lcdLevel = 0;
int maxLevel = 250;

enum lcdState {
  lcdWait,
  lcdUp,
  lcdDown
};
lcdState currentLcdState = lcdWait;



const byte ROWS = 1; // Four rows
const byte COLS = 4; // Three columns
// Define the Keymap
char keys[ROWS][COLS] = {
  {'+', '-', 'B', 'A'}
};

#define boilerPin DD2

bool isBoilerOnline = false;
bool isBoilerOnlinePrevious = false;
#define boilerOnlineArrayCount 5
unsigned long boilerOnTime;
unsigned long boilerStartDelay = 12000;


byte rowPins[ROWS] = { 8 };
byte colPins[COLS] = { 9, 10, 11, 12 };
Keypad kpd = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

Servo servo;
int servoPin = 7;

int minAngle = 180;
int maxAngle = 90;
int servoAngle = minAngle;   // servo position in degrees
bool isServoAttached = false;

enum servoState {
  none,
  up,
  down
};

servoState currentServoState = none;

//bool isServoWorks = false;
//bool isDecreasingGas = false;

double maxTemp = 50.0;
double minTemp = 20.0;
double targetTemp = 36.0;
double currentTemp = 25.0;
double threshold = 1.0;

Thread temperatureAdjustingThread = Thread();
Thread getCurrentTemperatureThread = Thread();
Thread boilerOnlineThread = Thread();
Thread lightUpThread = Thread();
Thread lightDownThread = Thread();

void lightUp() {
  if (lcdLevel < maxLevel) {
    Serial.println(lcdLevel);
    currentLcdState = lcdUp;
    lcdLevel += 1;
  } else {
    Serial.println("WAIT");
    currentLcdState = lcdWait;
  }
  analogWrite(lcd1Light, lcdLevel);
}

void lightDown() {
  if (lcdLevel > 0) {
    Serial.println("______");
    lcdLevel -= 1;
    currentLcdState = lcdDown;
  } else {
    currentLcdState = lcdWait;
  }
  analogWrite(lcd1Light, lcdLevel);  // analogRead values go from 0 to 1023, analogWrite values from 0 to 255
}

void adjustTemperature() {

  if (millis() - boilerOnTime < boilerStartDelay) {
    currentServoState = none;
    return;
  }
  currentServoState = none;

  if (currentTemp > (targetTemp + threshold) && servoAngle < minAngle) {
    int delta = 1;
    if ((currentTemp + threshold -  targetTemp) > 2) {
      delta = 2;
    }
    if ((currentTemp + threshold -  targetTemp) > 4) {
      delta = 3;
    }
    if ((currentTemp + threshold -  targetTemp) > 9) {
      delta = 5;
    }
    servoAngle += delta;
    currentServoState = down;
  }

  if (currentTemp < (targetTemp - threshold) && servoAngle > maxAngle) {
    int delta = 1;
    if ((currentTemp + threshold -  targetTemp) < -2) {
      delta = 2;
    }
    if ((currentTemp + threshold -  targetTemp) < -4) {
      delta = 3;
    }
    if ((currentTemp + threshold -  targetTemp) < -9) {
      delta = 5;
    }
    servoAngle -= delta;
    currentServoState = up;
  }

  Serial.println(servoAngle);
  servo.write(servoAngle);
}

void getTemperature() {
  uint8_t i;
  float average;

  for (i = 0; i < NUMSAMPLES; i++) {
    samples[i] = analogRead(THERMISTORPIN);
    delay(10);
  }

  // average all the samples out
  average = 0;
  for (i = 0; i < NUMSAMPLES; i++) {
    average += samples[i];
  }
  average /= NUMSAMPLES;

  //42-195, 35 - 247, 48 - 156 53 - 137, 40-220
  currentTemp = map(average, 247, 137, 35, 53);
}

void checkIfBoilerOnline() {
  uint8_t i;

  bool tempOnline = false;
  for (i = 0; i < boilerOnlineArrayCount; i++) {
    if (digitalRead(boilerPin) == HIGH) {
      tempOnline = true;
    }
    delay(10);
  }
  if (isBoilerOnline != tempOnline) {
    isBoilerOnlinePrevious = isBoilerOnline;
    isBoilerOnline = tempOnline;
    boilerOnTime = millis();
  }
}

void setup()
{
  pinMode(lcd1Light, OUTPUT);
  pinMode(boilerPin, INPUT);

  temperatureAdjustingThread.onRun(adjustTemperature);
  temperatureAdjustingThread.setInterval(3000);

  getCurrentTemperatureThread.onRun(getTemperature);
  getCurrentTemperatureThread.setInterval(500);

  boilerOnlineThread.onRun(checkIfBoilerOnline);
  boilerOnlineThread.setInterval(100);

  lightUpThread.onRun(lightUp);
  lightUpThread.setInterval(1);

  lightDownThread.onRun(lightDown);
  lightDownThread.setInterval(1);

  lcd.begin(16, 2);
  analogWrite(lcd1Light, maxLevel);

  Serial.begin(9600);
  servo.attach(servoPin);
  isServoAttached = true;

  servo.write(servoAngle);
  delay(1000);
  servo.write(servoAngle);
  delay(1000);
  servo.write(servoAngle);
  lastActionTime = millis();
}

void loop()
{
  Serial.print(currentLcdState);
  Serial.println("___");
  //  Serial.print(lcdLevel);

  if ((millis() - lastActionTime > lcdSleepDelay) && (lcdLevel >= 0) && (currentLcdState == lcdWait || currentLcdState == lcdDown)) {
    if (lightDownThread.shouldRun()) {
      lightDownThread.run();
      Serial.println("Down");
    }

    if (isServoAttached) {
      servo.detach();
      isServoAttached = false;
    }
  } else if ((millis() - lastActionTime < lcdSleepDelay) && (lcdLevel <= maxLevel) && (currentLcdState == lcdWait || currentLcdState == lcdUp)) {
    if (lightUpThread.shouldRun()) {
      lightUpThread.run();
      Serial.println("Up");
    }

    if (!isServoAttached) {
      servo.attach(servoPin);
      isServoAttached = true;
    }


  }

  if (!isBoilerOnline) {
    currentServoState = none;
  } else {
    lastActionTime = millis();
  }

  if (temperatureAdjustingThread.shouldRun() && isBoilerOnline == true)
    temperatureAdjustingThread.run();

  if (getCurrentTemperatureThread.shouldRun())
    getCurrentTemperatureThread.run();

  if (boilerOnlineThread.shouldRun())
    boilerOnlineThread.run();

  printToLcd1();
  //275-25, 220 - 40
  //  checkIfBoilerOnline();
  //  temp = map(analogRead(THERMISTORPIN), 800, 635, 31, 51);





  //  Serial.print("Average analog reading ");
  //  Serial.println(average);

  // convert the value to resistance
  //  average = 1023 / average - 1;
  //  average = SERIESRESISTOR / average;
  //  //  Serial.print("Thermistor resistance ");
  //  //  Serial.println(average);
  //
  //  float steinhart;
  //  steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
  //  steinhart = log(steinhart);                  // ln(R/Ro)
  //  steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  //  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  //  steinhart = 1.0 / steinhart;                 // Invert
  //  steinhart -= 273.15;                         // convert to C

  //  Serial.print("Temperature ");
  //  Serial.print(steinhart);
  //  Serial.println(" *C");

  //  delay(1000);



  //  for (int i = 254; i >= 0; i--)
  //  {
  //    Serial.println(i);
  //    analogWrite(lcd1Light, i);  // analogRead values go from 0 to 1023, analogWrite values from 0 to 255
  //    delay(20);
  //  }
  //
  //  for (int i = 1; i <= 255; i++)
  //  {
  //    Serial.println(i);
  //    analogWrite(lcd1Light, i);  // analogRead values go from 0 to 1023, analogWrite values from 0 to 255
  //    delay(20);
  //  }




  //  byte error, address;
  //  int nDevices;
  //
  //  Serial.println("Scanning...");
  //
  //  nDevices = 0;
  //  for(address = 1; address < 127; address++ )
  //  {
  //    // The i2c_scanner uses the return value of
  //    // the Write.endTransmisstion to see if
  //    // a device did acknowledge to the address.
  //    Wire.beginTransmission(address);
  //    error = Wire.endTransmission();
  //
  //    if (error == 0)
  //    {
  //      Serial.print("I2C device found at address 0x");
  //      if (address<16)
  //        Serial.print("0");
  //      Serial.print(address,HEX);
  //      Serial.println("  !");
  //
  //      nDevices++;
  //    }
  //    else if (error==4)
  //    {
  //      Serial.print("Unknown error at address 0x");
  //      if (address<16)
  //        Serial.print("0");
  //      Serial.println(address,HEX);
  //    }
  //  }
  //  if (nDevices == 0)
  //    Serial.println("No I2C devices found\n");
  //  else
  //    Serial.println("done\n");
  //
  //  delay(5000);           // wait 5 seconds for next scan
  //
  //



  char key = kpd.getKey();

  if (key) // Check for a valid key.
  {
    switch (key)
    {
      case '+':
        if (targetTemp < maxTemp)
          targetTemp += 1;
        lastActionTime = millis();
        break;

      case '-':
        if (targetTemp > minTemp)
          targetTemp -= 1;

        lastActionTime = millis();
        break;

      default:
        lastActionTime = millis();
        break;
    }
  }
}


void printToLcd1() {
  String boiler = " ON ";
  String servoString;

  switch (currentServoState) {
    case none:
      servoString = "    ";
      break;

    case up:
      servoString = " >> ";
      break;

    case down:
      servoString = " << ";
      break;
  }

  if (!isBoilerOnline) {
    boiler = " OFF";
  }
  lcd.home ();

  lcd.print("Target = " + String(targetTemp));
  lcd.setCursor ( 0, 1 );        // go to the next line
  lcd.print (String(currentTemp) + servoString + String(servoAngle) + boiler);
}




