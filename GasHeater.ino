#include <Servo.h>
#include <Keypad.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Thread.h>
#include <avr/wdt.h>

// which analog pin to connect
#define THERMISTORPIN A0
// resistance at 25 degrees C
#define THERMISTORNOMINAL 31000
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 100
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950
//#define BCOEFFICIENT 3000
// the value of the 'other' resistor
#define SERIESRESISTOR 97000

int loop_count = 0;

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
int startAngle = 120;
int maxAngle = 90;
int servoAngle = minAngle;   // servo position in degrees
bool isServoAttached = false;
bool isCycleSkipped = false;

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
double previousTemp = 0.0;

Thread temperatureAdjustingThread = Thread();
Thread getCurrentTemperatureThread = Thread();
Thread boilerOnlineThread = Thread();
Thread lightUpThread = Thread();
Thread lightDownThread = Thread();

void lightUp() {
  if (lcdLevel < maxLevel) {
    currentLcdState = lcdUp;
    lcdLevel += 2;
  } else {
    currentLcdState = lcdWait;
  }
  analogWrite(lcd1Light, lcdLevel);
}

void lightDown() {
  if (lcdLevel > 0) {
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
  //  currentServoState = none;
  isCycleSkipped = false;
  if (currentTemp > (targetTemp + threshold) && servoAngle < minAngle) {
    int delta = 1;
    if ((currentTemp + threshold - targetTemp) > 2) {
      delta = 2;
    }
    if ((currentTemp + threshold - targetTemp) > 4) {
      delta = 3;
    }
    if ((currentTemp + threshold - targetTemp) > 6) {
      delta = 4;
    }
    if ((currentTemp + threshold - targetTemp) > 9) {
      delta = 5;
    }

    if (
      (delta >= 3 && (previousTemp - currentTemp) >= 1.2) ||
      (delta < 2 && (previousTemp - currentTemp) >= 0.3)
    ) {
      isCycleSkipped = true;
      previousTemp = currentTemp;
      return;
    } else {
      isCycleSkipped = false;
    }
    previousTemp = currentTemp;
    servoAngle += delta;
    currentServoState = down;
  }

  if (currentTemp < (targetTemp - threshold) && servoAngle > maxAngle) {
    int delta = 1;
    if ((currentTemp + threshold - targetTemp) < -2) {
      delta = 2;
    }
    if ((currentTemp + threshold - targetTemp) < -4) {
      delta = 3;
    }
    if ((currentTemp + threshold - targetTemp) < -6) {
      delta = 4;
    }
    if ((currentTemp + threshold - targetTemp) < -9) {
      delta = 5;
    }

    if (
      (delta >= 3 && (previousTemp - currentTemp) <= -1.2) ||
      (delta < 2 && (previousTemp - currentTemp) <= -0.5)
    ) {
      isCycleSkipped = true;
      previousTemp = currentTemp;
      return;
    } else {
      isCycleSkipped = false;
    }
    previousTemp = currentTemp;
    servoAngle -= delta;
    currentServoState = up;
  }
  servo.write(servoAngle);
}

void getTemperature() {
  uint8_t i;
  float average;
  uint16_t samples[NUMSAMPLES];

  for (i = 0; i < NUMSAMPLES; i++) {
    samples[i] = analogRead(THERMISTORPIN);
    delay(3);
  }

  // average all the samples out
  average = 0.0;
  for (i = 0; i < NUMSAMPLES; i++) {
    average += samples[i];
  }
  average /= NUMSAMPLES;
  // TODO: add more diapasones


  // 33 - 265
  // 35 - 247
  // 36 - 238
  // 38 - 220
  // 42 - 195
  // 48 - 156
  // 53 - 137
  if (average <= 156 && average >= 137) {
    currentTemp = map(average, 156, 137, 48 * 10.0, 53 * 10) / 10.0;
  } else if (average <= 195) {
    currentTemp = map(average, 195, 156, 42 * 10.0, 48 * 10) / 10.0;
  } else if (average <= 220) {
    currentTemp = map(average, 220, 195, 38 * 10.0, 42 * 10) / 10.0;
  } else if (average <= 238) {
    currentTemp = map(average, 238, 220, 36 * 10.0, 38 * 10) / 10.0;
  } else if (average <= 247) {
    currentTemp = map(average, 247, 238, 35 * 10.0, 36 * 10) / 10.0;
  } else if (average <= 265) {
    currentTemp = map(average, 265, 247, 33 * 10.0, 35 * 10) / 10.0;
  } else {
    currentTemp = map(average, 265, 137, 33.0 * 10.0, 53.0 * 10.0) / 10.0;
  }

  //  currentTemp = map(average, 247, 137, 35.0, 53.0);
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
  if (!isBoilerOnline) {
    previousTemp = 0.0;
  }
}

void watchdogSetup(void) {
  cli(); // disable all interrupts
  wdt_reset(); // reset the WDT timer
  /*
    WDTCSR configuration:
    WDIE = 1: Interrupt Enable
    WDE = 1 :Reset Enable
    WDP3 = 0 :For 2000ms Time-out
    WDP2 = 1 :For 2000ms Time-out
    WDP1 = 1 :For 2000ms Time-out
    WDP0 = 1 :For 2000ms Time-out
  */
  // Enter Watchdog Configuration mode:
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  //  WDTCSR |= (1 << WDE);
  // Set Watchdog settings:
  WDTCSR = (1 << WDIE) | (1 << WDE) | (0 << WDP3) | (1 << WDP2) | (1 << WDP1) | (1 << WDP0);
  sei();
}

void setup()
{

  pinMode(lcd1Light, OUTPUT);
  pinMode(boilerPin, INPUT);

  temperatureAdjustingThread.onRun(adjustTemperature);
  temperatureAdjustingThread.setInterval(5000);

  getCurrentTemperatureThread.onRun(getTemperature);
  getCurrentTemperatureThread.setInterval(500);

  boilerOnlineThread.onRun(checkIfBoilerOnline);
  boilerOnlineThread.setInterval(200);

  lightUpThread.onRun(lightUp);
  lightUpThread.setInterval(1);

  lightDownThread.onRun(lightDown);
  lightDownThread.setInterval(1);

  lcd.begin(16, 2);
  analogWrite(lcd1Light, maxLevel);

  Serial.begin(9600);
  servo.attach(servoPin);
  isServoAttached = true;
  servoAngle = minAngle;
  servo.write(servoAngle);
  delay(2000);
  servoAngle = startAngle;
  servo.write(servoAngle);
  lastActionTime = millis();
  //   watchdogSetup();
}

//ISR(WDT_vect) // Watchdog timer interrupt. {
// Include your code here - be careful not to use functions they may cause the interrupt to hang and
// prevent a reset.
//}

void loop()
{
  //    loop_count++;
  //    wdt_reset();

  if ((millis() - lastActionTime > lcdSleepDelay) /*&& (currentLcdState == lcdWait || currentLcdState == lcdDown)*/) {
    if (lightDownThread.shouldRun()) {
      lightDownThread.run();
    }

    if (isServoAttached) {
      servo.detach();
      isServoAttached = false;
    }
  } else if ((millis() - lastActionTime < lcdSleepDelay) /*&& (currentLcdState == lcdWait || currentLcdState == lcdUp)*/) {
    if (lightUpThread.shouldRun()) {
      lightUpThread.run();
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

  String emptyLeft = "       ";
  String emptyRight = "       ";

  String waitLeft = "    =  ";
  String waitRight = "  =    ";

  String goLeft = "    -> ";
  String goRight = " <-    ";

  String pausedLeft = "  ||-> ";
  String pausedRight = " <-||  ";

  lcd.home ();
  switch (currentServoState) {
    case none:
      lcd.print(waitLeft + String((int)targetTemp) + waitRight);
      break;

    case up:
      if (isCycleSkipped) {
        lcd.print(pausedLeft + String((int)targetTemp) + emptyRight);
      } else {
        lcd.print(goLeft + String((int)targetTemp) + emptyRight);
      }
      break;

    case down:
      if (isCycleSkipped) {
        lcd.print(emptyLeft + String((int)targetTemp) + pausedRight);
      } else {
        lcd.print(emptyLeft + String((int)targetTemp) + goRight);
      }
      break;
  }

  if (!isBoilerOnline) {
    boiler = " OFF";
  }
  lcd.setCursor ( 0, 1 );        // go to the next line
  lcd.print (String(currentTemp) + "    " + String(180 - servoAngle) + boiler);
}




