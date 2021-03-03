/*
                _       _                             
   __ _ _ __ __| |_   _(_)_ __   ___                  
  / _` | '__/ _` | | | | | '_ \ / _ \                 
 | (_| | | | (_| | |_| | | | | | (_) |                
  \__,_|_|  \__,_|\__,_|_|_| |_|\___/                 
   __| | __ _| |_ __ _| | ___   __ _  __ _  ___ _ __  
  / _` |/ _` | __/ _` | |/ _ \ / _` |/ _` |/ _ \ '__| 
 | (_| | (_| | || (_| | | (_) | (_| | (_| |  __/ |    
  \__,_|\__,_|\__\__,_|_|\___/ \__, |\__, |\___|_|    
                               |___/ |___/            

  This project logs data from a 0-5 volts (0-200 psi) pressure transducer.  
  The log is stored on a file on the SD card, a time stamp is created based 
  on the time and date, from the RTC.  The case has two buttons and LED for
  user interaction, start logging, stop logging, error messages etc.  
  The 0-5 volt pin is connected to the ADC on the arduion which is a 
  10 bit adc,  or 1024 steps.
  version 0.1 - 3/3/2021 First working revision

*/

#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "RTClib.h"



LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27, 20 chars, 4 row display
RTC_PCF8523 rtc;
DateTime now;
unsigned long previousMillis = 0;
unsigned long intervalDelay = 500;

// constants won't change. They're used here to set pin numbers:
const int greenButtonPin = 1; // the number of the pushbutton pin green
const int redButtonPin = 3;   // the number of the pushbutton pin red
const int greenLedPin =  8;   // the number of the LED pin green
const int redLedPin =  9;     // the number of the LED pin red
const int chipSelect = 10;    // Adafruit SD shields and modules: pin 10
const int sensorPin = A2;     // select the input pin for the analog input
const int bandGapRef = 14;    // special indicator that we want to measure the bandgap
const float bandGapVoltage = 1.1; // this is not super guaranteed but its not -too- off
int sensorValue = 0;  // variable to store the value coming from the sensor
char filename[16];
bool printedFileHeader = false;
// these M and B values are obtained by solving the y=mx+b of the sensor when put under different pressures
// and logged,  you can do this in excel and use a scatter plot, and trendline, or the excel linest function
const float sensorMultiplierM = .2355; // Y=MX+B, this equals the M, Obtained
const float sensorAdderB = .421; // Y-Intercept or B in Y=MX+B

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// variables will change:
int greenButtonState = 0;     // variable for reading the pushbutton status
int redButtonState = 0;
int greenLEDState = 0;
int redLEDState = 0;

bool headerPrinted = false;   // used for lcd print header

// set up variables using the SD utility library functions:
Sd2Card card;
SdVolume volume;
SdFile root;

void setup() {


  // put your setup code here, to run once:
  lcd.init();                      // initialize the lcd

  // Print a message to the LCD.
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("datalogger ver 0.1");

  // initialize the LED pin as an output:
  pinMode(redLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);


  // initialize the pushbutton pin as an input:
  pinMode(greenButtonPin, INPUT);
  pinMode(redButtonPin, INPUT);


  //Begin RTC and check to see if its working at same time
  Wire.begin();
  if (! rtc.begin()) {
    lcd.setCursor(0, 0);
    lcd.print("Couldn't find RTC");
    lcd.setCursor(0, 1);
    lcd.print("Maybe dead battery");
    lcd.setCursor(0, 2);
    lcd.print("Or clock not set?");

    while (1) {
      digitalWrite(redLedPin, HIGH);
      digitalWrite(greenLedPin, LOW);
      delay(250);
      digitalWrite(redLedPin, LOW);
      digitalWrite(greenLedPin, HIGH);
      delay(250);
    }
  }
  // Begin SD card and check to see if its working at same time
  if (!SD.begin(chipSelect)) {

    lcd.setCursor(0, 0);
    lcd.print("SD card error.      ");
    lcd.setCursor(0, 1);
    lcd.print("SD Inserted?        ");
    lcd.setCursor(0, 2);
    lcd.print("is format FAT16/32? ");
    while (1) {
      digitalWrite(redLedPin, HIGH);
      digitalWrite(greenLedPin, LOW);
      delay(500);
      digitalWrite(redLedPin, LOW);
      digitalWrite(greenLedPin, HIGH);
      delay(500);
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  //read input buttons - maybe need debounce?
  greenButtonState = digitalRead(greenButtonPin);
  redButtonState = digitalRead(redButtonPin);

  if (greenButtonState == HIGH) {
    logging();
  }

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis > intervalDelay) {
    previousMillis = currentMillis;

    if (headerPrinted == false) {
      headerPrinted = true;   //reset varible to be changed in logging function
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("    PSI");
      lcd.setCursor(0, 1);
      lcd.print("Green to log to SD");
    }

    //    sensorValue = analogRead(sensorPin);
    //*************************************************************************
    //  This code is for testing the sensor input logic remove after verified
    //*************************************************************************
    if (sensorValue > 1024) {
      sensorValue = 0;
    } else {
      sensorValue = sensorValue + 50;
    }
    //**************************************************************************


    float psiFloat = sensorValue * sensorMultiplierM + sensorAdderB; //y=mx+b  value from testing
    int psiInt = round(psiFloat);
    lcd.setCursor(0, 0);
    lcd.print("   ");
    if (psiInt > 99) { // position the pressure reading based on how long 1, 2, or 3 digits
      lcd.setCursor(0, 0);
    } else if (psiInt > 9) {
      lcd.setCursor(1, 0);
    } else {
      lcd.setCursor(2, 0);
    }

    lcd.print(psiInt);

    digitalWrite(greenLedPin, HIGH);  // solid green led to indicate not logging but in a good state
    updateTimeOnLCD();
  }
}

void logging() {

  // create file name https://forum.sparkfun.com/viewtopic.php?t=38500
  int n = 0;
  snprintf(filename, sizeof(filename), "DATA%03d.CSV", n); // includes a three-digit sequence number in the file name
  while (SD.exists(filename)) {
    n++;
    snprintf(filename, sizeof(filename), "DATA%03d.CSV", n);
  }
  printedFileHeader = false;
  lcd.clear();
  lcd.setCursor(8, 0);
  lcd.print(filename);
  lcd.setCursor(0, 0);
  lcd.print("    PSI");
  headerPrinted = false;   // used for lcd print header on main screen
  while (redButtonState == LOW) {
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis > intervalDelay) {
      previousMillis = currentMillis;
      updateTimeOnLCD();

      //read analog pin once, then read again for more accuracy
      //      analogRead(sensorPin);
      //      delay(10);
      //      sensorValue = analogRead(sensorPin);

      //*************************************************************************
      //  This code is for testing the sensor input logic remove after verified
      //*************************************************************************
      if (sensorValue > 1024) {
        sensorValue = 0;
      } else {
        sensorValue = sensorValue + 50;
      }
      //**************************************************************************
      float psiFloat = sensorValue * sensorMultiplierM + sensorAdderB; //y=mx+b  value from
      int psiInt = round(psiFloat);
      lcd.print("   ");

      if (psiInt > 99) { // position the pressure reading based on how long 1, 2, or 3 digits
        lcd.setCursor(0, 0);
      } else if (psiInt > 9) {
        lcd.setCursor(1, 0);
      } else {
        lcd.setCursor(2, 0);
      }
      lcd.print(psiInt);
      lcd.setCursor(0, 3);
      lcd.print("Logging Red to stop");
      greenButtonState = digitalRead(greenButtonPin);
      redButtonState = digitalRead(redButtonPin);
      if (greenLEDState == LOW) {
        digitalWrite(greenLedPin, HIGH);
        greenLEDState = HIGH;
      } else {
        digitalWrite(greenLedPin, LOW);
        greenLEDState = LOW;
      }
      //Write to SD card
      String dataString = String(psiInt);

      File dataFile = SD.open(filename, FILE_WRITE);

      if (dataFile) {  //check to see if the SD card opened correctly
        if (printedFileHeader == false) {
          dataFile.println("Millis, Stamp, Date, Time, Counts, PSI, VCC");
          printedFileHeader = true;
        }
        // log milliseconds since starting
        uint32_t m = millis();
        dataFile.print(m);           // milliseconds since start
        dataFile.print(", ");

        // fetch the time
        now = rtc.now();
        // log time
        dataFile.print(now.unixtime()); // seconds since 1/1/1970
        dataFile.print(", ");
//        dataFile.print('"');  //add quotes around the date/time
        dataFile.print(now.year(), DEC);
        dataFile.print("/");
        dataFile.print(now.month(), DEC);
        dataFile.print("/");
        dataFile.print(now.day(), DEC);
        dataFile.print(", ");
        dataFile.print(now.hour(), DEC);
        dataFile.print(":");
        dataFile.print(now.minute(), DEC);
        dataFile.print(":");
        dataFile.print(now.second(), DEC);
//        dataFile.print('"');
        dataFile.print(", ");
        dataFile.print(sensorValue);
        dataFile.print(", ");
        dataFile.print(psiInt);

        // Log the estimated 'VCC' voltage by measuring the internal 1.1v ref
        analogRead(bandGapRef);
        delay(10);
        int refReading = analogRead(bandGapRef);
        float supplyvoltage = (bandGapVoltage * 1024) / refReading;

        dataFile.print(", ");
        dataFile.println(supplyvoltage);

        dataFile.close();

        digitalWrite(redLedPin, LOW);

      } else { // if the SD card didn't open turn on red LED and continue
        lcd.setCursor(0, 1);
        lcd.print("SD Card error");

        digitalWrite(redLedPin, HIGH);
      }
    }
  }
}
void updateTimeOnLCD() {
  now = rtc.now(); // Get the current time
  int h = now.hour(); // Get the hours right now and store them in an integer called h
  int m = now.minute(); // Get the minutes right now and store them in an integer called m
  int s = now.second(); // Get the seconds right now and store them in an integer called s
  lcd.setCursor(0, 2); // Set the cursor at the column zero, 3rd row... zero based
  lcd.print("Time: ");
  if (h < 10) { // Add a zero, if necessary,
    lcd.print("0");
  }
  lcd.print(h); //print the hour
  lcd.print(":");  // And print the colon
  if (m < 10) { // Add a zero, if necessary,
    lcd.print("0");
  }
  lcd.print(m); //print the minutes
  lcd.print(":");  // And print the colon
  if (s < 10) { // Add a zero, if necessary,
    lcd.print("0");
  }
  lcd.print(s); //print the seconds
}
