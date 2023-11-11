// ***************************************************************************
// ***************************************************************************

#include <SoftwareSerial.h>
#include "SevenSegmentTM1637.h"
#include "SevenSegmentExtended.h"
#include <EEPROM.h>
#include <CRCx.h>  //https://github.com/hideakitai/CRCx
#include <avr/wdt.h>


const byte PIN_TX = 6;  // define TX pin to Sensor
const byte PIN_RX = 7;  // define RX pin to Sensor
const byte PIN_CLK = 9;
const byte PIN_DIO = 8;

const byte BUTTON_CALI = A0;

const byte LED = 13;


const byte MB_PKT_7 = 7;    //MODBUS Packet Size
const byte MB_PKT_8 = 8;    //MODBUS Packet Size
const byte MB_PKT_17 = 17;  // MODBUS Packet Size



// SenseAir S8 MODBUS commands
const byte cmdReSA[MB_PKT_8] = { 0xFE, 0X04, 0X00, 0X03, 0X00, 0X01, 0XD5, 0XC5 };  // SenseAir Read CO2
const byte cmdOFFs[MB_PKT_8] = { 0xFE, 0x06, 0x00, 0x1F, 0x00, 0x00, 0xAC, 0x03 };  // SenseAir Close ABC
const byte cmdCal1[MB_PKT_8] = { 0xFE, 0x06, 0x00, 0x00, 0x00, 0x00, 0x9D, 0xC5 };  // SenseAir Calibration 1
const byte cmdCal2[MB_PKT_8] = { 0xFE, 0x06, 0x00, 0x01, 0x7C, 0x06, 0x6C, 0xC7 };  // SenseAir Calibration 2
const byte cmdCal3[MB_PKT_8] = { 0xFE, 0x03, 0x00, 0x00, 0x00, 0x01, 0x90, 0x05 };  // SenseAir Calibration 3
const byte cmdCalR[MB_PKT_7] = { 0xFE, 0x03, 0x02, 0x00, 0x20, 0xAD, 0x88 };        // SenseAir Calibration Response

static byte response[MB_PKT_8] = { 0 };

bool isLongPressCALI = false;
bool ExitCali = false;



//VALalti is altitude in meters divided by 50
//therefore Calgary is 1045m elevation, as 1045/50 = 20.9
byte VALalti = 21;

byte ConnRetry = 0;
int CO2 = 0;
int CO2value;
unsigned int crc_cmd;
unsigned long StartPress_ms = 0;
float CO2cor;
float hpa;


const int LongPress_ms = 5000;  // 5s button timeout


SevenSegmentExtended display(PIN_CLK, PIN_DIO);
SoftwareSerial co2sensor(PIN_RX, PIN_TX);

#define BAUDRATE 9600  // Device to SenseAir S8


void setup() {
  pinMode(BUTTON_CALI, INPUT_PULLUP);  //A0//pullup => high when open, low when closed
  pinMode(LED, OUTPUT);
  pinMode(A2, OUTPUT);

  digitalWrite(A2, LOW);  //for calibration button
  digitalWrite(LED, LOW);

  Serial.begin(9600);

  display.begin();           // initializes the display
  display.setBacklight(50);  // set the brightness in percent (ie: 100 => 100 %)
  delay(100);

  display.clear();
  display.print("co2");

  Serial.println(F("Start SenseAir S8 lecture"));
  co2sensor.begin(BAUDRATE);  // Sensor serial start
  delay(4000);


  CO2iniSenseAir();

  // Preheat routine: min 30 seconds for  SenseAir S8
  Serial.print(F("Preheat: "));

  for (int i = 30; i > -1; i--) {  // Preheat from 0 to 30 or to 180
    display.clear();
    display.print("HEAT");
    delay(1000);
    display.clear();
    display.printNumber(i);
    Serial.println(i);
    delay(1000);
    i--;
  }
  Serial.print(F("Start measurements compensated by Altitude: "));
  Serial.print(VALalti * 50);
  Serial.println(" m");
  display.clear();
  display.print("CO2-");
  delay(3000);
}

void loop() {
  CO2 = 0;

  CO2value = co2SenseAir();
  if (CO2value != 0) {
    hPaCalculation();
    CO2cor = float(CO2value) + (0.016 * ((1013 - float(hpa)) / 10) * (float(CO2value) - 400));  // Increment of 1.6% for every hPa of difference at sea level
    CO2 = round(CO2cor);
  } else
    CO2 = 0;




  if (CO2 > 0) {
    Serial.println(CO2);
    display.clear();
    delay(10);
    display.printNumber(CO2);
  } else {

    display.clear();
    delay(10);
    display.print("----");
  }




  check_calmode_active();
  delay(3967);  // for SenseAir S8 4 seconds
}

//Routine Bad connection
void BadConn() {
  Serial.println("Air sensor not detected. Please check wiring... Try# " + String(ConnRetry));

  if (ConnRetry > 1) {
    display.clear();
    delay(20);
    display.print("bad");
  }
  delay(2500);
  ConnRetry++;
}



void Done() {
  Serial.println(F("done"));
  display.clear();
  delay(10);
  display.print("done");
}

void Failed() {
  Serial.println(F("failed"));
  display.clear();
  delay(10);
  display.print("fail");
}

void DebugCO2val() {
  Serial.print("hpa: ");
  Serial.print(hpa);
  Serial.print(" CO2real: ");
  Serial.print(CO2value);
  Serial.print(" CO2adjust: ");
  Serial.println(CO2cor);
}

//Done or failed revision routine
void CheckResponse(uint8_t *a, uint8_t *b, uint8_t len_array_cmp) {
  bool check_match = false;
  for (int n = 0; n < len_array_cmp; n++) {
    if (a[n] != b[n]) {
      check_match = false;
      break;
    } else
      check_match = true;
  }

  if (check_match) {
    Done();
  } else {
    Failed();
  }
}


// Calculate of Atmospheric pressure

void hPaCalculation() {
  hpa = 1013 - 5.9 * float(VALalti) + 0.011825 * float(VALalti) * float(VALalti);  // quadratic regresion formula obtained PA (hpa) from high above the sea
  Serial.print(F("Atmospheric pressure calculated by the sea level inserted (hPa): "));
  Serial.println(hpa);
}




// SenseAir S8 routines

// Initialice SenseAir S8
void CO2iniSenseAir() {
  //Deactivate Automatic Self-Calibration
  co2sensor.write(cmdOFFs, MB_PKT_8);
  co2sensor.readBytes(response, MB_PKT_8);
  Serial.print(F("Deactivate Automatic Self-Calibration SenseAir S8: "));
  CheckResponse(cmdOFFs, response, MB_PKT_8);
  delay(2000);

  while (co2SenseAir() == 0 && (ConnRetry < 5)) {
    BadConn();
  }



  Serial.println(F(" Air sensor detected AirSense S8 Modbus"));

  display.clear();
  delay(10);
  display.print("good");
  Serial.println(F("SenseAir read OK"));
  delay(3000);
}

// CO2 lecture SenseAir S8
int co2SenseAir() {
  static byte responseSA[MB_PKT_7] = { 0 };
  memset(responseSA, 0, MB_PKT_7);
  co2sensor.write(cmdReSA, MB_PKT_8);
  co2sensor.readBytes(responseSA, MB_PKT_7);
  CO2 = (256 * responseSA[3]) + responseSA[4];


  if (CO2 != 0) {
    crc_cmd = crcx::crc16(responseSA, 5);
    if (responseSA[5] == lowByte(crc_cmd) && responseSA[6] == highByte(crc_cmd)) {
      return CO2;
    } else {
      while (co2sensor.available() > 0)
        char t = co2sensor.read();  // Clear serial input buffer;


      display.clear();
      display.print("----");
      return 0;
    }
  } else {

    display.clear();
    delay(10);
    display.print("----");
    return 0;
  }
}



void check_calmode_active() {
  unsigned long currentTime_ms = millis();

  // Test CALI button
  if (digitalRead(BUTTON_CALI) == LOW) {
    if (isLongPressCALI) {
      if (currentTime_ms > (StartPress_ms + LongPress_ms)) {
        ExitCali = false;
        display.clear();
        delay(100);


        Serial.println(F("Start calibration process: 300 seconds of 400 ppm stable"));
        for (int i = 300; i > -1; i--)

        {  // loop from 0 to 300

          display.clear();
          display.print("CAL-");
          delay(1000);
          Serial.print(i);
          Serial.print(" ");



          CO2 = co2SenseAir();

          Serial.print(F("CO2(ppm): "));
          Serial.println(CO2);
          display.clear();
          display.printNumber(i);
          delay(1000);
          i--;
        }
        if (ExitCali == false) {

          CalibrationSenseAir();

          isLongPressCALI = false;
          Done();
          delay(4000);
        }
        ExitCali = false;
      }
    } else {
      StartPress_ms = millis();
      isLongPressCALI = true;
      Serial.println(F("Button CALI has been pressed, hold 5s more to start calibration"));
      //blink led
      for (int i = 0; i < 10; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
        delay(100);
      }
    }
  } else {
    isLongPressCALI = false;
  }
}

// Calibration routine SenseAir S8
void CalibrationSenseAir() {
  for (int i = 0; i < 10; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }

  byte responseSA[MB_PKT_7] = { 0 };
  delay(100);

  //Step 1 Calibration
  co2sensor.write(cmdCal1, MB_PKT_8);
  co2sensor.readBytes(response, MB_PKT_8);

  Serial.print(F("Calibration Step1: "));
  CheckResponse(cmdCal1, response, MB_PKT_8);
  delay(1000);

  //Step 2 Calibration
  co2sensor.write(cmdCal2, MB_PKT_8);
  co2sensor.readBytes(response, MB_PKT_8);

  Serial.print(F("Calibration Step2: "));
  CheckResponse(cmdCal2, response, MB_PKT_8);
  delay(4000);

  //Step 3 Calibration
  co2sensor.write(cmdCal3, MB_PKT_8);
  co2sensor.readBytes(responseSA, MB_PKT_7);

  Serial.print(F("Resetting forced calibration factor to 400: "));
  CheckResponse(cmdCalR, responseSA, MB_PKT_7);
}