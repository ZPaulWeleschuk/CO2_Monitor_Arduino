
#define SenseAir_S8 // SenseAir S8
// UNCOMMENT FOR DEBUG MODE
//#define DEBUG

// ***************************************************************************
// ***************************************************************************

#include <SoftwareSerial.h>
#include "SevenSegmentTM1637.h"
#include "SevenSegmentExtended.h"
#include <EEPROM.h>
#include <CRCx.h> //https://github.com/hideakitai/CRCx
#include <avr/wdt.h>

const byte RevVersion = 147; // Firmware version 23 aug 2021

const byte PIN_TX = 6;  // define TX pin to Sensor
const byte PIN_RX = 7;  // define RX pin to Sensor
const byte PIN_CLK = 9;
const byte PIN_DIO = 8;

const byte BUTTON_CALI = A0;
const byte BUTTON_BEEP = A3;
const byte BUTTON_ALTI = 2;
const byte BUZZER = 11;
const byte LED = 13;

const byte addressBEEP = 0; // EEPROM address 0
const byte addressALTI = 1; // EEPROM address 1


const byte MB_PKT_7 = 7;   //MODBUS Packet Size
const byte MB_PKT_8 = 8;   //MODBUS Packet Size

const byte MB_PKT_17 = 17; // MODBUS Packet Size



// SenseAir S8 MODBUS commands
const byte cmdReSA[MB_PKT_8] = {0xFE, 0X04, 0X00, 0X03, 0X00, 0X01, 0XD5, 0XC5}; // SenseAir Read CO2
const byte cmdOFFs[MB_PKT_8] = {0xFE, 0x06, 0x00, 0x1F, 0x00, 0x00, 0xAC, 0x03}; // SenseAir Close ABC
const byte cmdCal1[MB_PKT_8] = {0xFE, 0x06, 0x00, 0x00, 0x00, 0x00, 0x9D, 0xC5}; // SenseAir Calibration 1
const byte cmdCal2[MB_PKT_8] = {0xFE, 0x06, 0x00, 0x01, 0x7C, 0x06, 0x6C, 0xC7}; // SenseAir Calibration 2
const byte cmdCal3[MB_PKT_8] = {0xFE, 0x03, 0x00, 0x00, 0x00, 0x01, 0x90, 0x05}; // SenseAir Calibration 3
const byte cmdCalR[MB_PKT_7] = {0xFE, 0x03, 0x02, 0x00, 0x20, 0xAD, 0x88};       // SenseAir Calibration Response

static byte response[MB_PKT_8] = {0};

bool isLongPressCALI = false;
bool isLongPressBEEP = false;
bool isLongPressALTI = false;
bool ExitCali = false;

byte VALbeep = 0;
int VALDIS = 0;
byte VALalti = 21;//TODO:calgary is 1045m elevation, as 1045/50 = 20.9
//int VALalti = 21;//TODO:calgary is 1045m elevation, as 1045/50 = 20.9
//TODO:we are having issue where alti is being set to max byte (ie 255) thus when times by 50 results in 12750m elevation. we need to figure this out
//TODO:perhaps we should write this as a float//TODO:
byte ConnRetry = 0;
int CO2 = 0;
int CO2value;
unsigned int CO2temp = 0;
unsigned int crc_cmd;
unsigned int delayIN = 0;
unsigned long StartPress_ms = 0;
float CO2cor;
float hpa;

const int LongPress_ms = 5000; // 5s button timeout


SevenSegmentExtended display(PIN_CLK, PIN_DIO);
SoftwareSerial co2sensor(PIN_RX, PIN_TX);


#define BAUDRATE 9600 // Device to SenseAir S8, MHZ19 and CM1106 Serial baudrate (should not be changed)


void setup()
{
  pinMode(BUTTON_CALI, INPUT_PULLUP);
  pinMode(BUTTON_BEEP, INPUT_PULLUP);
  pinMode(BUTTON_ALTI, INPUT_PULLUP);
  pinMode(BUZZER, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A5, OUTPUT);
  pinMode(LED, OUTPUT);

  digitalWrite(BUZZER, LOW);
  digitalWrite(4, LOW);
  digitalWrite(A2, LOW);
  digitalWrite(A5, LOW);
  digitalWrite(LED, LOW);

  Serial.begin(9600);

  display.begin();           // initializes the display
  display.setBacklight(100); // set the brightness to 100 %
  delay(100);

  Serial.print(F("Firmware version: "));
  Serial.println(RevVersion);

  display.clear();
  display.print("Lco2");//TODO:remove
  Serial.println(F("Start LibreCO2"));//TODO:rename

#ifdef SenseAir_S8
  Serial.println(F("Start SenseAir S8 lecture"));
#endif
  co2sensor.begin(BAUDRATE); // Sensor serial start
  delay(4000);

  VALbeep = EEPROM.read(addressBEEP);
  //displayVALbeep(1000);
  delay(1500);

  //VALalti = EEPROM.read(addressALTI);//TODO:do we need to save alti to the eeprom??. seems overly complex
  //displayVALalti();
  delay(1500);



#ifdef SenseAir_S8
  CO2iniSenseAir();
#endif

  // Preheat routine: min 30 seconds for SCD30 & CM1106 & SenseAir S8, 180 seconds for MH-Z14 or 19
  Serial.print(F("Preheat: "));

  for (int i = 30; i > -1; i--)
  { // Preheat from 0 to 30 or to 180
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
  delay(5000);
}

void loop()
{
  CO2 = 0;




  CO2value = co2SenseAir();
  if (CO2value != 0)
  {
    hPaCalculation();
    CO2cor = float(CO2value) + (0.016 * ((1013 - float(hpa)) / 10) * (float(CO2value) - 400)); // Increment of 1.6% for every hPa of difference at sea level
    CO2 = round(CO2cor);
  }
  else
    CO2 = 0;

#ifdef DEBUG
  DebugCO2val();
#endif


  if (CO2 > 0)
  {
#ifdef DEBUG
    Serial.print(F("CO2(ppm): "));
#else
    Serial.print(" ");
#endif
    Serial.println(CO2);
    display.clear();
    delay(10);
    display.printNumber(CO2);
  }
  else
  {
#ifdef DEBUG
    Serial.println(F("ERROR CO2=<0"));
#endif
    display.clear();
    delay(10);
    display.print("----");
  }

  // Alarm if CO2 is greater than VALDIS

  if (CO2 >= VALDIS && VALDIS != 1450)
  {
    Beep();

    digitalWrite(LED, !digitalRead(LED));
  }

  //check_calmode_active();


  delay(3967); // for SenseAir S8 4 seconds

}

//Routine Bad connection

void BadConn()
{
  Serial.println("Air sensor not detected. Please check wiring... Try# " + String(ConnRetry));

  if (ConnRetry > 1)
  {
    display.clear();
    delay(20);
    display.print("bad");
  }
  delay(2500);
  ConnRetry++;
}

// Beep 900Hhz
void Beep()
{
  tone(BUZZER, 900);
  delay(330);
  noTone(BUZZER);
}

void Done()
{
  Serial.println(F("done"));
  display.clear();
  delay(10);
  display.print("done");
}

void Failed()
{
  Serial.println(F("failed"));
  display.clear();
  delay(10);
  display.print("fail");
}

void DebugCO2val()
{
  Serial.print("hpa: ");
  Serial.print(hpa);
  Serial.print(" CO2real: ");
  Serial.print(CO2value);
  Serial.print(" CO2adjust: ");
  Serial.println(CO2cor);
}

//Done or failed revision routine
void CheckResponse(uint8_t *a, uint8_t *b, uint8_t len_array_cmp)
{
  bool check_match = false;
  for (int n = 0; n < len_array_cmp; n++)
  {
    if (a[n] != b[n])
    {
      check_match = false;
      break;
    }
    else
      check_match = true;
  }

  if (check_match)
  {
    Done();
  }
  else
  {
    Failed();
  }
}


// Calculate of Atmospheric pressure

void hPaCalculation()
{
  //hpa = 1013 - 5.9 * float(VALalti) + 0.011825 * float(VALalti) * float(VALalti); // Cuadratic regresion formula obtained PA (hpa) from high above the sea
  hpa = 1019.1;//we can pull kpa from BME so no need to calc. 
  //TODO:typically reading 1050ish units of co2 in office. need to take outside and see what reading is, should be about 412ppm
  //or why dont we define it as this?. . . //TODO://TODO:
  Serial.print(F("Atmospheric pressure calculated by the sea level inserted (hPa): "));
  Serial.println(hpa);
}






// SenseAir S8 routines

// Initialice SenseAir S8
void CO2iniSenseAir()
{
  //Deactivate Automatic Self-Calibration
  co2sensor.write(cmdOFFs, MB_PKT_8);
  co2sensor.readBytes(response, MB_PKT_8);
  Serial.print(F("Deactivate Automatic Self-Calibration SenseAir S8: "));
  CheckResponse(cmdOFFs, response, MB_PKT_8);
  delay(2000);

  while (co2SenseAir() == 0 && (ConnRetry < 5))
  {
    BadConn();
  }



  Serial.println(F(" Air sensor detected AirSense S8 Modbus"));

  display.clear();
  delay(10);
  display.print("good");
  //delay(2500);
  Serial.println(F("SenseAir read OK"));
  delay(4000);
}

// CO2 lecture SenseAir S8
int co2SenseAir()
{
  static byte responseSA[MB_PKT_7] = {0};
  memset(responseSA, 0, MB_PKT_7);
  co2sensor.write(cmdReSA, MB_PKT_8);
  co2sensor.readBytes(responseSA, MB_PKT_7);
  CO2 = (256 * responseSA[3]) + responseSA[4];

#ifdef DEBUG
  Serial.print(F("Read SenseAir S8: "));
#endif

  if (CO2 != 0)
  {
    crc_cmd = crcx::crc16(responseSA, 5);
    if (responseSA[5] == lowByte(crc_cmd) && responseSA[6] == highByte(crc_cmd))
    {
#ifdef DEBUG
      Serial.println(F("OK DATA"));
#else
      Serial.print("OK");
#endif
      return CO2;
    }
    else
    {
      while (co2sensor.available() > 0)
        char t = co2sensor.read(); // Clear serial input buffer;

#ifdef DEBUG
      Serial.println(F("FAIL CRC_CO2"));
#else
      Serial.println(F("FAIL"));
#endif
      display.clear();
      display.print("----");
      return 0;
    }
  }
  else
  {
#ifdef DEBUG
    Serial.println(F("FAILED"));
#else
    Serial.println(F("FAIL"));
#endif
    display.clear();
    delay(10);
    display.print("----");
    return 0;
  }
}
