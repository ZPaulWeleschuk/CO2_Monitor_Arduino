 #include <SoftwareSerial.h>
 #include <CRCx.h> //https://github.com/hideakitai/CRCx

const byte PIN_TX = 6;  // define TX pin to Sensor
const byte PIN_RX = 7;  // define RX pin to Sensor

const byte MB_PKT_7 = 7;   //MODBUS Packet Size
const byte MB_PKT_8 = 8;   //MODBUS Packet Size

// SenseAir S8 MODBUS commands
const byte cmdReSA[MB_PKT_8] = {0xFE, 0X04, 0X00, 0X03, 0X00, 0X01, 0XD5, 0XC5}; // SenseAir Read CO2
const byte cmdOFFs[MB_PKT_8] = {0xFE, 0x06, 0x00, 0x1F, 0x00, 0x00, 0xAC, 0x03}; // SenseAir Close ABC
//const byte cmdCal1[MB_PKT_8] = {0xFE, 0x06, 0x00, 0x00, 0x00, 0x00, 0x9D, 0xC5}; // SenseAir Calibration 1
//const byte cmdCal2[MB_PKT_8] = {0xFE, 0x06, 0x00, 0x01, 0x7C, 0x06, 0x6C, 0xC7}; // SenseAir Calibration 2
//const byte cmdCal3[MB_PKT_8] = {0xFE, 0x03, 0x00, 0x00, 0x00, 0x01, 0x90, 0x05}; // SenseAir Calibration 3
//const byte cmdCalR[MB_PKT_7] = {0xFE, 0x03, 0x02, 0x00, 0x20, 0xAD, 0x88};       // SenseAir Calibration Response
static byte response[MB_PKT_8] = {0};

byte VALalti = 21;//TODO:calgary is 1045m elevation, as 1045/50 = 20.9
//we might not even need this if we pull hpa form the bme sensor
byte ConnRetry = 0;
int CO2 = 0;
int CO2value;
unsigned int crc_cmd;
//unsigned long StartPress_ms = 0;
float CO2cor;
float hpa;

SoftwareSerial co2sensor(PIN_RX, PIN_TX);
#define BAUDRATE 9600 // Device to SenseAir S8, MHZ19 and CM1106 Serial baudrate (should not be changed)


void setup()
{
  Serial.begin(9600);
  delay(100);
  Serial.println(F("Start SenseAir S8 lecture"));

  co2sensor.begin(BAUDRATE); // Sensor serial start
  delay(4000);

  CO2iniSenseAir();

  // Preheat routine: min 30 seconds for SCD30 & CM1106 & SenseAir S8, 180 seconds for MH-Z14 or 19
  Serial.print(F("Preheat: "));
  for (int i = 30; i > -1; i--)
  { // Preheat from 0 to 30 or to 180
    delay(1000);
    Serial.println(i);
    delay(1000);
    i--;
  }
  Serial.print(F("Start measurements compensated by Altitude: "));
  Serial.print(VALalti * 50);
  Serial.println(" m");
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
  else{
    CO2 = 0;
  }


  if (CO2 > 0)
  {
Serial.print("CO2(ppm): ");
    Serial.println(CO2);
  }
  delay(3967); // for SenseAir S8 4 seconds
}

//Routine Bad connection
void BadConn()
{
  Serial.println("Air sensor not detected. Please check wiring... Try# " + String(ConnRetry));
  delay(2500);
  ConnRetry++;
}

void Done()
{
  Serial.println(F("done"));
}

void Failed()
{
  Serial.println(F("failed"));
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
  hpa = 1014.7;//we can pull kpa from BME so no need to calc. 
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



  if (CO2 != 0)
  {
    crc_cmd = crcx::crc16(responseSA, 5);
    if (responseSA[5] == lowByte(crc_cmd) && responseSA[6] == highByte(crc_cmd))
    {
      Serial.print("OK");
      return CO2;
    }
    else
    {
      while (co2sensor.available() > 0)
        char t = co2sensor.read(); // Clear serial input buffer;
      Serial.println(F("FAIL"));
      return 0;
    }
  }
  else
  {

    Serial.println("FAIL");

    return 0;
  }
}