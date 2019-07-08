/*
 * rare altimeter for FlyisFun app
 * version 2.0
 * Arduino NANO, TFT 1.8" SPI, BlueTooth HC06(04), BMP280 I2C.
 * - added rotary switch
 *  
 * spec.tnx2 tomware26@gmail.com WWW: flying.funsite.cz
 * 
 * license: PUBLIC, GNU/GPL
 * 2019,OK1VBR
 * 
 */

#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMono9pt7b.h>
#include <Fonts/FreeMonoBold12pt7b.h>

//------------BMP280 sensor address and connection Vcc=3V !!!
// used default i2c pins:
// SDA = pin A4
// SCL = pin A5
#define BMP280_ADDRESS (0x76)
// sensor init from BMP library
Adafruit_BMP280 bmp;

//------------TFT Adafruit 1.44" or 1.8" SPI display
#define TFT_CS        10  // CS for TFT
#define TFT_RST        9  // Reset for TFT
#define TFT_DC         8  // DC(A0) for TFT
#define TFT_BACKLIGHT  6  // Display backlight pin
// default HARDWARE SPI pins Uno/Nano: 
// SPI MOSI = pin 11 (sda)
// SPI MISO = pin 12
// SPI SCLK = pin 13 (sck)
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

//-------------Bluetooth modul
#define RX 5 //pin 5
#define TX 4 //pin 4
#define BTStatePin 7
SoftwareSerial bluetooth(TX, RX);

//--------------rotary switch
enum PinAssignments 
{
  encoderPinA = 2,   // pin 2, right increase QNH (labeled DT)
  encoderPinB = 3,   // pin 3, left decrease QNH (labeled CLK)
  clearButton = 12   // pin 12, reset to MSA QNH 1013.25hPa (labeled SW)
};
//--------------default QNH after reset and step
float encoderPos = 1013.25;         // a counter for the dial
float lastReportedPos = 0.25;       // change management
static boolean rotating=false;      // debounce management
//--------------interrupt service routine vars
boolean A_set = false;              
boolean B_set = false;

// Vario variables
#define varioBufLen 6 // 6 values i.e 5x differencies
#define varioWeightsTotal 15  // varioWeightsTotal 15.0 = 1+2+3+4+5 tj 1..varioBufLen-1
float varioBuffer[varioBufLen] = {0.0};
long int varioLastTime = 0.0;

///////////// SETUP /////////////////
void setup() 
{
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP); 
  pinMode(clearButton, INPUT_PULLUP); 
  attachInterrupt(0, doEncoderA, CHANGE); // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(1, doEncoderB, CHANGE); // encoder pin on interrupt 1 (pin 3)
  pinMode(TFT_BACKLIGHT, OUTPUT);  
  pinMode(BTStatePin, INPUT);
  Serial.begin(9600);
  bluetooth.begin(9600);
    
    if (!bmp.begin(BMP280_ADDRESS)) {
      Serial.println("BMP280 sensor not found, check your wiring!");
      digitalWrite(TFT_BACKLIGHT, HIGH);
      tft.initR(INITR_BLACKTAB);
      tft.setRotation(2);
      tft.fillScreen(ST77XX_BLACK);
      tft.setTextColor(ST77XX_RED);
      tft.setCursor(0,20);
      tft.setTextSize(1);
      tft.print("ERROR of BMP280");
    while (1);
  } 

  // -------------Vario data setup 
  varioLastTime = millis();
  float QNH = encoderPos;
  float altitude = bmp.readAltitude(QNH);
  for(int i=0;i<varioBufLen;i++)
  {
    varioBuffer[i] = altitude;
  }
  
  // -------------TFT Display init

  digitalWrite(TFT_BACKLIGHT, HIGH);
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(2);
  tft.fillScreen(ST77XX_BLACK);
  
  tft.fillRect(3,3,122,11,ST77XX_GREEN);
  tft.setTextColor(ST77XX_BLACK);
  tft.setCursor(24,5);
  tft.setTextSize(1);
  tft.print("ALTITUDE AMSL");

  tft.fillRect(3,45,122,11,ST77XX_BLUE);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(55,47);
  tft.setTextSize(1);
  tft.print("QNH");

  tft.fillRect(3,83,122,11,ST77XX_BLUE);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(50,85);
  tft.setTextSize(1);
  tft.print("VARIO");

  tft.fillRect(3,118,122,11,ST77XX_BLUE);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(30,120);
  tft.setTextSize(1);
  tft.print("TEMPERATURE");

}

// --------------rotary interrupts ---------------------------
// Interrupt on A changing state
void doEncoderA(){
  // debounce
  if ( rotating ) delay (1);  // wait a little until the bouncing is done
  // Test transition, did things really change? 
  if( digitalRead(encoderPinA) != A_set ) {  // debounce once more
    A_set = !A_set;
    // adjust counter + if A leads B
    if ( A_set && !B_set ) 
      encoderPos += 0.25;
    rotating = false;  // no more debouncing until loop() hits again
   }
}
// Interrupt on B changing state, same as A above
void doEncoderB(){
  if ( rotating ) delay (1);
  if( digitalRead(encoderPinB) != B_set ) {
    B_set = !B_set;
    //  adjust counter – 1 if B leads A
    if( B_set && !A_set ) 
      encoderPos -= 0.25;
    rotating = false;
    }
}

//----------------- drawTFT for re-drawing display 

void drawtft(float pressure, float altitude, float temperature, float vario, float QNH)
{
 char buffer1[6]; //calculated altitude (AMSL in feets)
 char buffer2[6]; //rare pressure from bm280 (in hPa)
 char buffer3[6]; //calculated vario (in meters per seconds)
 char buffer4[6]; //temperature from bm280 (in Celsius)
     
  //ALTITUDE
    dtostrf(altitude, 6, 0, buffer1);
    tft.fillRect(3,17,122,25,ST77XX_BLACK);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(1,35);
    tft.setFont(&FreeMonoBold12pt7b);
    tft.setTextSize(1);
    tft.print(buffer1);
    tft.print("ft");
  //PRESSURE - QNH
    dtostrf(pressure, 6, 1, buffer2);
    tft.fillRect(3,59,122,20,ST77XX_BLACK);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(5,74);
    tft.setFont(&FreeMono9pt7b);
    tft.setTextSize(1);
    //tft.print(buffer2);
    tft.print(QNH);
    tft.print(" hPa");
  //VARIO
    dtostrf(vario, 5, 1, buffer3);
    tft.fillRect(3,95,122,20,ST77XX_BLACK);
    tft.setCursor(5,110);
    tft.setFont(&FreeMonoBold9pt7b);
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_GREEN);
    tft.print(buffer3);
    tft.print(" m/s");
  //TEMPERATURE 
    dtostrf(temperature, 6, 1, buffer4);
    tft.fillRect(3,132,122,20,ST77XX_BLACK);
    tft.setCursor(10,147);
    tft.setFont(&FreeMono9pt7b);
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_GREEN);
    tft.print(buffer4);
    tft.print(" C");
}


// ------------- CHECKSUM --------------------------------
char toHex(uint8_t nibble)
{
  if (nibble >= 10)
    return nibble + 'A' - 10;
  else
    return nibble + '0';
}

void generateChecksum(const char* s, char* checksum)
   {
  uint8_t c = 0;
  // Initial $ is omitted from checksum, if present ignore it.
  if (*s == '$')
  ++s;
   
  while (*s != '\0' && *s != '*')
  c ^= *s++;
   
  if (checksum) {
  checksum[0] = '*';  
  checksum[1] = toHex(c / 16);
  checksum[2] = toHex(c % 16);
  checksum[3] = '\0';
  }
  return s;
}

// ----------------------- Formating and sending data via bluetooth

void sendToBT(float pressure, float altitude, float temperature, float vario)
{
   char buffer[10];
   char line[255] = "$LK8EX1,";
   
   dtostrf(pressure*100, 0, 0, buffer);
   strcat(line,buffer);   
   strcat(line,",");

   dtostrf(altitude, 0, 0, buffer);
   strcat(line,buffer);   
   strcat(line,",");

   dtostrf(vario*100, 0, 0, buffer);
   strcat(line,buffer);   
   strcat(line,",");
   
   dtostrf(temperature, 0, 1, buffer);
   strcat(line,buffer);   
   strcat(line,",999,");
   
   //checksum
   generateChecksum(line, buffer);
   strcat(line,buffer);
      
   Serial.println(line);
   Serial.println();
   bluetooth.println(line);
}

// --------------- Vario computing ------------------------------------

float calcVario(float altitude)
{
  float retval=0;
  // move buffer by one value 
  for(int i=1;i<varioBufLen;i++)
  {
    varioBuffer[i-1]=varioBuffer[i];
  }
  varioBuffer[varioBufLen-1] = altitude;
  
#ifdef DEBUG  
  for(int i=0;i<varioBufLen;i++)
  {
      Serial.print("[");
      Serial.print(i);
      Serial.print("]");
      Serial.println(varioBuffer[i]);
  }
#endif
// weighted average , weight is i
  for(int i=1;i<varioBufLen;i++)
  {
    retval = retval + ( (varioBuffer[i] - varioBuffer[i-1]) * i );
  }
  
#ifdef DEBUG  
  Serial.print("SUM:");
  Serial.println(retval);
#endif

  retval = retval /varioWeightsTotal; // weightsTotal 15.0 = 1+2+3+4+5 ...varioBufLen-1
  
  float time = (float) millis() - varioLastTime;
  varioLastTime = millis();
  time = time / 1000; // to seconds
  
  retval = retval / time; // to m/s
  
  return retval;
}

// ------------------ MAIN LOOP ---------------------------------
void loop() 
{
  //---------- start of rotary in loop and setup standart QNH
  rotating = true;  // reset the debouncer
  if (lastReportedPos != encoderPos) {
    Serial.print("Index:");
    Serial.println(encoderPos, DEC);
    lastReportedPos = encoderPos;
  }
  if (digitalRead(clearButton) == LOW )  {
    encoderPos = 1013.25;
  }
  //-----------end of rotary in loop
  
  float QNH = encoderPos;
  // reading values from sensor
  // reading of temperature
  float temperature = bmp.readTemperature();
  // reading of pressure
  float pressure = (bmp.readPressure()/100.00);
  // reading of altitude on QNH
  float altitude = bmp.readAltitude(QNH);
  // convert to feets
  float altitudeFt = altitude * 3.280839895;
  // calculate vario
  float vario = calcVario(altitude);

  
  // write all information to serial line (terminal)
  // temperature print
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" deg. Celsia.");
  // print pressure in hekto Pascals
  Serial.print("Barometric pressure: ");
  Serial.print(pressure);
  Serial.println(" hPa");
  // print altitude in m and ft on QNH
  Serial.print("Altitude on QNH ");
  Serial.print(QNH);
  Serial.print(": ");
  Serial.print(altitude);
  Serial.print(" m = ");
  Serial.print(altitudeFt);
  Serial.println(" ft");
  // print vario
  Serial.print("Vario: ");
  Serial.print(vario);
  Serial.println(" m/s");

  // empty line print
  Serial.println();
 
  // test for available data on bluetooth
  // if is there any data to read, return number of availabe bytes 
  if (bluetooth.available())
  {
    Serial.print("reading message: ");
    while (bluetooth.available()) {
      Serial.print((char)bluetooth.read());
      // short pause betweeen reading
      delay(10);
    }    
     Serial.println();
  }

  // draw content of TFT display
  drawtft(pressure,altitudeFt,temperature,vario,QNH);
  
  // send data on bluetooth only if is connected
  if (digitalRead(BTStatePin)==HIGH)
  {
     sendToBT(pressure, altitude, temperature, vario);  
  }
  
  // and delay
  delay(200);
}
