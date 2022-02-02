
/*
  NMEA Power Thing
 
  Provides a 4 channel current, voltage and coloumb counter
        with NMEA formatted XDR sentence broadcast.
 
  The circuit:
  * I2C bus: ADS1115 16-bit metering four ~12V DC supply via voltage divider
             ADS1115 16 bit metering four current shunts an eg 75mV 500A shunt
             20x4 LCD display
  * D5, D6   EC11 5 Pin Rotary Encoder (rotary contacts)
  * D7       EC11 5 Pin Rotary Encoder (push button switch)         
  
  Created 19 January 2022
  By Nicholas Taylor
 
  http://url/of/online/tutorial.cc
  https://stackr.ca
 
*/

#include <konfig.h>
#include <matrix.h>
#include <ukf.h>

#include <FS.h> //this needs to be first, or it all crashes and burn


#include <LiquidCrystal_I2C.h> // Library for LCD
#include <BigFont02_I2C.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 adsCurrent;  /* Use this for the 16-bit version */
Adafruit_ADS1115 adsVoltage;

#include <ESP_EEPROM.h>

#include <ESP8266WiFi.h>
#include <ESP8266TrueRandom.h>

#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>         // https://github.com/tzapu/WiFiManager
#include <ArduinoJson.h>          // https://github.com/bblanchon/ArduinoJson

#include <WiFiUdp.h>
#include <Wire.h>
#include <OneButton.h>

#include <RotaryEncoder.h>

// Example for ESP8266 NodeMCU with input signals on pin D5 and D6
#define PIN_IN1 D5
#define PIN_IN2 D6

byte uuidNumber[16]; // UUIDs in binary form are 16 bytes long
String uuidStr;
String nuuidStr;

//flag for saving data
bool shouldSaveConfig = false;

// A pointer to the dynamic created rotary encoder instance.
// This will be done in setup()
RotaryEncoder *encoder = nullptr;

int lastMillis = micros();

IRAM_ATTR void checkPosition()
{
  encoder->tick(); // just call tick() to check the state.
}

// Set web server port number to 80
WiFiServer server(80);

// Variable to store the HTTP request
String header;


IPAddress ipBroadCast(192, 168, 10, 255);

/* Bar graph display stuff

*/
int delt_t = 0;

static float G = 9.81;
unsigned char b, b2;
double a, a2;
unsigned int segment, segment2;
double perc = 100;
boolean logar;
byte bn;                 // bn BAR NUMBER every bar have to be numbered from 1 to 40
unsigned int xpos, ypos, blen;
double pkval[41];        // to store the last peak value for each bar
int    pkcyc[41][2];     // set the num. of printbar recall TO DO [1] and DONE [2] for each bar before the peak decays,

// if pkcyc[bn][1] == 0 no peaks wanted
// it's a workaround to avoid to waste time in delay functions (internal or external)
// that may interfere your application performances

// Bar characters definitions (8 maximum allowed)

byte blockHorizontal[8][8] =
{
  { 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10 }, // define characters for bar
  { 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18 },
  { 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C },
  { 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E },
  { 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08 }, // define characters for peak
  { 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04 },
  { 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02 },
  { 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01 },
};

// Bar characters definitions (8 maximum allowed)
byte blockVertical[8][8] =
{
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F }, // define characters for bar
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x1F },
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x1F, 0x1F },
  { 0x00, 0x00, 0x00, 0x00, 0x1F, 0x1F, 0x1F, 0x1F },
  { 0x00, 0x00, 0x00, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F },
  { 0x00, 0x00, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F },
  { 0x00, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F },
  { 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F },
};

/*
   OneButton menu stuff
*/


bool editFlag = false;
bool selectFlag = false;
bool scrollFlag = false;
boolean firstLoopFlag = true;

int changeAmount = 0;
bool encoderChangeFlag = false;
int encoderChangeAmount = 0;

/*
   Implementation specific
*/

// ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
// ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
// ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
// ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
// ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
// ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV


int adcNumBits = 32767;

int voltageMillis[4];
int currentMillis[4];

int voltageElapsedMillis[4];
int currentElapsedMillis[4];


int startTimeCurrentDisplay[4];
int startTimeVoltageDisplay[4];
int startTimeCoulombDisplay[4];

int elapsedTimeXDR[4];

float voltage[4];
float current[4];

float gainVoltage[4];
float biasVoltage[4];

float gainCurrent[4];
float biasCurrent[4];

float shuntMaxCurrent[4];
float shuntMaxVoltage[4];


int countAvailableUpdateFrequencies = 16;
float availableUpdateFrequencies[] = {0, 0.0000115741, 0.000034722222, 0.0000694444, 0.000277778, 0.001111111, 0.016666667, 0.0625, 0.125, 0.25, 0.5, 1, 2, 5, 10, 20};

char * availableUpdateFrequenciesLabels[] = {
  " OFF ",
    " 24hr",
    "  8hr",
    "  4hr",
  "  1hr",
  " 15m ",
  " 60s ",
  " 15s ",
  "7.5s ",
  "  4s ",
  "  2s ",
  "  1Hz",
  "  2Hz",
  "  5Hz",
  " 10Hz",
  " 20Hz"
};

/*
   Screen and menu related variables
*/

static int screenIndex = 0;
static int lastScreenIndex = 0;
static int maxScreenIndex = 25;

static int selectIndex = 0;
static int lastSelectIndex = 0;

static int editIndex = 0;
static int lastEditIndex = 0;
int screenChangeAmount = 0;
int selectChangeAmount = 0;
int editChangeAmount = 0;

static int scrollIndex = 0;
static int lastScrollIndex = 0;
int scrollChangeAmount = 0;
static int maxScrollIndex = 13;
static int scrollOffset = 0;

bool broadcastWiFiFlag = false;
bool broadcastNMEAFlag = true;
bool broadcastXDRFlag = true;

int startTimeNMEA = 0;
int startTimeXDR[4];

float broadcastXDRFrequency[4];

OneButton button(D7, true, true);

int lastCount = 0;

unsigned int udpRemotePort = 10110;

int maxHistoryCurrent[4];
int windowHistoryCurrent[4];

int maxHistoryVoltage[4];
int windowHistoryVoltage[4];

int maxHistoryCoulomb[4];
int windowHistoryCoulomb[4];

const int maxHistoryCurrent0  = 200;
static int windowHistoryCurrent0 = 50;

const int maxHistoryWindow  = 200;

const int maxHistoryCurrent1  = 200;
static int windowHistoryCurrent1 = 50;

const int maxHistoryCurrent2  = 200;
static int windowHistoryCurrent2 = 50;

const int maxHistoryCurrent3  = 200;
static int windowHistoryCurrent3 = 50;

float ampHour = 0.0;

float conversionAdcCurrent[4];
float conversionAdcVoltage[4];

float historyCurrent0 [maxHistoryWindow];
float historyCurrent1 [maxHistoryWindow];
float historyCurrent2 [maxHistoryWindow];
float historyCurrent3 [maxHistoryWindow];

float sumCoulomb[4];

const int maxHistoryCoulomb0  = 200;
static int windowHistoryCoulomb0 = 50;

const int maxHistoryVoltage0  = 200;
static int windowHistoryVoltage0 = 50;

const int maxHistoryVoltage1  = 200;
static int windowHistoryVoltage1 = 50;

const int maxHistoryVoltage2  = 200;
static int windowHistoryVoltage2 = 50;

const int maxHistoryVoltage3  = 200;
static int windowHistoryVoltage3 = 50;

int elapsedTimeCurrentDisplay[4];
int elapsedTimeVoltageDisplay[4];
int elapsedTimeCoulombDisplay[4];

int displayChannelCurrentFrequencyIndex[4];
int displayChannelVoltageFrequencyIndex[4];
int displayChannelCoulombFrequencyIndex[4];

int broadcastXDRFrequencyIndex[4];

float historyCurrent[4] [maxHistoryCurrent0];
float historyVoltage[4] [maxHistoryVoltage0];
float historyCoulomb[4] [maxHistoryCoulomb0];

//float historyAccMagnitude [maxHistoryAcc];

int indexHistoryCurrent[4];
int indexHistoryVoltage[4];
int indexHistoryCoulomb[4];

float minHistoryMagnitudeCurrent[4];
float maxHistoryMagnitudeCurrent[4];

float minHistoryMagnitudeVoltage[4];
float maxHistoryMagnitudeVoltage[4];


float minHistoryMagnitudeCoulomb[4];
float maxHistoryMagnitudeCoulomb[4];

float displayChannelCurrentFrequency[4];
float displayChannelVoltageFrequency[4];
float displayChannelCoulombFrequency[4];

/*
   WiFi broadcast settings
*/

const int UDP_PACKET_SIZE = 80;
char udpBuffer[ UDP_PACKET_SIZE ];

const byte UUIDLENGTH = 36;
char uuid[UUIDLENGTH + 1];

const byte NUUIDLENGTH = 4;
char nuuid[NUUIDLENGTH + 1];

//const char uuid[] = "0a65a21e-8c81-4581-8dcc-3818f3c2d53a";
//const char nuuid[] = "0a65";

char *nameThingWiFiAP = "Thing 0a65";

double microsStartTime = 0;

WiFiUDP udp;
WiFiClient espClient;

LiquidCrystal_I2C lcd(0x27, 20, 4); // I2C address 0x27, 16 column and 2 r
BigFont02_I2C     big(&lcd); // construct large font object, passing to it the name of our lcd object
#ifdef LCD


//char const *prefixNMEA = "KM"; // Feel free to change this if not suiting your needs

/* Setup WiFi */

#endif // LCD

#define AHRS true         // Set to false for basic data read
#define SerialDebug true  // Set to true to get Serial output for debugging

// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed  = 13;  // Set up pin 13 led for toggling

#define I2Cclock 400000
#define I2Cport Wire

float kalmanBaselineAccX = 0.0;
float kalmanBaselineAccY = 0.0;
float kalmanBaselineAccZ =   0.0;

int frequencyIndex = 0;


#include <SimpleKalmanFilter.h>

SimpleKalmanFilter kalmanAccX(1, 1, 0.01);
SimpleKalmanFilter kalmanAccY(1, 1, 0.01);
SimpleKalmanFilter kalmanAccZ(1, 1, 0.01);

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void saveUuid() {
  writeString(0, uuidStr);
}

String loadUuid() {

  String data;
  data = read_String(0);

  uuidStr = data;
  nuuidStr = uuidStr.substring(0, 4);
  strcpy(nameThingWiFiAP, "Thing ");
  strcat(nameThingWiFiAP, nuuidStr.c_str());


  return data;


}

String read_String(char add)
{
  int i;
  char data[100]; //Max 100 Bytes
  int len = 0;
  unsigned char k;
  k = EEPROM.read(add);
  while (k != '\0' && len < 500) //Read until null character
  {
    k = EEPROM.read(add + len);
    data[len] = k;
    len++;
  }
  data[len] = '\0';
  return String(data);
}

void writeString(char add, String data)
{
  int _size = data.length();
  int i;
  for (i = 0; i < _size; i++)
  {
    EEPROM.write(add + i, data[i]);
  }
  EEPROM.write(add + _size, '\0'); //Add termination null character for String Data
  EEPROM.commit();
}

void resetUuid() {
  ESP8266TrueRandom.uuid(uuidNumber);
  uuidStr = ESP8266TrueRandom.uuidToString(uuidNumber);
  nuuidStr = uuidStr.substring(0, 4);
  //Serial.println("The UUID number is " + uuidStr);
  strcpy(nameThingWiFiAP, "Thing ");
  strcat(nameThingWiFiAP, nuuidStr.c_str());
  //strcat(nameThingWiFiAP, "
  //  Serial.println("The NUUID number is " + nuuidStr);
  //    Serial.println(nameThingWiFiAP);
  saveUuid();
}

WiFiManager wifiManager;

void setup()
{

  //setupSPIFFS();
  lcd.init(); //initialize the lcd
  lcd.backlight(); //open the backlight

  big.begin();

  Serial.begin(38400);
  while (!Serial) {};
  EEPROM.begin(400);
  loadUuid();
  //resetUuid();
  displayAbout();

  delay(4000);

  defaultSettings();
  loadSettings();

  lcd.clear();



  Wire.begin();
  // TWBR = 12;  // 400 kbit/sec I2C speed



  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);

  button.attachDoubleClick(doubleClick);
  button.attachLongPressStop(longPress);
  button.attachClick(Click);

  encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::FOUR3);

  // register interrupt routine
  attachInterrupt(digitalPinToInterrupt(PIN_IN1), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_IN2), checkPosition, CHANGE);

  lcd.setCursor(0, 0);         // move cursor to   (2,
  lcd.print("ADS   Connecting ");

  if (!adsCurrent.begin(0x48)) {
    Serial.println("Failed to initialize ADS Current.");
    while (1);
  }

  adsCurrent.setGain(GAIN_SIXTEEN);

  if (!adsVoltage.begin(0x49)) {
    Serial.println("Failed to initialize ADS Voltage.");
    while (1);
  }
  adsVoltage.setGain(GAIN_ONE);

  lcd.setCursor(0, 0);         // move cursor to   (2,
  lcd.print("ADS   Connected  ");

  //lcd.clear();
  lcd.setCursor(0, 1);         // move cursor to   (2,
  lcd.print("WIFI  Connecting ");

  // WiFiManager

  //WiFiManager wifiManager;
  //    wifiManager.setConfigPortalTimeout(15);
  wifiManager.setAPCallback(configModeCallback);
  /*
  if ( digitalRead(D7) == LOW ) {

    lcd.setCursor(0, 1);
    lcd.print("WIFI  Portal opened");

    bool res;

    res = wifiManager.startConfigPortal(nameThingWiFiAP);

    if (!res) {
      lcd.setCursor(0, 1);         // move cursor to   (2,
      lcd.print("WIFI  Did not connect");
    }
    else {
      lcd.setCursor(0, 1);         // move cursor to   (2,
      lcd.print("WIFI  SSID SET   ");
    }

    delay(1000);

  }
  */
    lcd.setCursor(0, 1);
  lcd.print("WIFI  Portal opened");
  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
  bool res;
  wifiManager.setConfigPortalBlocking(false);
  //res = wifiManager.startConfigPortal(nameThingWiFiAP);
  //res = wifiManager.autoConnect(nameThingWiFiAP, "password");
  res = wifiManager.autoConnect(nameThingWiFiAP);
  if (!res) {
    lcd.setCursor(0, 1);         // move cursor to   (2,
    lcd.print("WIFI  Did not connect");
    Serial.println("Configportal running");
    // Start up AP as available.


  } else {
    lcd.setCursor(0, 1);         // move cursor to   (2,
    lcd.print("WIFI  SSID SET   ");
    Serial.println("WIFI SSID got.");
    sendPMTK("WIFI SSID got"); 
    // Connected to an SSID.
  }
  //if (displayFlag) {
    delay(1000);
  //}



  server.begin();
  //delay(2000);


  int wifiCount = 0;
  int maxWifiCount = 20;

  int wifiStartTime = micros();
  WiFi.begin();

  while (WiFi.status() != WL_CONNECTED && wifiCount < maxWifiCount) {

    delay(500);
    lcd.setCursor(17, 1);        // move cursor to   (2,

    int wifiElapsedTime = int((micros() - wifiStartTime) / 1e6);
    if (wifiElapsedTime < 10) {
      lcd.print(wifiElapsedTime);
      lcd.print("s ");

    } else {

      lcd.print(".  ");
    }

    if (SerialDebug)
    {
      Serial.print(".");
    }
    wifiCount += 1;


  }

  lcd.setCursor(0, 1);         // move cursor to   (2, 1)
  if (WiFi.status() == WL_CONNECTED) {
    lcd.print(     "WIFI  Connected     ");
  } else {
    lcd.print(     "WIFI  Not connected    ");

  }

  delay(1000);
  lcd.clear();
}


void displayAbout() {

  lcd.setCursor(0, 0);         // move cursor to   (0, 0)
  lcd.print("NMEA POWER THING");        // print message at (0, 0)

  lcd.setCursor(0, 1);         // move cursor to   (2, 1)
  lcd.print("stackr.ca");

  lcd.setCursor(0, 2);         // move cursor to   (2, 1)
  lcd.print("1 February 2022");

  lcd.setCursor(0, 3);         // move cursor to   (2, 1)
  lcd.print(nuuidStr);

}

// not used
void configModeCallback (WiFiManager * myWiFiManager) {
  // Serial.println("Entered config mode");
  // Serial.println(WiFi.softAPIP());
  // Serial.println(myWiFiManager->getConfigPortalSSID());
  return;
}

// dev
void setupSPIFFS() {
  /*
    //clean FS, for testing
    //SPIFFS.format();

    //read configuration from FS json
    Serial.println("mounting FS...");

    if (SPIFFS.begin()) {
      Serial.println("mounted file system");
      if (SPIFFS.exists("/config.json")) {
        //file exists, reading and loading
        Serial.println("reading config file");
        File configFile = SPIFFS.open("/config.json", "r");
        if (configFile) {
          Serial.println("opened config file");
          size_t size = configFile.size();
          // Allocate a buffer to store contents of the file.
          std::unique_ptr<char[]> buf(new char[size]);

          configFile.readBytes(buf.get(), size);
          DynamicJsonBuffer jsonBuffer;
          JsonObject& json = jsonBuffer.parseObject(buf.get());
          json.printTo(Serial);
          if (json.success()) {
            Serial.println("\nparsed json");
            strcpy(output, json["output"]);
          } else {
            Serial.println("failed to load json config");
          }
        }
      }
    } else {
      Serial.println("failed to mount FS");
    }
    //end read
  */
}

void webConfigPortal() {
  int timeout = 180;
  WiFiManager wm;

  //reset settings - for testing
  //wm.resetSettings();

  // set configportal timeout
  wm.setConfigPortalTimeout(timeout);

  if (!wm.startConfigPortal(nameThingWiFiAP)) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.restart();
    delay(5000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");

}

void sendUDP(char *inData)
{
  if (broadcastNMEAFlag == false) {
    return;
  }

  if (WiFi.status() == WL_CONNECTED) {

    udp.beginPacket(ipBroadCast, udpRemotePort);
    udp.write(inData);
    int response = udp.endPacket();

    if (response == 0) {
      Serial.print("sendUDP Did not send packet.\n");
    } else {

    }

  } else {
    Serial.print("No WiFi.\n");
  }
  Serial.print(inData);
  Serial.print("\n");

}

char* substr(char* arr, int begin, int len)
{
  char* res = new char[len + 1];
  for (int i = 0; i < len; i++)
    res[i] = *(arr + begin + i);
  res[len] = 0;
  return res;
}

int nmea0183_checksum(char *nmea_data)
{
  int crc = 0;
  int i;
  // ignore the first $ sign, no checksum in sentence
  for (i = 1;  i < strlen(nmea_data); i ++) { // remove the - three because no chksum is presnet
    crc ^= nmea_data[i];
  }
  return crc;
}

void defaultSettings() {


  for (int i = 0;  i < 4; i ++) {
    startTimeXDR[i] = 0;

    gainVoltage[i] = 5.01 / 0.41;
    biasVoltage[i] = 0.0;

    conversionAdcCurrent[i] = 0.0078125 / 1000;
    conversionAdcVoltage[i] = 0.125 / 1000;

    gainCurrent[i] = 1;
    biasCurrent[i] = 0.0;

    indexHistoryVoltage[i] = 0;
    indexHistoryCurrent[i] = 0;
    indexHistoryCoulomb[i] = 0;

    minHistoryMagnitudeCurrent[i] = 1e9;
    maxHistoryMagnitudeCurrent[i] = -1e9;

    minHistoryMagnitudeVoltage[i] = 1e9;
    maxHistoryMagnitudeVoltage[i] = -1e9;

    minHistoryMagnitudeCoulomb[i] = 1e9;
    maxHistoryMagnitudeCoulomb[i] = -1e9;

    maxHistoryCurrent[i] = 200;
    windowHistoryCurrent[i] = 50;

    maxHistoryVoltage[i] = 200;
    windowHistoryVoltage[i] = 50;

    maxHistoryCoulomb[i] = 200;
    windowHistoryCoulomb[i] = 50;

    shuntMaxCurrent[i] = 500;
    shuntMaxVoltage[i] = 0.075;

    sumCoulomb[i] = 0.0;

    displayChannelCurrentFrequencyIndex[i] = 2;
    displayChannelVoltageFrequencyIndex[i] = 2;
    displayChannelCoulombFrequencyIndex[i] = 2;

    displayChannelCurrentFrequency[i] = indexToFrequency(displayChannelCurrentFrequencyIndex[i]);
    displayChannelVoltageFrequency[i] = indexToFrequency(displayChannelVoltageFrequencyIndex[i]);
    displayChannelCoulombFrequency[i] = indexToFrequency(displayChannelCoulombFrequencyIndex[i]);

    broadcastXDRFrequencyIndex[i] = 7;
    broadcastXDRFrequency[i] = indexToFrequency(broadcastXDRFrequencyIndex[i]);

    startTimeCurrentDisplay[i] = micros();
    elapsedTimeCurrentDisplay[i] = 0;

    startTimeVoltageDisplay[i] = micros();
    elapsedTimeVoltageDisplay[i] = 0;

    startTimeCoulombDisplay[i] = micros();
    elapsedTimeCoulombDisplay[i] = 0;

  }

}

void sendPMTK(char * data) {
/*
 * Packet Length:
The maximum length of each packet is restricted to 255 bytes.
Packet Contents:
Preamble: 1 byte character. ‘$’
Talker ID: 4 bytes character string. “PMTK”
Packet Type: 3 bytes character string. From “000” to “999”
Data Field: The Data Field has variable length depending on the packet type.
A comma symbol ‘,’ must be inserted ahead each data field to help the decoder process the
Data Field.
*: 1 byte character. ‘*’
The start symbol is used to mark the end of Data Field.
CHK1, CHK2: 2 bytes character string. CHK1 and CHK2 are the checksum of data between Preamble and ‘*’.
CR, LF: 2 bytes binary data. (0x0D, 0x0A)
The 2 bytes are used to identify the end of a packet
 * 
 */
 char nmeaData[255];
  char *dollar = "$";
  char *sentenceNMEA = "PMTK";
  strcpy(nmeaData, dollar);
  //strcat(nmeaData, prefixNMEA);
  strcat(nmeaData, sentenceNMEA);

  strcat(nmeaData, ",");
  strcat(nmeaData, "000");
  strcat(nmeaData, ",");
  strcat(nmeaData, data);
  
  int crc =  nmea0183_checksum(nmeaData);
  char hex[2];
  sprintf(hex, "%02X", crc);

  strcat(nmeaData, "*");

  strcat(nmeaData, hex);

  sendUDP(nmeaData);
  
}

void sendXDR(char * prefixNMEA, char * sensorNameA, float roll, char * sensorNameB, float pitch, char * sensorNameC, float aY)
{
  char nmeaData[80];
  char *dollar = "$";
  char *sentenceNMEA = "XDR";
  strcpy(nmeaData, dollar);
  strcat(nmeaData, prefixNMEA);
  strcat(nmeaData, sentenceNMEA);
  strcat(nmeaData, ",");
  strcat(nmeaData, "A,");

  char append[5];
  sprintf(append, "%.1f", roll);
  strcat(nmeaData, append);

  strcat(nmeaData, ",");
  strcat(nmeaData, "D");
  strcat(nmeaData, ",");
  strcat(nmeaData, sensorNameA);
  strcat(nmeaData, ",");

  strcat(nmeaData, "A,");

  sprintf(append, "%.1f", pitch);
  strcat(nmeaData, append);

  strcat(nmeaData, ",");
  strcat(nmeaData, "D");
  strcat(nmeaData, ",");
  strcat(nmeaData, sensorNameB);
  strcat(nmeaData, ",");

  strcat(nmeaData, "A");
  strcat(nmeaData, ",");
  //  char append[5];
  sprintf(append, "%.1f", aY);
  strcat(nmeaData, append);

  strcat(nmeaData, ",");
  strcat(nmeaData, "X");
  strcat(nmeaData, ",");
  strcat(nmeaData, sensorNameC);

  int crc =  nmea0183_checksum(nmeaData);
  char hex[2];
  sprintf(hex, "%02X", crc);

  strcat(nmeaData, "*");

  strcat(nmeaData, hex);

  sendUDP(nmeaData);



}

void(* resetFunc) (void) = 0;  // declare reset fuction at address 0



void displayVariable(float variable, int column, int row, int width, int precision) {
  char variableString[5];

  dtostrf(variable, width, precision, variableString);

  lcd.setCursor(column, row);
  lcd.print(variableString);
}



void displayQuantityUnit(float quantity, String units, int precision, int column, int row) {
  char variableString[5];

  String multiplierText = "k";
  float multiplier = 1;
  int width = 4 + precision;
  //int precision = 1;
//Serial.println(quantity);


  if (abs(quantity) < 1e-6) {

    multiplier = 1e-9;
    multiplierText = " p";

  } else   if (abs(quantity) < 1e-3) {

    multiplier = 1e-6;
    multiplierText = " u";

  } else if (abs(quantity) < 0) {

    multiplier = 1e-3;
    multiplierText = " m";

  } else if (abs(quantity) < 1e3) {

    multiplier = 1;
    multiplierText = " ";

  } else if (quantity < 1e6) {

    multiplier = 1e3;
    multiplierText = " k";

  } else if (quantity < 1e9) {

    multiplier = 1e6;
    multiplierText = " M";

  } else if (abs(quantity) < 1e12) {

    multiplier = 1e9;
    multiplierText = " G";

  }

  float x = quantity/multiplier;

  dtostrf(x, width, precision, variableString);

  lcd.setCursor(column, row);
  lcd.print(variableString + multiplierText + units + (multiplier == 1 ? " " : ""));
}

// function to round the number
int roundN(int n, int precision)
{
  // Smaller multiple
  int a = (n / precision) * precision;

  // Larger multiple
  int b = a + precision;

  // Return of closest of two
  return (n - a > b - n) ? b : a;
}


// this function will be called when the button was pressed.
void Click()
{

  if (selectFlag == false && editFlag == false) {
    editFlag = true;
    return;
  }
  if (selectFlag == false && editFlag == true) {
    selectFlag = true;
    return;
  }
  if (selectFlag == true && editFlag == true) {
    selectFlag = false;
    return;
  }
  if (selectFlag == false && editFlag == true) {
    editFlag = false;
    return;
  }

} // Click


// this function will be called when the button was pressed 2 times in a short timeframe.
void doubleClick()
{

} // doubleClick

// this function will be called when the button was pressed 2 times in a short timeframe.
void longPress()
{
  //Serial.println("LONG");

  selectFlag = false;
  editFlag = false;

  saveSettings();

} // doubleClick

void saveSettings() {

  int eeAddress = 0;

  eeAddress += 50;
  EEPROM.put(eeAddress, (bool)broadcastNMEAFlag);



  for (int i = 0; i < 4; i++) {
    eeAddress += 4;
    EEPROM.put(eeAddress, (int)broadcastXDRFrequencyIndex[i]);

    eeAddress += 4;
    EEPROM.put(eeAddress, (int)displayChannelVoltageFrequencyIndex[i]);

    eeAddress += 4;
    EEPROM.put(eeAddress, (int)displayChannelCurrentFrequencyIndex[i]);

    eeAddress += 4;
    EEPROM.put(eeAddress, (int)displayChannelCoulombFrequencyIndex[i]);

  }

  for (int i = 0; i < 4; i++) {
    eeAddress += 4;
    EEPROM.put(eeAddress, biasCurrent[i]);

    eeAddress += 4;
    EEPROM.put(eeAddress, gainCurrent[i]);

    eeAddress += 4;
    EEPROM.put(eeAddress,   shuntMaxCurrent[i]);

    eeAddress += 4;
    EEPROM.put(eeAddress,   shuntMaxVoltage[i]);

    eeAddress += 4;
    EEPROM.put(eeAddress, biasVoltage[i]);

    eeAddress += 4;
    EEPROM.put(eeAddress, gainVoltage[i]);

  }


  boolean ok1 = EEPROM.commit();
  // EEPROM.end();
  // Serial.println((ok1) ? "First commit OK" : "Commit failed");

}

float getFloatEEPROM(int eeAddress, float defaultValue) {
  float floatVariable = 0.0;
  EEPROM.get(eeAddress, floatVariable);
  if (isnan(floatVariable)) {
    floatVariable = defaultValue;
  }
  return floatVariable;

}

int getIntEEPROM(int eeAddress, int defaultValue) {
  int intVariable = 0.0;
  EEPROM.get(eeAddress, intVariable);
  if (isnan(intVariable)) {
    intVariable = defaultValue;
  }
  return intVariable;

}

void loadSettings() {

  int eeAddress = 0;

  eeAddress += 50;
  EEPROM.get(eeAddress, broadcastNMEAFlag);
  if (isnan(broadcastNMEAFlag)) {
    broadcastNMEAFlag = false;
  }

  for (int i = 0; i < 4; i++) {
    eeAddress += 4;
    broadcastXDRFrequencyIndex[i] = getIntEEPROM(eeAddress, broadcastXDRFrequencyIndex[i]);

    eeAddress += 4;
    displayChannelVoltageFrequencyIndex[i] = getIntEEPROM(eeAddress, displayChannelVoltageFrequencyIndex[i]);

    eeAddress += 4;
    displayChannelCurrentFrequencyIndex[i] = getIntEEPROM(eeAddress, displayChannelCurrentFrequencyIndex[i]);

    eeAddress += 4;
    displayChannelCoulombFrequencyIndex[i] = getIntEEPROM(eeAddress, displayChannelCoulombFrequencyIndex[i]);


    broadcastXDRFrequency[i] = indexToFrequency(broadcastXDRFrequencyIndex[i]);
    displayChannelVoltageFrequency[i] = indexToFrequency(displayChannelVoltageFrequencyIndex[i]);
    displayChannelCurrentFrequency[i] = indexToFrequency(displayChannelCurrentFrequencyIndex[i]);
    displayChannelCoulombFrequency[i] = indexToFrequency(displayChannelCoulombFrequencyIndex[i]);
  }

  for (int i = 0; i < 4; i++) {

    eeAddress += 4;
    biasCurrent[i] = getFloatEEPROM(eeAddress, biasCurrent[i]);

    eeAddress += 4;
    gainCurrent[i] = getFloatEEPROM(eeAddress, gainCurrent[i]);

    eeAddress += 4;
    shuntMaxCurrent[i] = getFloatEEPROM(eeAddress, shuntMaxCurrent[i]);

    eeAddress += 4;
    shuntMaxVoltage[i] = getFloatEEPROM(eeAddress, shuntMaxVoltage[i]);

    eeAddress += 4;
    biasVoltage[i] = getFloatEEPROM(eeAddress, biasVoltage[i]);

    eeAddress += 4;
    gainVoltage[i] = getFloatEEPROM(eeAddress, gainVoltage[i]);

  }

}

float magnitudeVector(float x, float y, float z) {

  float magnitude = sqrt(x * x + y * y + z * z);
  return magnitude;

}

void updateEditIndex(int maxEditIndex, bool wrapFlag) {

  editIndex = lastEditIndex + editChangeAmount;
  if (editIndex < 0) {
    editIndex = 0;
  }
  if (editIndex > maxEditIndex) {
    editIndex = maxEditIndex;
  }
}

void updateScrollIndex(int maxScrollIndex, bool wrapFlag) {

  scrollIndex = lastScrollIndex + scrollChangeAmount;
  if (scrollIndex < 0) {
    scrollIndex = 0;
  }
  if (scrollIndex > maxScrollIndex) {
    scrollIndex = maxScrollIndex;
  }
}

float indexToFrequency(int index) {
  return availableUpdateFrequencies[index];
}

void displayVoltageChart(int chan) {

  for ( int i = 0 ; i < 8 ; i++ )
    lcd.createChar( i, blockVertical[i] );

  logar = false;

  maxHistoryMagnitudeVoltage[chan] = 0.0;
  minHistoryMagnitudeVoltage[chan] = 1000.0;

  for (int i = 1; i <= 20; i++) // print a series of short bars
  {
    int j = ((indexHistoryVoltage[chan] - (i - 1)) + indexHistoryVoltage[chan]) % windowHistoryVoltage[chan];
    float m = historyVoltage[chan][j];

    if (m > maxHistoryMagnitudeVoltage[chan]) {
      maxHistoryMagnitudeVoltage[chan] = m;
    }
    if (m < minHistoryMagnitudeVoltage[chan]) {
      minHistoryMagnitudeVoltage[chan] = m;
    }
  }

  for (int i = 1; i <= 20; i++) // print a series of short bars
  {
    int j = ((indexHistoryVoltage[chan] - (i - 1)) + windowHistoryVoltage[chan]) % windowHistoryVoltage[chan];

    float m = historyVoltage[chan][j];

    perc = (float)(m / 0.075 * 100);

    pkcyc[i][1] =  0;
    printVerticalBar (i, perc, 20 - i, 3, 3, false);
  }

  lcd.setCursor(0, 0);
  lcd.print("VOLTAGE ");

  lcd.setCursor(8, 0);
  lcd.print(chan + 1);

  displayEditBrackets(editIndex == 0 && editFlag, 0, 13, 19);
  lcd.setCursor(14, 0);

  if (editFlag) {
    lcd.print(availableUpdateFrequenciesLabels[displayChannelVoltageFrequencyIndex[chan]]);
  } else {
    //displayVariable(historyVoltage[chan][indexHistoryVoltage[chan]], 12, 0, 6, 3);
    displayQuantityUnit(voltage[chan], "V", 1, 12, 0);
    //lcd.setCursor(18, 0);
    //lcd.print(" V");
  }

}

void displayCurrentChart(int chan) {

  for ( int i = 0 ; i < 8 ; i++ )
    lcd.createChar( i, blockVertical[i] );

  logar = false;

  maxHistoryMagnitudeCurrent[chan] = 0.0;
  minHistoryMagnitudeCurrent[chan] = 1000.0;

  for (int i = 1; i <= 20; i++) // print a series of short bars
  {
    int j = ((indexHistoryCurrent[chan] - (i - 1)) + windowHistoryCurrent[chan]) % windowHistoryCurrent[chan];
    float m = historyCurrent[chan][j];

    if (m > maxHistoryMagnitudeCurrent[chan]) {
      maxHistoryMagnitudeCurrent[chan] = m;
    }
    if (m < minHistoryMagnitudeCurrent[chan]) {
      minHistoryMagnitudeCurrent[chan] = m;
    }
  }

    float chartMaxHistoryMagnitudeCurrent = maxHistoryMagnitudeCurrent[chan] * 1.1;
  float chartMinHistoryMagnitudeCurrent = minHistoryMagnitudeCurrent[chan] * 0.9;

  for (int i = 1; i <= 20; i++) // print a series of short bars
  {
    int j = ((indexHistoryCurrent[chan] - (i - 1)) + windowHistoryCurrent[chan]) % windowHistoryCurrent[chan];

    float m = historyCurrent[chan][j];

    perc = (float)(m / shuntMaxCurrent[chan] * 100);
    perc = (float)((m - minHistoryMagnitudeCurrent[chan]) / (chartMaxHistoryMagnitudeCurrent - chartMinHistoryMagnitudeCurrent) * 100) + 10;
    pkcyc[i][1] =  0;
    printVerticalBar (i, perc, 20 - i, 3, 3, false);
  }

  lcd.setCursor(0, 0);
  lcd.print("CURRENT ");

  lcd.setCursor(8, 0);
  lcd.print(chan + 1);

  displayEditBrackets(editIndex == 0 && editFlag, 0, 13, 19);
  lcd.setCursor(14, 0);

  if (editFlag) {
    lcd.print(availableUpdateFrequenciesLabels[displayChannelCurrentFrequencyIndex[chan]]);
  } else {
    displayQuantityUnit(current[chan], "A", 1, 12, 0);
  }

}

void displayCoulombChart(int chan) {


  for ( int i = 0 ; i < 8 ; i++ )
    lcd.createChar( i, blockVertical[i] );

  logar = false;

  maxHistoryMagnitudeCoulomb[chan] = -1e9;
  minHistoryMagnitudeCoulomb[chan] = 1e9;

  for (int i = 1; i <= 20; i++) // print a series of short bars
  {
    int j = ((indexHistoryCoulomb[chan] - (i - 1)) + windowHistoryCoulomb[chan]) % windowHistoryCoulomb[chan];
    float m = historyCoulomb[chan][j];

    if (m > maxHistoryMagnitudeCoulomb[chan]) {
      maxHistoryMagnitudeCoulomb[chan] = m;
    }
    if (m < minHistoryMagnitudeCoulomb[chan]) {
      minHistoryMagnitudeCoulomb[chan] = m;
    }
  }

  float chartMaxHistoryMagnitudeCoulomb = maxHistoryMagnitudeCoulomb[chan] * 1.1;
  float chartMinHistoryMagnitudeCoulomb = minHistoryMagnitudeCoulomb[chan] * 0.9;

  for (int i = 1; i <= 20; i++) // print a series of short bars
  {
    int j = ((indexHistoryCoulomb[chan] - (i - 1)) + windowHistoryCoulomb[chan]) % windowHistoryCoulomb[chan];

    float m = historyCoulomb[chan][j];

    perc = (float)((m - minHistoryMagnitudeCoulomb[chan]) / (chartMaxHistoryMagnitudeCoulomb - chartMinHistoryMagnitudeCoulomb) * 100) + 10;

    pkcyc[i][1] =  0;
    printVerticalBar (i, perc, 20 - i, 3, 3, false);
  }

  lcd.setCursor(0, 0);
  lcd.print("COULOMB");

  lcd.setCursor(8, 0);
  lcd.print(chan + 1);

  displayEditBrackets(editIndex == 0 && editFlag, 0, 13, 19);
  lcd.setCursor(14, 0);

  if (editFlag) {
    lcd.print(availableUpdateFrequenciesLabels[displayChannelCoulombFrequencyIndex[chan]]);
  } else {
       displayQuantityUnit(sumCoulomb[chan], "C", 1, 12, 0);

  }

}

void displayEditBrackets(bool editFlag, int row, int startColumn, int endColumn) {
  displayBrackets(editFlag, row, startColumn, endColumn, "[", "]");
  return;
  if (editFlag) {
    lcd.setCursor(startColumn, row);
    lcd.print("[");
    lcd.setCursor(endColumn, row);
    lcd.print("]");

  } else {
    lcd.setCursor(startColumn, row);
    lcd.print(" ");
    lcd.setCursor(endColumn, row);
    lcd.print(" ");
  }
}

void displayActionBrackets(bool editFlag, int row, int startColumn, int endColumn) {
  displayBrackets(editFlag, row, startColumn, endColumn, "<", ">");

}

void displayBrackets(bool editFlag, int row, int startColumn, int endColumn, char* startBracket, char* endBracket) {
  if (editFlag) {
    lcd.setCursor(startColumn, row);
    //lcd.print("[");
        lcd.print(startBracket);
    lcd.setCursor(endColumn, row);
    //lcd.print("]");
        lcd.print(endBracket);

  } else {
    lcd.setCursor(startColumn, row);
    lcd.print(" ");
    lcd.setCursor(endColumn, row);
    lcd.print(" ");
  }
}


void loop()
{
  button.tick();

  // Web server (http)
  wifiManager.process();
  webThing();

  int tempChan = 0;

  int16_t adcCurrent[4];
  int16_t adcVoltage[4];

  for (int i = 0;  i < 4; i ++) {

    voltageElapsedMillis[i] = micros() - voltageMillis[i];
    voltageMillis[i] = micros();
    adcVoltage[i] = adsVoltage.readADC_SingleEnded(i);

    currentElapsedMillis[i] = micros() - currentMillis[i];
    currentMillis[i] = micros();
    adcCurrent[i] = adsCurrent.readADC_SingleEnded(i);
    //voltage[i] = adcVoltage[i] * conversionAdcVoltage[i] * gainVoltage[i] + biasVoltage[i];
    //current[i] = adcCurrent[i] * conversionAdcCurrent[i] / shuntMaxVoltage[i] * shuntMaxCurrent[i] * gainCurrent[i] + biasCurrent[i];


    voltage[i] = (float) adsVoltage.computeVolts(adcVoltage[i]) * gainVoltage[i] + biasVoltage[i];
    current[i] = (float) adsCurrent.computeVolts(adcCurrent[i]) / shuntMaxVoltage[i] * shuntMaxCurrent[i] * gainCurrent[i] + biasCurrent[i];
    // shuntMaxVoltage[i] * shuntMaxCurrent[i] * gainCurrent[i] + biasCurrent[i];
    
    sumCoulomb[i] = sumCoulomb[i] + current[i] * currentElapsedMillis[i] / 1e6;

  }
/*
Serial.println(adcVoltage[0]);
Serial.println(voltage[0],4);
Serial.println(conversionAdcVoltage[0]);
Serial.println(gainVoltage[0],4);
Serial.println(biasVoltage[0],4);
*/
  static int pos = 0;

  int newPos = encoder->getPosition();


  if (pos != newPos) {
    encoderChangeFlag = true;

    encoderChangeAmount = newPos - pos;

    if (! (editFlag || selectFlag)) {
      screenChangeAmount = screenChangeAmount + newPos - pos;
    }

    if (selectFlag) {
      selectChangeAmount = selectChangeAmount + newPos - pos;
    }

    if (editFlag and !selectFlag) {
      editChangeAmount = editChangeAmount + newPos - pos;
    }

    pos = newPos;

  } // if


  if (true) {


    // Serial print and/or display at fixed rate independent of data rates
    delt_t = micros() - lastCount;

    // update LCD once per 100ms independent of read rate
    if (delt_t > 100 * 1000)
    {


      screenIndex = lastScreenIndex + screenChangeAmount;
      if (screenIndex < 0) {
        screenIndex = 0;
      }
      if (screenIndex > maxScreenIndex) {
        screenIndex = maxScreenIndex;
      }

      if (screenIndex != lastScreenIndex) {
        lcd.clear();
      }

      /*
         Render LCD page
      */
      switch (screenIndex) {
        case 0:
          selectFlag = false;
          editFlag = false;

          lcd.setCursor(12, 3);
          for (int i = 0;  i < 4; i ++) {

            lcd.setCursor(0, i);
            lcd.print("CH");
            lcd.setCursor(2, i);
            lcd.print(i + 1);

            displayQuantityUnit(current[i], "A", 1, 5, i);
            displayQuantityUnit(voltage[i], "V", 1, 12, i);
          }

          break;

        case 1:
        case 2:
        case 3:
        case 4:

          tempChan = screenIndex - 1;
          selectFlag = false;
          editFlag = false;

          lcd.setCursor(0, 0);
          lcd.print("CHANNEL");

          lcd.setCursor(8, 0);
          lcd.print(tempChan + 1);

          lcd.setCursor(0, 1);
          lcd.print("CURRENT");

          lcd.setCursor(0, 2);
          lcd.print("VOLTAGE");

          lcd.setCursor(0, 3);
          lcd.print("AMPHOUR");

          ampHour = sumCoulomb[tempChan] / 3600;

          displayQuantityUnit(current[tempChan], "A", 2, 8, 1);
          displayQuantityUnit(voltage[tempChan], "V", 3, 8, 2);
          displayQuantityUnit(ampHour, "A-hr", 1, 8, 3);

          break;

        case 5:
        case 6:
        case 7:
        case 8:

          tempChan = screenIndex - 5;
          if (lastScreenIndex != screenIndex) {
            editIndex = 0;
            lastEditIndex = 0;
            scrollOffset = 0;
          }

          displayVoltageChart(tempChan);

          if ((selectFlag || editFlag) && selectChangeAmount != 0) {
            //   switch (editIndex) {
            //     case 0:

            frequencyIndex = displayChannelVoltageFrequencyIndex[tempChan] + selectChangeAmount;

            if (frequencyIndex < 0) {
              frequencyIndex = 0;
            }
            if (frequencyIndex > countAvailableUpdateFrequencies - 1) {
              frequencyIndex = countAvailableUpdateFrequencies - 1;
            }
            displayChannelVoltageFrequency[tempChan] = (float)availableUpdateFrequencies[frequencyIndex];
            //   }

            displayChannelVoltageFrequencyIndex[tempChan] = frequencyIndex;
            displayChannelVoltageFrequency[tempChan] = indexToFrequency(frequencyIndex);
          }

          break;
        case 9:
        case 10:
        case 11:
        case 12:



          tempChan = screenIndex - 9;
          if (lastScreenIndex != screenIndex) {
            editIndex = 0;
            lastEditIndex = 0;
            scrollOffset = 0;
          }

          displayCurrentChart(tempChan);

          if ((selectFlag || editFlag) && selectChangeAmount != 0) {
            //  switch (editIndex) {
            //   case 0:
            frequencyIndex = displayChannelCurrentFrequencyIndex[tempChan] + selectChangeAmount;

            if (frequencyIndex < 0) {
              frequencyIndex = 0;
            }
            if (frequencyIndex > countAvailableUpdateFrequencies - 1) {
              frequencyIndex = countAvailableUpdateFrequencies - 1;
            }
            displayChannelCurrentFrequencyIndex[tempChan] = frequencyIndex;
            displayChannelCurrentFrequency[tempChan] = indexToFrequency(frequencyIndex);
          }
          break;

        case 13:
        case 14:
        case 15:
        case 16:
          tempChan = screenIndex - 13;
          if (lastScreenIndex != screenIndex) {
            editIndex = 0;
            lastEditIndex = 0;
            scrollOffset = 0;
          }

          displayCoulombChart(tempChan);

          if ((selectFlag || editFlag) && selectChangeAmount != 0) {
            //   switch (editIndex) {
            //    case 0:
            frequencyIndex = displayChannelCoulombFrequencyIndex[tempChan] + selectChangeAmount;

            if (frequencyIndex < 0) {
              frequencyIndex = 0;
            }
            if (frequencyIndex > countAvailableUpdateFrequencies - 1) {
              frequencyIndex = countAvailableUpdateFrequencies - 1;
            }
            displayChannelCoulombFrequencyIndex[tempChan] = frequencyIndex;
            displayChannelCoulombFrequency[tempChan] = indexToFrequency(frequencyIndex);


          }

          break;
        case 17:

          updateEditIndex(4, false);
          if (lastScreenIndex != screenIndex) {
            editIndex = 0;
            lastEditIndex = 0;
            scrollOffset = 0;
          }

          if ((editIndex - lastEditIndex) < 0) {
            scrollOffset = scrollOffset - 1;
          } else if ((editIndex - lastEditIndex) > 0) {
            scrollOffset = scrollOffset + 1;
          }

          if (scrollOffset > 2) {
            scrollOffset = 2;
          }
          if (scrollOffset < 0) {
            scrollOffset = 0;
          }

          lcd.setCursor(0, 0);
          lcd.print("NMEA SETTINGS");

          for (int i = 0; i < (2 + 1); i++) {

            displayEditBrackets(scrollOffset == i && editFlag , i + 1, 9, 17);


            switch (editIndex - scrollOffset + i) {
              case 0:
                lcd.setCursor(0, i + 1);
                lcd.print("TRANSMIT");
                break;
              case 1:
                lcd.setCursor(0, i + 1);
                lcd.print("XDR CH1 ");
                break;
              case 2:
                lcd.setCursor(0, i + 1);
                lcd.print("XDR CH2 ");
                break;

              case 3:
                lcd.setCursor(0, i + 1);
                lcd.print("XDR CH3 ");
                break;
              case 4:
                lcd.setCursor(0, i + 1);
                lcd.print("XDR CH4 ");
                break;

            }

            switch (editIndex - scrollOffset + i) {
              case 0:
                lcd.setCursor(10, i + 1);


                if (broadcastNMEAFlag) {
                  lcd.print("   ON  ");
                } else {

                  lcd.print("  OFF  ");
                }

                break;
              case 1:
              case 2:
              case 3:
              case 4:
                lcd.setCursor(10, i + 1);
                lcd.print(availableUpdateFrequenciesLabels[broadcastXDRFrequencyIndex[editIndex - scrollOffset + i - 1]]);
                break;

            }

            if (editFlag && selectFlag) {

              //saveSettings();
              //selectFlag = false;
              
            }

          }

          if (editFlag && selectFlag && selectChangeAmount != 0) {

            changeAmount = 0;

            switch (editIndex) {

              case 1:
              case 2:
              case 3:
              case 4:

                frequencyIndex = broadcastXDRFrequencyIndex[editIndex - 1] + selectChangeAmount;


                if (frequencyIndex < 0) {
                  frequencyIndex = 0;
                }
                if (frequencyIndex > countAvailableUpdateFrequencies - 1) {
                  frequencyIndex = countAvailableUpdateFrequencies - 1;

                }

                broadcastXDRFrequencyIndex[editIndex - 1] = frequencyIndex;
                broadcastXDRFrequency[editIndex - 1] = indexToFrequency(frequencyIndex);

                break;
            }

            switch (editIndex) {
              case 0:

                broadcastNMEAFlag = !broadcastNMEAFlag;

                break;


            }
          }

          break;


        case 18:
        case 19:
        case 20:
        case 21:
          tempChan = screenIndex - 18;

          updateEditIndex(5, false);
          if (lastScreenIndex != screenIndex) {
            editIndex = 0;
            lastEditIndex = 0;
            scrollOffset = 0;
          }

          if ((editIndex - lastEditIndex) < 0) {
            scrollOffset = scrollOffset - 1;
          } else if ((editIndex - lastEditIndex) > 0) {
            scrollOffset = scrollOffset + 1;
          }

          if (scrollOffset > 2) {
            scrollOffset = 2;
          }
          if (scrollOffset < 0) {
            scrollOffset = 0;
          }

          lcd.setCursor(0, 0);
          lcd.print("CHANNEL");
          lcd.setCursor(8, 0);
          lcd.print(tempChan + 1);

          lcd.setCursor(10, 0);
          lcd.print("SETTINGS");

          for (int i = 0; i < (2 + 1); i++) {

            displayEditBrackets(scrollOffset == i && editFlag , i + 1, 10, 19);

            switch (editIndex - scrollOffset + i) {
              case 0:
                lcd.setCursor(0, i + 1);
                lcd.print("CURR GAIN");
                break;
              case 1:
                lcd.setCursor(0, i + 1);
                lcd.print("CURR O/S ");
                break;
              case 2:
                lcd.setCursor(0, i + 1);
                lcd.print("SHNT AMPS");
                break;
              case 3:
                lcd.setCursor(0, i + 1);
                lcd.print("SHNT mV  ");
                break;
              case 4:
                lcd.setCursor(0, i + 1);
                lcd.print("VOLT GAIN ");
                break;
              case 5:
                lcd.setCursor(0, i + 1);
                lcd.print("VOLT O/S ");
                break;
            }

            //lcd.setCursor(0,0);
            //lcd.print(editIndex - scrollOffset + i);
            switch (editIndex - scrollOffset + i) {
              case 0:
                displayVariable(gainCurrent[tempChan], 12, i + 1, 6, 2);
                break;
              case 1:
                displayVariable(biasCurrent[tempChan], 12, i + 1, 6, 2);
                break;
              case 2:
                displayVariable(shuntMaxCurrent[tempChan], 12, i + 1, 6, 0);
                break;
              case 3:
                displayVariable(shuntMaxVoltage[tempChan], 12, i + 1, 6, 3);
                break;
              case 4:
                displayVariable(gainVoltage[tempChan], 12, i + 1, 6, 2);
                break;
              case 5:
                displayVariable(biasVoltage[tempChan], 12, i + 1, 6, 2);
                break;

            }



            if (editFlag && selectFlag) {

              //saveSettings();
              //selectFlag = false;
            }

          }

          if (editFlag && selectFlag && selectChangeAmount != 0) {

            changeAmount = 0;

            switch (editIndex) {
              case 0:
                gainCurrent[tempChan] += (float)selectChangeAmount / 100;
                break;
              case 1:
                biasCurrent[tempChan] += (float)selectChangeAmount / 100;
                break;
              case 2:
                shuntMaxCurrent[tempChan] += (float)selectChangeAmount * 5;
                break;
              case 3:
                shuntMaxVoltage[tempChan] += (float)selectChangeAmount / 200;
                break;
              case 4:
                gainVoltage[tempChan] +=  (float)selectChangeAmount / 100;
                break;
              case 5:
                biasVoltage[tempChan] +=  (float)selectChangeAmount / 100;
                break;

            }

          }

          break;

        case 22:
          lcd.setCursor(0,  0); lcd.print("UDP SETTINGS");

          lcd.setCursor(0, 1);
          lcd.print("IP");

          lcd.setCursor(0, 2);
          lcd.print("PORT");

          lcd.setCursor( 5, 1);
          lcd.print(WiFi.localIP());

          lcd.setCursor(5, 2);
          lcd.print(udpRemotePort);

          break;

        case 23:
          updateEditIndex(0, false);
          lcd.setCursor(0, 0);
          lcd.print("WIFI SETTINGS");

          lcd.setCursor(0, 1);
          lcd.print("SSID");

          lcd.setCursor(5, 1);
          lcd.print(WiFi.SSID().substring(0, 15));



          lcd.setCursor(1, 2);
          lcd.print("WEB CONFIG");


          //displayEditBrackets(editIndex == 0 && editFlag, 0 + 1, 0, 19);
          displayActionBrackets(editIndex == 0 && editFlag, 1 + 1, 0, 19);
          //displayEditBrackets(editIndex == 2 && editFlag, 2 + 1, 0, 19);

          if (editFlag && selectFlag) {
            switch (editIndex) {

              case 0:
                lcd.setCursor(1, 2);
                lcd.print("Thing " + nuuidStr + " open");
                webConfigPortal();
                lcd.print("WEB CONFIG OK.     ");
                //resetFunc(); //call reset
                selectFlag = false;

                break;

            }
          }




          break;

        case 24:
          updateEditIndex(2, false);
          lcd.setCursor(0, 0);
          lcd.print("CALIBRATION");


          lcd.setCursor(1, 1);
          lcd.print("RESTART       ");

          lcd.setCursor(1, 2);
          lcd.print("SET DEFAULTS  ");

          lcd.setCursor(1, 3);
          lcd.print("RESET THING UUID");

          displayActionBrackets(editIndex == 0 && editFlag, 0 + 1, 0, 19);
          displayActionBrackets(editIndex == 1 && editFlag, 1 + 1, 0, 19);
          displayActionBrackets(editIndex == 2 && editFlag, 2 + 1, 0, 19);

          if (editFlag && selectFlag) {
            switch (editIndex) {

              case 0:
                lcd.setCursor(1, 1);
                lcd.print("Restarting.      ");
                resetFunc(); //call reset
                // editFlag = false;
                // selectFlag = false;

                break;

              case 1:
                lcd.setCursor(1, 2);
                lcd.print("Setting defaults.   ");
                defaultSettings();
                saveSettings();
                //resetFunc(); //call reset
                editFlag = false;
                selectFlag = false;

                break;

              case 2:
                lcd.setCursor(1, 3);
                lcd.print("Resetting UUID.   ");
                //configWebPortal();
                resetUuid();
                lcd.print("UUID reset.     ");
                //saveSettings();
                //resetFunc(); //call reset
                //            editFlag = false;
                selectFlag = false;
                editFlag = false;

                break;

            }
          }

          break;
        case 25:
          displayAbout();
          break;

        default:
          break;
      }


      lastScreenIndex = screenIndex;
      lastEditIndex = editIndex;
      lastSelectIndex = selectIndex;

      lastCount = micros();

      encoderChangeFlag = false;
      encoderChangeAmount = 0;

      if (! (editFlag || selectFlag)) {
        screenChangeAmount = 0;
      }

      if (selectFlag) {
        selectChangeAmount = 0;
      }

      if (editFlag) {
        editChangeAmount = 0;
      }

    } // if (mpu.delt_t > 500)

  }

  //  int elapsedTimeNMEA = micros() - startTimeNMEA;

  for (int i = 0;  i < 4; i ++) {
    elapsedTimeCurrentDisplay[i] = micros() - startTimeCurrentDisplay[i];
    elapsedTimeVoltageDisplay[i] = micros() - startTimeVoltageDisplay[i];
    elapsedTimeCoulombDisplay[i] = micros() - startTimeCoulombDisplay[i];
    elapsedTimeXDR[i] = micros() - startTimeXDR[i];
  }

  for (int i = 0;  i < 4; i ++) {

    if (elapsedTimeXDR[i] > 1 / broadcastXDRFrequency[i] * 1e6) {
      startTimeXDR[i] = micros();


      char number[5];
      sprintf(number, "%d", i);

      char ampLabel[5];
      strcpy(ampLabel, "AMP");
      strcat(ampLabel, number);

      char voltLabel[5];
      strcpy(voltLabel, "VLT");
      strcat(voltLabel, number);


      char coulombLabel[5];
      strcpy(coulombLabel, "CLB");
      strcat(coulombLabel, number);

      sendXDR("TH", ampLabel, current[i], voltLabel, voltage[i], coulombLabel, sumCoulomb[i]);
    }
  }

  for (int i = 0;  i < 4; i ++) {

    if (elapsedTimeCurrentDisplay[i] > 1 / displayChannelCurrentFrequency[i] * 1e6) {
      startTimeCurrentDisplay[i] = micros();

      indexHistoryCurrent[i] = (indexHistoryCurrent[i] + 1) % windowHistoryCurrent[i];
      historyCurrent[i][indexHistoryCurrent[i]] = current[i];
    }


    if (elapsedTimeVoltageDisplay[i] > 1 / displayChannelVoltageFrequency[i] * 1e6) {
      startTimeVoltageDisplay[i] = micros();

      indexHistoryVoltage[i] = (indexHistoryVoltage[i] + 1) % windowHistoryVoltage[i];
      historyVoltage[i][indexHistoryVoltage[i]] = voltage[i];

    }

    if (elapsedTimeCoulombDisplay[i] > 1 / displayChannelCoulombFrequency[i] * 1e6) {

      startTimeCoulombDisplay[i] = micros();

      indexHistoryCoulomb[i] = (indexHistoryCoulomb[i] + 1) % windowHistoryCoulomb[i];
      historyCoulomb[i][indexHistoryCoulomb[i]] = sumCoulomb[i];

    }
  }
}

// https://forum.arduino.cc/t/lcd-bargraph-help-solved/350762
// https://forum.arduino.cc/u/robtillaart

//  METER BARS PRINTING FUNCTION
// passing percentage, x and y position, positions bar length, linear/audio logaritmic bar
// barnum is used to manage times of decayn of each peak separately (see definitions)
void printbar (byte bn, double perc, int xpos, int ypos, int blen, boolean logar)
{
  if ((logar == true) && (perc > 0))   // logaritmic bar
  {
    perc = ( log10(perc ) ) * 50;      // 10 * log10 (value) linear to logaritmic for AUDIO conversion
    if ( perc < 0 ) perc = 0;          // avoid negative values
  }

  a = blen / 99.5 * perc;              // calculate length of bar
  b = 0;

  if ( pkcyc[bn][1] > 0 )              // if PEAK is activated
  {
    if ( (a > (pkval[bn] - 0.01)) || (pkcyc[bn][2] > pkcyc[bn][1]) ) // new peak (w little histeresys) or expiration of peak
    {
      pkval[bn] = a;
      pkcyc[bn][2] = 0;                // reset cycles
    }
    pkcyc[bn][2]++;
  }

  // drawing filled rectangles
  if (a >= 1)
  {
    for (int i = 1; i < a; i++)
    {
      lcd.setCursor(xpos - 1 + i, ypos);
      lcd.write(255);
      b = i;
    }
    a = a - b;
  }
  segment = a * 5;

  // drawing final part of the bar
  if (b < blen)
  {
    lcd.setCursor(xpos + b, ypos);

    switch (segment) {
      case 0:
        lcd.print(" ");
        break;
      case 1:
        lcd.write((byte)0);
        break;
      case 2:
        lcd.write(1);
        break;
      case 3:
        lcd.write(2);
        break;
      case 4:
        lcd.write(3);
        break;
    }
  }

  // cleaning rest of line
  for (int i = 0; i < (blen - b - 1); i++)
  {
    lcd.setCursor(xpos + b + 1 + i, ypos);
    lcd.print(" ");
  }

  b2 = (int) pkval[bn];
  a2 = pkval[bn] - b2;
  segment2 = a2 * 5;

  // DRAWING PEAK
  if ( (pkcyc[bn][1] > 0) && (
         ((b + segment) == 0)                               // if bar empty
         || (b2 > b)                                        // or different box position
         || ( (b2 == b) && segment == 0 && segment2 > 0 )   // special case, too long to explain :-)
       ))
  {
    lcd.setCursor(xpos + b2, ypos);

    switch (segment2) {
      case 0:
        if ( (b2 > 0) || (b2 > b + 1))
        {
          lcd.setCursor(xpos + b2 - 1, ypos);
          lcd.write(7);
        };
        break;
      case 1:
        lcd.write(byte(0));
        break;
      case 2:
        lcd.write(4);
        break;
      case 3:
        lcd.write(5);
        break;
      case 4:
        lcd.write(6);
        break;
    }
  }
}

//  METER BARS PRINTING FUNCTION
// passing percentage, x and y position, positions bar length, linear/audio logaritmic bar
// barnum is used to manage STANDARD/PEAK bar and decayn of each peak separately (see definitions)

void printVerticalBar (byte bn, double perc, int xpos, int ypos, int blen, boolean logar)
{
  if ((logar == true) && (perc > 0))   // logaritmic bar
  {
    perc = ( log10(perc ) ) * 50;  // 10 * log10 (value) linear to logaritmic for AUDIO conversion
    if ( perc < 0 ) {
      perc = 0;  // avoid negative values
    }
  } else {

    if (perc > 99.5) {
      perc = 99.5;
    }

  }

  double a = blen / 99.5 * perc; // calculate length of bar
  b = 0;

  if ( pkcyc[bn][1] > 0 )              // if PEAK is activated
  {
    if ( a > (pkval[bn] - 0.1) | pkcyc[bn][2] > pkcyc[bn][1] ) // new peak (w little histeresys) or expiration of peak
    {
      pkval[bn] = a;
      pkcyc[bn][2] = 0;    // reset cycles
    }

    pkcyc[bn][2]++;
  }

  // drawing main bar
  if (pkcyc[bn][1] == 0)
  {
    drawrest (xpos, ypos, b, a, blen);
  }
  else

    // drawing peak bar
  {
    b2 = (int) pkval[bn];
    a = pkval[bn];
    drawrest (xpos, ypos, b2, a, blen);
  }
}

// trace bar and cleanes (also for peak)
void drawrest (int xpos, int ypos, unsigned char b, double a, int blen)
{
  // drawing filled rectangles
  if (a >= 1) {

    for (int i = 1; i < a; i++) {
      lcd.setCursor(xpos, ypos - i + 1);
      lcd.write(255);
      b = i;
    }
    a = a - b;
  }

  segment = a * 8;
  if (b < blen)
  {
    lcd.setCursor(xpos, ypos - b);

    switch (segment) {
      case 0:
        lcd.print(" ");
        break;
      case 1:
        lcd.write((byte)0);
        break;
      case 2:
        lcd.write(1);
        break;
      case 3:
        lcd.write(2);
        break;
      case 4:
        lcd.write(3);
        break;
      case 5:
        lcd.write(4);
        break;
      case 6:
        lcd.write(5);
        break;
      case 7:
        lcd.write(6);
        break;
    }
  }

  // cleaning rest of line
  for (int i = 0; i < (blen - b - 1); i++)
  {
    lcd.setCursor(xpos, ypos - b - i - 1);
    lcd.print(" ");
  }
}

void webThing() {

  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();

            // turns the GPIOs on and off
            if (header.indexOf("GET /5/on") >= 0) {
              Serial.println("GPIO 5 on");
              // output5State = "on";
              // digitalWrite(output5, HIGH);
            } else if (header.indexOf("GET /5/off") >= 0) {
              Serial.println("GPIO 5 off");
              // output5State = "off";
              //  digitalWrite(output5, LOW);
            } else if (header.indexOf("GET /4/on") >= 0) {
              Serial.println("GPIO 4 on");
              // output4State = "on";
              //  digitalWrite(output4, HIGH);
            } else if (header.indexOf("GET /4/off") >= 0) {
              Serial.println("GPIO 4 off");
              //  output4State = "off";
              // digitalWrite(output4, LOW);
            }

            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the on/off buttons
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #195B6A; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #77878A;}</style></head>");

            // Web Page Heading
            client.println("<body><h1>ESP8266 Web Server</h1>");

            // Display current state, and ON/OFF buttons for GPIO 5
            //client.println("<p>GPIO 5 - State " + output5State + "</p>");
            // If the output5State is off, it displays the ON button
            // if (output5State=="off") {
            //  client.println("<p><a href=\"/5/on\"><button class=\"button\">ON</button></a></p>");
            // } else {
            // client.println("<p><a href=\"/5/off\"><button class=\"button button2\">OFF</button></a></p>");
            //    }

            // Display current state, and ON/OFF buttons for GPIO 4
            //client.println("<p>GPIO 4 - State " + output4State + "</p>");
            // If the output4State is off, it displays the ON button
            //      if (output4State=="off") {
            //      client.println("<p><a href=\"/4/on\"><button class=\"button\">ON</button></a></p>");
            //  } else {
            //     client.println("<p><a href=\"/4/off\"><button class=\"button button2\">OFF</button></a></p>");
            //   }
            client.println("</body></html>");

            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }

  //}
}
