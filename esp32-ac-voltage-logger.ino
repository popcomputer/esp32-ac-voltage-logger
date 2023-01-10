#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <time.h> 
#include <WiFi.h>

const char* ssid       = "xxxxxxx";
const char* password   = "xxxxxxx";

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 25200;
const int   daylightOffset_sec = 0;

String dataMessage;
String timeStamp;

// Run function every PERIOD time (mS)
// logSDCard
unsigned long target_time = 0L;
#define PERIOD (1000L)
// getTimeServer
unsigned long target_time2 = 0L;
#define PERIOD2 (12*60*60*1000L)

/* This code works with ESP8266 12E or Arduino and ZMPT101B AC voltage sensor up to 250 VAC 50/60Hz
 * It permits the measure of True RMS value of any AC signal, not only sinewave
 * The code uses the Sigma "Standard deviation" method and displays the value every "printPeriod"
 * check www.SurtrTech.com for more details
 */

#include <Filters.h>                            //Library to use

#define ZMPT101B 36                            //Analog input

float testFrequency = 50;                     // test signal frequency (Hz)
float windowLength = 5/testFrequency;       // how long to average the signal, for statistist, changing this can have drastic effect
                                              // Test as you need

int RawValue = 0;     
float Volts_TRMS;     // estimated actual voltage in Volts
float Volts_TRMS_Prev;

float intercept = -5;  // to be adjusted based on calibration testin
float slope = 0.54353;      

/* How to get the intercept and slope? First keep them like above, intercept=0 and slope=1, 
 * also below keep displaying Calibrated and non calibrated values to help you in this process.
 * Put the AC input as 0 Volts, upload the code and check the serial monitor, normally you should have 0
 * if you see another value and it is stable then the intercept will be the opposite of that value
 * Example you upload first time and then you see a stable 1.65V so the intercept will be -1.65
 * To set the slope now you need to put the voltage at something higher than 0, and measure that using your reference TRMS multimeter
 * upload the new code with the new intercept and check the value displayed as calibrated values
 * Slope = (Measured values by multimeter)/(Measured values by the code)
 * Place your new slope and reupload the code, if you have problems with calibration try to adjust them both
 * or add a new line to calibrate further
 * Slope and intercept have nothing to do with the TRMS calculation, it's just an adjustement of the line of solutions
 */


unsigned long printPeriod = 50; //Measuring frequency, every 1s, can be changed
unsigned long previousMillis = 0;

RunningStatistics inputStats; //This class collects the value so we can apply some functions

#define LED_READVOLTAGE 2
#define LED_SDCARD 32
#define LED_AC_OK 27
#define LED_AC_FAIL 25
#define LED_HEARTBEAT 12
int LED_BLINK_State = LOW;  // Variable to store the LED state

struct tm timeinfo;
char timeStringBuff[50];    // timestamp string buffer for logSDCard function

bool AC_FAIL = false;

int ledStateHB = LOW;
unsigned long previousMillisHB = 0;
const long intervalHB = 1000;

// ================================================================================================================

void setup() {
  pinMode(LED_READVOLTAGE, OUTPUT);
  pinMode(LED_SDCARD, OUTPUT);
  pinMode(LED_AC_OK, OUTPUT);
  pinMode(LED_AC_FAIL, OUTPUT);
  pinMode(LED_HEARTBEAT, OUTPUT);

  inputStats.setWindowSecs( windowLength );
  Serial.begin(115200);    // start the serial port
  Serial.println("Serial started");

  // ==================
  Serial.println("====================================================");
  Serial.println("====================================================");

  //connect to WiFi and get time from NTP server
  getTimeServer();
  
  if(!SD.begin()){
      Serial.println("Card Mount Failed");
      return;
  }
  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE){
      Serial.println("No SD card attached");
      return;
  }

  Serial.print("SD Card Type: ");
  if(cardType == CARD_MMC){
      Serial.println("MMC");
  } else if(cardType == CARD_SD){
      Serial.println("SDSC");
  } else if(cardType == CARD_SDHC){
      Serial.println("SDHC");
  } else {
      Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

  // If the data.csv file doesn't exist
  // Create a file on the SD card and write the data labels
  File file = SD.open("/data.csv");
  if(!file) {
    Serial.println("File doens't exist");
    Serial.println("Creating file...");
    writeFile(SD, "/data.csv", "Timestamp,Voltage,Status\r\n");
  }
  else {
    Serial.println("File already exists");  
  }
  file.close();

  for (int i=0; i<10000; i++) readVoltage();
}

// ================================================================================================================

void loop() {
  readVoltage();  //The only function I'm running, be careful when using with this kind of boards
                  //Do not use very long delays, or endless loops inside the loop
  
  if(Volts_TRMS > 200) {            // 10% of 220V
    digitalWrite(LED_AC_OK, HIGH);  // Update the LED state
    digitalWrite(LED_AC_FAIL, LOW);  // Update the LED state
    Volts_TRMS_Prev = Volts_TRMS;
    if(AC_FAIL == true) {
      logSDCard(Volts_TRMS_Prev, false);    // Write to SD card
      AC_FAIL = false;
    }    
  } else {
    digitalWrite(LED_AC_OK, LOW);  // Update the LED state
    digitalWrite(LED_AC_FAIL, HIGH);  // Update the LED state
    AC_FAIL = true;
    if(millis() - target_time >= PERIOD) {
      target_time += PERIOD;
      if(Volts_TRMS - Volts_TRMS_Prev < -50) { // Different between current voltage and previous voltage more than 50 V
        Volts_TRMS_Prev = Volts_TRMS;
        logSDCard(Volts_TRMS_Prev, true);    // Write to SD card
      }
    }
  }

  if(millis() - target_time2 >= PERIOD2) {
    target_time2 += PERIOD2;   // change scheduled time exactly, no slippage will happen
    getTimeServer();  // Update time from NTP every 12 hours
  }

  // Heartbeat LED blink
  unsigned long currentMillisHB = millis();
  if (currentMillisHB - previousMillisHB >= intervalHB) {
    previousMillisHB = currentMillisHB;
    if (ledStateHB == LOW) {
      ledStateHB = HIGH;
    } else {
      ledStateHB = LOW;
    }
    digitalWrite(LED_HEARTBEAT, ledStateHB);
  }
}

// ================================================================================================================

void getTimeServer(){
  Serial.printf("Connecting to %s ", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" CONNECTED");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  delay(500);

  //init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();

  //disconnect WiFi as it's no longer needed
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  Serial.println("WIFI DISCONNECTED");
}

void printLocalTime(){
  Serial.print("Connecting to NTP server ...");
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(" CONNECTED");
  Serial.print("DateTime: ");
  Serial.println(&timeinfo, "%Y-%m-%d %H:%M:%S");
}

float readVoltage(){
  LED_BLINK_State = !LED_BLINK_State;  // Toggle the LED state
  digitalWrite(LED_READVOLTAGE, LED_BLINK_State);  // Update the LED state

  RawValue = analogRead(ZMPT101B);  // read the analog in value:
  inputStats.input(RawValue);       // log to Stats function
      
  if((unsigned long)(millis() - previousMillis) >= printPeriod) { // We calculate and display every 1s
    previousMillis = millis();   // update time
    
    Volts_TRMS = inputStats.sigma()* slope + intercept;
    // Volts_TRMS = Volts_TRMS*0.979;              // Further calibration if needed
    
    // Serial.print("Non Calibrated: ");
    // Serial.print("\t");
    // Serial.print(inputStats.sigma()); 
    // Serial.print("\t");
    // Serial.print("Calibrated: ");
    Serial.print("Voltage: ");
    Serial.println(Volts_TRMS);
  }
}

// Write the sensor readings on the SD card
void logSDCard(float Volts_TRMS_SD, bool AC_FAIL_SD) {
  getLocalTime(&timeinfo);
  strftime(timeStringBuff, sizeof(timeStringBuff), "%Y-%m-%d %H:%M:%S", &timeinfo);
  timeStamp = String(timeStringBuff);

  if(AC_FAIL_SD == false){
    dataMessage = timeStamp + "," + String(Volts_TRMS_SD) + ",OK" + "\r\n";
  } else {
    dataMessage = timeStamp + "," + String(Volts_TRMS_SD) + ",Fault" + "\r\n";
  }
  Serial.print("Save data: ");
  Serial.print(dataMessage);
  appendFile(SD, "/data.csv", dataMessage.c_str());
}

void writeFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file){
      Serial.println("Failed to open file for writing");
      return;
  }
  if(file.print(message)){
      Serial.println("File written");
  } else {
      Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file){
      Serial.println("Failed to open file for appending");
      return;
  }
  if(file.print(message)){
      Serial.println("Message appended");
      digitalWrite(LED_SDCARD, HIGH);
  } else {
      Serial.println("Append failed");
  }
  file.close();
  digitalWrite(LED_SDCARD, LOW);
}