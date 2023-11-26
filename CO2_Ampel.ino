/*
 * CO2 warner "traffic light"
 * (C) 2021 Alexander Schmitt, https://github.com/alexandermainz
 * This software runs under the MIT License (https://github.com/alexandermainz/remote-card-play/blob/main/LICENSE)
 * 
 * Additional libraries and licenses used:
 * https://github.com/WifWaf/MH-Z19 (LGPL-3.0)
 * https://github.com/jandrassy/ArduinoOTA (LGPL-2.1)
 */
#include <Arduino.h>
#include "MHZ19.h"                                        
#include <SoftwareSerial.h>        // Remove if using HardwareSerial or Arduino package without SoftwareSerial support
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>

// MH-Z19 hw setup
#define BAUDRATE 9600             // MH-Z19 serial connection baud rate (should not be changed)
#define RX_PIN D7                 // RX pin which the MHZ19 Tx pin is attached to
#define TX_PIN D6                 // TX pin which the MHZ19 Rx pin is attached to
#define PWM_PIN D5                // pin which the MHZ19 analogue PWM out pin is attached to

// signal LEDs hw setup
#define LED_Y_PIN D5              // pin which the yellow LED is attached to
#define LED_R_PIN D4              // pin which the red LED (or the red anode of an RGB LED) is attached to
#define LED_G_PIN D3              // pin which the greed LED (or the green anode of an RGB LED) is attached to

// WiFi & OTA passwords - adjust to your environment
#define STASSID "mySSID"      // WiFi SSID name to connect to
#define STAPSK  "myPass"      // WiFi WPA password
#define OTAPASS "otaPass"     // ArduinoOTA upload password

// Tago.io config: device token for the sensor - comment out or adjust to your environment
String Device_Token = "my-device-token";

// baud rate for the TTY serial port
const uint32_t SERIAL_SPEED = 115200;

// globals and class instatiations
MHZ19 myMHZ19;
SoftwareSerial mySerial(RX_PIN, TX_PIN);                   
WiFiClient client;
unsigned long lastread = 0;
const char* ssid = STASSID;
const char* password = STAPSK;

// CO2 status indicator variable: 0=red, 1=yellow, 2=green
unsigned int ampel = 3;

void setup() {
  pinMode(LED_R_PIN, OUTPUT);
  pinMode(LED_G_PIN, OUTPUT);
  pinMode(LED_Y_PIN, OUTPUT);
  digitalWrite(LED_R_PIN, HIGH);
  digitalWrite(LED_G_PIN, HIGH);
  digitalWrite(LED_Y_PIN, HIGH);
  
  // connect WiFi
  initWiFi();

  // Start serial port at Baud rate
  Serial.begin(SERIAL_SPEED);  

  // init MH-Z19A CO2 sensor, give some time to heat up
  Serial.println("\r\nInitializing MH-Z19A CO2 sensor...");
  delay(5000);
  mySerial.begin(BAUDRATE);                               
  myMHZ19.begin(mySerial);                           
  if (myMHZ19.errorCode == 1) {  //OK: shut off all LED after successful sensor init
    digitalWrite(LED_Y_PIN, LOW);
    digitalWrite(LED_R_PIN, LOW);
    digitalWrite(LED_G_PIN, LOW);
  }
  else {  // restart the ESP if sensor was not ready
    Serial.println("Sensor did not respond, restarting microcontroller...");
    digitalWrite(LED_G_PIN, LOW);
    delay(500);
    ESP.restart();
  }
  

  // turn off sensor autocalibration - for manually calibration only; otherwise set to true!
  myMHZ19.autoCalibration(true);
  // set the sensor's measurement range to 0-3000ppm
  myMHZ19.setRange(3000);                             

  // fetch some config values from the sensor for debug purposes
  Serial.print(" - sensor range: "); Serial.println(myMHZ19.getRange());
  Serial.print(" - autocal status: "); Serial.println(myMHZ19.getABC());
  Serial.print(" - sensor accuracy: "); Serial.println(myMHZ19.getAccuracy());
  Serial.print(" - background CO2 val: "); Serial.println(myMHZ19.getBackgroundCO2());
  
  // Init mDNS & set device name
  if (MDNS.begin("esp-co2ampel")) {
    Serial.println("MDNS responder started");
  }

  // Init OTA
  initOTA();
  Serial.println("ArduinoOTA started");

}  // of method setup()

void loop() {
  MDNS.update();
  ArduinoOTA.handle();

  // update sensor reading every 1 minute
  if (millis() - lastread > 1*60*1000) {
    lastread = millis();
    int CO2; 
    CO2 = myMHZ19.getCO2(false);     // Request CO2 (as ppm)
    Serial.print("CO2 (ppm): ");                      
    Serial.println(CO2);  
    //activate for PWM value reading:
    //Serial.print("CO2 (PWM read): "); Serial.println(readPWM());    
                              
    if (CO2 > 0) {   // only accept valid readings
      int8_t mhTemp;
      mhTemp = myMHZ19.getTemperature();           // Request sensor internal temperature (as Celsius)
      Serial.print("MH-Z19A Temperature (C): ");                  
      Serial.println(mhTemp);                               
      
      // send values to Tago.io - comment out if you're not using Tago.io for sensor visualization
      httpPostTago(CO2, mhTemp);

      // change "traffic lights" color according to CO2 ppm value
      setAmpel(CO2);
    }
    else
      Serial.println("Invalid reading from MH-Z19B! Ignoring values.");
  }
}  // of method loop()

void setAmpel(int co2_val) {
  unsigned int neueAmpel;
  // values below 1000ppm are considered as good
  if (co2_val < 1000)
    neueAmpel = 2;
  // values between 1000 and 1800ppm are considered as "warning"
  if (co2_val >= 1000 && co2_val <= 1800)
    neueAmpel = 1;
  // values above 1800ppm are considered as bad
  if (co2_val > 1800)
    neueAmpel = 0;

  if (ampel != neueAmpel) {
    ampel = neueAmpel;
    digitalWrite(LED_R_PIN, (ampel == 0));
    digitalWrite(LED_Y_PIN, (ampel == 1));
    digitalWrite(LED_G_PIN, (ampel == 2));
    // for RGB-LED with common cathode, set both R+G to high to simualate a "yellowish" color
    // comment out if using dictinct LEDs per color
    if (ampel == 1) {
      digitalWrite(LED_G_PIN, HIGH);
      digitalWrite(LED_R_PIN, HIGH);
    }
  }
}

// helper function for connecting to WiFi
void initWiFi() {
  Serial.print("Attempting to connect to SSID: ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  int tryout = 0;
  while (WiFi.status() != WL_CONNECTED && tryout++ < 3) {
    Serial.print(".");
    Serial.println(WiFi.begin(ssid, password));
    int wait = 0;
    while (WiFi.status() != WL_CONNECTED && wait++ < 12) {
      delay(500);
      Serial.print(".");
    }
    
    delay(5000);
  }
  if (WiFi.status() != WL_CONNECTED)  // reboot if WiFi cannot be connected
    ESP.restart();
    
  Serial.print("Connected to ");
  Serial.println(ssid);  
  Serial.println(WiFi.localIP());
}

// this method makes a HTTP POST request to the Tago.io server:
void httpPostTago(int CO2, int8_t mhTemp) {
  // close any connection before send a new request.
  // This will free the socket on the WiFi shield
  client.stop();

  Serial.print("\nStarting connection to Tago.io server... ");
  // if you get a connection, report back via serial:
  String postData = String("[{\"variable\":\"co2_sensor_temp\", \"value\":") + String(mhTemp) + String(",\"unit\":\"C\"},")
    + String("{\"variable\":\"co2\", \"value\":") + String(CO2) + String(",\"unit\":\"ppm\"}]");
  String dev_token = String("Device-Token: ") + String(Device_Token);
  Serial.println(postData); Serial.println(dev_token);
  if (client.connect("api.tago.io", 80)) {
    Serial.println("connected to server");
    client.println("POST /data? HTTP/1.1");
    client.println("Host: api.tago.io");
    client.println("_ssl: false");                        // for non-secured connection, use this option "_ssl: false"
    client.println("Connection: close");
    client.println(dev_token);
    client.println("Content-Type: application/json");
    client.print("Content-Length: ");
    client.println(postData.length());
    client.println();
    client.println(postData);
    Serial.println("request sent");

    // the following code can be commented in if you like to check the return value from http request:
/*
       int httpStatus = 0;
        delay(10);
        // get reply headers and check result code
        while (client.connected()) {
          String line = client.readStringUntil('\n');
          Serial.println("POST -> " +line);
          if (line.startsWith("HTTP/")) {
            if (line.indexOf("200 OK", 4) > -1) {
              httpStatus = 200;
            }
          }
          if (line == "\r")  // end of headers reached
            break;
          yield();
        } // while
        // Get body of answer
        if (httpStatus == 200 && client.connected()) {
          yield();
          String line = client.readStringUntil('\n');
          Serial.println(line);
        }
*/        
   
    client.stop();
  }
  else {
    Serial.println("connection failed");
  }
}

// helper function for reading analogue PWM value from the sensor
// formula taken from https://github.com/WifWaf/MH-Z19
int readPWM() {
  unsigned long th, tl, ppm_pwm = 0;
  do {
    th = pulseIn(PWM_PIN, HIGH, 1004000) / 1000;
    tl = 1004 - th;
    ppm_pwm = 5000 * (th - 2) / (th + tl - 4);
  } while (th == 0);
  Serial.print(F("\n # PPM PWM: "));
  Serial.println(ppm_pwm);
  return ppm_pwm;
}

// Init OTA (over-the-air-update)
void initOTA() {
  ArduinoOTA.setHostname("esp-co2ampel-OTA");
  ArduinoOTA.setPassword(OTAPASS);
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    });
    ArduinoOTA.onEnd([]() {
      Serial.println("\nEnd");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
  ArduinoOTA.begin();
}
