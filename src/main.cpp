/*
  WiFiTelnetToSerial - Example Transparent UART to Telnet Server for ESP32

  Copyright (c) 2017 Hristo Gochkov. All rights reserved.
  This file is part of the ESP32 WiFi library for Arduino environment.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#include <Arduino.h>

#include <WiFi.h>
#include <WiFiMulti.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>     // MQTT client library
#include <esp_wpa2.h>         // WPA2 library for enterprise networks like Eduroam
#include "arduino_secrets.h"

WiFiMulti wifiMulti;
// Load settings, TODO see arduino_secrets.h.template
const String wifiSSID        = SECRET_WIFI_SSID;
const char wifiPassword[]    = SECRET_WIFI_PASSWORD;
const char wifiAnonymousId[] = SECRET_WIFI_ANONYMOUSID;
const char wifiEduroamId[]   = SECRET_WIFI_EDUROAMID;
const char   mqttBroker[]    = SECRET_MQTT_BROKER;
const  int   mqttPort        = SECRET_MQTT_PORT;
const char   mqttUser[]      = SECRET_MQTT_USER;
const char   mqttPassword[]  = SECRET_MQTT_PASSWORD;
const String mqttPrefix      = SECRET_MQTT_PREFIX;
const int    mqttYear        = SECRET_MQTT_YEAR;
const byte   mqttBoardId     = SECRET_MQTT_BOARDID;

// Board-specific MQTT topics to be generated
String mqttPublish;   // Topic telemetry
String mqttSubscribe; // Topic telecommand

// Encryption using https://letsencrypt.org/certs/lets-encrypt-r3.pem
const char tlsPublicCertificate[] = ("\
-----BEGIN CERTIFICATE-----\n\
MIIFBjCCAu6gAwIBAgIRAIp9PhPWLzDvI4a9KQdrNPgwDQYJKoZIhvcNAQELBQAw\n\
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh\n\
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMjQwMzEzMDAwMDAw\n\
WhcNMjcwMzEyMjM1OTU5WjAzMQswCQYDVQQGEwJVUzEWMBQGA1UEChMNTGV0J3Mg\n\
RW5jcnlwdDEMMAoGA1UEAxMDUjExMIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIB\n\
CgKCAQEAuoe8XBsAOcvKCs3UZxD5ATylTqVhyybKUvsVAbe5KPUoHu0nsyQYOWcJ\n\
DAjs4DqwO3cOvfPlOVRBDE6uQdaZdN5R2+97/1i9qLcT9t4x1fJyyXJqC4N0lZxG\n\
AGQUmfOx2SLZzaiSqhwmej/+71gFewiVgdtxD4774zEJuwm+UE1fj5F2PVqdnoPy\n\
6cRms+EGZkNIGIBloDcYmpuEMpexsr3E+BUAnSeI++JjF5ZsmydnS8TbKF5pwnnw\n\
SVzgJFDhxLyhBax7QG0AtMJBP6dYuC/FXJuluwme8f7rsIU5/agK70XEeOtlKsLP\n\
Xzze41xNG/cLJyuqC0J3U095ah2H2QIDAQABo4H4MIH1MA4GA1UdDwEB/wQEAwIB\n\
hjAdBgNVHSUEFjAUBggrBgEFBQcDAgYIKwYBBQUHAwEwEgYDVR0TAQH/BAgwBgEB\n\
/wIBADAdBgNVHQ4EFgQUxc9GpOr0w8B6bJXELbBeki8m47kwHwYDVR0jBBgwFoAU\n\
ebRZ5nu25eQBc4AIiMgaWPbpm24wMgYIKwYBBQUHAQEEJjAkMCIGCCsGAQUFBzAC\n\
hhZodHRwOi8veDEuaS5sZW5jci5vcmcvMBMGA1UdIAQMMAowCAYGZ4EMAQIBMCcG\n\
A1UdHwQgMB4wHKAaoBiGFmh0dHA6Ly94MS5jLmxlbmNyLm9yZy8wDQYJKoZIhvcN\n\
AQELBQADggIBAE7iiV0KAxyQOND1H/lxXPjDj7I3iHpvsCUf7b632IYGjukJhM1y\n\
v4Hz/MrPU0jtvfZpQtSlET41yBOykh0FX+ou1Nj4ScOt9ZmWnO8m2OG0JAtIIE38\n\
01S0qcYhyOE2G/93ZCkXufBL713qzXnQv5C/viOykNpKqUgxdKlEC+Hi9i2DcaR1\n\
e9KUwQUZRhy5j/PEdEglKg3l9dtD4tuTm7kZtB8v32oOjzHTYw+7KdzdZiw/sBtn\n\
UfhBPORNuay4pJxmY/WrhSMdzFO2q3Gu3MUBcdo27goYKjL9CTF8j/Zz55yctUoV\n\
aneCWs/ajUX+HypkBTA+c8LGDLnWO2NKq0YD/pnARkAnYGPfUDoHR9gVSp/qRx+Z\n\
WghiDLZsMwhN1zjtSC0uBWiugF3vTNzYIEFfaPG7Ws3jDrAMMYebQ95JQ+HIBD/R\n\
PBuHRTBpqKlyDnkSHDHYPiNX3adPoPAcgdF3H2/W0rmoswMWgTlLn1Wu0mrks7/q\n\
pdWfS6PJ1jty80r2VKsM/Dj3YIDfbjXKdaFU5C+8bhfJGqU3taKauuz0wHVGT3eo\n\
6FlWkWYtbt4pgdamlwVeZEW+LM7qZEJEsMNPrfC03APKmZsJgpWCDWOKZvkZcvjV\n\
uYkQ4omYCTX5ohy+knMjdOmdH9c7SpqEWBDC86fiNex+O0XOMEZSa8DA\n\
-----END CERTIFICATE-----\n");

//how many clients should be able to telnet to this ESP32
#define MAX_SRV_CLIENTS 1
const char *ssid = "motorola edge 40_4386";
const char *password = "motowifi";

const int LED = 0;
const int touchUP = 1;
int threshold = 40000;
volatile bool switchState = false; //to store in ram instaead of registers?
unsigned long lastTouchTime = 0; // Stores last touch event time
const int debounceDelay = 500; // Debounce time in milliseconds

volatile bool tcReceived = false; // Set to true when a telecommand is received
volatile int newMode = -1;       // Shared variable to store the new mode from MQTT or touch

// PWM properties
const int freq = 2000;     // Frequency in Hz (Adjust for different tones)
const int pwmChannel = 0;  // PWM channel (0-15 for ESP32)
const int resolution = 8;  // 8-bit resolution (0-255 duty cycle)


const int buzzer = 14;
WiFiServer server(23);
WiFiClient serverClients[MAX_SRV_CLIENTS];
WiFiClientSecure wifiClient;
PubSubClient client(wifiClient);
// OLED display dimensions
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// OLED I2C settings
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
// GPIO pin definitions
const byte usbPin = 3;      // GPIO 3 for USB voltage
const byte batteryPin = 7;  // GPIO 7 for battery voltage
// Voltage divider scale factors (adjustable via telecommands)
float usbDividerScaleFactor = (470.0 + 220.0) / 470.0; // Default based on 470kΩ and 220kΩ resistors
float batteryDividerScaleFactor = (150.0 + 470.0) / 150.0; // Default based on 150kΩ and 470kΩ resistors

// ADC attenuation scale factors (adjustable via telecommands)
float usbAdcScaleFactor = 3.1 / 4095.0;  // Default for 11dB attenuation
float batteryAdcScaleFactor = 1.25 / 4095.0; // Default for 2.5dB attenuation
const float bat_threshold = 3;
// Function prototypes
void initializeDisplay();
void displayText(const char* text1,int,int);
float readAndPrintVoltages();
volatile int mode=0;
char buffer[20];  // Buffer to hold the formatted string
// Interrupt Service Routine (ISR) for touch sensing
void IRAM_ATTR touchISR() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastTouchTime > debounceDelay) { 
    switchState = true; // Set flag to process touch in the main loop
    lastTouchTime = currentMillis;
  }
}
void wifiConnect() {
  delay(10);
  Serial.print("\nAttempting Wi-Fi connection to ");
  Serial.println(wifiSSID.c_str());
  WiFi.begin(wifiSSID.c_str(), wifiPassword);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  randomSeed(micros());
  Serial.print(" done using IP address ");
  Serial.println(WiFi.localIP());
}

/*
 * Initialize Wi-Fi connection for eduroam
 */
void wifiConnectEduroam() {
  delay(10);
  Serial.print("\nAttempting Wi-Fi connection to ");
  Serial.println(wifiSSID);
  WiFi.disconnect(true);  //disconnect form wifi to set new wifi connection
  WiFi.mode(WIFI_STA); //init wifi mode
  esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)wifiAnonymousId, strlen(wifiAnonymousId)); //provide identity
  esp_wifi_sta_wpa2_ent_set_username((uint8_t *)wifiEduroamId, strlen(wifiEduroamId)); //provide username
  esp_wifi_sta_wpa2_ent_set_password((uint8_t *)wifiPassword, strlen(wifiPassword)); //provide password
  esp_wifi_sta_wpa2_ent_enable(); 
  WiFi.begin(wifiSSID.c_str());
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  randomSeed(micros());
  Serial.print(" done using IP address ");
  Serial.println(WiFi.localIP());
}

/*
 * Callback function for MQTT subscription handler
 */
void mqttCallback(char* topic, byte *payload, unsigned int length) {
  Serial.println("Info: Receiving message from ground...");
  Serial.print("      COM CHANNEL: ");
  Serial.println(topic);
  Serial.print("      UPLINK DATA: ");
 // Convert payload to a string
 String message;
 for (int i = 0; i < length; i++) {
   message += (char)payload[i];
 }
 Serial.println(message);

 // Check if the message is a valid number between 0 and 6
 if (message.length() == 1 && message[0] >= '0' && message[0] <= '6') {
   newMode = message.toInt(); // Update the shared variable
   tcReceived = true;         // Set flag to indicate telecommand received
   Serial.print("New mode received via telecommand: ");
   Serial.println(newMode);
 } else {
   Serial.println("Invalid mode command. Expected a number between 0 and 6.");
 }
}

/*
 * Initialize MQTT connection
 */
void mqttConnect() {
  // wifiClient.setInsecure(); // Disable SSL verification (not recommended for production)

  wifiClient.setCACert(tlsPublicCertificate);
  //wifiClient.setCertificate(tlsClientCertificate);
  //wifiClient.setPrivateKey(tlsClientPrivateKey);

  // Generate board-specific MQTT client ID
  String mqttClientId = mqttPrefix + "-" + String(mqttBoardId);

  // Loop until (re)connected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection to ");
    Serial.println(mqttBroker);
    Serial.print("... ");
    // Attempt to connect
    if (client.connect(mqttClientId.c_str(), mqttUser, mqttPassword)) {
      Serial.print("done using client ID ");
      Serial.println(mqttClientId);
      client.subscribe(mqttSubscribe.c_str());
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      delay(5000);
    }
  }
}
void setup() {
  Serial.begin(115200);
    // Configure GPIO pins
    pinMode(usbPin, INPUT);
    pinMode(batteryPin, INPUT);
    pinMode(LED, OUTPUT);
    // pinMode(buzzer,OUTPUT);
       // Configure the LEDC (PWM) channel
       ledcSetup(pwmChannel, freq, resolution);
    
       // Attach the channel to the GPIO
       ledcAttachPin(buzzer, pwmChannel);
    Serial.print("Initial Touch Value: ");
    Serial.println(touchRead(touchUP));
  
    touchAttachInterrupt(touchUP, touchISR, threshold);

  // Set ADC attenuation for each pin
  analogSetPinAttenuation(usbPin, ADC_11db);       // 11dB for USB (0V - 3.1V)
  analogSetPinAttenuation(batteryPin, ADC_2_5db);  // 2.5dB for Battery (0V - 1.2V)
  
  initializeDisplay();
  float bat_voltage = readAndPrintVoltages();
  if(bat_voltage<bat_threshold)
  displayText("Low Battery",0,0);
  else
  displayText("Battery : OK",0,0);
  delay(5000);

  // display.clearDisplay();
// display.display();
  Serial.println("\nConnecting");
  displayText(" Connecting......",0,8);

  wifiMulti.addAP(ssid, password);
  // wifiMulti.addAP("ssid_from_AP_2", "your_password_for_AP_2");
  // wifiMulti.addAP("ssid_from_AP_3", "your_password_for_AP_3");

  Serial.println("Connecting Wifi ");
 
  for (int loops = 10; loops > 0; loops--) {
    if (wifiMulti.run() == WL_CONNECTED) {
      Serial.println("");
      Serial.print("WiFi connected ");
      displayText("Wifi Connected",0,15);
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
      break;
    } else {
      Serial.println(loops);
      displayText("Wifi Connection Failed",0,15);
      delay(1000);
    }
  }
  if (wifiMulti.run() != WL_CONNECTED) {
    Serial.println("WiFi connect failed");
    displayText("Wifi Connection Failed",0,15);

    delay(1000);
    ESP.restart();
  }

  //start UART and the server
  Serial1.begin(9600);
  server.begin();
  server.setNoDelay(true);

  Serial.print("Ready! Use 'telnet ");
  Serial.print(WiFi.localIP());
  Serial.println(" 23' to connect");
  // Generate board-specific MQTT topics using the pattern [prefix]/[year]/[ID]/[variable]
  String mqttTopic = mqttPrefix + "/" + String(mqttYear) + "/" + String(mqttBoardId) + "/";
  mqttPublish   = mqttTopic + "tm"; // Topic telemetry
  mqttSubscribe = mqttTopic + "tc"; // Topic telecommand
  
  delay(10000);
  if(wifiSSID.compareTo(String("eduroam")) == 0){
    // wifiSSID equals eduroam
    wifiConnectEduroam(); // for eduroam
  } else {
    wifiConnect();        // for "normal" Wi-Fi (e.g. Hotspot, at home)
  }
  client.setServer(mqttBroker, mqttPort);
  client.setCallback(mqttCallback);
  mqttConnect();

}

void loop() {
  display.clearDisplay();
  
  Serial.print("Touch Value: ");
  Serial.println(touchRead(touchUP));
  
  Serial.print("Switch State: ");
  Serial.println(switchState);
  
  Serial.print("Mode : ");
  Serial.println(mode);

  if (switchState or tcReceived) { 
    if(switchState){
    if (mode < 6) mode++;
    else mode = 0;

    switchState = false; // Reset flag after processing
  }
  if(tcReceived){
    mode = newMode;
    tcReceived=false;
  }
    Serial.print("New Mode: ");
    Serial.println(mode);

    // Beep the buzzer as many times as the mode number
    for (int i = 0; i < mode; i++) {
        ledcWrite(pwmChannel, 127); // Start buzzer
        delay(200); // Beep duration
        ledcWrite(pwmChannel, 0);   // Stop buzzer
        delay(200); // Pause between beeps
    }
}
  // digitalWrite(buzzer,LOW);
      //  ledcWrite(pwmChannel, 0);

  // Update OLED Display
  sprintf(buffer, "Mode: %d", mode);
  displayText(buffer, 0, 25);

  // Toggle LED
  digitalWrite(LED, switchState ? HIGH : LOW);
  
  delay(500); // Small delay to stabilize readings
  uint8_t i;
 
  if (wifiMulti.run() == WL_CONNECTED) {
    // displayText("Wifi Connected");

    //check if there are any new clients
    if (server.hasClient()) {
      for (i = 0; i < MAX_SRV_CLIENTS; i++) {
        //find free/disconnected spot
        if (!serverClients[i] || !serverClients[i].connected()) {
          if (serverClients[i]) {
            serverClients[i].stop();
          }
          serverClients[i] = server.accept();
          if (!serverClients[i]) {
            Serial.println("available broken");
          }
          Serial.print("New client: ");
          Serial.print(i);
          Serial.print(' ');
          Serial.println(serverClients[i].remoteIP());
          break;
        }
      }
      if (i >= MAX_SRV_CLIENTS) {
        //no free/disconnected spot so reject
        server.accept().stop();
      }
    }
    //check clients for data
    for (i = 0; i < MAX_SRV_CLIENTS; i++) {
      if (serverClients[i] && serverClients[i].connected()) {
        if (serverClients[i].available()) {
          //get data from the telnet client and push it to the UART
          while (serverClients[i].available()) {
            Serial1.write(serverClients[i].read());
          }
        }
      } else {
        if (serverClients[i]) {
          serverClients[i].stop();
        }
      }
    }
    //check UART for data
    if (Serial1.available()) {
      size_t len = Serial1.available();
      uint8_t sbuf[len];
      Serial1.readBytes(sbuf, len);
      //push UART data to all connected telnet clients
      for (i = 0; i < MAX_SRV_CLIENTS; i++) {
        if (serverClients[i] && serverClients[i].connected()) {
          serverClients[i].write(sbuf, len);
          delay(1);
        }
      }
    }
          // displayText("Wifi Connected");

  } else {
    Serial.println("WiFi not connected!");
    // displayText("Wifi Connection failed");

    for (i = 0; i < MAX_SRV_CLIENTS; i++) {
      if (serverClients[i]) {
        serverClients[i].stop();
      }
    }
    // delay(5000);
  }
  client.loop();
  if (!client.connected()) {
    mqttConnect();
  }
  Serial.print("Info: Transmitting status via channel ");
  Serial.println(mqttPublish);
  String payload = "Berlin, Cyber Base here. The CADSE no. " + String(mqttBoardId) + " has landed!";
  client.publish(mqttPublish.c_str(), payload.c_str());
  delay(1000);
}

/**
 * Initialize the OLED display.
 */
void initializeDisplay() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Infinite loop if initialization fails
  }
  display.clearDisplay();
    // display.setCursor(0, 0);
    display.display();

}

/**
 * Display text on the OLED screen.
 * text1 : First line of text
 * text2 : Second line of text
 */
void displayText(const char* text1,int posx,int posy) {
  // display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(posx, posy);
  display.println(text1);
  display.display();
}

/**
 * Read ADC values, calculate voltages, and display them on the OLED screen.
 */
float readAndPrintVoltages() {
  // Read raw ADC values
  int usbValue = analogRead(usbPin);
  int batteryValue = analogRead(batteryPin);
 
  // // Calculate voltages
  float usbVoltage = usbValue * usbAdcScaleFactor;
  float batteryVoltage = batteryValue * batteryAdcScaleFactor;

  // Calculate actual voltages
  float actualUsbVoltage = usbVoltage * usbDividerScaleFactor;
  float actualBatteryVoltage = batteryVoltage * batteryDividerScaleFactor;

  // Print values to Serial Monitor

  // Serial.printf("Actual USB Voltage: %.6f V\n", actualUsbVoltage);
  // Serial.printf("Actual Battery Voltage: %.6f V\n", actualBatteryVoltage);
  return actualBatteryVoltage;
  // Display voltages on OLED
  // char usbDisplay[16];
  // char batteryDisplay[16];
  // snprintf(usbDisplay, sizeof(usbDisplay), "USB: %.2fV", actualUsbVoltage);
  // snprintf(batteryDisplay, sizeof(batteryDisplay), "BAT: %.2fV", actualBatteryVoltage);
  // displayText(usbDisplay, batteryDisplay);
}
