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

WiFiMulti wifiMulti;

//how many clients should be able to telnet to this ESP32
#define MAX_SRV_CLIENTS 1
const char *ssid = "motorola edge 40_4386";
const char *password = "motowifi";

const int LED = 0;
const int touchUP = 1;
int threshold = 60000;
volatile bool switchState = false; //to store in ram instaead of registers?


WiFiServer server(23);
WiFiClient serverClients[MAX_SRV_CLIENTS];

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
volatile bool touchTriggered=false;
void touchUpISR() {
      Serial.println("ISR Triggered!");

  if(touchRead(touchUP)>threshold)
    digitalWrite(LED,LOW);
  else
    digitalWrite(LED,HIGH);
  
}

void setup() {
  Serial.begin(115200);
    // Configure GPIO pins
    pinMode(usbPin, INPUT);
    pinMode(batteryPin, INPUT);
    pinMode(LED, OUTPUT);
    Serial.print("Initial Touch Value: ");
    Serial.println(touchRead(touchUP));
  
    touchAttachInterrupt(touchUP, touchUpISR, threshold);

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


}

void loop() {
  // display.clearDisplay();
  // display.display();
  // Serial.print("Touch Value: ");
  // Serial.println(touchRead(touchUP));
  // Serial.println(touchRead(touchUP));

  // if (touchTriggered) {
  //   touchTriggered = false; // Reset flag
  //   Serial.println("ISR Triggered!");
  // }
  uint8_t i;
  // Serial.println("touch : "); // Debugging message
  // Serial.println(touchTriggered); // Read and print the touch sensor value

  // if (touchTriggered) {  
  //   touchTriggered = false; // Reset flag  
  //   Serial.println("ISR Executed!"); // Debugging message
  // }

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
    delay(5000);
  }
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
