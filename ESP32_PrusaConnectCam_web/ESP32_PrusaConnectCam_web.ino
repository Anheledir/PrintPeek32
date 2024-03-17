/*
   This code is adapted for the ESP32-CAM board from Ai Thinker.
   Thanks to Miroslav Pivovarsky (miroslav.pivovarsky@gmail.com) for his initial work.

   It's neccesary to install support for ESP32 board to the Arduino IDE. In the board manager we need add next link
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   Then we can install "ESP32 by Espressif Systems" board in the board manager.

   Tools -> Board -> ESP32 -> AI Thinker ESP32-CAM

   Project: ESP32 PrusaConnect Camera
   Author: Gordon Breuer, Andreas Rothmann 
   e-mail: gordon@outlook.de, andreas.rothmann@web.de
   Version 1.0

*/

/* includes */
#include <WiFi.h>
#include "Arduino.h"
#include <esp_task_wdt.h>
#include <ESPmDNS.h>

#include "server.h"
#include "cfg.h"
#include "var.h"
#include "mcu_cfg.h"

void setup() {
  /* Serial port for debugging purposes */
  Serial.begin(SERIAL_PORT_SPEER);
  Serial.println("Start MCU!");
  Serial.print("SW Version: ");
  Serial.println(SW_VERSION);

  /* read configuration from flash */
  WifiMacAddr = WiFi.macAddress();
  Cfg_Init();

  /* If WiFi credentials are empty, start AP mode immediately */
  if (sWiFiSsid == "" || sWiFiPsw == "") {
    startAPMode();
    return;
  }
  
  /* Connect to Wi-Fi */
  WiFi.begin(sWiFiSsid, sWiFiPsw);
  Serial.println("Connecting to WiFi");
  int connectionAttempts = 0;

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
    connectionAttempts++;

    if (connectionAttempts >= 5) {
      startAPMode();
      return;
    }
  }
  
  Serial.println();
  Serial.print("Signal Strength (RSSI): ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");

  /* Print ESP32 Local IP Address */
  Serial.print("IP Address: http://");
  Serial.println(WiFi.localIP());

  /* Init MDNS record */
  Serial.println(F("----------------------------------------------------------------"));
  Serial.print("Starting mDNS record: http://");
  Serial.print(MDNS_RECORD_HOST);
  Serial.println(".local");
  if (!MDNS.begin(MDNS_RECORD_HOST)) {
    Serial.println("Error starting mDNS");
  } else {
    Serial.println("Starting mDNS OK");
  }

  /* init camera interface */
  Camera_InitCamera();
  Camera_CapturePhoto();

  /* GPIO configuration */
  pinMode(FLASH_GPIO_NUM, OUTPUT);
  /* On any (re)start the LED should be off
  digitalWrite(FLASH_GPIO_NUM, low);

  /* init WEB server */
  Server_InitWebServer();

  /* init watchdog */
   (WDT_TIMEOUT, true); /* enable panic so ESP32 restarts */
  esp_task_wdt_add(NULL);               /* add current thread to WDT watch */
  esp_task_wdt_reset();                 /* reset wdg */

  Serial.println("MCU configuration done!");
}

void startAPMode() {
    WiFi.softAP("PrintPeek32", NULL); // No password

    Serial.println("Started AP Mode");
    Serial.print("AP IP Address: ");
    Serial.println(WiFi.softAPIP());

    Server_InitApServer();
}

void loop() {
    Serial.println("----------------------------------------------------------------");

    /* check wifi reconnecting */
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Reconnecting to WiFi...");
        WiFi.disconnect();
        WiFi.reconnect();
    } else if (WiFi.status() == WL_CONNECTED) {
        char cstr[150];
        sprintf(cstr, "Wifi connected. SSID: %s, RSSI: %d dBm, IP: %s \n", WiFi.SSID().c_str(), WiFi.RSSI(), WiFi.localIP().toString().c_str());
        Serial.printf(cstr);
    }

    /* take photo */
    if (Camera_CapturePhoto()) {
        /* send photo to backend */
        Server_SendPhotoToPrusaBackend();
    }


    /* wait for the next interval to take a photo */
    for (uint32_t i = 0; i < RefreshInterval; i++) {
        /* reset watchdog */
        esp_task_wdt_reset();
        delay(1000);
    }
}

/* EOF */