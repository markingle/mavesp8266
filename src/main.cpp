/****************************************************************************
 *
 * Copyright (c) 2015, 2016 Gus Grubba. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file main.cpp
 * ESP8266 Wifi AP, MavLink UART/UDP Bridge
 *
 * @author Gus Grubba <mavlink@grubba.com>
 */

#include "mavesp8266.h"
#include "mavesp8266_parameters.h"
#include "mavesp8266_gcs.h"
#include "mavesp8266_vehicle.h"
#include "mavesp8266_httpd.h"
#include "mavesp8266_component.h"

#if !defined(ARDUINO_ESP32_DEV) && !defined(ARDUINO_ESP32S3_DEV) && !defined(ARDUINO_ESP32C3_DEV)
#include <ESP8266mDNS.h>
#else
/* ESP32 */
typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
#include <esp_event.h>
#include <esp_event.h>
#include <esp_wifi.h>
#include <esp_wifi_types.h>
#include <esp_err.h>
#include <mdns.h>
#endif

IPAddress local_IP(192, 168, 1, 117);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

const char* routerSSID = "Skybrush_2Ghz";
const char* routerPWD = "";

#define GPIO02  2

//---------------------------------------------------------------------------------
//-- HTTP Update Status
class MavESP8266UpdateImp : public MavESP8266Update {
public:
    MavESP8266UpdateImp ()
        : _isUpdating(false)
    {

    }
    void updateStarted  ()
    {
        _isUpdating = true;
    }
    void updateCompleted()
    {
        //-- TODO
    }
    void updateError    ()
    {
        //-- TODO
    }
    bool isUpdating     () { return _isUpdating; }
private:
    bool _isUpdating;
};



//-- Singletons
IPAddress               localIP;
MavESP8266Component     Component;
MavESP8266Parameters    Parameters;
MavESP8266GCS           GCS;
MavESP8266Vehicle       Vehicle;
MavESP8266Httpd         updateServer;
MavESP8266UpdateImp     updateStatus;
MavESP8266Log           Logger;

//---------------------------------------------------------------------------------
//-- Accessors
class MavESP8266WorldImp : public MavESP8266World {
public:
    MavESP8266Parameters*   getParameters   () { return &Parameters;    }
    MavESP8266Component*    getComponent    () { return &Component;     }
    MavESP8266Vehicle*      getVehicle      () { return &Vehicle;       }
    MavESP8266GCS*          getGCS          () { return &GCS;           }
    MavESP8266Log*          getLogger       () { return &Logger;        }
};

MavESP8266WorldImp      World;

MavESP8266World* getWorld()
{
    return &World;
}

//---------------------------------------------------------------------------------
//-- Wait for a DHCPD client
void wait_for_client() {
    DEBUG_LOG("Waiting for a client...\n");
    Serial.println("Waiting for a client....");
#ifdef ENABLE_DEBUG
    int wcount = 0;
#endif
#if defined(ARDUINO_ESP32_DEV) || defined(ARDUINO_ESP32S3_DEV) || defined(ARDUINO_ESP32C3_DEV)
    uint8 client_count = WiFi.softAPgetStationNum();
#else    
    uint8 client_count = wifi_softap_get_station_num();
#endif    
    while (!client_count) {
#ifdef ENABLE_DEBUG
        Serial1.print(".");
        if(++wcount > 80) {
            wcount = 0;
            Serial1.println();
        }
#endif
        delay(1000);
#if defined(ARDUINO_ESP32_DEV)|| defined(ARDUINO_ESP32S3_DEV) || defined(ARDUINO_ESP32C3_DEV)
	client_count = WiFi.softAPgetStationNum();
#else	
        client_count = wifi_softap_get_station_num();
#endif	
    }
    DEBUG_LOG("Got %d client(s)\n", client_count);
}

//---------------------------------------------------------------------------------
//-- Reset all parameters whenever the reset gpio pin is active
void reset_interrupt(){
    Parameters.resetToDefaults();
    Parameters.saveAllToEeprom();
#if defined(ARDUINO_ESP32_DEV)|| defined(ARDUINO_ESP32S3_DEV) || defined(ARDUINO_ESP32C3_DEV)
    ESP.restart();
#else    
    ESP.reset();
#endif    
}

//---------------------------------------------------------------------------------
//-- Set things up
void setup() {
    delay(1000);
    Parameters.begin();
    Serial.begin(115200);
#ifdef ENABLE_DEBUG
    //   We only use it for non debug because GPIO02 is used as a serial
    //   pin (TX) when debugging.
    //Serial1.begin(115200);
#else
    //-- Initialized GPIO02 (Used for "Reset To Factory")
    pinMode(GPIO02, INPUT_PULLUP);
    attachInterrupt(GPIO02, reset_interrupt, FALLING);
#endif
    Logger.begin(2048);

    DEBUG_LOG("\nConfiguring access point...\n");
    DEBUG_LOG("Free Sketch Space: %u\n", ESP.getFreeSketchSpace());
    Serial.println("Configuring access point/station");

    WiFi.disconnect(true);

    if(Parameters.getWifiMode() == WIFI_MODE_STA){
        //-- Connect to an existing network
#if defined(ARDUINO_ESP32_DEV) || defined(ARDUINO_ESP32S3_DEV) 
        WiFi.mode(WIFI_MODE_STA);  
        Serial.println("Station Mode 1");
        //WiFi.config(local_IP, gateway, subnet, 0U, 0U);
        //WiFi.begin(routerSSID, routerPWD); 
        WiFi.config(Parameters.getWifiStaIP(), Parameters.getWifiStaGateway(), Parameters.getWifiStaSubnet(), 0U, 0U);
        WiFi.begin(Parameters.getWifiStaSsid(), Parameters.getWifiStaPassword());    
#else            
        WiFi.mode(WIFI_STA);
        Serial.println("Station Mode 2");
        WiFi.config(Parameters.getWifiStaIP(), Parameters.getWifiStaGateway(), Parameters.getWifiStaSubnet(), 0U, 0U);
        WiFi.begin(Parameters.getWifiStaSsid(), Parameters.getWifiStaPassword()); 
#endif
#if defined(ARDUINO_ESP32S3_DEV) || defined(ARDUINO_ESP32C3_DEV)
        WiFi.config(Parameters.getWifiStaIP(), Parameters.getWifiStaGateway(), Parameters.getWifiStaSubnet(), IPAddress((uint32_t)0), IPAddress((uint32_t)0));
#else
        WiFi.config(Parameters.getWifiStaIP(), Parameters.getWifiStaGateway(), Parameters.getWifiStaSubnet(), 0U, 0U);
#endif	
        WiFi.begin(Parameters.getWifiStaSsid(), Parameters.getWifiStaPassword());

        //-- Wait a minute to connect
        for(int i = 0; i < 120 && WiFi.status() != WL_CONNECTED; i++) {
            //#ifdef ENABLE_DEBUG
            Serial.print(".");
            //#endif
            delay(500);
        }
        if(WiFi.status() == WL_CONNECTED) {
            localIP = WiFi.localIP();
            WiFi.setAutoReconnect(true);
        } else {
            //-- Fall back to AP mode if no connection could be established
            WiFi.disconnect(true);
            Parameters.setWifiMode(WIFI_MODE_AP);
        }
    }

    if(Parameters.getWifiMode() == WIFI_MODE_AP){
        //-- Start AP
#if defined(ARDUINO_ESP32_DEV) || defined(ARDUINO_ESP32S3_DEV) || defined(ARDUINO_ESP32C3_DEV)
	WiFi.mode(WIFI_MODE_AP);      /* Default to WPA2 */
    Serial.println("AP Mode 1");
#else      
        WiFi.mode(WIFI_AP);
        Serial.println("AP Mode 2");
        //WiFi.encryptionType(AUTH_WPA2_PSK);	
#endif
        WiFi.softAP(Parameters.getWifiSsid(), Parameters.getWifiPassword(), Parameters.getWifiChannel());
        localIP = WiFi.softAPIP();
        Serial.print("IP Address ");
        Serial.println(localIP);
        wait_for_client();
    }
    //-- Boost power to Max
#if defined(ARDUINO_ESP32_DEV) || defined(ARDUINO_ESP32S3_DEV) || defined(ARDUINO_ESP32C3_DEV)
    {
         int8_t power;
	 esp_wifi_get_max_tx_power(&power);
	 esp_wifi_set_max_tx_power(power);
    }
#else
    WiFi.setOutputPower(20.5);
#endif    
    //-- MDNS
    char mdsnName[256];
#if defined(ARDUINO_ESP32_DEV) || defined(ARDUINO_ESP32S3_DEV) || defined(ARDUINO_ESP32C3_DEV)
    sprintf(mdsnName, "MavEsp32-%d",localIP[3]);
    mdns_init();
    mdns_service_add(NULL, "http", "tcp", 80, NULL, 0);
#else
    sprintf(mdsnName, "MavEsp8266-%d",localIP[3]);    
    MDNS.begin(mdsnName);
    MDNS.addService("http", "tcp", 80);
#endif    
    //-- Initialize Comm Links
    DEBUG_LOG("Start WiFi Bridge\n");
    Serial.println("Start Wifi Bridge");
    DEBUG_LOG("Local IP: %s\n", localIP.toString().c_str());
    Serial.print("Local IP: ");
    Serial.println(localIP);

    Parameters.setLocalIPAddress(localIP);
    IPAddress gcs_ip(localIP);
    //-- I'm getting bogus IP from the DHCP server. Broadcasting for now.
    gcs_ip[3] = 255;
    //Serial.println(" ");
    Serial.println("---- Starting GCS ----");
    Serial.print("GCS IP : ");
    Serial.println(gcs_ip);
    GCS.begin(&Vehicle, gcs_ip);
    Serial.println("---- Starting Vehicle----");
    Vehicle.begin(&GCS);
    //-- Initialize Update Server
    updateServer.begin(&updateStatus);
}

//---------------------------------------------------------------------------------
//-- Main Loop
void loop() {
    if(!updateStatus.isUpdating()) {
        //Serial.println("Is Updating...");
        //delay(1000);
        if (Component.inRawMode()) {
            GCS.readMessageRaw();
            //Serial.println("Read CGS Msg Raw");
            delay(0);
            Vehicle.readMessageRaw();
            //Serial.println("Read Vehicle Msg Raw");

        } else {
            //Serial.println("--------- GCS --------");
            GCS.readMessage();
            //Serial.println("Read GCS Msg");
            //Serial.println(" ");
            delay(0);
            //Serial.println("------ Vehicle -------");
            Vehicle.readMessage();
            //Serial.println("Read Vehicle Msg");
            //Serial.println(" ");
        }
    }
    updateServer.checkUpdates();
    //Serial.println("Check Updates....");
}
