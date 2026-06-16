/**
 * vs-wrc058メガローバーで使用するwifi関連の定義
 * 
*/
#ifndef WRC058_WIFI_H
#define WRC058_WIFI_H

#include "vs_wrc058_memmap.h"
#include "vs_wrc058_motor.h"
#include "vs_wrc058_spi.h"
#include <Arduino.h>
#include <WiFi.h>
#include <FS.h>
#include <SPIFFS.h>

enum responses{
    INDEX       = 0,
    WIFI_WRITE  = 1,
    WIFI_READ   = 2,
    GET_BUTTON  = 3,
    WIFI_VIN    = 4,
    BAD_REQUEST = 5,
    NOT_FOUND   = 99
};

#define BUF_SIZE 10240
extern uint8_t buf[BUF_SIZE];

extern WiFiServer server;

void wifiInit(char* ssid, char* password);
void serverInit(char* ui_path);

int loadUI(char* ui_path);
int checkClient();

#endif /* WIFI_H */