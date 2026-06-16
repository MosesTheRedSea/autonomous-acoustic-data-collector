#include "vs_wrc058_wifi.h"
#include "vs_wrc058_memmap.h"
#include "vs_wrc058_spi.h"
#include "vs_wrc058_motor.h"
#include "vs_wrc058_fwdsrover.h"
#include <Arduino.h>
#include <FS.h>
#include <SPIFFS.h>
#include <WiFi.h>

WiFiServer server(80);

uint8_t buf[BUF_SIZE];

WiFiClient client;

int responseCode = INDEX;
uint32_t connectTime = 0;

String wifiRead;
String wifiWrite;
uint8_t i2cAddr = 0x00;

String currentLine = "";

// WiFiネットワークに接続
void wifiInit(char* ssid, char* password){
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  int printDotTime = millis();
  while (WiFi.status() != WL_CONNECTED) {
    //if(updatePad()){
    //  chkPadInput();
    //}
    //pidControl();
    //wheel.sendWriteMap();

    if(millis() - printDotTime > 500){
      Serial.print(".");
      printDotTime = millis();
    }

    delay(10);
  }

  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

//SPIFFSからUIページをロード
int loadUI(char* ui_path){
  Serial.println(ui_path);
  if(!SPIFFS.begin()){
    Serial.println("SPIFFS maunt failed");
    return 0;
  }
  File ui = SPIFFS.open(ui_path, "r");
  if (!ui) {
    Serial.println("index.html does not exist.");
    return 0;
  }else if(ui.size() >= BUF_SIZE){
    Serial.print("Could not load file. Make file smaller.");
    return 0;
  }else{
    ui.read(buf,ui.size());
    ui.close();
    SPIFFS.end();
    return 1;
  }
}

// サーバー起動
void serverInit(char* ui_path){
  loadUI(ui_path);
  server.begin();
}

//I2Cアドレスの取得
uint8_t getI2CAddr(String currentLine){
  int i,j;
  uint8_t i2cAddr = 0xff;

  i = currentLine.indexOf("i2caddr=");

  if(i == -1){
    return 0x10;
  }

  j=8;
  if(isHexadecimalDigit(currentLine[i+j]) && isHexadecimalDigit(currentLine[i+j+1])){
        i2cAddr = ((cToHex(currentLine[i+j]) << 4) | (cToHex(currentLine[i+j+1])));
  }

  return i2cAddr;
}

//UIページの送信
void handleRoot(WiFiClient* client){
  // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
  // and a content-type so the client knows what's coming, then a blank line:
  client->println("HTTP/1.1 200 OK");
  client->println("Content-type:text/html");
  client->println();

  // the content of the HTTP response follows the header:
  client->print((char *)buf);

  // The HTTP response ends with another blank line:
  client->println();
}

void Vin(WiFiClient* client){
  if(wheel.u8Map(MU8_P_STTS)){
    return;
  }

  double voltage = 0.0;
  voltage = wheel.getVin();
  String res = String(voltage);
  client->println("HTTP/1.1 200 OK");
  client->println("Content-type:text/plain");
  client->println();
  client->print(res);
  client->println();
  Serial.println("Vin");
}

void Go(){
  setCtrlMode(MODE_SPD);
  wheel.s16Map(MS16_S_XS, 300);
  wheel.s16Map(MS16_S_ZS, 0);
  setO_EN(ON_ON, &wheel);
  Serial.println("Go"); 
}

void Left(){
  setCtrlMode(MODE_SPD);
  wheel.s16Map(MS16_S_XS,    0);
  wheel.s16Map(MS16_S_ZS, 1000);
  setO_EN(ON_ON, &wheel);
  Serial.println("Left");
}

void Right(){
  setCtrlMode(MODE_SPD);
  wheel.s16Map(MS16_S_XS,     0);
  wheel.s16Map(MS16_S_ZS, -1000);
  setO_EN(ON_ON, &wheel);
  Serial.println("Right");
}

void Back(){
  setCtrlMode(MODE_SPD);
  wheel.s16Map(MS16_S_XS, -300);
  wheel.s16Map(MS16_S_ZS, 0);
  setO_EN(ON_ON, &wheel);
  Serial.println("Back");
}

void Stop(){
  setCtrlMode(MODE_SPD);
  wheel.s16Map(MS16_S_XS, 0);
  wheel.s16Map(MS16_S_ZS, 0);
  setO_EN(ON_ON, &wheel);
  Serial.println("Stop");
}

void badRequest(WiFiClient* client){
  client->println("HTTP/1.1 400 BadRequest");
  client->println("Content-Length: 0");
  client->println();
  client->println();
  Serial.println("BadRequest");
}

void notFound(WiFiClient* client){
  client->println("HTTP/1.1 404 NotFound");
  client->println("Content-Length: 0");
  client->println();
  client->println();
  Serial.println("NotFound");
}

// クライアント接続時の処理
int checkClient(){
  
  int i, j;
  bool getCom = false;

  if(!client){
    client = server.available();
    responseCode = NOT_FOUND;
  }

  if (client) {
    connectTime = millis();                 // recode the time of connection from client                   
    while (client.connected()) {            // loop while the client's connected
      if(client.available()){               // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        //Serial.print(c);                    // print it out the serial monitor
        if(c=='\n'){                        // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if(currentLine.length() == 0){

            if(responseCode == WIFI_READ){
              if(i2cAddr == 0x10){
                wheel.checkMsg(wifiRead, &client);
                Serial.println("WIFI_READ");
              }else if(i2cAddr == 0x1F){
                //wrc022.checkMsg(wifiRead, &client);
              }else{
                badRequest(&client);
              }
              wifiRead = "";
              
            }else if(responseCode == INDEX || responseCode == GET_BUTTON){
              handleRoot(&client);
              Serial.println("index or get button");
            }else if(responseCode == WIFI_WRITE){
              if(i2cAddr == 0x10){
                wheel.checkMsg(wifiWrite, &client);
                client.println("HTTP/1.1 200 OK");
                client.println("Content-type:text/html");
                client.println();
                client.println();
                Serial.println("WIFI_WRITE");
              }else if(i2cAddr == 0x1F){
                steer.checkMsg(wifiWrite, &client);
                client.println("HTTP/1.1 200 OK");
                client.println("Content-type:text/html");
                client.println();
                client.println();
              }else{
                badRequest(&client);
              }
              wifiWrite = "";   
              
            }else if(responseCode == WIFI_VIN){
              Vin(&client);
            }else if(responseCode == NOT_FOUND){
              notFound(&client);
            }else{
              badRequest(&client);
            }
            currentLine = "";
            responseCode = NOT_FOUND;
            
            // break out of the while loop:
            break;
          }else{    // if you got a newline, then clear currentLine:
            //Serial.println(currentLine);
            currentLine = "";
          }
        }else if(c != '\r'){  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        if(responseCode == NOT_FOUND && currentLine.length() > 5){

          // Check to the client request :
          if(currentLine.endsWith("GET /vin")){
            responseCode = WIFI_VIN;
          }else if(currentLine.endsWith("GET / HTTP")){
            responseCode = INDEX;
          }else if(currentLine.endsWith("GET /stop")){
            responseCode = GET_BUTTON;
            Stop();
          }else if(currentLine.endsWith("GET /go")){
            responseCode = GET_BUTTON;
            Go();
          }else if(currentLine.endsWith("GET /left")){
            responseCode = GET_BUTTON;
            Left();
          }else if(currentLine.endsWith("GET /right")){
            responseCode = GET_BUTTON;
            Right();
          }else if(currentLine.endsWith("GET /back")){
            responseCode = GET_BUTTON;
            Back();
          }else if(currentLine.endsWith("GET /f1")){
            responseCode = GET_BUTTON;
            setCtrlMode(MODE_POS, &wheel);
            wheel.u8Map(MU8_TRIG, 0x10);
            wheel.s16Map(MS16_P_DIS, 1000);
            wheel.s16Map(MS16_P_RAD, 0);
            wheel.s16Map(MS16_P_ACC, 1000);
            wheel.s16Map(MS16_P_SPD, 250);
            Serial.printf("Go 1m");
          }else if(currentLine.endsWith("GET /f2")){
            responseCode = GET_BUTTON;
            setCtrlMode(MODE_POS, &wheel);
            wheel.u8Map(MU8_TRIG, 0x10);
            wheel.s16Map(MS16_P_DIS, -1000);
            wheel.s16Map(MS16_P_RAD, 0);
            wheel.s16Map(MS16_P_ACC, 1000);
            wheel.s16Map(MS16_P_SPD, 250);
            Serial.printf("Back 1m");
          }else if(currentLine.endsWith("GET /f3")){
            responseCode = GET_BUTTON;
            setCtrlMode(MODE_POS, &wheel);
            wheel.u8Map(MU8_TRIG, 0x10);
            wheel.s16Map(MS16_P_DIS, 0);
            wheel.s16Map(MS16_P_RAD, 1570);
            wheel.s16Map(MS16_P_ACC, 1000);
            wheel.s16Map(MS16_P_SPD, 250);
            Serial.printf("Left 90deg");
          }else if(currentLine.endsWith("GET /f4")){
            responseCode = GET_BUTTON;
            setCtrlMode(MODE_POS, &wheel);
            wheel.u8Map(MU8_TRIG, 0x10);
            wheel.s16Map(MS16_P_DIS, 0);
            wheel.s16Map(MS16_P_RAD, -1570);
            wheel.s16Map(MS16_P_ACC, 1000);
            wheel.s16Map(MS16_P_SPD, 250);
            Serial.printf("Right 90deg");
          }

          if(currentLine.startsWith("GET /write")){   //GET /writeで始まり
            if(currentLine.endsWith("HTTP")){         //HTTP/1.1で終わった場合は、メモリマップへのwrite命令                      
              if(currentLine.indexOf('?') == 10){     //10文字目'？'でなければフォーマットエラー
                responseCode = WIFI_WRITE;

                //write命令セット
                wifiWrite += "w ";

                //アドレスの取得
                i =  currentLine.indexOf("addr=");
                if(i == -1){
                  responseCode = BAD_REQUEST;
                }
              
                for(j = 5; j <= 6; j++){
                  if(isHexadecimalDigit(currentLine[i+j])){
                    wifiWrite += currentLine[i+j];
                  }else{
                    responseCode = BAD_REQUEST;
                  }
                }
                wifiWrite += ' ';

                //データセット
                i = currentLine.indexOf("data=");
                if(i == -1){
                  responseCode = BAD_REQUEST;
                }

                j = 5;
                while(isHexadecimalDigit(currentLine[i+j])){
                  wifiWrite += currentLine[i+j];
                  j++;
                }
                wifiWrite += '\n';

                //I2Cアドレスの取得
                i2cAddr = getI2CAddr(currentLine);
                if(i2cAddr == 0xff){
                  responseCode = BAD_REQUEST;
                }
              
              }else{
                responseCode = BAD_REQUEST;
              }
            }
          }

          if(currentLine.startsWith("GET /read")){   //GET /readで始まり
            if(currentLine.endsWith("HTTP")){         //HTTP/1.1で終わった場合は、メモリマップへのread命令
              if(currentLine.indexOf('?') == 9){     //9文字目'？'でなければフォーマットエラー
                responseCode = WIFI_READ;
                //write命令セット
                wifiRead += "r ";
                //アドレスの取得
                i =  currentLine.indexOf("addr=");
                if(i == -1){
                  responseCode = BAD_REQUEST;
                }
              
                for(j = 5; j <= 6; j++){
                  if(isHexadecimalDigit(currentLine[i+j])){
                    wifiRead += currentLine[i+j];
                  }else{
                    responseCode = BAD_REQUEST;
                  }
                }
                wifiRead += ' ';

                //読み込みバイト数取得
                i = currentLine.indexOf("length=");
                if(i == -1){
                  responseCode = BAD_REQUEST;
                }

                for(j = 7; j <= 8; j++){
                  if(isHexadecimalDigit(currentLine[i+j])){
                    wifiRead += currentLine[i+j];
                  }else{
                    responseCode = BAD_REQUEST;
                  }
                }
                wifiRead += '\n';

                //I2Cアドレスの取得
                i2cAddr = getI2CAddr(currentLine);
                if(i2cAddr == 0xff){
                  responseCode = BAD_REQUEST;
                }               
              
              }else{
                responseCode = BAD_REQUEST;
              }
            }
          }
        }
      }
    }
    // close the connection:
    client.stop();
  }else{
    client.stop();
  }

  return responseCode;
} 