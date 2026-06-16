/**
 * BLE接続のXboxコントローラに関連するライブラリ
 * 
 * 
 * 
 */

#ifndef WRC051_BLE_PAD_H
#define WRC051_BLE_PAD_H

//#include "vs_wrc058_memmap.h"
#include "esp_log.h"
#include "vs_wrc058_spi.h"
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <FS.h>
#include <SPIFFS.h>

extern const char* BLETAG;
extern bool doConnect;
extern bool connected;
extern uint32_t scanTime;

extern NimBLEAdvertisedDevice* advDevice;

void scanEndedCB(NimBLEScanResults results);

/**  None of these are required as they will be handled by the library with defaults. **
 **                       Remove as you see fit for your needs                        */
class ClientCallbacks : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* pClient) {
    //Serial.println("Connected");
    /** After connection we should change the parameters if we don't need fast response times.
     *  These settings are 150ms interval, 0 latency, 450ms timout.
     *  Timeout should be a multiple of the interval, minimum is 100ms.
     *  I find a multiple of 3-5 * the interval works best for quick response/reconnect.
     *  Min interval: 120 * 1.25ms = 150, Max interval: 120 * 1.25ms = 150, 0 latency, 60 * 10ms = 600ms timeout
     */
    //pClient->updateConnParams(120,120,0,60);
    //pClient->updateConnParams(6,6,0,3);
    connected = true;
  };

  void onDisconnect(NimBLEClient* pClient) {
    ///Serial.print(pClient->getPeerAddress().toString().c_str());
    //ESP_LOGI(BLETAG, pClient->getPeerAddress().toString().c_str());
    Serial.println(" Disconnected - Starting scan");
    NimBLEDevice::getScan()->start(scanTime, scanEndedCB);
    connected = false;

    pad_data.button = 0x0000;
    pad_data.right_stick.x = 0;
    pad_data.right_stick.y = 0;
    pad_data.left_stick.x = 0;
    pad_data.left_stick.y = 0;
    prev_pad_data = pad_data;

  };

  /** Called when the peripheral requests a change to the connection parameters.
   *  Return true to accept and apply them or false to reject and keep
   *  the currently used parameters. Default will return true.
   */
  bool onConnParamsUpdateRequest(NimBLEClient* pClient, const ble_gap_upd_params* params) {
    if(params->itvl_min < 6) { /** 1.25ms units */
      return false;
    } else if(params->itvl_max > 40) { /** 1.25ms units */
      return false;
    } else if(params->latency > 2) { /** Number of intervals allowed to skip */
      return false;
    } else if(params->supervision_timeout > 100) { /** 10ms units */
      return false;
    }

    return true;
  };

  /********************* Security handled here **********************
  ****** Note: these are the same return values as defaults ********/
  uint32_t onPassKeyRequest(){
    Serial.println("Client Passkey Request");
    /** return the passkey to send to the server */
    return 123456;
  };

  bool onConfirmPIN(uint32_t pass_key){
    Serial.print("The passkey YES/NO number: ");
    Serial.println(pass_key);
  /** Return false if passkeys don't match. */
    return true;
  };

  /** Pairing process complete, we can check the results in ble_gap_conn_desc */
  void onAuthenticationComplete(ble_gap_conn_desc* desc){
    if(!desc->sec_state.encrypted) {
      Serial.println("Encrypt connection failed - disconnecting");
      /** Find the client with the connection handle provided in desc */
      NimBLEDevice::getClientByID(desc->conn_handle)->disconnect();
      return;
    }
  };
};

/** Define a class to handle the callbacks when advertisments are received */
class AdvertisedDeviceCallbacks: public NimBLEAdvertisedDeviceCallbacks {

    void onResult(NimBLEAdvertisedDevice* advertisedDevice) {
        //Serial.print("Advertised Device found: ");
        //Serial.println(advertisedDevice->toString().c_str());
        //Serial.println(advertisedDevice->getAppearance());
        //Serial.println(advertisedDevice->getName().c_str());
        //Serial.println(advertisedDevice->getAddress().toString().c_str());
        
        if(advertisedDevice->getAppearance() == 964)
        {
            static uint32_t getTime = millis();
            //ESP_LOGI(BLETAG, advertisedDevice->toString().c_str());

            while(!SPIFFS.begin()){
                Serial.println("SPIFFS maunt failed");
                delay(10);
            }
            File padAddrTxt = SPIFFS.open("/padAddr.txt", "r");

            uint8_t padAddr[] = "44:16:22:96:ca:8c";
            if (!padAddrTxt) {
                ///Serial.println("padAddr.txt does not exist.");
                //ESP_LOGI(BLETAG, "padAddr.txt does not exist.");
                padAddrTxt.close();

            }else{
                padAddrTxt.read(padAddr,padAddrTxt.size());
                padAddrTxt.close();
            }
            NimBLEAddress targetDeviceAddress((const char*)padAddr);

            if(advertisedDevice->getAddress().equals(targetDeviceAddress)){
                Serial.println("Paired Controller!");
                /** stop scan before connecting */
                NimBLEDevice::getScan()->stop();
                /** Save the device reference in a global for the client to use*/
                advDevice = advertisedDevice;
                /** Ready to connect now */
                doConnect = true;

            }else{
                if(millis() - getTime > 14000){
                    getTime = millis();
                }

                if(millis() - getTime > 10000){
                    Serial.println("New Controller!");
                    padAddrTxt = SPIFFS.open("/padAddr.txt", FILE_WRITE);
                    padAddrTxt.print(advertisedDevice->getAddress().toString().c_str());
                    padAddrTxt.close();

                    /** stop scan before connecting */
                    NimBLEDevice::getScan()->stop();
                    /** Save the device reference in a global for the client to use*/
                    advDevice = advertisedDevice;
                    /** Ready to connect now */
                    doConnect = true;                
                
                }
            }
            SPIFFS.end();
        }
    };
};

void notifyCB(NimBLERemoteCharacteristic* pRemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify);
void charaPrintId(NimBLERemoteCharacteristic* pChara);
void printValue(std::__cxx11::string str);
void charaRead(NimBLERemoteCharacteristic* pChara);
void charaSubscribeNotification(NimBLERemoteCharacteristic* pChara);
bool afterConnect(NimBLEClient* pClient);
bool connectToServer();

void parseXboxPad(uint8_t* pData);
int updateXboxPad();


#endif /* BLE_PAD */