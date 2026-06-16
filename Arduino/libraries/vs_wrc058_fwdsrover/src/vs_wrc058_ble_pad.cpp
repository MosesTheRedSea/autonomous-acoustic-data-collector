#include "vs_wrc058_ble_pad.h"

const char* BLETAG = "BLE";

NimBLEUUID uuidServiceUnknown("00000001-5f60-4c4f-9c83-a7953298d40d");
NimBLEUUID uuidServiceGeneral("1801");
NimBLEUUID uuidServiceBattery("180f");
NimBLEUUID uuidServiceHid("1812");
NimBLEUUID uuidCharaReport("2a4d");
NimBLEUUID uuidCharaPnp("2a50");
NimBLEUUID uuidCharaHidInformation("2a4a");
NimBLEUUID uuidCharaPeripheralAppearance("2a01");
NimBLEUUID uuidCharaPeripheralControlParameters("2a04");

NimBLEAdvertisedDevice* advDevice;

bool doConnect = false;
bool connected = false;
uint32_t scanTime = 0; /** 0 = scan forever */

/** Notification / Indication receiving handler callback */
void notifyCB(NimBLERemoteCharacteristic* pRemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify){

    parseXboxPad(pData);

}



/** Callback to process the results of the last scan or restart it */
void scanEndedCB(NimBLEScanResults results){
    ///Serial.println("Scan Ended");
    ESP_LOGI(BLETAG, "Scan Ended");
}

void charaPrintId(NimBLERemoteCharacteristic* pChara){
  
  Serial.printf("s:%s c:%s h:%d",
                pChara->getRemoteService()->getUUID().toString().c_str(),
                pChara->getUUID().toString().c_str(), pChara->getHandle());
  
}

void printValue(std::__cxx11::string str){
  Serial.printf("str: %s\n", str.c_str());
  Serial.printf("hex:");
  for (auto v : str) {
    Serial.printf(" %02x", v);
  }
  Serial.println("");
}

void charaRead(NimBLERemoteCharacteristic* pChara){
  if (pChara->canRead()) {
    //charaPrintId(pChara);
    ///Serial.println(" canRead");
    ESP_LOGI(BLETAG, " canRead");
    auto str = pChara->readValue();
    if (str.size() == 0) {
      str = pChara->readValue();
    }
    //printValue(str);
  }
}

void charaSubscribeNotification(NimBLERemoteCharacteristic* pChara){
  if (pChara->canNotify()) {
    //charaPrintId(pChara);
    ///Serial.println(" canNotify ");
    ESP_LOGI(BLETAG, " canNotify ");
    if (pChara->subscribe(true, notifyCB, true)) {
      ///Serial.println("set notifyCb");
      ESP_LOGI(BLETAG, "set notifyCb");
      // return true;
    } else {
      ///Serial.println("failed to subscribe");
      ESP_LOGE(BLETAG, "failed to subscribe");
    }
  }
}

bool afterConnect(NimBLEClient* pClient){
  for (auto pService : *pClient->getServices(true)) {
    auto sUuid = pService->getUUID();
    if (!sUuid.equals(uuidServiceHid)) {
      continue;  // skip
    }
    Serial.println(pService->toString().c_str());
    for (auto pChara : *pService->getCharacteristics(true)) {
      charaSubscribeNotification(pChara);
      charaRead(pChara);
    }
  }
  return true;
}


/** Create a single global instance of the callback class to be used by all clients */
static ClientCallbacks clientCB;


/** Handles the provisioning of clients and connects / interfaces with the server */
bool connectToServer() {
    NimBLEClient* pClient = nullptr;

    /** Check if we have a client we should reuse first **/
    if(NimBLEDevice::getClientListSize()) {
        /** Special case when we already know this device, we send false as the
         *  second argument in connect() to prevent refreshing the service database.
         *  This saves considerable time and power.
         */
        pClient = NimBLEDevice::getClientByPeerAddress(advDevice->getAddress());
        if(pClient){
            //pClient->updateConnParams(12,12,0,51);
            if(!pClient->connect(advDevice, false)) {
                ///Serial.println("Reconnect failed");
                ESP_LOGE(BLETAG, "Reconnect failed");
                return false;
            }
            ///Serial.println("Reconnected client");
            ESP_LOGI(BLETAG, "Reconnected client");
        }
        /** We don't already have a client that knows this device,
         *  we will check for a client that is disconnected that we can use.
         */
        else {
            pClient = NimBLEDevice::getDisconnectedClient();
        }
    }

    /** No client to reuse? Create a new one. */
    if(!pClient) {
        if(NimBLEDevice::getClientListSize() >= NIMBLE_MAX_CONNECTIONS) {
            ///Serial.println("Max clients reached - no more connections available");
            ESP_LOGI(BLETAG, "Max clients reached - no more connections available");
            return false;
        }

        pClient = NimBLEDevice::createClient();

        ///Serial.println("New client created");
        ESP_LOGI(BLETAG, "New client created");

        pClient->setClientCallbacks(&clientCB, false);
        /** Set initial connection parameters: These settings are 15ms interval, 0 latency, 120ms timout.
         *  These settings are safe for 3 clients to connect reliably, can go faster if you have less
         *  connections. Timeout should be a multiple of the interval, minimum is 100ms.
         *  Min interval: 12 * 1.25ms = 15, Max interval: 12 * 1.25ms = 15, 0 latency, 51 * 10ms = 510ms timeout
         */
        pClient->setConnectionParams(12,12,0,51);
        /** Set how long we are willing to wait for the connection to complete (seconds), default is 30. */
        pClient->setConnectTimeout(5);


        if (!pClient->connect(advDevice)) {
            /** Created a client but failed to connect, don't need to keep it as it has no data */
            NimBLEDevice::deleteClient(pClient);
            ///Serial.println("Failed to connect, deleted client");
            ESP_LOGI(BLETAG, "Failed to connect, deleted client");
            return false;
        }
    }

    if(!pClient->isConnected()) {
        if (!pClient->connect(advDevice)) {
            ///Serial.println("Failed to connect");
            ESP_LOGI(BLETAG, "Failed to connect");
            return false;
        }
    }

    Serial.print("Connected to: ");
    Serial.println(pClient->getPeerAddress().toString().c_str());
    ///Serial.print("RSSI: ");
    ///Serial.println(pClient->getRssi());
    ESP_LOGI(BLETAG, "RSSI: %d", pClient->getRssi());

    

    bool result = afterConnect(pClient);
        if (!result) {
        return result;
    }

    Serial.println("Done with this device!");
    return true;
}

void parseXboxPad(uint8_t* pData){

  pad_data.button = 0x0000;

  if(pData[9] > 0x01){
    //左トリガー
    pad_data.button += S_L2;
  }
  if(pData[11] > 0x01){
    //右トリガー
    pad_data.button += S_R2;
  }
  if(pData[13] & 0x40){
    //左ショルダー
    pad_data.button += S_L1;
  }
  if(pData[13] & 0x80){
    //左ショルダー
    pad_data.button += S_R1;
  }

  if(pData[12] == 0x01){
    //十字　上
    pad_data.button += CROSS_U;
  }else if(pData[12] == 0x02){
    //十字　右上
    pad_data.button += CROSS_U;
    pad_data.button += CROSS_R;
  }else if(pData[12] == 0x03){
    //十字　右
    pad_data.button += CROSS_R;
  }else if(pData[12] == 0x04){
    //十字　右下
    pad_data.button += CROSS_R;
    pad_data.button += CROSS_D;
  }else if(pData[12] == 0x05){
    //十字　下
    pad_data.button += CROSS_D;
  }else if(pData[12] == 0x06){
    //十字　左下
    pad_data.button += CROSS_L;
    pad_data.button += CROSS_D;
  }else if(pData[12] == 0x07){
    //十字　左
    pad_data.button += CROSS_L;
  }else if(pData[12] == 0x08){
    //十字　左上
    pad_data.button += CROSS_L;
    pad_data.button += CROSS_U;
  }

  if(pData[13] & 0x01){
    //A = ×
    pad_data.button += CROSS;
  }
  if(pData[13] & 0x02){
    //B = 〇
    pad_data.button += CIRCLE;
  }
  if(pData[13] & 0x08){
    //X = □
    pad_data.button += SQUARE;
  }
  if(pData[13] & 0x10){
    //Y = △
    pad_data.button += TRIANGLE;
  }

  if(pData[14] & 0x04){
    //select
    pad_data.button += BTN_SELECT;
  }
  if(pData[14] & 0x08){
    //start
    pad_data.button += BTN_START;
  }
  if(pData[14] & 0x10){
    //xbox
    
  }
  if(pData[14] & 0x20){
    //left stick btn
    pad_data.button += ANBTN_L;
  }
  if(pData[14] & 0x40){
    //right stick btn
    pad_data.button += ANBTN_R;
  }


  int8_t tmp = 0;
  if(pData[5] <= 0x7F){
    pad_data.right_stick.x = 127 - pData[5];
  }else{
    pad_data.right_stick.x = -(pData[5] - 128);
  }
  if(pData[7] <= 0x7F){
    pad_data.right_stick.y = 127 - pData[7];
  }else{
    pad_data.right_stick.y = -(pData[7] - 128);
  }
  if(pData[1] <= 0x7F){
    pad_data.left_stick.x = 127 - pData[1];
  }else{
    pad_data.left_stick.x = -(pData[1] - 128);
  }
  if(pData[3] <= 0x7F){
    pad_data.left_stick.y = 127 - pData[3];
  }else{
    pad_data.left_stick.y = -(pData[3] - 128);
  }

  if(abs(pad_data.right_stick.x) < 10){
    pad_data.right_stick.x = 0;
  }
  if(abs(pad_data.right_stick.y) < 10){
    pad_data.right_stick.y = 0;
  }
  if(abs(pad_data.left_stick.x) < 10){
    pad_data.left_stick.x = 0;
  }
  if(abs(pad_data.left_stick.y) < 10){
    pad_data.left_stick.y = 0;
  }


}

//PADの入力情報を更新
int updateXboxPad(){

  //前回取得時とコントローラの状態が変化していない場合はリターン
  if(prev_pad_data.button == pad_data.button && 
     prev_pad_data.left_stick.x == pad_data.left_stick.x && prev_pad_data.left_stick.y == pad_data.left_stick.y &&
     prev_pad_data.right_stick.x == pad_data.right_stick.x && prev_pad_data.right_stick.y == pad_data.right_stick.y){
    return 0;
  }

  prev_pad_data = pad_data;

  return 1;
}



