#ifndef BLE_CONFIG_H
#define BLE_CONFIG_H

#include <NimBLEDevice.h>

class BLEConfig {
public:
  static void init();
  static NimBLEServer* getServer();
  static NimBLEService* getService();
  static NimBLECharacteristic* getWriteCharacteristic();
  static NimBLECharacteristic* getCharacteristic();
  static NimBLECharacteristic* getNewCharacteristic();
  static NimBLECharacteristic* getCurveCharacteristic();

private:
    static NimBLEServer* pServer;
    static NimBLEService* pService;
    static NimBLECharacteristic* pCharacteristic;
    static NimBLECharacteristic* pNewCharacteristic;
    static NimBLECharacteristic* pWriteCharacteristic;
    static NimBLECharacteristic* pCurveCharacteristic;

};

#endif  // BLE_CONFIG_H