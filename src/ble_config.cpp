#include "ble_config.h"

NimBLEServer* BLEConfig::pServer = nullptr;
NimBLEService* BLEConfig::pService = nullptr;
NimBLECharacteristic* BLEConfig::pWriteCharacteristic = nullptr;
NimBLECharacteristic* BLEConfig::pCharacteristic = nullptr;
NimBLECharacteristic* BLEConfig::pNewCharacteristic = nullptr;
NimBLECharacteristic* BLEConfig::pCurveCharacteristic = nullptr;

void BLEConfig::init() {
  NimBLEDevice::init("OxiMed <3"); // Nome do Bluetooth
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY); // Define o tipo de pareamento
  NimBLEDevice::setSecurityAuth(true, true, true); // Habilita o pareamento com autenticação e solicitação de PIN

  pServer = NimBLEDevice::createServer();
  pService = pServer->createService("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
  pWriteCharacteristic = pService->createCharacteristic(
                         "f14a557e-a13d-4b00-9353-913027a5cd89",
                         NIMBLE_PROPERTY::READ |
                         NIMBLE_PROPERTY::WRITE |
                         NIMBLE_PROPERTY::NOTIFY
                       );

  pCharacteristic = pService->createCharacteristic(
                    "beb5483e-36e1-4688-b7f5-ea07361b26a8",
                    NIMBLE_PROPERTY::READ |
                    NIMBLE_PROPERTY::WRITE |
                    NIMBLE_PROPERTY::NOTIFY
                  );

  pNewCharacteristic = pService->createCharacteristic(
                       "26e2b12b-85f0-4f3f-9fdd-91d114270e6e",
                       NIMBLE_PROPERTY::READ |
                       NIMBLE_PROPERTY::WRITE |
                       NIMBLE_PROPERTY::NOTIFY
                     );

  pCurveCharacteristic = pService->createCharacteristic(
                        "26e2b12b-85f0-4f3f-9fdd-91d114270ea5",
                        NIMBLE_PROPERTY::READ |
                        NIMBLE_PROPERTY::WRITE |
                        NIMBLE_PROPERTY::NOTIFY
                      );

  pService->start();
  NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
  pAdvertising->start();
}

NimBLEServer* BLEConfig::getServer() {
  return pServer;
}

NimBLEService* BLEConfig::getService() {
  return pService;
}

NimBLECharacteristic* BLEConfig::getWriteCharacteristic() {
  return pWriteCharacteristic;
}

NimBLECharacteristic* BLEConfig::getCharacteristic() {
  return pCharacteristic;
}

NimBLECharacteristic* BLEConfig::getNewCharacteristic() {
  return pNewCharacteristic;
}

NimBLECharacteristic* BLEConfig::getCurveCharacteristic() {
  return pCurveCharacteristic;
}