#include "BLESerial.h"

// #define BLE_SERIAL_DEBUG

#define ADVERTISING_INTERVAL 1000

BLESerial* BLESerial::_instance = NULL;

BLESerial::BLESerial(unsigned char req, unsigned char rdy, unsigned char rst) :
  BLEPeripheral(req, rdy, rst)
{
  this->_txCount = 0;
  this->_rxHead = this->_rxTail = 0;
  this->_flushed = 0;
  BLESerial::_instance = this;

  addAttribute(this->_uartService);
  addAttribute(this->_uartNameDescriptor);
  setAdvertisedServiceUuid(this->_uartService.uuid());
  addAttribute(this->_rxCharacteristic);
  addAttribute(this->_rxNameDescriptor);
  this->_rxCharacteristic.setEventHandler(BLEWritten, BLESerial::_received);
  addAttribute(this->_txCharacteristic);
  addAttribute(this->_txNameDescriptor);
  
}

void BLESerial::begin(...) {
  BLEPeripheral::begin();  
}

void BLESerial::poll() {
  if (millis() < this->_flushed + 100) {
    BLEPeripheral::poll();
  } else {
    flush();
  }
}

void BLESerial::end() {
  this->_rxCharacteristic.setEventHandler(BLEWritten, NULL);
  this->_rxHead = this->_rxTail = 0;
  flush();
  BLEPeripheral::disconnect();
}

int BLESerial::available(void) {
  BLEPeripheral::poll();
  int retval = (this->_rxHead - this->_rxTail + sizeof(this->_rxBuffer)) % sizeof(this->_rxBuffer);
  return retval;
}

int BLESerial::peek(void) {
  BLEPeripheral::poll();
  if (this->_rxTail == this->_rxHead) return -1;
  uint8_t byte = this->_rxBuffer[this->_rxTail];
 
  return byte;
}

int BLESerial::read(void) {
  BLEPeripheral::poll();
  if (this->_rxTail == this->_rxHead) return -1;
  this->_rxTail = (this->_rxTail + 1) % sizeof(this->_rxBuffer);
  uint8_t byte = this->_rxBuffer[this->_rxTail];
  
  return byte;
}

void BLESerial::flush(void) {
  if (this->_txCount == 0) return;
  if (millis() < this->_flushed) this->_flushed = millis();
  while (millis() < this->_flushed + 100){}
  this->_txCharacteristic.setValue(this->_txBuffer, this->_txCount);
  this->_flushed = millis();
  this->_txCount = 0;
  BLEPeripheral::poll();
  
}

size_t BLESerial::write(uint8_t byte) {
  BLEPeripheral::poll();
  if (this->_txCharacteristic.subscribed() == false) return 0;
  this->_txBuffer[this->_txCount++] = byte;
  if (this->_txCount == sizeof(this->_txBuffer)) flush();
  
  return 1;
}

BLESerial::operator bool() {
  bool retval = BLEPeripheral::connected();
  
  return retval;
}

void BLESerial::_received(const uint8_t* data, size_t size) {
  for (int i = 0; i < size; i++) {
    this->_rxHead = (this->_rxHead + 1) % sizeof(this->_rxBuffer);
    this->_rxBuffer[this->_rxHead] = data[i];
  }
 
}

void BLESerial::_received(BLECentral& /*central*/, BLECharacteristic& rxCharacteristic) {
  BLESerial::_instance->_received(rxCharacteristic.value(), rxCharacteristic.valueLength());
}

void BLESerial::txmsg(const uint8_t* data, size_t size) {
  if (millis() < this->_flushed) this->_flushed = millis();
  while (millis() < this->_flushed + 100){}
  this->_txCount = size;
  for (int i = 0; i < size; i++) {
    this->_txBuffer[i] = data[i];
  }
  flush(); 
}
