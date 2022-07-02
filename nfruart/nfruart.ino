// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

/*
 * Serial Port over BLE
 * Create UART service compatible with Nordic's *nRF Toolbox* and Adafruit's *Bluefruit LE* iOS/Android apps.
 *
 * BLESerial class implements same protocols as Arduino's built-in Serial class and can be used as it's wireless
 * replacement. Data transfers are routed through a BLE service with TX and RX characteristics. To make the
 * service discoverable all UUIDs are NUS (Nordic UART Service) compatible.
 *
 * Please note that TX and RX characteristics use Notify and WriteWithoutResponse, so there's no guarantee
 * that the data will make it to the other end. However, under normal circumstances and reasonable signal
 * strengths everything works well.
 */


// Import libraries (BLEPeripheral depends on SPI)
#include <SPI.h>
#include <BLEPeripheral.h>
#include "BLESerial.h"
#include "nrf_soc.h"
#include "nrf_nvic.h"

// define pins (varies per shield/board)
#define BLE_REQ   10
#define BLE_RDY   2
#define BLE_RST   9

#define ADVERTISING_INTERVAL 1000
#define DEBOUNCE_SENSOR_MS 600

// LED and button pin
#define LED_PIN     18
#define BUTTON_PIN1  16

volatile bool g_sensorValueChanged = false;
volatile uint32_t g_lastSensorValueChanged = 0;
volatile uint8_t g_counterValue = 0;


// create ble serial instance, see pinouts above
BLESerial BLESerial(BLE_REQ, BLE_RDY, BLE_RST);



static void gpio_init(void)
{

  pinMode(LED_PIN, OUTPUT);
 

  pinMode(BUTTON_PIN1, INPUT_PULLDOWN);

  attachInterruptLowAccuracy(digitalPinToInterrupt(BUTTON_PIN1), sensorValueChanged, RISING);
}

void setup() {
  // custom services and characteristics can be added as well
  BLESerial.setLocalName("UARTpir");

  gpio_init();   


  BLESerial.begin();
}

void loop() {

   // Enter Low power mode
  sd_app_evt_wait();
  // Exit Low power mode

 
  sd_nvic_ClearPendingIRQ(SWI2_IRQn);
  
  BLESerial.poll();
  BLESerial.write(0x33);

  forward();
  // loopback();
   spam();
}


// forward received from Serial to BLESerial and vice versa
void forward() {
  if (BLESerial && Serial) {
    int byte;
    while ((byte = BLESerial.read()) > 0) Serial.write((char)byte);
    while ((byte = Serial.read()) > 0) BLESerial.write((char)byte);
  }
}

// echo all received data back
void loopback() {
  if (BLESerial) {
    int byte;
    while ((byte = BLESerial.read()) > 0) BLESerial.write(byte);
  }
}

// periodically sent time stamps
void spam() {
  if (BLESerial) {
    BLESerial.print(millis());
    BLESerial.write(0x33);
    delay(1000);
  }
}

void sensorValueChanged()
{
  uint32_t diff = millis() - g_lastSensorValueChanged;
  if (diff > DEBOUNCE_SENSOR_MS || diff < 0)
  {
    g_sensorValueChanged = true;
    g_counterValue = g_counterValue + 1;
  }
  g_lastSensorValueChanged = millis();
}
