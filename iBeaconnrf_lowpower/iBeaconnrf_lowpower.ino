// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <BLEPeripheral.h>
#include <iBeacon.h>
#include "nrf_soc.h"
#include "nrf_nvic.h"

#if !defined(NRF51) && !defined(NRF52) && !defined(__RFduino__)
#error "This example only works with nRF51 boards"
#endif
#define ADVERTISING_INTERVAL 1000
iBeacon beacon;

void setup() {
  char* uuid                   = "a196c876-de8c-4c47-ab5a-d7afd5ae7127";
  unsigned short major         = 128;
  unsigned short minor         = 16;
  unsigned short measuredPower = -55;
  
  beacon.setAdvertisingInterval(ADVERTISING_INTERVAL);
  
  
  beacon.begin(uuid, major, minor, measuredPower);

   // enable low power mode without interrupt
  sd_power_mode_set(NRF_POWER_MODE_LOWPWR);

}

void loop() {
  
   // Enter Low power mode
  sd_app_evt_wait();
  // Exit Low power mode
  
  // Clear IRQ flag to be able to go to sleep if nothing happens in between
  sd_nvic_ClearPendingIRQ(SWI2_IRQn);

  
  
  beacon.loop();
}
