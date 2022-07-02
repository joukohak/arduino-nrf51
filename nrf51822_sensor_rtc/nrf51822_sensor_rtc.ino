#ifdef __cplusplus
extern "C" {
#endif
void RTC1_IRQHandler();
#ifdef __cplusplus
}
#endif
#include <Arduino.h>
#include <SPI.h>
#include <BLEPeripheral.h>
#include "nrf_soc.h"
#include "nrf_nvic.h"

#define ADVERTISING_INTERVAL 5000
#define DEBOUNCE_SENSOR_MS 600

#define SR_LEN 15
#define VBAT_MAX_IN_MV 3600
#define TX_POWER 4

#define COUNTER_PRESCALER   0  /* f = LFCLK/(prescaler + 1) 125ms*/


// LED and button pin
#define LED_PIN    29
#define BUTTON_PIN1  28

BLEPeripheral blePeripheral = BLEPeripheral();

BLEService mainBleService("c83e0fa6-f5b7-473a-a2a2-3957117a2f58");
BLEUnsignedIntCharacteristic sensorCharacteristic("8afc0cf0-19bf-4bd2-9413-6e7259765edf", BLERead | BLEWrite | BLENotify);
BLEService batteryService("180F"); 
BLEUnsignedCharCharacteristic batteryLevelCharacteristic("2A19", BLERead);

volatile bool g_sensorValueChanged = false;
volatile uint32_t g_lastSensorValueChanged = 0;
volatile uint32_t g_counterValue = 0;
volatile uint32_t settimeValue = 0;
volatile uint32_t rtcValue = 0;
volatile uint8_t seconds = 0, minutes = 0, hours = 0;
volatile bool light_on = false;
const char g_localName[] = "NRF Sensor RTC";

static void gpio_init(void);
void bleConnectedCallback(BLECentral &);
void sensorCharacteristicWrittenCallback(BLECentral &, BLECharacteristic &);
void log(String s);
void digitalWriteLog(uint32_t ulPin, uint32_t ulVal);
void updateAdvertisingScanData();
void sensorValueChanged();
unsigned char getBatteryLevel(void);
static void rtc1_config();



static void gpio_init(void)
{

  pinMode(LED_PIN, OUTPUT);
 
  pinMode(BUTTON_PIN1, INPUT_PULLUP);

  attachInterruptLowAccuracy(digitalPinToInterrupt(BUTTON_PIN1), sensorValueChanged, FALLING);
}

void setup()
{


  gpio_init();

  blePeripheral.setDeviceName(g_localName);
  blePeripheral.setLocalName(g_localName);

  blePeripheral.addAttribute(batteryService);
  blePeripheral.addAttribute(batteryLevelCharacteristic);

  blePeripheral.setAdvertisedServiceUuid(mainBleService.uuid());
  blePeripheral.addAttribute(mainBleService);
  blePeripheral.addAttribute(sensorCharacteristic);

  blePeripheral.setAdvertisingInterval(ADVERTISING_INTERVAL);
  blePeripheral.setEventHandler(BLEConnected, bleConnectedCallback);

  sensorCharacteristic.setValue(g_counterValue);
  sensorCharacteristic.setEventHandler(BLEWritten, sensorCharacteristicWrittenCallback);

  blePeripheral.begin();

  blePeripheral.setTxPower(TX_POWER);

  // Manualy updates advertising scan data
  updateAdvertisingScanData();

  // enable low power mode
  sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
  
  rtc1_config();

}

void loop()
{ 
  // Enter Low power mode
  sd_app_evt_wait();
// Exit Low power mode
  
  sd_nvic_ClearPendingIRQ(SWI2_IRQn);

 
  if (g_sensorValueChanged)
  {
    g_sensorValueChanged = false;
    digitalWrite(LED_PIN, HIGH);   // turn the LED on
    delay(1000);
    
    // Sets characteristic value
    sensorCharacteristic.setValue(g_counterValue);

    // Update advertising scan data with counter value
    updateAdvertisingScanData();
    digitalWrite(LED_PIN, LOW);    // turn the LED off

  }

  // poll peripheral
  blePeripheral.poll();
}

void bleConnectedCallback(BLECentral &bleCentral)
{
  unsigned char batteryLevel = getBatteryLevel();
  
  batteryLevelCharacteristic.setValue(batteryLevel);
}

void sensorValueChanged()
{
   uint32_t diff = millis() - g_lastSensorValueChanged;
  if (diff > DEBOUNCE_SENSOR_MS || diff < 0)
  {
    g_sensorValueChanged = true;
    
    // update time to counter value
    g_counterValue =((getBatteryLevel()<<24) & 0xFF000000)|((hours <<16 ) & 0x00FF0000)|((minutes << 8) & 0x0000FF00)|(seconds & 0x00FF);
  }
  g_lastSensorValueChanged = millis();
}
  

void sensorCharacteristicWrittenCallback(BLECentral &central, BLECharacteristic &characteristic)
{
  // The new value has already been written on characteristic
  // We still save it and flag the value as changed to allow the advertising packet to be updated
  g_sensorValueChanged = true;
  g_counterValue = sensorCharacteristic.value();
  hours = (g_counterValue >> 16) & 0xFF;
  minutes = (g_counterValue >> 8) & 0xFF;  
  seconds = g_counterValue  & 0xFF;
  
}

// https://www.bluetooth.com/specifications/assigned-numbers/generic-access-profile/
void updateAdvertisingScanData()
{
  unsigned char srData[31];
  unsigned char srDataLen = 0;
  int scanDataSize = 3;
  BLEEirData scanData[scanDataSize];

  // - Local name
  scanData[0].length = strlen(g_localName);
  scanData[0].type = 0x09;
  memcpy(scanData[0].data, g_localName, scanData[0].length);

  // - Tx Power
  scanData[1].length = 1;
  scanData[1].type = 0x0A;
  scanData[1].data[0] = TX_POWER;

  // - Manufacturer Data
  scanData[2].length = 2 + 4;
  scanData[2].type = 0xFF;
  // Manufacturer ID
  scanData[2].data[0] = 0xFF;
  scanData[2].data[1] = 0xFF;
  // Manufacturer data content
  scanData[2].data[2] = seconds;
  scanData[2].data[3] = minutes;
  scanData[2].data[4] = hours;
  scanData[2].data[5] = getBatteryLevel();

  if (scanDataSize && scanData)
  {
    for (int i = 0; i < scanDataSize; i++)
    {
      srData[srDataLen + 0] = scanData[i].length + 1;
      srData[srDataLen + 1] = scanData[i].type;
      srDataLen += 2;

      memcpy(&srData[srDataLen], scanData[i].data, scanData[i].length);

      srDataLen += scanData[i].length;
    }
  }

  // - Sets only avertising scan data
  sd_ble_gap_adv_data_set(NULL, 0, srData, srDataLen);
}

//  Battery voltage 20 mV resolution  (160 = 3200 mV)

unsigned char getBatteryLevel(void)
{
  // Configure ADC
  NRF_ADC->CONFIG = (ADC_CONFIG_RES_8bit << ADC_CONFIG_RES_Pos) |
                    (ADC_CONFIG_INPSEL_SupplyOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) |
                    (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos) |
                    (ADC_CONFIG_PSEL_Disabled << ADC_CONFIG_PSEL_Pos) |
                    (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos);
  NRF_ADC->EVENTS_END = 0;
  NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled;

  NRF_ADC->EVENTS_END = 0; // Stop any running conversions.
  NRF_ADC->TASKS_START = 1;

  while (!NRF_ADC->EVENTS_END)
  {
  }
  uint16_t vbg_in_mv = 1200;
  uint8_t adc_max = 255;
  uint16_t vbat_current_in_mv = (NRF_ADC->RESULT * 3 * vbg_in_mv) / adc_max;

  NRF_ADC->EVENTS_END = 0;
  NRF_ADC->TASKS_STOP = 1;
  NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Disabled;

  return (unsigned char)((vbat_current_in_mv * 180) / VBAT_MAX_IN_MV);  // 
}

void log(String s)
{
#ifdef ENABLE_LOG
  Serial.println(s);
#endif
}



/** Configures the RTC1 with TICK for 100Hz and COMPARE0 to 10 sec
 */
static void rtc1_config(void)
{
   NRF_RTC1->TASKS_STOP = 1;
   NRF_RTC1->TASKS_CLEAR = 1;
   NRF_RTC1->PRESCALER = 0;

 
  NVIC_ClearPendingIRQ(RTC1_IRQn);
  NVIC_EnableIRQ(RTC1_IRQn);

  // Configure timer for clock
  NRF_RTC1->EVTENSET = RTC_EVTEN_COMPARE3_Msk;
  NRF_RTC1->INTENSET = RTC_INTENSET_COMPARE3_Msk;
   
  NRF_RTC1->CC[3] = NRF_RTC1->COUNTER + (10 * 32768) ;    //If not defined, the chip will freeze in 512 seconds
  
  NRF_RTC1->TASKS_START = 1;
  
}

/** RTC1 interrupt handler.
 * Triggered on COMPARE3 match.
 */

void RTC1_IRQHandler(void)
{
  NRF_RTC1->EVENTS_OVRFLW = 0;
  NRF_RTC1->EVENTS_COMPARE[0] = 0;
  NRF_RTC1->EVENTS_COMPARE[1] = 0;
  if (NRF_RTC1->EVENTS_COMPARE[3] != 0)
  {
    NRF_RTC1->EVENTS_COMPARE[3] = 0;
    NRF_RTC1->CC[3] = (NRF_RTC1->COUNTER + 10 * 32768)& 0xffffff;     // Compare0 after approx COMPARE_COUNTERTIME 1 second
   
    seconds=seconds+10;
    if (seconds>= 59){
      seconds=0;
      if (minutes++ >= 59){
        minutes=0;
         if (hours++ >= 23) hours=0;        
      }
    }
    updateAdvertisingScanData();
       
  }
}
