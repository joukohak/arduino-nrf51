


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
#ifdef __cplusplus
extern "C" {
#endif
void RTC1_IRQHandler();
#ifdef __cplusplus
}
#endif

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
#define WAIT_SENSOR_MS 30000  //

// LED and button pin
#define LED_PIN     18
#define BUTTON_PIN1  16


volatile bool g_sensorValueChanged = false;
volatile uint32_t g_lastSensorValueChanged = 0;
volatile uint8_t pir_counterValue = 0;
volatile uint8_t seconds = 0, minutes = 0, hours = 0;
volatile uint8_t pir_On = 0;
volatile uint16_t bat_Volt = 500 , solar_Volt = 550, temp_Volt = 500;
uint8_t pirFrame[10] = { 0x2A , 25 , 0 , 0 , pir_On , pir_counterValue , 1  , 3 , 50 ,0x23 } ;
uint8_t   BatLog[500],SolarLog[500],TempLog[500];

uint16_t   storeindex=0,storeof=0;
uint16_t   adcBuffer[3];
uint8_t   iVbat = 130, iTempC = 25, iVsolar = 20,adc_counter=0;

// 1023-ADC table from -30 to 50C on 5C steps
// ntc 30k with 43k pullup 3.3V with 1:3 div and 1.2V ref
const uint16_t ADCntcvalue[17] = {71,93,121,155,194,238,287,339,393,448,501,553,600,644,684,719,749};


int8_t Temp_Linear(uint16_t ntcADC);
unsigned int getAdcLevel(uint8_t channel_sel);
void  adc_channel_select(uint8_t channel);
void  DisplayLog(void);
void  rtc1_config(void);
void  saveSamples(void);
void  adc_Buffer_read(void);

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
  BLESerial.setLocalName("NRF51 PIR");
  BLESerial.setAdvertisingInterval(ADVERTISING_INTERVAL);
  
  gpio_init();

  rtc1_config();

  BLESerial.begin();  
}

void loop() {

   // Enter Low power mode
   sd_app_evt_wait();
  // Exit Low power mode
 
  sd_nvic_ClearPendingIRQ(SWI2_IRQn);

  command_received();
  send_message();
 
}





void command_received() {
  if (BLESerial) {
    int byte;
    byte = BLESerial.read();
    if (byte  == 0x34) send_info();
    if (byte  == 0x35){
      send_info();
      DisplayLog();
    }
  }
}


//  send message 10 bytes

void send_message() {
  if (BLESerial) {
    if (( pir_On == 1)&&(pir_counterValue < 25)) {    // 25 alarms max
      digitalWrite(LED_PIN, HIGH);                  // turn the LED on
      pir_On = 0;
      pirFrame[4] = 1;
      pirFrame[5] = pir_counterValue;
      adc_Volt_read();
      BLESerial.txmsg(pirFrame , 10);
           
      delay(1000);
      digitalWrite(LED_PIN, LOW);
    }
    

  }
}

//  send info message 10 bytes

void send_info() {
      pir_counterValue == 0;      // clear alarm counter
      pirFrame[4] = 0;
      pirFrame[5] = pir_counterValue;
      adc_Volt_read();

      BLESerial.txmsg(pirFrame , 10);
           
      delay(1000);

}

void sensorValueChanged()
{
  uint32_t diff = millis() - g_lastSensorValueChanged;
  if (diff > WAIT_SENSOR_MS || diff < 0)
  {
    if (++pir_counterValue <=20){  
      g_sensorValueChanged = true;
      // pir_counterValue = pir_counterValue + 1;
      pir_On = 1;
    }
  }
  g_lastSensorValueChanged = millis();
}

void adc_Volt_read(void)
{
  adc_Buffer_read();
    // battery voltage
  bat_Volt = adcBuffer[0];
  pirFrame[2] = (bat_Volt >>8) & 0x0F;
  pirFrame[3] = bat_Volt & 0x00FF;
  iVbat = bat_Volt * 36 / 256;
  // solar voltage
  solar_Volt = adcBuffer[1];
  pirFrame[7] = (solar_Volt >>8) & 0x0F;
  pirFrame[8] = solar_Volt & 0x00FF;
  iVsolar = solar_Volt * 36 / 256;
  // ntc temperature voltage
  temp_Volt = 938 - adcBuffer[2];
  iTempC = Temp_Linear(temp_Volt);
  pirFrame[1] = iTempC;    
}

void adc_Buffer_read(void)
{
  NRF_ADC->EVENTS_END = 0;
  NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled;
  uint8_t idx=0;
  for (idx =0; idx < 3; idx++){
    adc_channel_select(idx);
        
    NRF_ADC->EVENTS_END = 0; // Stop any running conversions.
    NRF_ADC->TASKS_START = 1;
 // uint32_t conversion_wait_max = micros();
  
    while (!NRF_ADC->EVENTS_END )
    {
    }
  
    adcBuffer[idx] = NRF_ADC->RESULT;
  }
  NRF_ADC->EVENTS_END = 0;
  NRF_ADC->TASKS_STOP = 1;
  NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Disabled;
      
}

// ADC input channel 2/3/4 voltage read
unsigned int getAdcLevel(uint8_t channel_sel)
{
  
  // Configure ADC
  if (channel_sel == 0){
    NRF_ADC->CONFIG = (ADC_CONFIG_RES_10bit << ADC_CONFIG_RES_Pos) |
                    (ADC_CONFIG_PSEL_AnalogInput2 << ADC_CONFIG_PSEL_Pos) |
                    (ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) |
                    (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos) |
                    (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos);
  }
  if (channel_sel == 1){
    NRF_ADC->CONFIG = (ADC_CONFIG_RES_10bit << ADC_CONFIG_RES_Pos) |
                    (ADC_CONFIG_PSEL_AnalogInput3 << ADC_CONFIG_PSEL_Pos) |
                    (ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) |
                    (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos) |
                    (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos);
  }
  if (channel_sel == 2){
    NRF_ADC->CONFIG = (ADC_CONFIG_RES_10bit << ADC_CONFIG_RES_Pos) |
                    (ADC_CONFIG_PSEL_AnalogInput4 << ADC_CONFIG_PSEL_Pos) |
                    (ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) |
                    (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos) |
                    (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos);
  }
  NRF_ADC->EVENTS_END = 0;
  NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled;

  NRF_ADC->EVENTS_END = 0; // Stop any running conversions.
  NRF_ADC->TASKS_START = 1;
 // uint32_t conversion_wait_max = micros();
  
  while (!NRF_ADC->EVENTS_END )
  {
  }
  

  uint16_t adc_in_mv = NRF_ADC->RESULT;

  NRF_ADC->EVENTS_END = 0;
  NRF_ADC->TASKS_STOP = 1;
  NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Disabled;
  
  return (unsigned int)(adc_in_mv);
}


void storeData(void)
{
    
    adc_Volt_read();
    
    BatLog[storeindex]= iVbat;
    TempLog[storeindex]= iTempC;
    SolarLog[storeindex]= iVsolar;
    if (++storeindex >= 500){
      storeindex=0;
      storeof=1;
    }
}


// save sample buffer to log data

void saveSamples(void)
{
  // battery voltage
  bat_Volt = adcBuffer[0];
  pirFrame[2] = (bat_Volt >>8) & 0x0F;
  pirFrame[3] = bat_Volt & 0x00FF;
  iVbat = bat_Volt * 36 / 255;
  // solar voltage
  solar_Volt = adcBuffer[1];
  pirFrame[7] = (solar_Volt >>8) & 0x0F;
  pirFrame[8] = solar_Volt & 0x00FF;
  iVsolar = solar_Volt * 36 / 255;
  // ntc temperature voltage
  temp_Volt = 938 - adcBuffer[2];
  iTempC = Temp_Linear(temp_Volt);
  pirFrame[1] = iTempC;
    
    
    BatLog[storeindex]= iVbat;
    TempLog[storeindex]= iTempC;
    SolarLog[storeindex]= iVsolar;
    if (++storeindex >= 500){
      storeindex=0;
      storeof=1;
    }
}


void rtc1_config(void) {
   NRF_RTC1->TASKS_STOP = 1;
   NRF_RTC1->TASKS_CLEAR = 1;
   NRF_RTC1->PRESCALER = 0;

//  NVIC_SetPriority(RTC1_IRQn, 3);
  NVIC_ClearPendingIRQ(RTC1_IRQn);
  NVIC_EnableIRQ(RTC1_IRQn);
  // Configure rtc1 counter overflow
  NRF_RTC1->INTENSET = RTC_INTENSET_OVRFLW_Msk;
  NRF_RTC1->EVTENSET = RTC_EVTEN_OVRFLW_Msk;
  // Configure cc3 events and interrups
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
  //digitalWrite(LED_PIN, HIGH);   // turn the LED on
  if (NRF_RTC1->EVENTS_OVRFLW != 0){      // RTC1 24bit counter overflow
    
    NRF_RTC1->EVENTS_OVRFLW = 0;
    RTC1_overflow_inc();

  }
  //NRF_RTC1->EVENTS_COMPARE[0] = 0;
  //NRF_RTC1->EVENTS_COMPARE[1] = 0;
  //NRF_RTC1->EVENTS_COMPARE[2] = 0;
  //NRF_RTC1->EVENTS_TICK       = 0;
  if (NRF_RTC1->EVENTS_COMPARE[3] != 0)
  { 

    NRF_RTC1->EVENTS_COMPARE[3] = 0;
    NRF_RTC1->CC[3] = (NRF_RTC1->COUNTER + 10 * 32768)& 0xffffff;     // Compare0 after approx COMPARE_COUNTERTIME 5 second
   
    seconds=seconds+10;
    if (seconds>= 59){
      seconds=0; 
      if (minutes++ >= 59){
        minutes=0;
         if (hours++ >= 23) {
            pir_counterValue=0;
            hours=0;
         }    
      }
    }
    if (seconds ==30){
      adc_Buffer_read();
      saveSamples();
    }
    

//   digitalWrite(LED_PIN, LOW);    // turn the LED off 
  }
    
}

void DisplayLog(void)
{
  uint16_t  i,index,maxcnt;
 
  
  BLESerial.write(0x2B);
   
  maxcnt=storeindex;
  index=0;
 
  if (storeof==1) {
    maxcnt = 500;
    index = storeindex;
  }
  for(i=0; i < maxcnt; i++)
    {
      BLESerial.write(BatLog[index]);
      BLESerial.write(SolarLog[index]);
      BLESerial.write(TempLog[index]);     
           
      if (++index >= 500) index=0;
    }
  BLESerial.write(0xFF);
  BLESerial.flush();
}

//ADC ntc value to  Temperature value

int8_t Temp_Linear(uint16_t ntcADC)
{
   uint8_t j=0;
   int8_t retC;
   if (ntcADC <= ADCntcvalue[0]){
    retC = -30;
    return retC;
   }
   for (j = 0; j < 17; j++) {
     if (ntcADC < ADCntcvalue[j+1])
     {
       ntcADC= ntcADC - ADCntcvalue[j];
       retC = ((j * 5) + ((5* ntcADC) /(ADCntcvalue[j+1]-ADCntcvalue[j])) - 30);
       return retC;
     }
   }
   return 50;
}




// adc channel select
void adc_channel_select(uint8_t channel)
{
  
  // configure adc channel ADC
  if (channel == 0){
    NRF_ADC->CONFIG = (ADC_CONFIG_RES_10bit << ADC_CONFIG_RES_Pos) |
                    (ADC_CONFIG_PSEL_AnalogInput2 << ADC_CONFIG_PSEL_Pos) |
                    (ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) |
                    (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos) |
                    (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos);
  }
  if (channel == 1){
    NRF_ADC->CONFIG = (ADC_CONFIG_RES_10bit << ADC_CONFIG_RES_Pos) |
                    (ADC_CONFIG_PSEL_AnalogInput3 << ADC_CONFIG_PSEL_Pos) |
                    (ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) |
                    (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos) |
                    (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos);
  }
  if (channel == 2){
    NRF_ADC->CONFIG = (ADC_CONFIG_RES_10bit << ADC_CONFIG_RES_Pos) |
                    (ADC_CONFIG_PSEL_AnalogInput4 << ADC_CONFIG_PSEL_Pos) |
                    (ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) |
                    (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos) |
                    (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos);
  }
  NRF_ADC->TASKS_START = 1;
}
