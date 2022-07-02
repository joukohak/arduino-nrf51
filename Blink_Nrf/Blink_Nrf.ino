

/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the Uno and
  Leonardo, it is attached to digital pin 13. If you're unsure what
  pin the on-board LED is connected to on your Arduino model, check
  the documentation at http://arduino.cc

  This example code is in the public domain.

  modified 8 May 2014
  by Scott Fitzgerald
  
  Modified by Roger Clark. www.rogerclark.net for Maple mini 25th April 2015 , where the LED is on PB1
  
 */
#ifdef __cplusplus
extern "C" {
#endif
void RTC1_IRQHandler();
#ifdef __cplusplus
}
#endif



#include <Arduino.h>
 
// LED and button pin
#define LED_PIN     29
#define PIN_BUTTON1  28
volatile bool light_on = false;

void RTC1_IRQHandler(void);

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin PB1 as an output.
  pinMode(LED_PIN, OUTPUT);
  rtc1_config();

  //pinMode(PIN_SW_SENSOR, INPUT_PULLUP);
}

// the loop function runs over and over again forever
void loop() {
/*    if (!light_on) {
      digitalWrite(LED_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)
      light_on = true;
    }else{
      digitalWrite(LED_PIN, LOW);    // turn the LED off by making the voltage LOW
      light_on = false;
    }
 // delay(100);*/

 
  mydelay();              // wait for a second
  
}


/** Configures the RTC1 with TICK for 100Hz and COMPARE0 to 10 sec
 */
static void rtc1_config(void)
{
   NRF_RTC1->TASKS_STOP = 1;
   NRF_RTC1->TASKS_CLEAR = 1;
   NRF_RTC1->PRESCALER = 0;

    NRF_RTC1->CC[0] = 5 * 32768 ;    //If not defined, the chip will freeze in 512 seconds    
//  NVIC_ClearPendingIRQ(RTC1_IRQn);
//  NVIC_EnableIRQ(RTC1_IRQn);

  // Configure timer for clock
//  NRF_RTC1->EVTENSET = RTC_EVTEN_COMPARE0_Msk;
//  NRF_RTC1->INTENSET = RTC_INTENSET_COMPARE0_Msk;
 // Enable EVENTS_COMPARE[0] generation
  NRF_RTC1->EVTENSET = RTC_EVTENSET_COMPARE0_Enabled << RTC_EVTENSET_COMPARE0_Pos;
  // Enable IRQ on EVENTS_COMPARE[0]
  NRF_RTC1->INTENSET = RTC_INTENSET_COMPARE0_Enabled << RTC_INTENSET_COMPARE0_Pos;

  
  NVIC_ClearPendingIRQ(RTC1_IRQn);
  NVIC_EnableIRQ(RTC1_IRQn);
   
//  NRF_RTC1->CC[3] = NRF_RTC1->COUNTER + (5 * 32768) ;    //If not defined, the chip will freeze in 512 seconds
  
  NRF_RTC1->TASKS_START = 1;
  
  nrf_delay_us(100);
  
}


/** RTC1 interrupt handler.
 * Triggered on COMPARE3 match.
 */

void RTC1_IRQHandler()
{

  
  if ((NRF_RTC1->EVENTS_COMPARE[0] != 0) && ((NRF_RTC1->INTENSET & RTC_INTENSET_COMPARE0_Msk) != 0))
  {
    NRF_RTC1->EVENTS_COMPARE[0] = 0;
    NRF_RTC1->CC[0] = (NRF_RTC1->COUNTER + 2* 32768)& 0xffffff;     // Compare0 after approx COMPARE_COUNTERTIME 1 second
    if (!light_on) {
     digitalWrite(LED_PIN, HIGH);   // turn the LED on
      light_on = true;
    }else{
      digitalWrite(LED_PIN, LOW);    // turn the LED off
      light_on = false;
    }
   


/*    if (seconds++ >= 59){
      seconds=0;
      if (minutes++ >= 59){
        minutes=0;
         if (hours++ >= 23) hours=0;        
      }
    }*/
    
  }
}
void mydelay(void) {
  volatile uint32_t waiting = NRF_RTC1->COUNTER ;
  volatile uint32_t diff = 0 ;
  while (diff < 32768){
    diff = NRF_RTC1->COUNTER - waiting;
  }
}
