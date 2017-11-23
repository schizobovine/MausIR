//
// wdt.cpp - Watchdog shit for the SAMD
//

#include <Arduino.h>
#include "wdt.h"

// Enable watchdog timer which will wake us from sleep
void Watchdog::enable() {

  if (!this->_is_initialized) this->_initialize_wdt();

  // Disable WDT for configuration
  WDT->CTRL.reg = 0;
  SYNC_WDT();

  // Configure watchdog interrupt
  WDT->INTENSET.bit.EW   = 1;      // Enable early warning interrupt
  WDT->CONFIG.bit.PER    = 0xB;    // Period = max
  WDT->CONFIG.bit.WINDOW = 0xB;    // Set time of interrupt
  WDT->CTRL.bit.WEN      = 1;      // Enable window mode
  SYNC_WDT();

  this->reset();                   // Clear interval
  WDT->CTRL.bit.ENABLE = 1;        // Start WDT
  SYNC_WDT();

}

void Watchdog::disable() {
  WDT->CTRL.bit.ENABLE = 0;        // Stop WDT
  SYNC_WDT();
}

// Write the watchdog clear key value (0xA5) to the watchdog clear register to
// clear the watchdog timer and reset it.
void Watchdog::reset() {
  WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
  SYNC_WDT();
}

// Executes a device DSB (Data Synchronization Barrier) instruction to ensure
// all ongoing memory accesses have completed, then a WFI (Wait For Interrupt)
// instruction to place the device into the sleep mode specified until woken by
// an interrupt.
void Watchdog::sleep() {
  __DSB();
  __WFI();
}

// One time init stuff
void Watchdog::_initialize_wdt() {

  // Set clock divisor to 32 (2^(DIV+1)) for generic clock generator 2; this
  // should give a frequency of around 1024Hz
  GCLK->GENDIV.reg = GCLK_GENDIV_ID(2) | GCLK_GENDIV_DIV(4);

  // Set generic clock gen 2 to use the low power 32k osc with the configured
  // divisor
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(2)
    | GCLK_GENCTRL_GENEN
    | GCLK_GENCTRL_SRC_OSCULP32K
    | GCLK_GENCTRL_DIVSEL
    ;

  // Wait for sync - note that this is for the GCLK register and NOT the WDT
  // register
  SYNC_GCLK();

  // Associate clock with watchdog
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_WDT
    | GCLK_CLKCTRL_CLKEN
    | GCLK_CLKCTRL_GEN_GCLK2
    ;

  // Register watchdog interrupts
  NVIC_DisableIRQ(WDT_IRQn);
  NVIC_ClearPendingIRQ(WDT_IRQn);
  NVIC_SetPriority(WDT_IRQn, 0); // max prio
  NVIC_EnableIRQ(WDT_IRQn);

  // Ripped from atmel's generated code in Adafruit_ASFCore
#if (SAMD20 || SAMD21)
	/* Errata: Make sure that the Flash does not power all the way down
	 * when in sleep mode. */
	NVMCTRL->CTRLB.bit.SLEEPPRM = NVMCTRL_CTRLB_SLEEPPRM_DISABLED_Val;
#endif

  // For IDLE 0 to 3 sleep modes
  SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
  PM->SLEEP.reg = 0;

  // For standby
  //SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

  this->_is_initialized = true;

}

////////////////////////////////////////////////////////////////////////
// BEGIN C SHIT
////////////////////////////////////////////////////////////////////////

// Magical ISR name
void WDT_Handler(void) {
  WDT->CTRL.bit.ENABLE = 0;        // Disable WDT
  SYNC_GCLK();                     // Wait for sync
  WDT->INTFLAG.bit.EW = 1;         // Clear irq flag
}
