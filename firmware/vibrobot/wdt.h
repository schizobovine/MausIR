//
// wdt.cpp - Watchdog shit for the SAMD
//

#ifndef _WDT_H
#define _WDT_H

#define SYNC_WDT()  do {} while (WDT->STATUS.bit.SYNCBUSY)
#define SYNC_GCLK() do {} while (GCLK->STATUS.bit.SYNCBUSY)

class Watchdog {
  public:
    Watchdog(): _is_initialized(false) {}
    void enable();
    void disable();
    void reset();
    void sleep();

  private:
    void _initialize_wdt();
    bool _is_initialized;
};

#endif
