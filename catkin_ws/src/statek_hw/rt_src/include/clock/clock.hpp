#pragma once

void clockStart(void); //!< Start the clock.
unsigned long  clockNow(void); //!< Get time since start of the clock in ms. Valid for short periods of time (minutes max).