#ifndef _BEEPFUNC_H
#define _BEEPFUNC_H
#include "config.h"
#include "Arduino.h"

#include <ESP32Tone.h>

extern boolean NoBeep;
extern const int pinSpeaker;
extern int beepingFrequency;
extern int pos;

extern void beepAltitude(long altitude);
extern void beginBeepSeq();
extern void longBeep();
extern void shortBeep();
extern void beepAltiVersion (int majorNbr, int minorNbr);
extern void longBeepRepeat( int digit );
extern void shortBeepRepeat( int digit );
extern void beepAltitudeNew( long value);

#endif
