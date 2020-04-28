#include "scheduledIO.h"
#include "scheduler.h"
#include "globals.h"
#include "timers.h"

// changes for new tacho
#if defined(CORE_TEENSY)
#define TACHO_OUTPUT_FLAG_READY
#else
#undef TACHO_OUTPUT_FLAG_READY
#define TACHO_OUTPUT_FLAG_READY tachoOutputFlag = READY
#endif


inline void beginCoil1Charge() { digitalWrite(pinCoil1, coilHIGH); TACHO_OUTPUT_FLAG_READY; }
inline void endCoil1Charge() { digitalWrite(pinCoil1, coilLOW); }

inline void beginCoil2Charge() { digitalWrite(pinCoil2, coilHIGH); TACHO_OUTPUT_FLAG_READY; }
inline void endCoil2Charge() { digitalWrite(pinCoil2, coilLOW); }

inline void beginCoil3Charge() { digitalWrite(pinCoil3, coilHIGH); TACHO_OUTPUT_FLAG_READY; }
inline void endCoil3Charge() { digitalWrite(pinCoil3, coilLOW); }

inline void beginCoil4Charge() { digitalWrite(pinCoil4, coilHIGH); TACHO_OUTPUT_FLAG_READY; }
inline void endCoil4Charge() { digitalWrite(pinCoil4, coilLOW); }

inline void beginCoil5Charge() { digitalWrite(pinCoil5, coilHIGH); TACHO_OUTPUT_FLAG_READY; }
inline void endCoil5Charge() { digitalWrite(pinCoil5, coilLOW); }

inline void beginCoil6Charge() { digitalWrite(pinCoil6, coilHIGH); TACHO_OUTPUT_FLAG_READY; }
inline void endCoil6Charge() { digitalWrite(pinCoil6, coilLOW); }

inline void beginCoil7Charge() { digitalWrite(pinCoil7, coilHIGH); TACHO_OUTPUT_FLAG_READY; }
inline void endCoil7Charge() { digitalWrite(pinCoil7, coilLOW); }

inline void beginCoil8Charge() { digitalWrite(pinCoil8, coilHIGH); TACHO_OUTPUT_FLAG_READY; }
inline void endCoil8Charge() { digitalWrite(pinCoil8, coilLOW); }

//The below 3 calls are all part of the rotary ignition mode
inline void beginTrailingCoilCharge() { digitalWrite(pinCoil2, coilHIGH); }
inline void endTrailingCoilCharge1() { digitalWrite(pinCoil2, coilLOW); *ign3_pin_port |= ign3_pin_mask; } //Sets ign3 (Trailing select) high
inline void endTrailingCoilCharge2() { digitalWrite(pinCoil2, coilLOW); *ign3_pin_port &= ~(ign3_pin_mask); } //sets ign3 (Trailing select) low

//As above but for ignition (Wasted COP mode)
void beginCoil1and3Charge() { digitalWrite(pinCoil1, coilHIGH); digitalWrite(pinCoil3, coilHIGH); TACHO_OUTPUT_FLAG_READY; }
void endCoil1and3Charge()   { digitalWrite(pinCoil1, coilLOW);  digitalWrite(pinCoil3, coilLOW);  }
void beginCoil2and4Charge() { digitalWrite(pinCoil2, coilHIGH); digitalWrite(pinCoil4, coilHIGH); TACHO_OUTPUT_FLAG_READY; }
void endCoil2and4Charge()   { digitalWrite(pinCoil2, coilLOW);  digitalWrite(pinCoil4, coilLOW);  }

void nullCallback() { return; }
