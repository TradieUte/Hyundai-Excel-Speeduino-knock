#ifndef TEENSY35_H
#define TEENSY35_H
#if defined(CORE_TEENSY) && defined(CORE_TEENSY35)

/*
***********************************************************************************************************
* General
*/
  void initBoard();
  uint16_t freeRam();
  #define PORT_TYPE uint8_t //Size of the port variables
  #define PINMASK_TYPE uint8_t
  #define COMPARE_TYPE uint16_t
  #define COUNTER_TYPE uint16_t
  #define BOARD_DIGITAL_GPIO_PINS 34
  #define BOARD_NR_GPIO_PINS 34
  #define USE_SERIAL3
  #ifdef USE_SPI_EEPROM
    #define EEPROM_LIB_H "src/SPIAsEEPROM/SPIAsEEPROM.h"
  #else
    #define EEPROM_LIB_H <EEPROM.h>
  #endif

  #define micros_safe() micros() //timer5 method is not used on anything but AVR, the micros_safe() macro is simply an alias for the normal micros()

/*
***********************************************************************************************************
* Schedules
*/
  //shawnhymel.com/661/learning-the-teensy-lc-interrupt-service-routines/
  #define FUEL1_COUNTER FTM0_CNT
  #define FUEL2_COUNTER FTM0_CNT
  #define FUEL3_COUNTER FTM0_CNT
  #define FUEL4_COUNTER FTM0_CNT
  #define FUEL5_COUNTER FTM3_CNT
  #define FUEL6_COUNTER FTM3_CNT
  #define FUEL7_COUNTER FTM3_CNT
  #define FUEL8_COUNTER FTM3_CNT

  #define IGN1_COUNTER  FTM0_CNT
  #define IGN2_COUNTER  FTM0_CNT
  #define IGN3_COUNTER  FTM0_CNT
  #define IGN4_COUNTER  FTM0_CNT
  #define IGN5_COUNTER  FTM3_CNT
  #define IGN6_COUNTER  FTM3_CNT
  #define IGN7_COUNTER  FTM3_CNT
  #define IGN8_COUNTER  FTM3_CNT

  #define FUEL1_COMPARE FTM0_C0V
  #define FUEL2_COMPARE FTM0_C1V
  #define FUEL3_COMPARE FTM0_C2V
  #define FUEL4_COMPARE FTM0_C3V
  #define FUEL5_COMPARE FTM3_C0V
  #define FUEL6_COMPARE FTM3_C1V
  #define FUEL7_COMPARE FTM3_C2V
  #define FUEL8_COMPARE FTM3_C3V

  #define IGN1_COMPARE  FTM0_C4V
  #define IGN2_COMPARE  FTM0_C5V
  #define IGN3_COMPARE  FTM0_C6V
  #define IGN4_COMPARE  FTM0_C7V
  #define IGN5_COMPARE  FTM3_C4V
  #define IGN6_COMPARE  FTM3_C5V
  #define IGN7_COMPARE  FTM3_C6V
  #define IGN8_COMPARE  FTM3_C7V

  #define FUEL1_TIMER_ENABLE() FTM0_C0SC |= FTM_CSC_CHIE //Write 1 to the CHIE (Channel Interrupt Enable) bit of channel 0 Status/Control
  #define FUEL2_TIMER_ENABLE() FTM0_C1SC |= FTM_CSC_CHIE
  #define FUEL3_TIMER_ENABLE() FTM0_C2SC |= FTM_CSC_CHIE
  #define FUEL4_TIMER_ENABLE() FTM0_C3SC |= FTM_CSC_CHIE
  #define FUEL5_TIMER_ENABLE() FTM3_C0SC |= FTM_CSC_CHIE
  #define FUEL6_TIMER_ENABLE() FTM3_C1SC |= FTM_CSC_CHIE
  #define FUEL7_TIMER_ENABLE() FTM3_C2SC |= FTM_CSC_CHIE
  #define FUEL8_TIMER_ENABLE() FTM3_C3SC |= FTM_CSC_CHIE

  #define FUEL1_TIMER_DISABLE() FTM0_C0SC &= ~FTM_CSC_CHIE //Write 0 to the CHIE (Channel Interrupt Enable) bit of channel 0 Status/Control
  #define FUEL2_TIMER_DISABLE() FTM0_C1SC &= ~FTM_CSC_CHIE
  #define FUEL3_TIMER_DISABLE() FTM0_C2SC &= ~FTM_CSC_CHIE
  #define FUEL4_TIMER_DISABLE() FTM0_C3SC &= ~FTM_CSC_CHIE
  #define FUEL5_TIMER_DISABLE() FTM3_C0SC &= ~FTM_CSC_CHIE //Write 0 to the CHIE (Channel Interrupt Enable) bit of channel 0 Status/Control
  #define FUEL6_TIMER_DISABLE() FTM3_C1SC &= ~FTM_CSC_CHIE
  #define FUEL7_TIMER_DISABLE() FTM3_C2SC &= ~FTM_CSC_CHIE
  #define FUEL8_TIMER_DISABLE() FTM3_C3SC &= ~FTM_CSC_CHIE

  #define IGN1_TIMER_ENABLE() FTM0_C4SC |= FTM_CSC_CHIE
  #define IGN2_TIMER_ENABLE() FTM0_C5SC |= FTM_CSC_CHIE
  #define IGN3_TIMER_ENABLE() FTM0_C6SC |= FTM_CSC_CHIE
  #define IGN4_TIMER_ENABLE() FTM0_C7SC |= FTM_CSC_CHIE
  #define IGN5_TIMER_ENABLE() FTM3_C4SC |= FTM_CSC_CHIE
  #define IGN6_TIMER_ENABLE() FTM3_C5SC |= FTM_CSC_CHIE
  #define IGN7_TIMER_ENABLE() FTM3_C6SC |= FTM_CSC_CHIE
  #define IGN8_TIMER_ENABLE() FTM3_C7SC |= FTM_CSC_CHIE

  #define IGN1_TIMER_DISABLE() FTM0_C4SC &= ~FTM_CSC_CHIE
  #define IGN2_TIMER_DISABLE() FTM0_C5SC &= ~FTM_CSC_CHIE
  #define IGN3_TIMER_DISABLE() FTM0_C6SC &= ~FTM_CSC_CHIE
  #define IGN4_TIMER_DISABLE() FTM0_C7SC &= ~FTM_CSC_CHIE
  #define IGN5_TIMER_DISABLE() FTM3_C4SC &= ~FTM_CSC_CHIE
  #define IGN6_TIMER_DISABLE() FTM3_C5SC &= ~FTM_CSC_CHIE
  #define IGN7_TIMER_DISABLE() FTM3_C6SC &= ~FTM_CSC_CHIE
  #define IGN8_TIMER_DISABLE() FTM3_C7SC &= ~FTM_CSC_CHIE

  #define MAX_TIMER_PERIOD 139808 // 2.13333333uS * 65535
  #define uS_TO_TIMER_COMPARE(uS) ((uS * 15) >> 5) //Converts a given number of uS into the required number of timer ticks until that time has passed.

/*
***********************************************************************************************************
* Auxilliaries
*/
  #define ENABLE_BOOST_TIMER()  FTM1_C0SC |= FTM_CSC_CHIE
  #define DISABLE_BOOST_TIMER() FTM1_C0SC &= ~FTM_CSC_CHIE

  #define ENABLE_VVT_TIMER()    FTM1_C1SC |= FTM_CSC_CHIE
  #define DISABLE_VVT_TIMER()   FTM1_C1SC &= ~FTM_CSC_CHIE

  #define BOOST_TIMER_COMPARE   FTM1_C0V
  #define BOOST_TIMER_COUNTER   FTM1_CNT
  #define VVT_TIMER_COMPARE     FTM1_C1V
  #define VVT_TIMER_COUNTER     FTM1_CNT

  static inline void boostInterrupt();
  static inline void vvtInterrupt();

/*
***********************************************************************************************************
* Idle
*/
  #define IDLE_COUNTER FTM2_CNT
  #define IDLE_COMPARE FTM2_C0V

  #define IDLE_TIMER_ENABLE() FTM2_C0SC |= FTM_CSC_CHIE
  #define IDLE_TIMER_DISABLE() FTM2_C0SC &= ~FTM_CSC_CHIE

  static inline void idleInterrupt();

/*
***********************************************************************************************************
* CAN / Second serial
*/
  //Uart CANSerial (&sercom3, 0, 1, SERCOM_RX_PAD_1, UART_TX_PAD_0);
  
/*
  New tacho
 */
static inline void startTacho(void);  // includes skip setup
static inline void start_tacho(void); // does skip if required

/*
  Knock Stuff
 */
static inline void launchKnockWindow();
void initialiseKnock();
uint8_t sendCmd(uint8_t);
//static inline void getKnockValue();
volatile int knock_threshold = 0;
volatile byte rpmModGain = 0; // gain as a function of rpm
volatile bool knockRecoveryFirstStepDelay = false;

// SPI commands for TPIC8101
#define PS_SDO_STAT_CMD  0b01000110 // 8MHz in, SDO active
#define CHAN1_SEL_CMD    0b11100000 // channel 1
#define CHAN2_SEL_CMD    0b11100001 // channel 2
#define BPCF_CMD         0b00000000 // band pass center frequency SPI cmd
#define INT_GAIN_CMD     0b10000000 // Integrator gain
#define INT_TC_CMD       0b11000000 // Integrator Time constant
#define ADV_MODE_CMD     0b01110001 // switch to advanced mode
#define REQUEST_LOW_BYTE     0b01000110 // (when in advanced mode)
#define REQUEST_HIGH_BYTE    0b11100000 // (when in advanced mode)

// TPIC8101 values
int timeConst[] = {40,45,50,55,60,65,70,75,80,90,100,110,120,130,140,150,160,180,200,220,240,260,280,300,320,360,400,440,480,520,560,600};
// the gainK array is inverted from that required by TPIC8101 to allow both timeConst and gainK to be searched with same routine. The array index of gainK from the search is adjusted when calculating rpmModGain.
int gainK[] = {111,118,125,129,133,138,143,148,154,160,167,174,182,190,200,211,222,236,250,258,267,276,286,296,308,320,333,348,364,381,400,421,444,471,500,548,567,586,607,630,654,680,708,739,773,810,850,895,944,1000,1063,1143,1185,1231,1280,1333,1391,1455,1523,1600,1684,1778,1882,2000};
byte getClosestIndex(int, int [], byte);
byte closestIndex(int, int, int [], int);

int16_t knockWindowSize = 0;  //The current crank angle delta that defines the knock window duration
int16_t knockWindowDelay = 0; //The current crank angle after end of ign dwell for a knock pulse to be valid
int16_t knockWindowGainFactor = 0;  // to control the sensor sensitivity as rpm (noise) increases
volatile byte integrator_time_constant = 0;

#endif //CORE_TEENSY
#endif //TEENSY35_H