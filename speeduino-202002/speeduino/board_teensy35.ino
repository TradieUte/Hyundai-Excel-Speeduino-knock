#if defined(CORE_TEENSY) && defined(CORE_TEENSY35)
#include "board_teensy35.h"
#include "globals.h"
#include "auxiliaries.h"
#include "idle.h"
#include "scheduler.h"
#include <SPI.h>

SPISettings knockSettings(1000000, MSBFIRST, SPI_MODE1);  // SPI comms setup

void initBoard()
{
    /*
    ***********************************************************************************************************
    * General
    */

    /*
    ***********************************************************************************************************
    * Idle
    */
    if ((configPage6.iacAlgorithm == IAC_ALGORITHM_PWM_OL) || (configPage6.iacAlgorithm == IAC_ALGORITHM_PWM_CL))
    {
        //FlexTimer 2, compare channel 0 is used for idle
        FTM2_MODE |= FTM_MODE_WPDIS; // Write Protection Disable
        FTM2_MODE |= FTM_MODE_FTMEN; //Flex Timer module enable
        FTM2_MODE |= FTM_MODE_INIT;

        FTM2_SC = 0x00;      // Set this to zero before changing the modulus
        FTM2_CNTIN = 0x0000; //Shouldn't be needed, but just in case
        FTM2_CNT = 0x0000;   // Reset the count to zero
        FTM2_MOD = 0xFFFF;   // max modulus = 65535

        /*
        * Enable the clock for FTM0/1
        * 00 No clock selected. Disables the FTM counter.
        * 01 System clock
        * 10 Fixed frequency clock (32kHz)
        * 11 External clock
        */
        FTM2_SC |= FTM_SC_CLKS(0b10);

        /*
        * Trim the slow clock from 32kHz down to 31.25kHz (The slowest it will go)
        * This is somewhat imprecise and documentation is not good.
        * I poked the chip until I figured out the values associated with 31.25kHz
        */
        MCG_C3 = 0x9B;

        /*
        * Set Prescaler
        * This is the slowest that the timer can be clocked (Without used the slow timer, which is too slow). It results in ticks of 2.13333uS on the teensy 3.5:
        * 32000 Hz = F_BUS
        * 128 * 1000000uS / F_BUS = 2.133uS
        *
        * 000 = Divide by 1
        * 001 Divide by 2
        * 010 Divide by 4
        * 011 Divide by 8
        * 100 Divide by 16
        * 101 Divide by 32
        * 110 Divide by 64
        * 111 Divide by 128
        */
        FTM2_SC |= FTM_SC_PS(0b0); //No prescaler

        //Setup the channels (See Pg 1014 of K64 DS).
        FTM2_C0SC &= ~FTM_CSC_MSB; //According to Pg 965 of the K64 datasheet, this should not be needed as MSB is reset to 0 upon reset, but the channel interrupt fails to fire without it
        FTM2_C0SC |= FTM_CSC_MSA;  //Enable Compare mode
        //The below enables channel compare interrupt, but this is done in idleControl()
        //FTM2_C0SC |= FTM_CSC_CHIE;

        FTM2_C1SC &= ~FTM_CSC_MSB; //According to Pg 965 of the K64 datasheet, this should not be needed as MSB is reset to 0 upon reset, but the channel interrupt fails to fire without it
        FTM2_C1SC |= FTM_CSC_MSA;  //Enable Compare mode
        //Enable channel compare interrupt (This is currently disabled as not in use)
        //FTM2_C1SC |= FTM_CSC_CHIE;

        //Enable IRQ Interrupt
        NVIC_ENABLE_IRQ(IRQ_FTM2);
    }

    /*
    ***********************************************************************************************************
    * PI Timers
    */
    SIM_SCGC6 |= SIM_SCGC6_PIT; // enable PIT clock (60MHz)
    __asm__ volatile("nop");    // e7914, Mask 1N83J, Errata
    PIT_MCR = 0;                // enable PIT

    // Use PIT0 for 1mS interval   (free running)
    PIT_LDVAL0 = 0xEA5F; // 1mS (count down by 60,000 - 1)
    PIT_TFLG0 = 1;       // clear any interrupts
    NVIC_ENABLE_IRQ(IRQ_PIT_CH0);
    PIT_TCTRL0 |= PIT_TCTRL_TIE; // enable interrupt;
    PIT_TCTRL0 |= PIT_TCTRL_TEN; // timer 0 enable

    // Use PIT1 for Tacho duration (oneshot)
    PIT_LDVAL1 = tach_pulse_duration;
    PIT_TFLG1 = 1;               // clear any interrupts
    PIT_TCTRL1 |= PIT_TCTRL_TIE; // enable interrupt;
    NVIC_ENABLE_IRQ(IRQ_PIT_CH1);

    /*
    ***********************************************************************************************************
    * Auxilliaries
    */

    /*
    ***********************************************************************************************************
    * BOOST and VVT
    */
    if (configPage6.boostEnabled == 1 || configPage6.vvtEnabled == 1)
    {
        //FlexTimer 2, compare channel 0 is used for idle
        FTM1_MODE |= FTM_MODE_WPDIS; // Write Protection Disable
        FTM1_MODE |= FTM_MODE_FTMEN; //Flex Timer module enable
        FTM1_MODE |= FTM_MODE_INIT;

        FTM1_SC = 0x00;      // Set this to zero before changing the modulus
        FTM1_CNTIN = 0x0000; //Shouldn't be needed, but just in case
        FTM1_CNT = 0x0000;   // Reset the count to zero
        FTM1_MOD = 0xFFFF;   // max modulus = 65535

        /*
        * Enable the clock for FTM0/1
        * 00 No clock selected. Disables the FTM counter.
        * 01 System clock
        * 10 Fixed frequency clock (32kHz)
        * 11 External clock
        */
        FTM1_SC |= FTM_SC_CLKS(0b10);

        /*
        * Trim the slow clock from 32kHz down to 31.25kHz (The slowest it will go)
        * This is somewhat imprecise and documentation is not good.
        * I poked the chip until I figured out the values associated with 31.25kHz
        */
        MCG_C3 = 0x9B;

        /*
        * Set Prescaler
        * This is the slowest that the timer can be clocked (Without used the slow timer, which is too slow). It results in ticks of 2.13333uS on the teensy 3.5:
        * 32000 Hz = F_BUS
        * 128 * 1000000uS / F_BUS = 2.133uS
        *
        * 000 = Divide by 1
        * 001 Divide by 2
        * 010 Divide by 4
        * 011 Divide by 8
        * 100 Divide by 16
        * 101 Divide by 32
        * 110 Divide by 64
        * 111 Divide by 128
        */
        FTM2_SC |= FTM_SC_PS(0b0); //No prescaler

        //Setup the channels (See Pg 1014 of K64 DS).
        FTM1_C0SC &= ~FTM_CSC_MSB; //According to Pg 965 of the K64 datasheet, this should not be needed as MSB is reset to 0 upon reset, but the channel interrupt fails to fire without it
        FTM1_C0SC |= FTM_CSC_MSA;  //Enable Compare mode
        //The below enables channel compare interrupt, but this is done in idleControl()
        //FTM1_C0SC |= FTM_CSC_CHIE;

        FTM1_C1SC &= ~FTM_CSC_MSB; //According to Pg 965 of the K64 datasheet, this should not be needed as MSB is reset to 0 upon reset, but the channel interrupt fails to fire without it
        FTM1_C1SC |= FTM_CSC_MSA;  //Enable Compare mode
        //Enable channel compare interrupt (This is currently disabled as not in use)
        //FTM1_C1SC |= FTM_CSC_CHIE;

        //Enable IRQ Interrupt
        NVIC_ENABLE_IRQ(IRQ_FTM1);

        boost_pwm_max_count = 1000000L / (32 * configPage6.boostFreq * 2); //Converts the frequency in Hz to the number of ticks (at 16uS) it takes to complete 1 cycle. Note that the frequency is divided by 2 coming from TS to allow for up to 512hz
        vvt_pwm_max_count = 1000000L / (32 * configPage6.vvtFreq * 2);     //Converts the frequency in Hz to the number of ticks (at 16uS) it takes to complete 1 cycle. Note that the frequency is divided by 2 coming from TS to allow for up to 512hz

    }

    /*
    ***********************************************************************************************************
    * Schedules
    */
/*
  //FlexTimer 0 is used for 4 ignition and 4 injection schedules. There are 8 channels on this module, so no other timers are needed
  FTM0_SC = 0x00;      // Set this to zero before changing the modulus
  FTM0_MOD = 0xFFFF;   //max modulus = 65535

  //FlexTimer 3 is used for schedules on channel 5+. Currently only channel 5 is used, but will likely be expanded later
  FTM3_SC = 0x00;      // Set this to zero before changing the modulus
  FTM3_MOD = 0xFFFF;   //max modulus = 65535

  *
    * Enable the clock for FTM0/1
    * 00 No clock selected. Disables the FTM counter.
    * 01 System clock
    * 10 Fixed frequency clock
    * 11 External clock
    *
  FTM0_SC |= FTM_SC_CLKS(0b1);
  FTM3_SC |= FTM_SC_CLKS(0b1);

  *
    * Set Prescaler
    * This is the slowest that the timer can be clocked (Without used the slow timer, which is too slow). It results in ticks of 2.13333uS on the teensy 3.5:
    * 60000000 Hz = F_BUS
    * 128 * 1000000uS / F_BUS = 2.133uS
    *
    * 000 = Divide by 1
    * 001 Divide by 2
    * 010 Divide by 4
    * 011 Divide by 8
    * 100 Divide by 16
    * 101 Divide by 32
    * 110 Divide by 64
    * 111 Divide by 128
    *
  FTM0_SC |= FTM_SC_PS(0b111);
  FTM3_SC |= FTM_SC_PS(0b111);

  FTM0_C0SC = 0; 
  FTM0_C0SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

  FTM0_C1SC = 0; 
  FTM0_C1SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

  FTM0_C2SC = 0; 
  FTM0_C2SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

  FTM0_C3SC = 0; 
  FTM0_C3SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

  FTM0_C4SC = 0; 
  FTM0_C4SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

  FTM0_C5SC = 0; 
  FTM0_C5SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

  FTM0_C6SC = 0; 
  FTM0_C6SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

  FTM0_C7SC = 0; 
  FTM0_C7SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

  NVIC_ENABLE_IRQ(IRQ_FTM0);
  //increase interrupt priority (default 128, h/w serial 64, systick 0)
  NVIC_SET_PRIORITY(IRQ_FTM0, 48); // higher priority than comms

#if ( (INJ_CHANNELS>=5)&&(IGN_CHANNELS>=5) )
  FTM3_C0SC = 0; 
  FTM3_C0SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

  FTM3_C1SC = 0; 
  FTM3_C1SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

  FTM3_C2SC = 0; 
  FTM3_C2SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

  FTM3_C3SC = 0; 
  FTM3_C3SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

  FTM3_C4SC = 0; 
  FTM3_C4SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

  FTM3_C5SC = 0; 
  FTM3_C5SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

  FTM3_C6SC = 0; 
  FTM3_C6SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

  FTM3_C7SC = 0; 
  FTM3_C7SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

  NVIC_ENABLE_IRQ(IRQ_FTM3);
  NVIC_SET_PRIORITY(IRQ_FTM3, 48);
#endif
*/
    //FlexTimer 0 is used for 4 ignition and 4 injection schedules. There are 8 channels on this module, so no other timers are needed
    FTM0_MODE |= FTM_MODE_WPDIS; //Write Protection Disable
    FTM0_MODE |= FTM_MODE_FTMEN; //Flex Timer module enable
    FTM0_MODE |= FTM_MODE_INIT;

    FTM0_SC = 0x00;      // Set this to zero before changing the modulus
    FTM0_CNTIN = 0x0000; //Shouldn't be needed, but just in case
    FTM0_CNT = 0x0000;   //Reset the count to zero
    FTM0_MOD = 0xFFFF;   //max modulus = 65535

    //FlexTimer 3 is used for schedules on channel 5+. Currently only channel 5 is used, but will likely be expanded later
    FTM3_MODE |= FTM_MODE_WPDIS; //Write Protection Disable
    FTM3_MODE |= FTM_MODE_FTMEN; //Flex Timer module enable
    FTM3_MODE |= FTM_MODE_INIT;

    FTM3_SC = 0x00;      // Set this to zero before changing the modulus
    FTM3_CNTIN = 0x0000; //Shouldn't be needed, but just in case
    FTM3_CNT = 0x0000;   //Reset the count to zero
    FTM3_MOD = 0xFFFF;   //max modulus = 65535

    /*
    * Enable the clock for FTM0/1
    * 00 No clock selected. Disables the FTM counter.
    * 01 System clock
    * 10 Fixed frequency clock
    * 11 External clock
    */
    FTM0_SC |= FTM_SC_CLKS(0b1);
    FTM3_SC |= FTM_SC_CLKS(0b1);

    /*
    * Set Prescaler
    * This is the slowest that the timer can be clocked (Without used the slow timer, which is too slow). It results in ticks of 2.13333uS on the teensy 3.5:
    * 60000000 Hz = F_BUS
    * 128 * 1000000uS / F_BUS = 2.133uS
    *
    * 000 = Divide by 1
    * 001 Divide by 2
    * 010 Divide by 4
    * 011 Divide by 8
    * 100 Divide by 16
    * 101 Divide by 32
    * 110 Divide by 64
    * 111 Divide by 128
    */
    FTM0_SC |= FTM_SC_PS(0b111);
    FTM3_SC |= FTM_SC_PS(0b111);

    //Setup the channels (See Pg 1014 of K64 DS).
    //The are probably not needed as power on state should be 0
    //FTM0_C0SC &= ~FTM_CSC_ELSB;
    //FTM0_C0SC &= ~FTM_CSC_ELSA;
    //FTM0_C0SC &= ~FTM_CSC_DMA;
    FTM0_C0SC &= ~FTM_CSC_MSB; //According to Pg 965 of the K64 datasheet, this should not be needed as MSB is reset to 0 upon reset, but the channel interrupt fails to fire without it
    FTM0_C0SC |= FTM_CSC_MSA;  //Enable Compare mode
    FTM0_C0SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

    FTM0_C1SC &= ~FTM_CSC_MSB; //According to Pg 965 of the datasheet, this should not be needed as MSB is reset to 0 upon reset, but the channel interrupt fails to fire without it
    FTM0_C1SC |= FTM_CSC_MSA;  //Enable Compare mode
    FTM0_C1SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

    FTM0_C2SC &= ~FTM_CSC_MSB; //According to Pg 965 of the datasheet, this should not be needed as MSB is reset to 0 upon reset, but the channel interrupt fails to fire without it
    FTM0_C2SC |= FTM_CSC_MSA;  //Enable Compare mode
    FTM0_C2SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

    FTM0_C3SC &= ~FTM_CSC_MSB; //According to Pg 965 of the datasheet, this should not be needed as MSB is reset to 0 upon reset, but the channel interrupt fails to fire without it
    FTM0_C3SC |= FTM_CSC_MSA;  //Enable Compare mode
    FTM0_C3SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

    FTM0_C4SC &= ~FTM_CSC_MSB; //According to Pg 965 of the datasheet, this should not be needed as MSB is reset to 0 upon reset, but the channel interrupt fails to fire without it
    FTM0_C4SC |= FTM_CSC_MSA;  //Enable Compare mode
    FTM0_C4SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

    FTM0_C5SC &= ~FTM_CSC_MSB; //According to Pg 965 of the datasheet, this should not be needed as MSB is reset to 0 upon reset, but the channel interrupt fails to fire without it
    FTM0_C5SC |= FTM_CSC_MSA;  //Enable Compare mode
    FTM0_C5SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

    FTM0_C6SC &= ~FTM_CSC_MSB; //According to Pg 965 of the datasheet, this should not be needed as MSB is reset to 0 upon reset, but the channel interrupt fails to fire without it
    FTM0_C6SC |= FTM_CSC_MSA;  //Enable Compare mode
    FTM0_C6SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

    FTM0_C7SC &= ~FTM_CSC_MSB; //According to Pg 965 of the datasheet, this should not be needed as MSB is reset to 0 upon reset, but the channel interrupt fails to fire without it
    FTM0_C7SC |= FTM_CSC_MSA;  //Enable Compare mode
    FTM0_C7SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

    //Do the same, but on flex timer 3 (Used for channels 5-8)
    FTM3_C0SC &= ~FTM_CSC_MSB; //According to Pg 965 of the K64 datasheet, this should not be needed as MSB is reset to 0 upon reset, but the channel interrupt fails to fire without it
    FTM3_C0SC |= FTM_CSC_MSA;  //Enable Compare mode
    FTM3_C0SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

    FTM3_C1SC &= ~FTM_CSC_MSB; //According to Pg 965 of the K64 datasheet, this should not be needed as MSB is reset to 0 upon reset, but the channel interrupt fails to fire without it
    FTM3_C1SC |= FTM_CSC_MSA;  //Enable Compare mode
    FTM3_C1SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

    FTM3_C2SC &= ~FTM_CSC_MSB; //According to Pg 965 of the K64 datasheet, this should not be needed as MSB is reset to 0 upon reset, but the channel interrupt fails to fire without it
    FTM3_C2SC |= FTM_CSC_MSA;  //Enable Compare mode
    FTM3_C2SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

    FTM3_C3SC &= ~FTM_CSC_MSB; //According to Pg 965 of the K64 datasheet, this should not be needed as MSB is reset to 0 upon reset, but the channel interrupt fails to fire without it
    FTM3_C3SC |= FTM_CSC_MSA;  //Enable Compare mode
    FTM3_C3SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

    FTM3_C4SC &= ~FTM_CSC_MSB; //According to Pg 965 of the K64 datasheet, this should not be needed as MSB is reset to 0 upon reset, but the channel interrupt fails to fire without it
    FTM3_C4SC |= FTM_CSC_MSA;  //Enable Compare mode
    FTM3_C4SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

    FTM3_C5SC &= ~FTM_CSC_MSB; //According to Pg 965 of the K64 datasheet, this should not be needed as MSB is reset to 0 upon reset, but the channel interrupt fails to fire without it
    FTM3_C5SC |= FTM_CSC_MSA;  //Enable Compare mode
    FTM3_C5SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

    FTM3_C6SC &= ~FTM_CSC_MSB; //According to Pg 965 of the K64 datasheet, this should not be needed as MSB is reset to 0 upon reset, but the channel interrupt fails to fire without it
    FTM3_C6SC |= FTM_CSC_MSA;  //Enable Compare mode
    FTM3_C6SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

    FTM3_C7SC &= ~FTM_CSC_MSB; //According to Pg 965 of the K64 datasheet, this should not be needed as MSB is reset to 0 upon reset, but the channel interrupt fails to fire without it
    FTM3_C7SC |= FTM_CSC_MSA;  //Enable Compare mode
    FTM3_C7SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

    // enable IRQ Interrupt
    NVIC_ENABLE_IRQ(IRQ_FTM0);
    NVIC_ENABLE_IRQ(IRQ_FTM3);
}

uint16_t freeRam()
{
    uint32_t stackTop;
    uint32_t heapTop;

    // current position of the stack.
    stackTop = (uint32_t)&stackTop;

    // current position of heap.
    void *hTop = malloc(1);
    heapTop = (uint32_t)hTop;
    free(hTop);

    // The difference is the free, available ram.
    return (uint16_t)stackTop - heapTop;
}

/*
void initialiseKnock()
{
  if ((configPage10.knock_mode == KNOCK_MODE_DIGITAL))
  {
    // do this here once, rather than in pit3_isr
    knock_threshold = map(configPage10.knock_threshold, 0, 50, 0, 1024); // T/S (0-50)/10 Volt; TPIC8101 0 -1024

    // setup SPI for knock detector TPIC8101 - NB the chip uses CS to load values to internal registers
    uint8_t ret;
#if !defined(PCB_FLASH)    
    SPI.setSCK(SCK0); // alternate clock pin
    SPI.begin();      // init CPU hardware
#endif
    SPI.beginTransaction(knockSettings);
    ret = sendCmd(PS_SDO_STAT_CMD); // set prescaler for 8MHz input; SDO active
    if (ret == PS_SDO_STAT_CMD)     // if not already in advanced mode, continue
    {
      sendCmd(CHAN1_SEL_CMD);                               // select channel 1
      sendCmd(BPCF_CMD | configPage10.band_pass_frequency); // set bandpassCenterFrequency
      sendCmd(INT_GAIN_CMD | rpmModGain);                   // set gain
      sendCmd(INT_TC_CMD | integrator_time_constant);       // set integrationTimeConstant
      sendCmd(ADV_MODE_CMD);                                // set advanced mode - allows receiving integrator data - clearing advanced mode requires power off
    }
    SPI.endTransaction();

    // Use PIT2 for Knock window start (oneshot)
    PIT_TFLG2 = 1;               // clear any interrupts
    PIT_TCTRL2 |= PIT_TCTRL_TIE; // enable interrupt;
    NVIC_ENABLE_IRQ(IRQ_PIT_CH2);

    // Use PIT3 for Knock window duration (oneshot)
    PIT_TFLG3 = 1;               // clear any interrupts
    PIT_TCTRL3 |= PIT_TCTRL_TIE; // enable interrupt;
    NVIC_ENABLE_IRQ(IRQ_PIT_CH3);
  }
  // KNOCK_MODE_ANALOG - nothing here yet
}
*/

void pit0_isr() // free running one mS timer
{
  PIT_TFLG0 = 1; // clear interrupt - loads timer value, restarts timer countdown
  oneMSInterval();
}

void pit1_isr() // Tach pulse end (oneshot)
{
  PIT_TCTRL1 &= ~PIT_TCTRL_TEN; // disable PIT1
  PIT_TFLG1 = 1;                // clear interrupt flag - reloads (fixed) countdown value from PIT_LDVAL1
  TACHO_PULSE_LOW();
}
/*
// PIT2 (oneshot) determines the time between ignition pulse end and knock window start
// when PIT2 interrupts, start the knock window duration timer PIT3
void pit2_isr() // window start delay end (oneshot)
{
  PIT_TCTRL2 &= ~PIT_TCTRL_TEN; // stop PIT2
  PIT_TFLG2 = 1;                // clear interrupt flag - reloads countdown value from PIT_LDVAL2
  // start knock window duration timer PIT3
  PIT_LDVAL3 = 10000;
  PIT_TCTRL3 |= PIT_TCTRL_TEN; // start PIT3
  OPEN_KNOCK_WINDOW();
}

void pit3_isr() // knock window end (oneshot)
{
  byte lowByte;
  byte highByte;
  int knockValue = 0;

  PIT_TCTRL3 &= ~PIT_TCTRL_TEN; // stop PIT3
  PIT_TFLG3 = 1;                // clear interrupt flag - reloads countdown value from PIT_LDVAL3
  CLOSE_KNOCK_WINDOW();
  //delayMicroseconds(1);
  // get knock Value - takes 16 uSec with 2MHz clock
  SPI.beginTransaction(knockSettings);
  CS0_ASSERT();  
  sendCmd(REQUEST_LOW_BYTE);  // also sets prescaler
  lowByte = sendCmd(REQUEST_HIGH_BYTE); // also sets the channel
  // rpmModGain and integrator_time_constant set once per second in main loop
  highByte = sendCmd(INT_GAIN_CMD | rpmModGain); //  refresh gain - changes with rpm
  sendCmd(INT_TC_CMD | integrator_time_constant); // refresh itc - changes with rpm
  SPI.endTransaction();
  knockValue = (knockValue | highByte) << 2; // already shifted 6 bits
  knockValue |= lowByte;
  if (knockValue > knock_threshold)
  {
    knockCounter++;
  }
}

uint8_t sendCmd(uint8_t cmd)
{
  uint8_t val = 0;
  CS0_ASSERT();
  val = SPI.transfer(cmd);
  CS0_RELEASE();
  return (val);
}


byte getClosestIndex(int val, int array[], byte sz)
{
  // binary search
  int i = 0;
  int j = sz;
  int k = 0;
  while (i < j)
  {
    k = (i + j) / 2;     // middle of search range
    if (array[k] == val) // best hope
    {
      return (k); // return matched value index
    }

    if (val < array[k]) // search left of array[k]
    {
      // integrator_time_constant is the index to the internal TCIP8101 tc table

      if (k > 0 && val > array[k - 1])
      {
        byte idx = closestIndex(k - 1, k, array, val);
        return (idx);
      }
      j = k;
    }
    else  // search right of array[k]
    {
      if ((k < sz - 1) && (val < array[k + 1]))
      {
        byte idx = closestIndex(k, k + 1, array, val);
        return (idx);
      }
      i = k + 1;
    }
  }
  return (k);
}

byte closestIndex(int idx1, int idx2, int array[], int reqVal)
{
  if ((reqVal - array[idx1]) >= (array[idx2] - reqVal))
  {
    return (idx1);
  }
  else
  {
    return (idx2);
  }
}

// called by each ignition isr at the end of the ign pulse
static inline void launchKnockWindow()
{
  PIT_LDVAL2 = knockWindowStartDelay;
  PIT_TFLG2 = 1;               // clear interrupt flag
  PIT_TCTRL2 |= PIT_TCTRL_TEN; // start timer
}
*/

static inline void startTacho(void) // skip pulses if required
{
  if (skip_factor == 0)
  {
    skip_factor = configPage2.tachoDiv;
    TACHO_PULSE_HIGH();
    PIT_LDVAL1 = tach_pulse_duration;
    PIT_TCTRL1 |= PIT_TCTRL_TEN; // start PIT1
  }
  else
  {
    skip_factor--;
  }
}

#endif
