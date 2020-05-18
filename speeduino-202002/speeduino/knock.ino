#include "knock.h"
#include "globals.h"
#include <SPI.h>


void initialiseKnock()
{
  if ((configPage10.knock_mode == KNOCK_MODE_DIGITAL))
  {
    // do this here once, rather than in pit3_isr
    knock_threshold = map(configPage10.knock_threshold, 0, 50, 0, 1024); // T/S (0-50)/10 Volt; TPIC8101 0 -1024

    // setup SPI for knock detector TPIC8101 - NB the chip uses CS to load values to internal registers
    uint8_t ret;
    SPI.setSCK(SCK0); // alternate clock pin
    SPI.begin();      // init CPU hardware
    knockSettings = SPISettings(1000000, MSBFIRST, SPI_MODE1);
    SPI.beginTransaction(knockSettings);
    ret = sendCmd(PS_SDO_STAT_CMD); // set prescaler for 8MHz input; SDO active
    if (ret == PS_SDO_STAT_CMD)     // if not already in advanced mode, continue
    {
      sendCmd(CHAN1_SEL_CMD);                               // select channel 1
//      sendCmd(BPCF_CMD | configPage10.band_pass_frequency); // set bandpassCenterFrequency
      sendCmd(BPCF_CMD | BAND_PASS_FREQUENCY); // set bandpassCenterFrequency
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
  else
  {
    // KNOCK_MODE_ANALOG - nothing here yet
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

void refreshKnockParameters()
{
  int integratorGain = (608/KNOCK_SENSOR_OUTPUT);  // x 10e-1; where knock_sensor_output is in mV
//  int integratorGain = (608/configPage10.knock_sensor_output);  // x 10e-1; where knock_sensor_output is in mV
    // 608, is reduction of TCIP8101 Ap value. (actually 0.06076), knockSensorSensitivity is in mV.
      
  // Reduce gain with RPM, to compensate for increasing engine noise
//  knockWindowGainFactor = table2D_getValue(&knockWindowGainTable, currentStatus.RPM/100); // 100 to 0 (%)
    knockWindowGainFactor = KNOCK_GAIN; // table yet to be included
  integratorGain = integratorGain * knockWindowGainFactor;  // integratorGain is 10e1 high and knockWindowGainFactor is 10e2 high; total 10e3, which matches multiplier in TPIC8101 gain array
  // rpmModGain is the index to the internal TCIP8101 gain table
  rpmModGain = 63 - getClosestIndex(integratorGain, gainK, (byte)64); // correct for inverted gainK array
  // Calculate integrator_time_constant - sticking with int arithmetic.
  // knockWindowSize*1000000000/6/rpm/28.274; (28.274 = 2x Pi x Vout);  (knockWindowSize in deg)
  // reducing further: knockWindowSize * 5895 / rpm
  int itc = knockWindowSize * 5895 / currentStatus.RPM;  // uSec
  integrator_time_constant = getClosestIndex(itc, timeConst, (byte)32); // timeConst[]
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

