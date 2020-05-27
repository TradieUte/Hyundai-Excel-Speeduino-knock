#if defined (KNOCK)
#include "knock.h"
#include "globals.h"
#include "sensors.h"
#include <SPI.h>

// two driving functions are required to use this code.
// the first function runs the macro OPEN_KNOCK_WINDOW at the correct time following each ignition event
// the second function calls getKnockValue() at the end of each knock window. (CLOSE_KNOCK_WINDOW is embedded)
// initialiseKnock() is called once, during program initialisation.

void initialiseKnock()
{
  if ((configPage10.knock_mode == KNOCK_MODE_DIGITAL))  // TPIC8101 setup
  {
    // Tuner Studio scales band pass freq down by 100 and bpFreq array scales down by 10, so need to scale up by 10
    band_pass_frequency_idx = getClosestIndex(configPage10.band_pass_frequency*10, bpFreq, (uint8_t)64); 
    knock_threshold = map(configPage10.knock_threshold, 0, 50, 0, 1024); // T/S (0-50)/10 Volt; TPIC8101 0 -1024
    // reduction of TPIC8101 gain equation, gives Ap = 60.764/Vin (Vin in mVp), or 121.53 x Vin (with Vin mVpp)
    integratorGain = 121530 / configPage10.knock_sensor_output; // sensor out in mVpp scaled to Vpp
    // setup SPI for knock detector TPIC8101 - NB the chip uses CS to load values to internal registers
    uint8_t ret;
    SPI.setSCK(SCK0); // alternate clock pin
    SPI.begin();      // init CPU hardware
    knockSettings = SPISettings(1000000, MSBFIRST, SPI_MODE1);
    SPI.beginTransaction(knockSettings);
    ret = sendCmd(PS_SDO_STAT_CMD); // set prescaler for 8MHz input; SDO active
    if (ret == PS_SDO_STAT_CMD)     // if not already in advanced mode, continue
    {
      sendCmd(CHAN1_SEL_CMD);       // select channel 1
      sendCmd(BPCF_CMD | band_pass_frequency_idx); // set bandpassCenterFrequency
      // just set median values for now
      sendCmd(INT_GAIN_CMD | 22);   // set gain
      sendCmd(INT_TC_CMD | 20);     // set integrationTimeConstant
      sendCmd(ADV_MODE_CMD);        // set advanced mode - allows receiving integrator data - clearing advanced mode requires power off
    }
    SPI.endTransaction();
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

void refreshKnockParameters() // must only be called when RPM > 0
{
  uint16_t rpm = currentStatus.RPM/100; // scale to Tuner Studio rpm values
  // get the values to program the TPIC8101;

  // calc integrator time Constant 
  int degKnockWin = table2D_getValue(&knockWindowDurationTable, rpm);
  int knock_win_duration = (degKnockWin*166667)/(currentStatus.RPM); // scaled to uSec (required by TPIC8101)
  // itc = knock_win_duration / (2 * pi * Vout))
  int itc = (double)knock_win_duration / 28.27; // uSec
  // integrator_time_constant_idx is the index to the internal TPIC8101 timeConst table
  integrator_time_constant_idx = getClosestIndex(itc, timeConst, (uint8_t)32); // timeConst[]

  // integrator gain is a constant, calculated at initialiseKnock()
  // Reduce gain with RPM, to compensate for increasing engine noise
  // This is an experiment to "fix" the timing retard that occurs in
  // the OEM ecu on X3 variants of Hyundai Excel cars (low cost racing),
  // when engine revs and mechanical noise, gets high.
  knockWindowGainFactor = table2D_getValue(&knockWindowSensivityTable, rpm); // 100 to 0 (%)
  // rpmModGain_idx is the index to the internal TPIC8101 gain table
  rpmModGain_idx = 64 - getClosestIndex((integratorGain * knockWindowGainFactor)/100, gainK, (uint8_t)64); // compensating for for inverted gainK array
}

static inline void getKnockValue()
{
  uint8_t lowByte;
  uint8_t highByte;
  int knockValue = 0;

  CLOSE_KNOCK_WINDOW();
  // get knock Value - takes 16 uSec with 2MHz clock
  SPI.beginTransaction(knockSettings);
  CS0_ASSERT();  
  sendCmd(REQUEST_LOW_BYTE);  // also sets prescaler
  lowByte = sendCmd(REQUEST_HIGH_BYTE); // also sets the channel
  // rpmModGain_idx and integrator_time_constant_idx set in main loop (4Hz)
  highByte = sendCmd(INT_GAIN_CMD | rpmModGain_idx); //  refresh gain - changes with rpm
  sendCmd(INT_TC_CMD | integrator_time_constant_idx); // refresh itc - changes with rpm
  SPI.endTransaction();
  knockValue = (knockValue | highByte) << 2; // already shifted 6 bits
  knockValue |= lowByte;
DIAG2=knockValue;
DIAG3=knock_threshold;
  if (knockValue > knock_threshold)
  {
    knockCounter++; // used in 100 mS timer loop
  }
}

uint8_t getClosestIndex(int val, int array[], uint8_t sz)
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
      // integrator_time_constant is the index to the internal TPIC8101 tc table

      if (k > 0 && val > array[k - 1])
      {
        uint8_t idx = closestIndex(k - 1, k, array, val);
        return (idx);
      }
      j = k;
    }
    else  // search right of array[k]
    {
      if ((k < sz - 1) && (val < array[k + 1]))
      {
        uint8_t idx = closestIndex(k, k + 1, array, val);
        return (idx);
      }
      i = k + 1;
    }
  }
  return (k);
}

uint8_t closestIndex(int idx1, int idx2, int array[], int reqVal)
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


#endif