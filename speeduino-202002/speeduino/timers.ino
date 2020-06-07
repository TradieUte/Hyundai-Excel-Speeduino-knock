/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/

/*
Timers are used for having actions performed repeatedly at a fixed interval (Eg every 100ms)
They should not be confused with Schedulers, which are for performing an action once at a given point of time in the future

Timers are typically low resolution (Compared to Schedulers), with maximum frequency currently being approximately every 10ms
*/
#include "timers.h"
#include "globals.h"
#include "sensors.h"
#include "scheduler.h"
#include "scheduledIO.h"
#include "speeduino.h"
#include "auxiliaries.h"
#include "knock.h"

#if defined(CORE_AVR)
  #include <avr/wdt.h>
#endif

void initialiseTimers()
{
  lastRPM_100ms = 0;
  loop33ms = 0;
  loop66ms = 0;
  loop100ms = 0;
  loop250ms = 0;
  loopSec = 0;
}


//Timer2 Overflow Interrupt Vector, called when the timer overflows.
//Executes every ~1ms.
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER2_OVF_vect, ISR_NOBLOCK) //This MUST be no block. Turning NO_BLOCK off messes with timing accuracy
#else
void oneMSInterval() //Most ARM chips can simply call a function
#endif
{
  ms_counter++;

  //Increment Loop Counters
  loop33ms++;
  loop66ms++;
  loop100ms++;
  loop250ms++;
  loopSec++;

  unsigned long targetOverdwellTime;

  //Overdwell check
  targetOverdwellTime = micros() - dwellLimit_uS; //Set a target time in the past that all coil charging must have begun after. If the coil charge began before this time, it's been running too long
  bool isCrankLocked = configPage4.ignCranklock && (currentStatus.RPM < currentStatus.crankRPM); //Dwell limiter is disabled during cranking on setups using the locked cranking timing. WE HAVE to do the RPM check here as relying on the engine cranking bit can be potentially too slow in updating
  //Check first whether each spark output is currently on. Only check it's dwell time if it is

  if(ignitionSchedule1.Status == RUNNING) { if( (ignitionSchedule1.startTime < targetOverdwellTime) && (configPage4.useDwellLim) && (isCrankLocked != true) ) { ign1EndFunction(); ignitionSchedule1.Status = OFF; } }
  if(ignitionSchedule2.Status == RUNNING) { if( (ignitionSchedule2.startTime < targetOverdwellTime) && (configPage4.useDwellLim) && (isCrankLocked != true) ) { ign2EndFunction(); ignitionSchedule2.Status = OFF; } }
  if(ignitionSchedule3.Status == RUNNING) { if( (ignitionSchedule3.startTime < targetOverdwellTime) && (configPage4.useDwellLim) && (isCrankLocked != true) ) { ign3EndFunction(); ignitionSchedule3.Status = OFF; } }
  if(ignitionSchedule4.Status == RUNNING) { if( (ignitionSchedule4.startTime < targetOverdwellTime) && (configPage4.useDwellLim) && (isCrankLocked != true) ) { ign4EndFunction(); ignitionSchedule4.Status = OFF; } }
  if(ignitionSchedule5.Status == RUNNING) { if( (ignitionSchedule5.startTime < targetOverdwellTime) && (configPage4.useDwellLim) && (isCrankLocked != true) ) { ign5EndFunction(); ignitionSchedule5.Status = OFF; } }
  if(ignitionSchedule6.Status == RUNNING) { if( (ignitionSchedule6.startTime < targetOverdwellTime) && (configPage4.useDwellLim) && (isCrankLocked != true) ) { endCoil6Charge(); ignitionSchedule6.Status = OFF; } }
  if(ignitionSchedule7.Status == RUNNING) { if( (ignitionSchedule7.startTime < targetOverdwellTime) && (configPage4.useDwellLim) && (isCrankLocked != true) ) { endCoil7Charge(); ignitionSchedule7.Status = OFF; } }
  if(ignitionSchedule8.Status == RUNNING) { if( (ignitionSchedule8.startTime < targetOverdwellTime) && (configPage4.useDwellLim) && (isCrankLocked != true) ) { endCoil8Charge(); ignitionSchedule8.Status = OFF; } }

#if !defined(CORE_TEENSY)  // Teensy uses PIT for Tacho
  //Tacho output check
  //Tacho is flagged as being ready for a pulse by the ignition outputs. 
  if(tachoOutputFlag == READY)
  {
    //Check for half speed tacho
    if( (configPage2.tachoDiv == 0) || (tachoAlt == true) ) 
    { 
      TACHO_PULSE_HIGH();
      //ms_counter is cast down to a byte as the tacho duration can only be in the range of 1-6, so no extra resolution above that is required
      tachoEndTime = (uint8_t)ms_counter + configPage2.tachoDuration;
      tachoOutputFlag = ACTIVE;
    }
    else
    {
      //Don't run on this pulse (Half speed tacho)
      tachoOutputFlag = DEACTIVE;
    }
    tachoAlt = !tachoAlt; //Flip the alternating value incase half speed tacho is in use. 
  }
  else if(tachoOutputFlag == ACTIVE)
  {
    //If the tacho output is already active, check whether it's reached it's end time
    if((uint8_t)ms_counter == tachoEndTime)
    {
      TACHO_PULSE_HIGH();
      tachoOutputFlag = DEACTIVE;
    }
  }
#endif

  //30Hz loop
  if (loop33ms == 33)
  {
    loop33ms = 0;
    BIT_SET(TIMER_mask, BIT_TIMER_30HZ);
  }

  //15Hz loop
  if (loop66ms == 66)
  {
    loop66ms = 0;
    BIT_SET(TIMER_mask, BIT_TIMER_15HZ);
  }

  //Loop executed every 100ms loop
  //Anything inside this if statement will run every 100ms.
  if (loop100ms == 100)
  {
    loop100ms = 0; //Reset counter
    BIT_SET(TIMER_mask, BIT_TIMER_10HZ);

#if defined(KNOCK)
    if (configPage10.knock_mode==KNOCK_MODE_DIGITAL)
    {
      // knock intervals all in 100ms increments in Tuner Studio
      // knockCounter is incremented when using Teensy by pit3_isr
      // knockRetard is used for ignition corrections (corrections.ino)
      static int lastKnockCount = 1000; // arbitrarily large count to ensure first use is valid
      static int retardStepTime = 0;   // retard knock sample interval counter
      static int advanceStepTime = 0;  // retard recovery (advance) interval counter; incremented in advance process
      static int advanceDelay = 0;     // accumulates the delay between last no-knock condition and start of advance process
      static bool reset = true;
      // algorithm first checks if retard is required, then allows advance to happen. The consequence is that 
      // the minimum advance recovery interval is set by the knock sample retard interval.

      if (++retardStepTime >= configPage10.knock_stepTime) // accumulate knock events during this time
      {
        retardStepTime=0;
        if ((knockCounter <= configPage10.knock_count) || (currentStatus.RPM >= configPage10.knock_maxRPM*100) ||(currentStatus.MAP >= (configPage10.knock_maxMAP<<1)) ) // "no knock" condition (knockCounter incremented in pit3_isr)
        {
          if (reset == true)  // only do following when required
          {
            reset = false;
            currentStatus.knockActive = false;
            if (knockRecoveryFirstStepDelay == true)
            {
              knockRecoveryFirstStepDelay = false;
              // the following ensures knock advance recovery process starts on time after last instance of knock,
              // for any value of configPage10.knock_duration and configPage10.knock_recoveryStepTime
              // delay can not be less than configPage10.knock_stepTime, as that has already occurred to arrive at this point
              advanceDelay = configPage10.knock_duration - configPage10.knock_recoveryStepTime - configPage10.knock_stepTime;
              if (advanceDelay <= 0)
              {
                advanceDelay = 0;
              // configPage10.knock_stepTime is added to advanceStepTime, because that time has already passed to get here.
              advanceStepTime = configPage10.knock_recoveryStepTime - configPage10.knock_duration + configPage10.knock_stepTime; // for first step of recovery
            }
              advanceStepTime = configPage10.knock_stepTime - configPage10.knock_recoveryStepTime;  // if knock check time greater than advance recovery step time
              if (advanceStepTime <= 0) {advanceStepTime = configPage10.knock_recoveryStepTime;}    // setup for no advance recovery delay
            }
          }
        }
        else  // retard spark
        {
          reset = true;
          currentStatus.knockActive = true;
          knockRecoveryFirstStepDelay = true; // setup for advance process
          int knock_retard = knockRetard; // do calcs on local variable knock_retard
          if (knock_retard < configPage10.knock_maxRetard) // knockRetard used in corrections.ino
          {
            if ((knockCounter > lastKnockCount) || (knockCounter > (configPage10.knock_count << 4))) // high or increasing knock count
            {
              knock_retard += configPage10.knock_firstStep;  // agressive ign retard
            }
            else
            {
              knock_retard += configPage10.knock_stepSize;   // normal ign retard
            }
            lastKnockCount = knockCounter;  // store last knock count
            if (knock_retard > configPage10.knock_maxRetard) {knock_retard = configPage10.knock_maxRetard;}
          }
          knockRetard = knock_retard; // update global variable
        }
        knockCounter = 0; // incremented by pit3_isr (when using Teensy) each time a knock is registered
      }
      // advance spark - if knock just finished, wait advanceDelay before advancing
      if ( (currentStatus.knockActive == false) && (knockRetard > 0) && (--advanceDelay <= 0) ) // no knock; positive retard value; advance start delay expired
      {
        reset = true;
        if (++advanceStepTime >= configPage10.knock_recoveryStepTime) // time, in 100mS increments, between advance steps
        {
          advanceStepTime = 0;
          knockRetard -= configPage10.knock_recoveryStep;
        }
        advanceDelay = 0;
      }
    }
#endif 

    currentStatus.rpmDOT = (currentStatus.RPM - lastRPM_100ms) * 10; //This is the RPM per second that the engine has accelerated/decelleratedin the last loop
    lastRPM_100ms = currentStatus.RPM; //Record the current RPM for next calc
  }

  //Loop executed every 250ms loop (1ms x 250 = 250ms)
  //Anything inside this if statement will run every 250ms.
  if (loop250ms == 250)
  {
    loop250ms = 0; //Reset Counter
    BIT_SET(TIMER_mask, BIT_TIMER_4HZ);
    #if defined(CORE_STM32) //debug purpose, only visual for running code
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    #endif

    #if defined(CORE_AVR)
      //Reset watchdog timer (Not active currently)
      //wdt_reset();
      //DIY watchdog
      //This is a sign of a crash:
      //if( (initialisationComplete == true) && (last250msLoopCount == mainLoopCount) ) { setup(); }
      //else { last250msLoopCount = mainLoopCount; }
    #endif
  }

  //Loop executed every 1 second (1ms x 1000 = 1000ms)
  if (loopSec == 1000)
  {
    loopSec = 0; //Reset counter.
    BIT_SET(TIMER_mask, BIT_TIMER_1HZ);

    dwellLimit_uS = (1000 * configPage4.dwellLimit); //Update uS value incase setting has changed
    currentStatus.crankRPM = ((unsigned int)configPage4.crankRPM * 10);

    //**************************************************************************************************************************************************
    //This updates the runSecs variable
    //If the engine is running or cranking, we need ot update the run time counter.
    if (BIT_CHECK(currentStatus.engine, BIT_ENGINE_RUN))
    { //NOTE - There is a potential for a ~1sec gap between engine crank starting and ths runSec number being incremented. This may delay ASE!
      if (currentStatus.runSecs <= 254) //Ensure we cap out at 255 and don't overflow. (which would reset ASE and cause problems with the closed loop fueling (Which has to wait for the O2 to warmup))
        { currentStatus.runSecs++; } //Increment our run counter by 1 second.
    }
    //**************************************************************************************************************************************************
    //This records the number of main loops the system has completed in the last second
    currentStatus.loopsPerSecond = mainLoopCount;
    mainLoopCount = 0;
    //**************************************************************************************************************************************************
    //increament secl (secl is simply a counter that increments every second and is used to track whether the system has unexpectedly reset
    currentStatus.secl++;
    //**************************************************************************************************************************************************
    //Check the fan output status
    if (configPage6.fanEnable == 1)
    {
       fanControl();            // Fucntion to turn the cooling fan on/off
    }

    //Check whether fuel pump priming is complete
    if(fpPrimed == false)
    {
      //fpPrimeTime is the time that the pump priming started. This is 0 on startup, but can be changed if the unit has been running on USB power and then had the ignition turned on (Which starts the priming again)
      if( (currentStatus.secl - fpPrimeTime) >= configPage2.fpPrime)
      {
        fpPrimed = true; //Mark the priming as being completed
        if(currentStatus.RPM == 0)
        {
          //If we reach here then the priming is complete, however only turn off the fuel pump if the engine isn't running
          digitalWrite(pinFuelPump, LOW);
          currentStatus.fuelPumpOn = false;
        }
      }
    }
    //**************************************************************************************************************************************************
    //Set the flex reading (if enabled). The flexCounter is updated with every pulse from the sensor. If cleared once per second, we get a frequency reading
    if(configPage2.flexEnabled == true)
    {
      if(flexCounter < 50)
      {
        currentStatus.ethanolPct = 0; //Standard GM Continental sensor reads from 50Hz (0 ethanol) to 150Hz (Pure ethanol). Subtracting 50 from the frequency therefore gives the ethanol percentage.
        flexCounter = 0;
      }
      else if (flexCounter > 151) //1 pulse buffer
      {

        if(flexCounter < 169)
        {
          currentStatus.ethanolPct = 100;
          flexCounter = 0;
        }
        else
        {
          //This indicates an error condition. Spec of the sensor is that errors are above 170Hz)
          currentStatus.ethanolPct = 0;
          flexCounter = 0;
        }
      }
      else
      {
        currentStatus.ethanolPct = flexCounter - 50; //Standard GM Continental sensor reads from 50Hz (0 ethanol) to 150Hz (Pure ethanol). Subtracting 50 from the frequency therefore gives the ethanol percentage.
        flexCounter = 0;
      }

      //Off by 1 error check
      if (currentStatus.ethanolPct == 1) { currentStatus.ethanolPct = 0; }

    }

  }
#if defined(CORE_AVR) //AVR chips use the ISR for this
    //Reset Timer2 to trigger in another ~1ms
    TCNT2 = 131;            //Preload timer2 with 100 cycles, leaving 156 till overflow.
#endif
}

