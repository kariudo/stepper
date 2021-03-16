//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                          //
//                                               Stepper Test                                               //
//                                                                                                          //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

//
//  In preparation for building an analog styled clock, I chose to use an ESP32 board and a 5vdc stepper with
// controller.  While the analog clock I was designing would only have an hour and minute hand, I tested the
// hardware, wiring and software using seconds from the ESP32 real time clock (rtc).
//

// Hardware:
//
//  1) The ESP32 I used: https://www.adafruit.com/product/3405
//  2) The stepper and controller I used: https://www.amazon.com/gp/product/B077YGWRHK/ref=oh_aui_detailpage_o01_s00?ie=UTF8&psc=1
//
//
// Wiring:
//
//  1) Add male to female jumper wires (included in the stepper motor kit) between the ESP32 and stepper controller boards as follows:
//
//     a) ESP32 pin 14 (male) to stepper board pin IN4 (female).
//     b) ESP32 pin 32 (male) to stepper board pin IN3 (female).
//     c) ESP32 pin 15 (male) to stepper board pin IN2 (female).
//     d) ESP32 pin 33 (male) to stepper board pin IN1 (female).
//     e) ESP32 pin GND (male) to the stepper board pin "-" (female).
//     f) ESP32 pin USB (male) to the stepper board pin "+" (female).
//

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Includes.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include                              <time.h>                              // for time
#include                              <WiFi.h>                              // for wifi (not used but required for timeval structure)

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Constants.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define       MILLISECONDS_STEP_HOLD  1                                   // time in milliseconds between motor steps
#define       MOTOR_PHASE_A           32                                  // motor phase a pin
#define       MOTOR_PHASE_B           14                                  // motor phase b pin
#define       MOTOR_PHASE_C           33                                  // motor phase c pin
#define       MOTOR_PHASE_D           15                                  // motor phase d pin
#define       STEPS_PER_REVOLUTION    4096                                // steps for one full revolution of the stepper

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Global Variables.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

int             nDelay =                        100;                     // main loop delay
long            nSeconds =                      0;                        // seconds
long            nSecondsIndicated =             0;                        // secondsindicated by clock
struct timeval  tvTimeValue;                                              // time value structure

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Setup.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() 
{
  // Serial.
  
  Serial.begin(115200);
  while(!Serial){};

  // I/O.

    // Set motor drive pins as outputs.
    
    pinMode(MOTOR_PHASE_A, OUTPUT);
    pinMode(MOTOR_PHASE_B, OUTPUT);
    pinMode(MOTOR_PHASE_C, OUTPUT);
    pinMode(MOTOR_PHASE_D, OUTPUT);

   // Set esp32 rtc to 00:00:00.
  
  memset(& tvTimeValue, 0, sizeof(tvTimeValue));
  settimeofday(& tvTimeValue, NULL);

  // End of setup.
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Loop.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() 
{
  // Update nSeconds from the ESP32 rtc.
  
  gettimeofday(& tvTimeValue, NULL);
  
  struct tm * tmPointer = localtime(& tvTimeValue.tv_sec);
  nSeconds = tmPointer->tm_sec;
  
  // Update the stepper position.
  
  Update();
  
  // Delay nDelay seconds.

  delay(nDelay);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// MotorOff
// Turn off motor drive.
// Entry   : nothing
// Returns : nothing
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void  MotorOff()
{
  digitalWrite(MOTOR_PHASE_A, LOW);
  digitalWrite(MOTOR_PHASE_B, LOW);
  digitalWrite(MOTOR_PHASE_C, LOW);
  digitalWrite(MOTOR_PHASE_D, LOW);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Step
// Step the stepper motor.
// Entry   : direction (>= 0 for clockwise step, < 0 for counter clockwise step)
// Returns : nothing
//
// Notes   : 1) For this stepper motor, 1 step is (1 / STEPS_PER_REVOLUTION) degrees.
//
//           2) Forward clock motion is performed in 8 steps by driving the motor phases, in sequence, as follows:
//
//              a) phase d
//              b) phase d and c
//              c) phase c
//              d) phase c and b
//              e) phase b
//              f) phase b and a
//              g) phase a
//              h) phase a and d
//
//            3) Reverse clock motion is performed in the reverse order of 2).
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void  Step(int nDirection)
{
  // Local variables.
  
  static  int nPhase = 0;

  // Update phase.
  
  nPhase = ((nDirection < 0) ? (nPhase - 1) : (nPhase + 1)) & 7;

  // Step this phase.
  
  switch(nPhase)
  {
    case 0:
    {
      digitalWrite(MOTOR_PHASE_D, HIGH);
      digitalWrite(MOTOR_PHASE_C, LOW);
      digitalWrite(MOTOR_PHASE_B, LOW);
      digitalWrite(MOTOR_PHASE_A, LOW);
    }
    break;

    case 1:
    {
      digitalWrite(MOTOR_PHASE_D, HIGH);
      digitalWrite(MOTOR_PHASE_C, HIGH);
      digitalWrite(MOTOR_PHASE_B, LOW);
      digitalWrite(MOTOR_PHASE_A, LOW);
    }
    break;

    case 2:
    {
      digitalWrite(MOTOR_PHASE_D, LOW);
      digitalWrite(MOTOR_PHASE_C, HIGH);
      digitalWrite(MOTOR_PHASE_B, LOW);
      digitalWrite(MOTOR_PHASE_A, LOW);
    }
    break;

    case 3:
    {
      digitalWrite(MOTOR_PHASE_D, LOW);
      digitalWrite(MOTOR_PHASE_C, HIGH);
      digitalWrite(MOTOR_PHASE_B, HIGH);
      digitalWrite(MOTOR_PHASE_A, LOW);
    }
    break;

    case 4:
    {
      digitalWrite(MOTOR_PHASE_D, LOW);
      digitalWrite(MOTOR_PHASE_C, LOW);
      digitalWrite(MOTOR_PHASE_B, HIGH);
      digitalWrite(MOTOR_PHASE_A, LOW);
    }
    break;

    case 5:
    {
      digitalWrite(MOTOR_PHASE_D, LOW);
      digitalWrite(MOTOR_PHASE_C, LOW);
      digitalWrite(MOTOR_PHASE_B, HIGH);
      digitalWrite(MOTOR_PHASE_A, HIGH);
    }
    break;

    case 6:
    {
      digitalWrite(MOTOR_PHASE_D, LOW);
      digitalWrite(MOTOR_PHASE_C, LOW);
      digitalWrite(MOTOR_PHASE_B, LOW);
      digitalWrite(MOTOR_PHASE_A, HIGH);
    }
    break;

    case 7:
    {
      digitalWrite(MOTOR_PHASE_D, HIGH);
      digitalWrite(MOTOR_PHASE_C, LOW);
      digitalWrite(MOTOR_PHASE_B, LOW);
      digitalWrite(MOTOR_PHASE_A, HIGH);
    }
    break;
  }

  // Hold this step for MILLISECONDS_STEP_HOLD milliseconds.
   
  delay(MILLISECONDS_STEP_HOLD);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Update
// Update stepper position.
// Entry   : nothing
// Returns : -1 if update occurred, 0 if not.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

int   Update()
{
  // Local variables.

  long              nClockwiseSeconds = 0;
  long              nCounterClockwiseSeconds = 0;
  long              nStepCount = 0;
  long              nStepDirection = 0;
  long              nStepPosition = 0;
  long              nStepPositionN1 = 0;
  
  // Calculate clockwise and counterclockwise seconds required to drive indicated seconds to actual seconds.
  
  if(nSeconds > nSecondsIndicated)
  {
    nClockwiseSeconds = nSeconds - nSecondsIndicated;
    nCounterClockwiseSeconds = (nSecondsIndicated + 60) - nSeconds;
  }
  else if(nSeconds < nSecondsIndicated)
  {
    nClockwiseSeconds = (nSeconds + 60) - nSecondsIndicated;
    nCounterClockwiseSeconds = nSecondsIndicated - nSeconds;
  }
  
  // Check if update is needed.
  
  if((nClockwiseSeconds) || (nCounterClockwiseSeconds))
  {
    // Update is needed, determine shortest direction.
    
    if(nClockwiseSeconds < nCounterClockwiseSeconds)
    {
      // Clockwise movement is shorter.
      
      nStepDirection = 1; 
    }
    else
    {
      // Counterclockwise movement is shorter.
      
      nStepDirection = -1;
    }

    // Drive indicated seconds to seconds.

    while(nSeconds != nSecondsIndicated)
    {
      // Calculate n-1 step position.
      
      nStepPositionN1 = ((nSecondsIndicated % 60) * STEPS_PER_REVOLUTION) / 60;
      
      // Update seconds.
      
      nSecondsIndicated = (nSecondsIndicated + nStepDirection) % 60;
      nSecondsIndicated = (nSecondsIndicated < 0) ? 60 + nSecondsIndicated : nSecondsIndicated;
      
      // Calculate current step position.

      nStepPosition = ((nSecondsIndicated % 60) * STEPS_PER_REVOLUTION) / 60;

      // Calculate step count.
      
      nStepCount = ((nStepDirection > 0) ? nStepPosition - nStepPositionN1 : nStepPositionN1 - nStepPosition);
      nStepCount = (nStepCount < 0) ? STEPS_PER_REVOLUTION + nStepCount : nStepCount;
                     
      // Step the required steps.
      
      while(nStepCount)
      {
        Step(nStepDirection);
        nStepCount = nStepCount - 1;
      }
    }
  }
  else
  {
    // No update performed.
    
    return 0;
  }

  // Update was performed, remove motor power.

  MotorOff();
  
  // Update performed.
  
  return -1;
}

