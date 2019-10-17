/*
 * Programmable Appliance Timer (PAT) with contact-closure input/trip, status relay, and status 12V supply
 * Author: Chris Hamilton <tauchris@utexas.edu>
 * Date:   October 15, 2019
 * 
 * Designed to control a hot-water recirculation pump, but can control any on-demand appliance that should run between
 * 1 and 20 minutes*.  PAT has one on-board status/mode indicator LED, and two on-board buttons -- one mode-select
 * button to switch between normal operation mode, in which the pump relay can be operated, and program mode, in which
 * the appliance runtime setting can be displayed and changed. PAT has terminals for external connections as follows:
 * 
 *   Terminal 0:1 :  Normally-open external trip button (e.g. doorbell switch) to trigger pump
 *   Terminal 2:3 :  12V/0.3A power supply to drive external LEDs or other low-power device while pump is running
 *   Terminal 4:6 :  Contact-closure relay terminals (no/nc/co) to interface w/external devices (e.g., Insteon/X10)
 * 
 * Operation:
 * +---------+----------------------------+---------------------+---------------------+-------------------------------+
 * | STATE   | DESCRIPTION                | LED                 | TRIP_BUTTON         | MODE_BUTTON                   |
 * +---------+----------------------------+---------------------+---------------------+-------------------------------+
 * | IDLE    | Ready state (relays off)   | 2 dim green flashes | Goes to ACTIVE      | Hold for 5s,                  |
 * |         |                            | @10s interval       |                     | goes to PROGRAM               |
 * +---------+----------------------------+---------------------+---------------------+-------------------------------+
 * | ACTIVE  | Runs pump (relays on) then | Solid blue          | Cancels pump --     | (ignored)                     |
 * |         | goes back to IDLE mode     |                     | goes back to IDLE   |                               |
 * +---------+----------------------------+---------------------+---------------------+-------------------------------+
 * | PROGRAM | Change or confirm relay    | Flickering magenta  | Press and hold to   | Press and release to trigger  |
 * |         | runtime setting.  If no    | bursts              | enter new setting   | ECHO (confirm setting); Press |
 * |         | buttons pressed for 90s,   | @2s interval        | (goes to COUNT)     | and *hold* for 5s to SAVE     |
 * |         | goes to CANCEL mode.       |                     |                     | new setting (SAVE mode)       |
 * +---------+----------------------------+---------------------+---------------------+-------------------------------+
 * | COUNT   | Sets new runtime setting   | long bright amber   | Release to exit     | (ignored)                     |
 * |         | while trip button is held; | flashes             | COUNT and return    |                               |
 * |         | Each flash = 1 minute      | @1s interval        | to PROGRAM          |                               |
 * +---------+----------------------------+---------------------+---------------------+-------------------------------+
 * | ECHO    | Displays runtime value,    | short bright amber  | (ignored)           | (ignored)                     |
 * |         | then goes back to PROGRAM. | flashes             |                     |                               |
 * |         | Each flash = 1 minute      | @0.5s interval      |                     |                               |
 * +---------+----------------------------+---------------------+---------------------+-------------------------------+
 * | SAVE    | Saves new runtime setting  | Solid bright green  | (ignored)           | (ignored)                     |
 * |         | to device memory, goes     | for 1s              |                     |                               |
 * |         | back to IDLE mode          |                     |                     |                               |
 * +---------+----------------------------+---------------------+---------------------+-------------------------------+
 * | CANCEL  | Discards changes to run-   | Flickering          | (ignored)           | (ignored)                     |
 * |         | time setting, then goes    | red burst for       |                     |                               |
 * |         | back to IDLE mode          | 1s                  |                     |                               |
 * +---------+----------------------------+---------------------+---------------------+-------------------------------+
 * 
 * External trip buttons on terminals 0:1 operate same as TRIP_BUTTON in modes IDLE and RELAY_ON.  External buttons are 
 * ignored in all other modes.
 * 
 * 12V supply on terminals 2:3 is inactive while pump relay is off, and is active while pump relay is on.  Similarly,
 * contact-closure relay terminals 4:6 are in "normal" state (no:co is open, nc:co is closed) while pump relay is off,
 * and are in "active" state (no:co is CLOSED, and nc:co is OPEN) while pump relay is on.
 * 
 * Pump relay is on ONLY during ACTIVE mode.
 * 
 * *20 minutes is more than enough runtime for a hot-water recirculation pump even in large homes, so I chose it as
 * an upper limit here.  If a larger limit is needed, then the interface probably needs to be redesigned, as the 
 * button-holding and LED-flashing I/O pardigms are suited to small numbers only.  As an alternative, perhaps programming
 * via a serial port would be more appropriate.
 */
 
/*
 * DESIGN NOTES:
 *  - First iteration of design was done on Arduino Uno, and used Timer1 interrupts.  Porting the design to an 
 *    Arudino Nano Every, which is ATmega4809-based, broke that, because the timers and interrupts are different.
 *    Even if it *had* worked, it would have had to be different, because the Uno (ATmega328P) runs at 16Mhz, while
 *    the Nano Every runs at 20Mhz.  Timers are clock-frequency dependent.  So I switched to this millis()-based
 *    approach, and found that it is actually easier to understand, much more platform independent, and every bit
 *    as accurate for devices with any sufficiently-fast clock.  Only downside is that it probably requires a bit
 *    more global variables and possibly more program storage space.  Hard to measure that, and honestly, I doubt
 *    the difference is significant.
 *  - I added a "firstIterationAction()" function to be called from loop().  This is an unfortunate compromise.  
 *    I originally added some Serial debug messages and LED light settings in setup() to try to get an idea of 
 *    when the device had successfully started -- but the lights never did what I expected, and the Serial message
 *    was always dropped-- or at best, garbled.  Also, putting a delay() statement in setup() also turns out to
 *    be bad (or pointless).  Not really sure why that is.  Moved the EEPROM read code to firstIterationAction() 
 *    as well, just to be safe.
 * 
 * TODO/FIXMEs:
 * 
 * 1. Actually read setting from EEPROM and write setting to EEPROM
 *    Assuming "normal" way of setting initial EEPROM value is to load and run a separate setup program on 
 *    the Arduino, and ensure that the address chosen for the value is the same in both programs.
 * 2. Consider long-term EEPROM stability.  What is expected lifetime of device?  What is expected number of
 *    power-up/programming cycles for a given period of time (e.g., a month).  How long would a single EEPROM
 *    memory address be safe to use given the stated max read/write cycles given?
 * 3. Actually test external switches with relay-controlled 12V power supply
 * 4. Consider whether 12V power supply can be controlled with just a transistor (instead of a relay)
 * 5. Consider whether NO/NC relays can be controlled with transistors (instead of a relay)
 * 
 */

#include <EEPROM.h>

// enum types
enum BUTTON_STATE { BUTTON_OFF, BUTTON_PRESSED, BUTTON_STATE_MAX };
enum PIN_INDEX { P_RED, P_GREEN, P_BLUE, PIN_INDEX_MAX };
enum RGB_COLOR { RGB_OFF, RGB_DIM_GREEN, RGB_BLUE, RGB_MAGENTA, RGB_AMBER, RGB_RED, RGB_GREEN, RGB_WHITE, RGB_COLOR_MAX }; 
enum MODE { M_IDLE, M_ACTIVE, M_PROGRAM, M_COUNT, M_ECHO, M_SAVE, M_CANCEL, MODE_MAX };
enum STEP_RECORD_FIELD { STEP_COLOR, STEP_DURATION, STEP_NEXT_STEP, STEP_RECORD_FIELD_MAX };

// constants : pins and addresses
const byte modeButton = 3;
const byte tripButton = 2;
const byte remoteButton = 4;
const byte ledRed = 10;
const byte ledGreen = 5;
const byte ledBlue = 6;
const byte relayControl = 8;
const int eeAddrRelayTimerMs = 0x00;
// constants : limits, defaults, and conversion ratios
const unsigned long debounceMs = 25;
const unsigned long secToMs = 1000;
const unsigned long minToMs = 60 * secToMs;
const unsigned long relayTimeoutToEchoTimeoutRatio = 120;    // Ratio is 1m/0.5s = 60s/0.5s = 120.
const unsigned long pgmModeTimeoutMs = 90 * secToMs;
const unsigned long longPressMs = 5 * secToMs;
const unsigned long minSaveOrCancelModeMs = 1 * secToMs;

// constant LUTs
const byte color[RGB_COLOR_MAX][PIN_INDEX_MAX]={
//{   R,   G,   B }
  {   0,   0,   0 },    // RGB_OFF
  {   0,  15,   0 },    // RGB_DIM_GREEN
  {   0,   0, 255 },    // RGB_BLUE
  {  40,   0,  10 },    // RGB_MAGENTA
  { 200,  15,   0 },    // RGB_AMBER
  { 220,   0,   0 },    // RGB_RED
  {   0, 220,   0 },    // RGB_GREEN
  { 255, 255, 255 }     // RGB_WHITE
};

// This table holds multiple linked-lists of indicator pattern steps (one list of steps per mode), and the
// next array is start index of each mode's linked list in this table.  Not the most maintenance-friendly
// choice.  Consider refactoring for better data cohesion and robustness.
const unsigned short indicatorStepTable[][STEP_RECORD_FIELD_MAX]={
//{ STEP_COLOR,      STEP_DURATION, STEP_NEXT_STEP }, // index  : MODE
  { RGB_OFF,         500,           1              }, // 0      : M_IDLE
  { RGB_DIM_GREEN,   250,           2              }, // 1
  { RGB_OFF,         250,           3              }, // 2
  { RGB_DIM_GREEN,   250,           4              }, // 3
  { RGB_OFF,         8750,          0              }, // 4
  { RGB_BLUE,        30000,         5              }, // 5      : M_RUN_TIMER
  { RGB_MAGENTA,     30,            7              }, // 6      : M_PROGRAM
  { RGB_OFF,         30,            8              }, // 7
  { RGB_MAGENTA,     30,            9              }, // 8
  { RGB_OFF,         30,            10             }, // 9
  { RGB_MAGENTA,     30,            11             }, // 10
  { RGB_OFF,         30,            12             }, // 11
  { RGB_MAGENTA,     30,            13             }, // 12
  { RGB_OFF,         1790,          6              }, // 13
  { RGB_OFF,         325,           15             }, // 14     : M_COUNT
  { RGB_AMBER,       675,           14             }, // 15
  { RGB_OFF,         75,            17             }, // 16     : M_ECHO
  { RGB_AMBER,       325,           18             }, // 17
  { RGB_OFF,         100,           16             }, // 18
  { RGB_GREEN,       30000,         19             }, // 19     : M_SAVE
  { RGB_RED,         30,            21             }, // 20     : M_CANCEL
  { RGB_OFF,         30,            20             }, // 21
};
const byte modeIndicatorStartStep[MODE_MAX]={
  0,   // M_IDLE
  5,   // M_RUN_TIMER
  6,   // M_PROGRAM
  14,  // M_COUNT
  16,  // M_ECHO
  19,  // M_SAVE
  20,  // M_CANCEL
};

// global variables: event-tracking times
// Note: with unsigned longs, "newtime - oldtime > diff" is correct even when newtime (millis()) rolls over to 0
unsigned long currentMs = 0;
unsigned long lastModeButtonDebounceMs = 0;
unsigned long lastTripButtonDebounceMs = 0;
unsigned long lastRemoteButtonDebounceMs = 0;
unsigned long lastModeChangeMs = 0;
unsigned long lastModeButtonPressedMs = 0;
unsigned long lastIndiciatorStepMs = 0;
unsigned long lastUpdateIndicatorsMs = 0;
unsigned long nextUpdateIndicatorsMs = 0;
// global variables: program state
unsigned long relayTimerMs = 0;        // Will be read from EEPROM (defaults to 5 minutes if EEPROM value is garbage)
unsigned long echoTimeoutMs = 0;
unsigned long newRelayTimerMs = 0;
byte modeButtonState = BUTTON_OFF;
byte tripButtonState = BUTTON_OFF;
byte remoteButtonState = BUTTON_OFF;
byte mode = M_IDLE;
unsigned short indicatorStepIndex = 0;
bool synchronousModeWorkDone = false;
bool indicatorsNeedUpdate = true;


#define DEBUGGING_MESSAGES_ENABLED

#ifdef DEBUGGING_MESSAGES_ENABLED
#define DebugMsg(x)     do { Serial.print(x); } while (0)
#define DebugMsgLn(x)   do { Serial.println(x); } while (0)
const char *buttonStateToStr(int buttonState) {
  static const char* const str[] = {
    "BUTTON_OFF",
    "BUTTON_PRESSED"
  };
  return str[buttonState];
}
const char *modeToStr(int mode) {
  static const char* const str[] = {
    "M_IDLE",
    "M_ACTIVE",
    "M_PROGRAM",
    "M_COUNT",
    "M_ECHO",
    "M_SAVE",
    "M_CANCEL"
  };
  return str[mode];
}
#else
#define DebugMsg(x)
#define DebugMsgLn(x)
#endif


void setLEDColor(const byte rgb[]) {
  analogWrite(ledRed,   rgb[P_RED]);
  analogWrite(ledGreen, rgb[P_GREEN]);
  analogWrite(ledBlue,  rgb[P_BLUE]);
}

int normalizeRelayTimerMinutes(int timerMinutes) {
  static const int minRelayTimerMinutes = 1;
  static const int maxRelayTimerMinutes = 20;
  timerMinutes = max(timerMinutes, minRelayTimerMinutes);
  timerMinutes = min(timerMinutes, maxRelayTimerMinutes);
  return timerMinutes;
}

void setup() {
  #ifdef DEBUGGING_MESSAGES_ENABLED
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect
  }
  // and delay an extra second, because Serial port isn't *really* connected correctly yet...
  delay(1000);
  #endif
  
  DebugMsgLn("PAT setup(): Initializing...");

  pinMode(ledRed, OUTPUT);
  pinMode(ledGreen, OUTPUT);
  pinMode(ledBlue, OUTPUT);
 
  digitalWrite(modeButton, LOW);
  pinMode(modeButton, INPUT);
  digitalWrite(tripButton, LOW);
  pinMode(tripButton, INPUT);
  digitalWrite(remoteButton, LOW);
  pinMode(remoteButton, INPUT);

  digitalWrite(relayControl, HIGH);  // relay is active-low; pullup to default as OFF
  pinMode(relayControl, OUTPUT);

  setLEDColor(color[RGB_WHITE]);
  delay(1000);

  EEPROM.get(eeAddrRelayTimerMs, relayTimerMs);
  int relayTimerMinutes = relayTimerMs / minToMs;
  DebugMsg("PAT setup(): Read saved relayTimerMs = "); DebugMsg(relayTimerMs);
  DebugMsg("; ["); DebugMsg(relayTimerMinutes); DebugMsgLn(" minutes]");
  int normalizedTimerMinutes = normalizeRelayTimerMinutes(relayTimerMinutes);
  if (relayTimerMinutes != normalizedTimerMinutes) {
    DebugMsgLn("PAT setup(): Stored value of relayTimerMs is invalid.  Defaulting to 5 minutes (300000ms)");
    relayTimerMs = 5 * minToMs;
    EEPROM.put(eeAddrRelayTimerMs, relayTimerMs);
  }

  delay(1000);
  setLEDColor(color[RGB_OFF]);
  DebugMsgLn("PAT is READY.");
}


void changeMode(int newMode) {
  if (mode == newMode) { return; }
  switch(newMode) {
    case M_IDLE:
      // See note at doSynchronousModeWork() regarding relay control
      digitalWrite(relayControl, HIGH);
      break;
    case M_ACTIVE:
      // See note at doSynchronousModeWork() regarding relay control
      digitalWrite(relayControl, LOW);
      break;
    case M_ECHO:
      DebugMsg("changeMode(): Entering ECHO mode: Current setting for relayTimerMinutes is: ");
      DebugMsgLn(newRelayTimerMs / minToMs);
      echoTimeoutMs = newRelayTimerMs / relayTimeoutToEchoTimeoutRatio;
      break;
    case M_PROGRAM:
      if (mode == M_COUNT) {
        unsigned long newRelayTimerMinutes = (currentMs - lastModeChangeMs) / secToMs;
        DebugMsg("changeMode(): M_COUNT --> M_PROGRAM :: Time spent in M_COUNT was ");
        DebugMsg(currentMs - lastModeChangeMs);
        DebugMsg("ms; newRelayTimerMinutes => ");
        DebugMsgLn(newRelayTimerMinutes);
        newRelayTimerMinutes = normalizeRelayTimerMinutes(newRelayTimerMinutes);
        newRelayTimerMs = newRelayTimerMinutes * minToMs;
        DebugMsg("changeMode(): After normalizing newRelayTimerMinutes, newRelayTimerMs => ");
        DebugMsgLn(newRelayTimerMs);
      } else if (mode == M_IDLE) {
        newRelayTimerMs = relayTimerMs;
        DebugMsg("changeMode(): M_IDLE --> M_PROGRAM :: newRelayTimerMs reset to current relayTimerMs = ");
        DebugMsgLn(newRelayTimerMs);
      }
      break;
    default:
      break;
  }
  mode = newMode;
  synchronousModeWorkDone = false;
  lastModeChangeMs = currentMs;
  indicatorStepIndex = modeIndicatorStartStep[mode];
  indicatorsNeedUpdate |= true;
  DebugMsg("changeMode(): Mode changed to: ");
  DebugMsg(modeToStr(mode));
  DebugMsg(" at currentMs = ");
  DebugMsgLn(currentMs);
}

bool modeButtonValidPress() {
  return (currentMs - lastModeButtonPressedMs) < (currentMs - lastModeChangeMs);
}

bool modeButtonLongPress() {
  return modeButtonState == BUTTON_PRESSED && 
         modeButtonValidPress() &&
         (currentMs - lastModeButtonPressedMs) >= longPressMs;
}

void checkButtons() {
  int state;

  // Check mode button first, as mode button supercedes all other input
  state = digitalRead(modeButton) ? BUTTON_PRESSED : BUTTON_OFF;
  if (state != modeButtonState && currentMs - lastModeButtonDebounceMs > debounceMs) {
    lastModeButtonDebounceMs = currentMs;
    modeButtonState = state;
    DebugMsg("checkButtons(): MODE button state changed to: "); DebugMsgLn(buttonStateToStr(modeButtonState));
    if (modeButtonState == BUTTON_PRESSED) { lastModeButtonPressedMs = currentMs; }
    else {
      // button released
      if (mode == M_PROGRAM && modeButtonValidPress()) {
        changeMode(M_ECHO);
      }
    }
  }

  // Next check (local) trip button, as local trip button supercedes external trip button
  state = digitalRead(tripButton) ? BUTTON_PRESSED : BUTTON_OFF;
  if (state != tripButtonState && currentMs - lastTripButtonDebounceMs > debounceMs) {
    lastTripButtonDebounceMs = currentMs;
    tripButtonState = state;
    DebugMsg("checkButtons(): TRIP button state changed to: "); DebugMsgLn(buttonStateToStr(tripButtonState));
    if (tripButtonState == BUTTON_PRESSED) {
      switch(mode) {
        case M_IDLE:
          changeMode(M_ACTIVE);
          break;
        case M_ACTIVE:
          changeMode(M_IDLE);
          break;
        case M_PROGRAM:
        case M_ECHO:
          changeMode(M_COUNT);
          break;
        default:
          break;
      }
    } else if (mode == M_COUNT) {
      changeMode(M_PROGRAM);
    }
  }

  // Finally, check remote trip button events
  state = digitalRead(remoteButton) ? BUTTON_PRESSED : BUTTON_OFF;
  if (state != remoteButtonState && currentMs - lastRemoteButtonDebounceMs > debounceMs) {
    lastRemoteButtonDebounceMs = currentMs;
    remoteButtonState = state;
    DebugMsg("checkButtons(): REMOTE (trip) button state changed to: "); DebugMsgLn(buttonStateToStr(remoteButtonState));
    if (remoteButtonState == BUTTON_PRESSED) {
      switch(mode) {
        case M_IDLE:
          changeMode(M_ACTIVE);
          break;
        case M_ACTIVE:
          changeMode(M_IDLE);
          break;
        default:
          break;
      }
    }
  }
}

void checkTimeouts() {
  switch (mode) {
    case M_IDLE:
      if (modeButtonLongPress()) {
        DebugMsgLn("checkTimeouts(): mode button long-press in M_IDLE");
        changeMode(M_PROGRAM);
      }
      break;
    case M_ACTIVE:
      if (currentMs - lastModeChangeMs >= relayTimerMs) {
        DebugMsgLn("checkTimeouts(): mode timer expired for M_ACTIVE");
        changeMode(M_IDLE);
      }
      break;
    case M_PROGRAM:
      if (modeButtonState == BUTTON_PRESSED) {
        if (modeButtonLongPress()) {
          DebugMsgLn("checkTimeouts(): mode button long-press in M_PROGRAM");
          changeMode(M_SAVE);
        }
      }
      else {
        if (currentMs - lastModeChangeMs >= pgmModeTimeoutMs) {
          DebugMsgLn("checkTimeouts(): mode timer expired for M_PROGRAM");
          changeMode(M_CANCEL);
        }
      }
      break;
    case M_ECHO:
      if (currentMs - lastModeChangeMs >= echoTimeoutMs) {
        DebugMsgLn("checkTimeouts(): mode timer expired for M_ECHO");
        changeMode(M_PROGRAM);
      }
      break;
    case M_SAVE:
    case M_CANCEL:
      if (currentMs - lastModeChangeMs >= minSaveOrCancelModeMs) {
        DebugMsg("checkTimeouts(): mode timer expired for "); DebugMsgLn(modeToStr(mode));
        changeMode(M_IDLE);
      }
      break;
    default:
      break;
  }
  indicatorsNeedUpdate |= (currentMs - lastUpdateIndicatorsMs >= nextUpdateIndicatorsMs);
}

// Any work that must be done uninterrupted in a given mode should go here.  Controller will be
// non-responsive (buttons ignored, indicators will be static) while this function is executing.
//
// At present, only writing new relayTimer values to EEPROM (in SAVE mode) needs to be uninterrupted.
//
// Note: if relay control turns out to require some buffer time (e.g., if relay inrush current 
// is high enough that I need to serialize the relays; or if rapid power cycling of the pump
// turns out to be a bad thing, and I decide to impose a minimum steady-state time for pump 
// power state), that relay control should be moved here.
//
void doSynchronousModeWork() {
  if (!synchronousModeWorkDone) {
    switch (mode) {
      case M_SAVE:
        if (relayTimerMs != newRelayTimerMs) {
          relayTimerMs = newRelayTimerMs;
          EEPROM.put(eeAddrRelayTimerMs, relayTimerMs);
        }
        break;
      default:
        break;
    }
    synchronousModeWorkDone = true;
  }
}

void updateIndicators() {
  if (indicatorsNeedUpdate) {
    setLEDColor(color[indicatorStepTable[indicatorStepIndex][STEP_COLOR]]);
    nextUpdateIndicatorsMs = indicatorStepTable[indicatorStepIndex][STEP_DURATION];
    indicatorStepIndex = indicatorStepTable[indicatorStepIndex][STEP_NEXT_STEP];
    lastUpdateIndicatorsMs = currentMs;
    indicatorsNeedUpdate = false;
  }
}

void loop() {
  currentMs = millis();
  checkButtons();
  checkTimeouts();
  updateIndicators();
  doSynchronousModeWork();
}
