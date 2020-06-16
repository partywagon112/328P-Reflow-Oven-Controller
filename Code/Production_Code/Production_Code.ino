/*******************************************************************************
  Title: Reflow Oven Controller
  Version: 1.20
  Date: 26-11-2012
  Company: Rocket Scream Electronics
  Author: Lim Phang Moh
  Website: www.rocketscream.com
  Modified by Patrick Curtain for use with use with attached PCB.

  Temperature (Degree Celcius)                 Magic Happens Here!
  224-|                                               x  x
      |                                            x        x
      |                                         x              x
      |                                      x                    x
  180-|                                   x                          x
      |                              x    |                          |   x
      |                         x         |                          |       x
      |                    x              |                          |
  150-|               x                   |                          |
      |             x |                   |                          |
      |           x   |                   |                          |
      |         x     |                   |                          |
      |       x       |                   |                          |
      |     x         |                   |                          |
      |   x           |                   |                          |
  30 -| x             |                   |                          |
      |<  60 - 90 s  >|<    90 - 120 s   >|<       90 - 120 s       >|
      | Preheat Stage |   Soaking Stage   |       Reflow Stage       | Cool
   0  |_ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
                                                                 Time (Seconds)

  This firmware owed very much on the works of other talented individuals as
  follows:
  ==========================================
  Brett Beauregard (www.brettbeauregard.com)
  ==========================================
  Author of Arduino PID library. On top of providing industry standard PID
  implementation, he gave a lot of help in making this reflow oven controller
  possible using his awesome library.

  ==========================================
  Limor Fried of Adafruit (www.adafruit.com)
  ==========================================
  Author of Arduino MAX6675 library. Adafruit has been the source of tonnes of
  tutorials, examples, and libraries for everyone to learn.

  Disclaimer
  ==========
  Dealing with high voltage is a very dangerous act! Please make sure you know
  what you are dealing with and have proper knowledge before hand. Your use of
  any information or materials on this reflow oven controller is entirely at
  your own risk, for which we shall not be liable.

  Licences
  ========
  This reflow oven controller hardware and firmware are released under the
  Creative Commons Share Alike v3.0 license
  http://creativecommons.org/licenses/by-sa/3.0/
  You are free to take this piece of code, use it and modify it.
  All we ask is attribution including the supporting libraries used in this
  firmware.
  Huge thankyou to Lim Phang Moh at ROCKETSTREAM for providing such an open platform
  to work on and build these things.

  Required Libraries
  ==================
  - Arduino PID Library:
    >> https://github.com/br3ttb/Arduino-PID-Library
  - MAX6675 Library:
    >> https://github.com/adafruit/MAX6675-library
  - U8G2 Library:
    >> https://github.com/olikraus/u8g2
*******************************************************************************/

// ***** INCLUDES *****
#include <U8g2lib.h>
#include <max6675.h>
#include <PID_v1.h>

// ***** TYPE DEFINITIONS *****
typedef enum REFLOW_STATE
{
  REFLOW_STATE_IDLE,
  REFLOW_STATE_PREHEAT,
  REFLOW_STATE_SOAK,
  REFLOW_STATE_REFLOW,
  REFLOW_STATE_COOL,
  REFLOW_STATE_COMPLETE,
  REFLOW_STATE_TOO_HOT,
  REFLOW_STATE_TIMEOUT,
  REFLOW_STATE_ERROR
} reflowState_t;

typedef enum REFLOW_STATUS
{
  REFLOW_STATUS_OFF,
  REFLOW_STATUS_ON
} reflowStatus_t;

typedef enum SWITCH
{
  SWITCH_NONE,
  SWITCH_1,
  SWITCH_2
} switch_t;

typedef enum DEBOUNCE_STATE
{
  DEBOUNCE_STATE_IDLE,
  DEBOUNCE_STATE_CHECK,
  DEBOUNCE_STATE_RELEASE
} debounceState_t;

// ***** CONSTANTS *****
#define TEMPERATURE_ROOM 50
#define TEMPERATURE_SOAK_MIN 150
#define TEMPERATURE_SOAK_MAX 180
#define TEMPERATURE_REFLOW_MAX 224
#define TEMPERATURE_COOL_MIN 100
#define SENSOR_SAMPLING_TIME 1000
#define SOAK_TEMPERATURE_STEP 5
#define SOAK_MICRO_PERIOD 10000
#define DEBOUNCE_PERIOD_MIN 50
#define REFLOW_TIMEOUT 500

// ***** PID PARAMETERS *****
// ***** PRE-HEAT STAGE *****
#define PID_KP_PREHEAT 100
#define PID_KI_PREHEAT 0.025
#define PID_KD_PREHEAT 20
// ***** SOAKING STAGE *****
#define PID_KP_SOAK 300
#define PID_KI_SOAK 0.05
#define PID_KD_SOAK 250
// ***** REFLOW STAGE *****
#define PID_KP_REFLOW 300
#define PID_KI_REFLOW 0.05
#define PID_KD_REFLOW 350
#define PID_SAMPLE_TIME 1000

// ***** LCD MESSAGES *****
const char* lcdMessagesReflowStatus[] = {
  "Ready",
  "Preheat",
  "Soak",
  "Reflow",
  "Cool",
  "Complete",
  "WAIT, HOT!",
  "Error"
};

// ***** PIN ASSIGNMENT *****
int ssrPin = 5;
int thermocoupleSOPin = 12;
int thermocoupleCSPin = 10;
int thermocoupleCLKPin = 13;
int switch1Pin = 2;
int switch2Pin = 3;

// ***** PID CONTROL VARIABLES *****
double setpoint;
double input;
double output;
double kp = PID_KP_PREHEAT;
double ki = PID_KI_PREHEAT;
double kd = PID_KD_PREHEAT;
int windowSize;
unsigned long windowStartTime;
unsigned long nextCheck;
unsigned long nextRead;
unsigned long timerSoak;
unsigned long buzzerPeriod;

// Reflow oven controller state machine state variable
reflowState_t reflowState;
// Reflow oven controller status
reflowStatus_t reflowStatus;
// Switch debounce state machine state variable
debounceState_t debounceState;
// Switch debounce timer
long lastDebounceTime;
// Switch press status
switch_t switchStatus;
// Seconds timer
int timerSeconds;

// Specify PID control interface
PID reflowOvenPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

// Specify OLED interface
U8G2_SSD1306_128X32_UNIVISION_F_SW_I2C oled(U8G2_R2, SCL, SDA, U8X8_PIN_NONE);

// Specify thermocouple interface
MAX6675 thermocouple(thermocoupleCLKPin, thermocoupleCSPin, thermocoupleSOPin);


// FUNCTIONS


void stateMachine() {
  // Reflow oven controller state machine
  switch (reflowState)
  {
    case REFLOW_STATE_IDLE:
      // If oven temperature is still above room temperature
      setpoint = 0;
      timerSeconds = 0;
      if (input >= TEMPERATURE_ROOM)
      {
        reflowState = REFLOW_STATE_TOO_HOT;
      }
      else
      {
        // If switch is pressed to start reflow process
        if (switchStatus == SWITCH_1)
        {
          // Send header for CSV file
          Serial.println("Time Setpoint Input Output");
          // Intialize seconds timer for serial debug information
          timerSeconds = 0;
          // Initialize PID control window starting time
          windowStartTime = millis();
          // Ramp up to minimum soaking temperature
          setpoint = TEMPERATURE_SOAK_MIN;
          // Tell the PID to range between 0 and the full window size
          reflowOvenPID.SetOutputLimits(0, windowSize);
          reflowOvenPID.SetSampleTime(PID_SAMPLE_TIME);
          // Turn the PID on
          reflowOvenPID.SetMode(AUTOMATIC);
          // Proceed to preheat stage
          reflowState = REFLOW_STATE_PREHEAT;
        }
      }
      break;

    case REFLOW_STATE_PREHEAT:
      reflowStatus = REFLOW_STATUS_ON;
      // If minimum soak temperature is achieve
      if (input >= TEMPERATURE_SOAK_MIN)
      {
        // Chop soaking period into smaller sub-period
        timerSoak = millis() + SOAK_MICRO_PERIOD;
        // Set less agressive PID parameters for soaking ramp
        reflowOvenPID.SetTunings(PID_KP_SOAK, PID_KI_SOAK, PID_KD_SOAK);
        // Ramp up to first section of soaking temperature
        setpoint = TEMPERATURE_SOAK_MIN + SOAK_TEMPERATURE_STEP;
        // Proceed to soaking state
        reflowState = REFLOW_STATE_SOAK;
      }
      break;

    case REFLOW_STATE_SOAK:
      // If micro soak temperature is achieved
      if (millis() > timerSoak)
      {
        timerSoak = millis() + SOAK_MICRO_PERIOD;
        // Increment micro setpoint
        setpoint += SOAK_TEMPERATURE_STEP;
        if (setpoint > TEMPERATURE_SOAK_MAX)
        {
          // Set agressive PID parameters for reflow ramp
          reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
          // Ramp up to first section of soaking temperature
          setpoint = TEMPERATURE_REFLOW_MAX;
          // Proceed to reflowing state
          reflowState = REFLOW_STATE_REFLOW;
        }
      }
      break;

    case REFLOW_STATE_REFLOW:
      // We need to avoid hovering at peak temperature for too long
      // Crude method that works like a charm and safe for the components
      if (input >= (TEMPERATURE_REFLOW_MAX - 5))
      {
        // Set PID parameters for cooling ramp
        reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
        // Ramp down to minimum cooling temperature
        setpoint = TEMPERATURE_COOL_MIN;
        // Proceed to cooling state
        reflowState = REFLOW_STATE_COOL;
      }
      break;

    case REFLOW_STATE_COOL:
      // If minimum cool temperature is achieve
      if (input <= TEMPERATURE_COOL_MIN)
      {
        buzzerPeriod = millis() + 1000;
        // Turn off reflow process
        reflowStatus = REFLOW_STATUS_OFF;
        // Proceed to reflow Completion state
        reflowState = REFLOW_STATE_COMPLETE;
      }
      break;

    case REFLOW_STATE_COMPLETE:
      if (millis() > buzzerPeriod)
      {
        // Reflow process ended
        reflowState = REFLOW_STATE_IDLE;
        setpoint = 0;
      }
      break;

    case REFLOW_STATE_TOO_HOT:
      // If oven temperature drops below room temperature
      if (input < TEMPERATURE_ROOM)
      {
        // Ready to reflow
        reflowState = REFLOW_STATE_IDLE;
      }
      break;

    case REFLOW_STATE_TIMEOUT:
      if (timerSeconds > REFLOW_TIMEOUT)
        reflowState = REFLOW_STATE_IDLE;
      break;    

    case REFLOW_STATE_ERROR:
      // If thermocouple problem is still present
      if (isnan(input))
      {
        // Wait until thermocouple wire is connected
        reflowState = REFLOW_STATE_ERROR;
      }
      else
      {
        // Clear to perform reflow process
        reflowState = REFLOW_STATE_IDLE;
      }
      break;
  }
}

void updateInput() {
  // If switch 1 is pressed
  if (switchStatus == SWITCH_1)
  {
    // If currently reflow process is on going
    if (reflowStatus == REFLOW_STATUS_ON)
    {
      // Button press is for cancelling
      // Turn off reflow process
      reflowStatus = REFLOW_STATUS_OFF;
      // Reinitialize state machine
      reflowState = REFLOW_STATE_IDLE;
    }
  }

  // Simple switch debounce state machine (for switch #1 (both analog & digital
  // switch supported))
  switch (debounceState)
  {
    case DEBOUNCE_STATE_IDLE:
      // No valid switch press
      switchStatus = SWITCH_NONE;
      // If switch #1 is pressed
      if (digitalRead(switch1Pin) == LOW)
      {
        // Intialize debounce counter
        lastDebounceTime = millis();
        // Proceed to check validity of button press
        debounceState = DEBOUNCE_STATE_CHECK;
      }
      break;

    case DEBOUNCE_STATE_CHECK:
      // If switch #1 is still pressed
      if (digitalRead(switch1Pin) == LOW)
      {
        // If minimum debounce period is completed
        if ((millis() - lastDebounceTime) > DEBOUNCE_PERIOD_MIN)
        {
          // Proceed to wait for button release
          debounceState = DEBOUNCE_STATE_RELEASE;
        }
      }
      // False trigger
      else
      {
        // Reinitialize button debounce state machine
        debounceState = DEBOUNCE_STATE_IDLE;
      }
      break;

    case DEBOUNCE_STATE_RELEASE:
      if (digitalRead(switch1Pin) == HIGH)
      {
        // Valid switch 1 press
        switchStatus = SWITCH_1;
        // Reinitialize button debounce state machine
        debounceState = DEBOUNCE_STATE_IDLE;
      }
      break;
  }
}

void updateRelay() {
  unsigned long now;
  // PID computation and SSR control
  if (reflowStatus == REFLOW_STATUS_ON) {
    now = millis();
    reflowOvenPID.Compute();
    if ((now - windowStartTime) > windowSize) {
      // Time to shift the Relay Window
      windowStartTime += windowSize;
    }
    if (output > (now - windowStartTime))
      digitalWrite(ssrPin, HIGH);
    else
      digitalWrite(ssrPin, LOW);
  }
  // Reflow oven process is off, ensure oven is off

  else
  {
    digitalWrite(ssrPin, LOW);
  }
}

void updateTemperature() {
  // Time to read thermocouple?
  if (millis() > nextRead)
  {
    // Read thermocouple next sampling period
    nextRead += SENSOR_SAMPLING_TIME;
    // Read current temperature
    input = thermocouple.readCelsius();

    // If thermocouple problem detected
    if (isnan(input))
    {
      // Illegal operation
      reflowState = REFLOW_STATE_ERROR;
      reflowStatus = REFLOW_STATUS_OFF;
    }
  }
}

void printDisplay() {
  if (millis() > nextCheck)
  {
    // Check input in the next seconds
    nextCheck += 1000;
    // If reflow process is on going
    if (reflowStatus == REFLOW_STATUS_ON) {
      // Increase seconds timer for reflow curve analysis
      timerSeconds++;
      // Send temperature and time stamp to serial
      Serial.print(timerSeconds);
      Serial.print(" ");
      Serial.print(setpoint);
      Serial.print(" ");
      Serial.print(input);
      Serial.print(" ");
      Serial.println(output);
    }
    
    oled.firstPage();
    do {
      oled.setDrawColor(2);
      oled.setFontMode(0);
      
      oled.drawStr(0,10, lcdMessagesReflowStatus[reflowState]);
         
      if (reflowState == REFLOW_STATE_ERROR) {
          Serial.println("Something's gone wrong!");
        oled.drawStr(0, 30, "TC Error!");
      }
      else {
        // Print current temperature
        oled.setCursor(0, 30);
        oled.print(input);
        if (setpoint > 0){
          oled.print("->");
          oled.setDrawColor(0);
          oled.print(setpoint);
        }
        oled.setDrawColor(2);
        oled.print("°C");
        if (timerSeconds > 0){
          char str[20];
          oled.drawStr(oled.getDisplayWidth()-oled.getStrWidth(itoa(timerSeconds,str, 10)) - 10, 10, itoa(timerSeconds,str, 3));
          // ^ i'll admit it, this is the most wasteful function in history. WHAT OF IT!
        }
                
      }
    } while (oled.nextPage());
  }
}

void setup() {
  Serial.println("STARTING");
  // SSR pin initialization to ensure reflow oven is off
  digitalWrite(ssrPin, LOW);
  pinMode(ssrPin, OUTPUT);

  pinMode(switch1Pin, INPUT);
  pinMode(switch2Pin, INPUT);


  oled.begin();
  oled.setFont(u8g2_font_8x13_mf);
  oled.clearDisplay();
  oled.clearBuffer();
  oled.firstPage();
  oled.enableUTF8Print();

  // Start-up splash
  oled.drawStr(0, 10, "I am no longer ");
  oled.drawStr(0, 30, "a toaster oven!");
  oled.sendBuffer();
  delay(2500);

  // Serial communication at 57600 bps
  Serial.begin(115200);

  // Set window size
  windowSize = 2000;
  // Initialize time keeping variable
  nextCheck = millis();
  // Initialize thermocouple reading variable
  nextRead = millis();

  oled.clearBuffer();
}

void loop() {
  printDisplay();
  stateMachine();
  updateTemperature();
  updateInput();
  updateRelay();
}
