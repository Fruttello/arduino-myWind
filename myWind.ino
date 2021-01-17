/*
  myWind_v5.2

  Wind meter and safeguard for remote-controlled sun awnings: when the wind is too strong, rolls up the awnings to prevent damage

  Tutorial for Arduino self-learning purposes only, licensed by Fruttello 2021 under Creative Commons Zero v1.0 Universal (CC0).

  Requirements:
  1) Measure wind speed through a wind mill with pulse sensor (a Reed switch that generates 1 pulse per revolution)
  2) Compute average and max wind speeds over a sliding window of measurements
  3) Display instantaneous, average and max speeds on LCD with the selected speed unit
  4) Cycle speed display unit between m/s, km/h, mph and knots on short keypresses of a button (with debounce)
  5) Adjust LCD brightess on long keypresses of the same button used for display mode (ramps brightness progressively up to max then down to zero; repeat until button is released)
  6) If either average speed or max speed are above safety limits, light an alarm LED and send command to roll up the awnings

  N.B. this prototype cannot sense the current awning status (up, down or in transit), so the safeguard will keep sending "roll up"
  commands at set intervals until the alarm condition ends

  WIRING:

  Wind sensor:
  * Reed switch contact 1 --> +5 V
  * Reed switch contact 2 --> digital pin 3 (interrupt-enabled), connected in parallel to ground through a pull-down resistor (10 kohm)

  Display mode/brightness adjustment button:
  * Button contact 1 --> +5V
  * Button contact 2 --> digital pin 4, connected in parallel to ground through a pull-down resistor (10 kohm)

  Hitachi HD44780-compatible LCD:
  * Ground             VSS       LCD pin 1   --> ground
  * Power supply       VDD       LCD pin 2   --> +5 V
  * Contrast adjust    V0 or VEE LCD pin 3   --> ground (max contrast)
  * Register Select    RS        LCD pin 4   --> board digital pin 5
  * Read/Write         R/W       LCD pin 5   --> ground (write)
  * Enable             E         LCD pin 6   --> board digital pin 6
  * Data bit 0         DB0       LCD pin 7   disconnected (in 4 bit mode)
  * Data bit 1         DB1       LCD pin 8   disconnected (in 4 bit mode)
  * Data bit 2         DB2       LCD pin 9   disconnected (in 4 bit mode)
  * Data bit 3         DB3       LCD pin 10  disconnected (in 4 bit mode)
  * Data bit 4         DB4       LCD pin 11  --> board digital pin 7
  * Data bit 5         DB5       LCD pin 12  --> board digital pin 8
  * Data bit 6         DB6       LCD pin 13  --> board digital pin 9
  * Data bit 7         DB7       LCD pin 14  --> board digital pin 10
  * Back-light anode   A or LED+ LCD pin 15  --> +5V
  * Back-light cathode K or LED- LCD pin 16  --> drain (center pin) of a FET transistor (e.g. IRF520)
                                                 gate (left pin) --> digital pin 11 (PWM-enabled)
                                                 source (right pin) --> ground through a current-limiting 220 ohm resistor

  Alarm LED:
  * LED anode (long)     --> digital pin 12
  * LED cathode (short)  --> ground through a current-limiting 220 ohm resistor

  Awning remote control "UP" button, wired through a 6 pin 4N35 optocoupler (wiring for a 4 pin 817 optocoupler is similar, just omit the disconnected pins):
  * Optocoupler pin 1 (dot - anode)  --> digital pin 13
  * Optocoupler pin 2 (cathode)      --> ground through a current-limiting 220 ohm resistor
  * Optocoupler pin 3 (unused)       disconnected
  * Optocoupler pin 4 (emitter)      --> remote control button contact 1
  * Optocoupler pin 5 (collector)    --> remote control button contact 2
  * Optocoupler pin 6 (base)         disconnected
  N.B. pins are numbered counter-clockwise; pin 1 is identified with a dot; therefore pin 6 is opposite pin 1
*/



// ---- INCLUDE LIBRARIES

#include <Arduino.h>
#include <LiquidCrystal.h>



// ---- FUNCTION DECLARATIONS

void readWind();
void windPulseISR();
void checkButton();
void checkLimits();



// ---- CONSTANTS

const int WIND_SENSOR_PIN = 3;            // digital pin number for wind speed sensor (must be interrupt-enabled, i.e. pin 2 or 3 on the Uno)
const int WIND_SENSOR_INTERVAL = 1000;    // number of milliseconds between wind measurements
const double WIND_SENSOR_DIAMETER = 0.25; // diameter of the wind sensor mill in meters (to be adjusted by calibration)
const int WIND_PULSE_INTERVAL = 20;       // minimum number of milliseconds between valid wind sensor pulses (for debouncing); max measurable rotational speed is 1000/WIND_PULSE_INTERVAL rps
const int WIND_AVG_WINDOW = 30;           // number of samples for average wind speed calculation (sliding window); window duration = WIND_SENSOR_INTERVAL * WIND_AVG_WINDOW / 1000 seconds
const double WIND_AVG_LIMIT = 5.5;        // average wind speed limit in m/s; when average wind is above the limit then the awnings must be rolled up
const double WIND_MAX_LIMIT = 8.5;        // max wind speed limit in m/s; when max wind is above the limit then the awnings must be rolled up

const int BUTTON_PIN = 4;                 // digital pin number for display mode/brightness adjustment button
const int BUTTON_PRESS_LEVEL = HIGH;      // logical input level when button is pressed; here a pull-down resistor in the circuit ensures the button defaults to LOW when not pressed
const int BUTTON_SHORT_INTERVAL = 50;     // minimum duration of a short keypress in milliseconds (for debouncing display mode changes)
const int BUTTON_LONG_INTERVAL = 2000;    // minimum duration of a long keypress in milliseconds (for brightness adjustment)
const int BRIGHTNESS_INTERVAL = 250;      // number of milliseconds between brightness changes
const int BRIGHTNESS_INCREMENT = 10;      // percent increment (or decrement) for each brightness change during long keypress (submultiple of 100)
const size_t NUM_MODES = 4;               // number of display modes that can be cycled through; must match the number of units defined by the enum SpeedUnit
const String UNIT_LABEL[NUM_MODES] = { " m/s", " km/h", " mph", " kn" };  // label for the speed unit in each display mode, with a leading space
const double UNIT_FACTOR[NUM_MODES] = { 1.0, 3.6, 2.236936, 1.943844 }; // multiplicative conversion factor (from m/s) to be applied in each display mode

const int LCD_RS_PIN = 5;                 // LCD register-select
const int LCD_EN_PIN = 6;                 // LCD enable
const int LCD_D4_PIN = 7;                 // LCD data bit 4
const int LCD_D5_PIN = 8;                 // LCD data bit 5
const int LCD_D6_PIN = 9;                 // LCD data bit 6
const int LCD_D7_PIN = 10;                // LCD data bit 7
const int LCD_BKL_PIN = 11;               // LCD backlight brightness control (must be PWM-enabled, i.e. pin 3, 5, 6, 9, 10 or 11 on the Uno)

const int ALARM_LED_PIN = 12;             // digital pin number for alarm LED

const int COMMAND_UP_PIN = 13;            // digital pin number for awning roll-up command
const int COMMAND_INTERVAL = 10000;       // minimum number of milliseconds between commands
const int COMMAND_DURATION = 250;         // duration (in milliseconds) of a simulated keypress on the awning remote



// ---- GLOBAL VARIABLES

long currentMillis = 0;             // value of the on-board timer (in milliseconds) for the current iteration of loop()
volatile int pulseCount = 0;        // counter for wind sensor pulses, incremented by the interrupt handler (windPulseISR); the volatile qualifier is needed to disable compiler optimizations
double avgWind = 0.0;               // average wind speed in m/s over the sampling window
double maxWind = 0.0;               // max wind speed in m/s over the sampling window
LiquidCrystal lcd(LCD_RS_PIN, LCD_EN_PIN, LCD_D4_PIN, LCD_D5_PIN, LCD_D6_PIN, LCD_D7_PIN); // instance of the LiquidCrystal class, in 4 data bits mode
enum SpeedUnit {                    // speed display modes; the number of speed units must match the constant NUM_MODES; constant arrays UNIT_FACTOR[] and UNIT_LABEL[] must be initialized accordingly
  MS,     // m/s
  KMH,    // km/h
  MPH,    // customary US miles per hour
  KN      // knots (nautical miles per hour)
};
SpeedUnit displayMode = MS;         // current speed display mode, initialized to m/s


// ---- SOURCE CODE

// automatically runs once when the board is powered-up or reset
void setup() {

  // initialize digital pin for wind sensor as input
  pinMode(WIND_SENSOR_PIN, INPUT);

  // attach interrupt to the wind sensor pin; set windPulseISR as the interrupt handler; interrupts are triggered on rising signal fronts (a pull-down resistor in the circuit ensures the sensor defaults to LOW)
  attachInterrupt(digitalPinToInterrupt(WIND_SENSOR_PIN), windPulseISR, RISING);

  // initialize LCD as a 16x2 display (2 lines, 16 characters per line)
  lcd.begin(16,2);
  lcd.clear();

  // initialize LCD brightness control pin as output (PWM) and set brightness to max
  pinMode(LCD_BKL_PIN, OUTPUT);
  analogWrite(LCD_BKL_PIN, 255);

  // initialize digital pin for button as input; a pull-down resistor in the circuit ensures the button defaults to LOW (=button not pressed).
  pinMode(BUTTON_PIN, INPUT);

  // initialize alarm LED pin as output and switch it off
  pinMode(ALARM_LED_PIN, OUTPUT);
  digitalWrite(ALARM_LED_PIN, LOW);

  // initialize awning command pin as output and switch it off
  pinMode(COMMAND_UP_PIN, OUTPUT);
  digitalWrite(COMMAND_UP_PIN, LOW);
}

// automatically starts after setup and runs over and over forever
void loop () {
  currentMillis = millis(); // read time of current loop
  checkButton();
  readWind();
  checkLimits();
}

/* N.B. there is NO need to explicitily manage timer wrap (aka rollover, i.e. when millis() overflows and resets to 0, approx. every 50 days) when performing interval calculations (e.g. currentMillis - lastWindMillis in readWind) because
   two's complement arithmetics will automagically ensure that results are correct even across overflows IF all variables involved in the calculation are UNSIGNED integers (works with unsigned long, unsigned int and unsigned byte).
   Signed integers (e.g. WIND_SENSOR_INTERVAL) are implicitly converted by the compiler to unsigned long when mixed in expressions with unsigned long variables.
*/

// read sensor pulse counter, compute current wind speed, update the sliding window, compute average wind speed and update the display
void readWind() {
  static unsigned long lastWindMillis = 0;  // last time the wind speed was measured
  static double wind[WIND_AVG_WINDOW];      // array (sampling window) of most recent wind speed measurements in m/s; does not need to be initialized, see readWind()
  static int sampleCount = 0;               // number of wind speed samples collected so far; if < WIND_AVG_WINDOW then the sliding window is incomplete (initial transient)
  static int currentSample = 0;             // index of the current wind speed sample within the wind[] array

  int pulses = 0;                           // local buffer for wind pulse counter
  double rps = 0.0;                         // rotation speed of the sensor in revolutions per second (separate variable for debugging and clarity)
  double sum = 0.0;                         // sum of wind speed samples for average calculation

  // if time elapsed since last measurement < WIND_SENSOR_INTERVAL --> do nothing
  if (currentMillis - lastWindMillis >= WIND_SENSOR_INTERVAL) {

    // read and reset the wind pulse counter, disabling interrupts only for the minimum time required
    noInterrupts();
    pulses = pulseCount;
    pulseCount = 0;
    interrupts();

    // compute rotation speed in revolutions per second; if the wind sensor has multiple magnets (typically 2 or 4, for better mechanical balance and measurement accuracy) then just divide by the number of magnets (i.e. the number of pulses per revolution)
    rps = 1000.0 * pulses / (currentMillis - lastWindMillis); //  multiply by 1000.0 (float) because time interval is measured in milliseconds

    // compute tangential speed in m/s and store it in the array, overwriting the oldest sample
    wind[currentSample] = 3.141593 * WIND_SENSOR_DIAMETER * rps;

    // increment the number of samples collected during the initial transient (i.e. when less than WIND_AVG_WINDOW samples have been collected)
    if (sampleCount < WIND_AVG_WINDOW)
      sampleCount++;

    // sum the wind speed samples in the array and find the max; sampleCount is used as the upper bound index for the loop (instead of WIND_AVG_WINDOW) so the wind[] array does not need to be initialized
    maxWind = 0.0;
    for (int i = 0; i < sampleCount; i++) {
      sum += wind[i]; // sum is a local variable initialized to 0
      if (maxWind < wind[i])
        maxWind = wind[i];
    }

    // compute average wind speed
    avgWind = sum / sampleCount;

    // display current, average and max wind speed on LCD with the selected unit (display mode)
    lcd.clear();
    lcd.setCursor(0, 0);  // line 0 = first line
    lcd.print("Wind=");
    lcd.print(wind[currentSample] * UNIT_FACTOR[displayMode]);
    lcd.print(UNIT_LABEL[displayMode]);
    lcd.setCursor(0, 1);  // line 1 = second line
    lcd.print("A=");
    lcd.print(avgWind * UNIT_FACTOR[displayMode]);
    lcd.print("  M=");
    lcd.print(maxWind * UNIT_FACTOR[displayMode]);

    // increment or reset the current sample index
    if (currentSample < WIND_AVG_WINDOW - 1)
      currentSample++;
    else
      currentSample = 0;

    // lastWindMillis is set to the time when the last measurement was scheduled to occur (as opposed to when it did actually occur, e.g. lastWindMillis = currentMillis)
    // if - for any reason - an interval between measurements turns out to be longer than nominal, then the following interval will automatically be shorter to compensate
    // on average, interval duration will be close to nominal (no drift) and average speed calculation will be correct
    lastWindMillis += WIND_SENSOR_INTERVAL;
  }
}

// check button status to cycle display mode (short keypress) or adjust display brightness (long keypress)
void checkButton() {
  static int wasButtonPressed = 0;                              // wasButtonPressed = TRUE (1) if the button was pressed during the previous call of checkButton
  static unsigned long lastButtonMillis;                        // last time a button was pressed (rising signal front)
  static int displayBrightness = 100;                           // current LCD brightness in percent
  static int brightnessDirection = -1;                          // current brightness change direction (1 = increase; -1 = decrease)
  static unsigned long lastBrightnessMillis = 0;                // last time brightness was changed

  int isButtonPressed = (digitalRead(BUTTON_PIN) == BUTTON_PRESS_LEVEL); // isButtonPressed = TRUE (1) if the button is pressed
  int onPress = isButtonPressed && !wasButtonPressed;           // onPress = TRUE (1) on a rising signal front
  int onRelease = !isButtonPressed && wasButtonPressed;         // onRelease = TRUE (1) on a falling signal front
  wasButtonPressed = isButtonPressed;

  if (onPress) lastButtonMillis = currentMillis;                // record the time of the latest rising signal front
  unsigned long pressDuration = currentMillis - lastButtonMillis; // time elapsed since last rising signal front

  // cycle display mode on a falling signal front after a "short" keypress
  // keypresses shorter than BUTTON_SHORT_INTERVAL are ignored as noise (debouncing)
  // keypresses longer than BUTTON_LONG_INTERVAL are ignored as brightness adjustments (only the falling signal front is processed here)
  if (onRelease && (pressDuration >= BUTTON_SHORT_INTERVAL) && (pressDuration < BUTTON_LONG_INTERVAL))
  {
    if ( displayMode < NUM_MODES-1 )
      displayMode = (SpeedUnit)(displayMode+1); // explicit type casting is required to appease the compiler: we know what we're doing by assigning an integer (result of the calculation) to an enum
    else
      displayMode = MS;
  }

  // adjust display brightness on a "long" continuous keypress, but only once every BRIGHTNESS_INTERVAL milliseconds
  if (isButtonPressed && (pressDuration >= BUTTON_LONG_INTERVAL) && (currentMillis - lastBrightnessMillis >= BRIGHTNESS_INTERVAL))
  {
    displayBrightness += BRIGHTNESS_INCREMENT*brightnessDirection;
    analogWrite(LCD_BKL_PIN, 255*displayBrightness/100);
    if (displayBrightness >= 100) brightnessDirection = -1;
    if (displayBrightness <= 0) brightnessDirection = 1;
    lastBrightnessMillis = currentMillis;
  }
}

// check wind speed limits and roll up the awnings if needed
void checkLimits() {
  static unsigned long lastCommandMillis = 0;// last time an awning command was sent

  if (avgWind >= WIND_AVG_LIMIT || maxWind >= WIND_MAX_LIMIT) {
    digitalWrite(ALARM_LED_PIN, HIGH);

    // if the time elapsed since last awning command is less than COMMAND_INTERVAL --> do nothing
    if (currentMillis - lastCommandMillis > COMMAND_INTERVAL) {
      digitalWrite(COMMAND_UP_PIN, HIGH);
      delay(COMMAND_DURATION);
      digitalWrite(COMMAND_UP_PIN, LOW);
      lastCommandMillis = currentMillis;
    }
  }
  else
    digitalWrite(ALARM_LED_PIN, LOW);
}

// ISR = Interrupt Service Routine (interrupt handler): callback function triggered by rising signal fronts on WIND_SENSOR_PIN
void windPulseISR() {
  static unsigned long lastPulseMillis = 0;  // last time a sensor pulse was processed

  // if the time elapsed since last pulse < WIND_PULSE_INTERVAL --> do nothing (ignore the pulse as noise)
  if (currentMillis - lastPulseMillis >= WIND_PULSE_INTERVAL) {
    pulseCount++; // pulseCount does not overflow because it's reset every WIND_SENSOR_INTERVAL milliseconds by readWind()
    lastPulseMillis=currentMillis;
  }
}
