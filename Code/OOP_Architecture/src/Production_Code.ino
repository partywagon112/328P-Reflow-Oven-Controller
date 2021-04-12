/*******************************************************************************
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

// ***** CONSTANTS *****
char SPASH_LINE1[] = {"Reflowinator"};
char SPASH_LINE2[] = {"PCB COOKER V1"};

// ***** PARAMETERS *****
const int TEMPERATURE_PREHEAT_MAX = 150;
const int TEMPERATURE_COOL_MIN = 100;
const int TEMPERATURE_SOAK_STEP = 5;      // Temperature steps in which to increment temperature.

const int LEADED_TEMPERATURE_SOAK_MAX = 180;
const int LEADED_TEMPERATURE_REFLOW_MAX = 224;
const int LEADED_SOAK_PERIOD = 9000;

const int UNLEADED_TEMPERATURE_SOAK_MAX = 200;
const int UNLEADED_TEMPERATURE_REFLOW_MAX = 250;
const int UNLEADED_SOAK_PERIOD = 9000;

const int DEBOUNCE_PERIOD_MIN = 50;       // 50 ms for debouncing input switches.
const int RELAY_UPDATE_PERIOD = 2000;     // Relay PWM Frequency 0.5 Hz.
const int COMPLETE_TIME = 5000;           // 5 seconds to show completed status.

// ***** LIMITS *****
const float TEMPERATURE_SAFE = 50;
const float MAX_TEMP = 1000;
const float MIN_TEMP = -40; 
const int MAX_TIMEOUT = 1000000;          //1000 seconds

// ***** PID PARAMETERS *****
// ***** DEFAULT *****
#define PID_KP_DEFAULT 300
#define PID_KI_DEFAULT 0.05
#define PID_KD_DEFAULT 250
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

// ***** PIN ASSIGNMENT *****
#define SSR_PIN 5
#define THERMOCOUPLE_SO_PIN 12
#define THERMOCOUPLE_CS_PIN 10
#define THERMOCOUPLE_CLK_PIN 13
#define SWITCH_MODE_PIN 2
#define SWITCH_START_STOP_PIN 3

// ***** TYPE DEFINITIONS *****
typedef enum MACHINE_STATE {
  IDLE_LEADED,
  IDLE_UNLEADED,
  PREHEAT,
  SOAK,
  REFLOW,
  COOL,
  COMPLETE,
  HOT,
  ERROR,
  INTERACTIVE
} machine_state_t;

typedef enum ERROR_STATE {
  ERROR_STATE,
  ERROR_TEMP_SENSOR_FAILURE,
  ERROR_TIMEOUT_EXCEEDED
} error_state_t;

typedef enum VERBOSE_STATE {
  VERBOSE_0,    // No logging except for when REFLOW_STATUS_ON.
  VERBOSE_1,    // Log temperature and time every second.
  VERBOSE_2     // Not quite sure yet.
} verbose_state_t;

typedef enum DEBOUNCE_STATE {
  DEBOUNCE_STATE_IDLE,
  DEBOUNCE_STATE_CHECK,
  DEBOUNCE_STATE_RELEASE
} debounce_state_t;

typedef enum HEATER_STATE {
  HEATER_ACTIVE,
  HEATER_DISABLE
} heater_state_t;

// ***** OLED MESSAGES *****
const char* message_machine_state[] = {
  "Leaded",
  "Unleaded",
  "Preheat",
  "Soak",
  "Reflow",
  "Cool",
  "Complete",
  "Wait, Hot!",
  "Error",
  "Interactive"
};

const char* message_error_status[] = {
  "Safe",
  "Sensor Fail",
  "Timeout"
};

// ***** OBJECTS *****

// CLASS: SWITCH
// 
// Object describiing a physically button interface
// - Use update_switch() with program loop to constantly poll the state of the button,
//   and perform debouncing function.
// - Use get_switch() to access the state of the switch. 
// 
class Switch {
  public:
    Switch::Switch(){}
    Switch::Switch(int i_switch_pin){
      switch_pin = i_switch_pin;
    }

    void Switch::update_switch(){
      switch_state = 0;

      switch (debounce_state){
        case DEBOUNCE_STATE_IDLE:
          switch_state = 0;
          if (digitalRead(switch_pin) == LOW){
            last_debounce_time = millis();
            debounce_state = DEBOUNCE_STATE_CHECK;
          }
          break;

        case DEBOUNCE_STATE_CHECK:
          if (digitalRead(switch_pin) == LOW){
            if ((millis() - last_debounce_time) > DEBOUNCE_PERIOD_MIN){
              debounce_state = DEBOUNCE_STATE_RELEASE;
            } 
          }
          break;
        
        case DEBOUNCE_STATE_RELEASE:
          if (digitalRead(switch_pin) == HIGH){
            switch_state = 1;
            debounce_state = DEBOUNCE_STATE_IDLE;
          }
          break;
      }
    }

    bool Switch::get_state(){
      return switch_state;
    }

  private:
    int switch_pin;
    bool switch_state;
    debounce_state_t debounce_state;
    unsigned long last_debounce_time;
};

// CLASS: HEATER INTERFACE
// 
// Object describiing the relation of the heating element and the temperature sensor. Maintains
// internal PID controller to control the duty cycle of the temperature sensor.
// - Use update_relay() within program loop to constantly update the solid state relay used
//   to control the heating element. 
// - Use update_temperature() within program loop to constatly update the current measured temperature
//   and check the validity of the sensors measurements.
// - Use configure_PID() to set the parameters of the PID. 
// 
class Heater{
  public:
    Heater::Heater(const int i_ssr_relay_pin, int i_thermocouple_clk_pin, int i_thermocouple_cs_pin, int i_thermocouple_so_pin) : 
      thermocouple(i_thermocouple_clk_pin, i_thermocouple_cs_pin, i_thermocouple_so_pin), 
      heater_pid(&input, &output, &setpoint, kp, ki, kd, DIRECT) {
      ssr_relay_pin = i_ssr_relay_pin;

      pinMode(ssr_relay_pin, OUTPUT);
      digitalWrite(ssr_relay_pin, LOW);

      heater_state = HEATER_DISABLE;

      heater_pid.SetTunings(PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT);
      heater_pid.SetOutputLimits(0, RELAY_UPDATE_PERIOD);
      heater_pid.SetMode(AUTOMATIC);  
    }

    bool Heater::update_temperature(){
      bool safe = false;
      if (millis() - last_set_time > MAX_TIMEOUT){
        timeout_failure_flag = ERROR_TIMEOUT_EXCEEDED;
        safe = false;
      }

      double thermocouple_temperature = thermocouple.readCelsius();
      if (isnan(thermocouple_temperature) || thermocouple_temperature > MAX_TEMP || thermocouple_temperature < MIN_TEMP) {
        temp_sensor_failure_flag = ERROR_TEMP_SENSOR_FAILURE;
        safe = false;
      } else {
        input = thermocouple_temperature;
        temp_sensor_failure_flag = ERROR_STATE;
        safe = true;
      }

      return safe;
    }

    void Heater::update_relay(){
      heater_pid.Compute();

      if (heater_state == HEATER_ACTIVE){      
        // Checks if current period has been exceeded, reset initial value to start of
        // next period.
        if (millis() - window_start_time > RELAY_UPDATE_PERIOD){
          window_start_time += RELAY_UPDATE_PERIOD;
        }
        
        // The value from output is a peroid, if that period is exceeded within this window
        // disable relay.
        if (output > millis() - window_start_time){
          digitalWrite(ssr_relay_pin, HIGH);
        } else {
          digitalWrite(ssr_relay_pin, LOW);
        }

      } else if (heater_state == HEATER_DISABLE) {
        digitalWrite(ssr_relay_pin, LOW);
      }
    }

    void Heater::configure_PID(double i_kp, double i_ki, double i_kd){
      kp = i_kp;
      ki = i_ki;
      kd = i_kd;
      heater_pid.SetTunings(kp, ki, kd);
    }

    bool Heater::set_temperature(double set_temperature){
      if (set_temperature > MAX_TEMP || set_temperature < MIN_TEMP){
        return false;
      } else {
        last_set_time = millis();

        heater_state = HEATER_ACTIVE;

        setpoint = set_temperature;
        return true;
      }
    }

    float Heater::get_temperature(){
      return input;
    }

    float Heater::get_set_temperature(){
      return setpoint;
    }

    error_state_t Heater::get_first_error(){
      error_state_t temp = ERROR_STATE;
      if (temp_sensor_failure_flag != ERROR_STATE){
        temp = ERROR_TEMP_SENSOR_FAILURE;
        return temp;
      }

      if (timeout_failure_flag != ERROR_STATE){
        temp = ERROR_TIMEOUT_EXCEEDED;
        return temp;
      }
    }

    float return_PID_effort(){
      return output/RELAY_UPDATE_PERIOD;
    }

  private:
    heater_state_t heater_state;

    error_state_t temp_sensor_failure_flag;
    error_state_t timeout_failure_flag;

    unsigned long last_set_time;

    int ssr_relay_pin;
    bool ssr_relay;

    double input;
    double output;
    double setpoint;

    double kp;
    double ki;
    double kd;

    int window_size;
    unsigned long window_start_time;
    unsigned long nextCheck;
    unsigned long nextRead;

    MAX6675 thermocouple;

    PID heater_pid;
};

// CLASS: MACHINE INTERFACE
// 
// Object describiing the state controller that subsecuentially controls the temperature interface, switch interfaces
// and 
// - Use update_state() with program loop to persistently monitor and update the state machine, and update the
//   states accordingly. 
// - Use get_state() to return the current machine state.
// 
class State_Machine{
  public:
    void update_state(Heater &heater_interface, Switch &mode, Switch &start){

      // Update GPIO
      mode.update_switch();
      start.update_switch();

      // Update Heater
      heater_interface.update_relay();
      if (heater_interface.update_temperature() == false){
        machine_state = ERROR;
      }
      
      switch(machine_state){
        case IDLE_LEADED:
          start_time = 0;
          heater_interface.set_temperature(0);

          if (mode.get_state() == true)
            machine_state = IDLE_UNLEADED;

          if (start.get_state()){
            machine_state = PREHEAT;

            start_time = millis();

            heater_interface.set_temperature(TEMPERATURE_PREHEAT_MAX);
            heater_interface.configure_PID(PID_KP_PREHEAT, PID_KI_PREHEAT, PID_KD_PREHEAT);

            temperature_soak_max = LEADED_TEMPERATURE_SOAK_MAX;
            temperature_reflow_max = LEADED_TEMPERATURE_REFLOW_MAX;
            soak_period = LEADED_SOAK_PERIOD;          
          }

          if (heater_interface.get_temperature() > TEMPERATURE_SAFE)
            machine_state = HOT;
         break;

        case IDLE_UNLEADED:
          start_time = 0;
          heater_interface.set_temperature(0);

          if (mode.get_state() == true)
            machine_state = INTERACTIVE;

          if (start.get_state()){
            machine_state = PREHEAT;

            start_time = millis();

            heater_interface.set_temperature(TEMPERATURE_PREHEAT_MAX);
            heater_interface.configure_PID(PID_KP_PREHEAT, PID_KI_PREHEAT, PID_KD_PREHEAT);

            temperature_soak_max = LEADED_TEMPERATURE_SOAK_MAX;
            temperature_reflow_max = LEADED_TEMPERATURE_REFLOW_MAX;
            soak_period = LEADED_SOAK_PERIOD;  
          }

          if (heater_interface.get_temperature() > TEMPERATURE_SAFE)
            machine_state = HOT;
          break;

        case PREHEAT:
          if (start.get_state() || mode.get_state())
            machine_state = IDLE_LEADED;

          if (heater_interface.get_temperature() >= TEMPERATURE_PREHEAT_MAX - 5){
            soak_timer = millis() + soak_period;
            
            heater_interface.configure_PID(PID_KP_SOAK, PID_KI_SOAK, PID_KD_SOAK); 
            heater_interface.set_temperature(TEMPERATURE_PREHEAT_MAX + TEMPERATURE_SOAK_STEP);            
            
            machine_state = SOAK;
          }
          break;

        case SOAK:
          if (start.get_state() || mode.get_state())
              machine_state = IDLE_LEADED;
          
          if (millis() > soak_timer){
            soak_timer = millis() + soak_period;
            heater_interface.set_temperature(heater_interface.get_set_temperature() + TEMPERATURE_SOAK_STEP);            
            
            if (heater_interface.get_set_temperature() > temperature_soak_max){
              heater_interface.configure_PID(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
              heater_interface.set_temperature(temperature_reflow_max);
              machine_state = REFLOW;
            }
          }
          break;

        case REFLOW:
          if (start.get_state() || mode.get_state())
              machine_state = IDLE_LEADED;
        
          if (heater_interface.get_temperature() >= temperature_reflow_max - 5){
            heater_interface.configure_PID(PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT);
            heater_interface.set_temperature(TEMPERATURE_COOL_MIN);
            machine_state = COOL;
          }
          break;

        case COOL:
          if (start.get_state() || mode.get_state())
              machine_state = IDLE_LEADED;

          if (heater_interface.get_temperature() <= TEMPERATURE_COOL_MIN){
            machine_state = COMPLETE;
            complete_start_time = millis();
          }
          break;

        case COMPLETE:
          if (millis() - complete_start_time > COMPLETE_TIME)
            machine_state = IDLE_LEADED;
          break;
          
        case HOT:
          start_time = 0;
          heater_interface.set_temperature(0);

          if (heater_interface.get_temperature() < TEMPERATURE_SAFE)
            machine_state = IDLE_LEADED;
          break;
        
        case ERROR:
          if (heater_interface.update_temperature())
            machine_state = IDLE_LEADED;
          break;

        case INTERACTIVE:
          if (mode.get_state() == true)
              machine_state = IDLE_LEADED;

          interactive_handler(heater_interface);
          break;
      }

      // Print Status
      print_status(heater_interface);
    }

    machine_state_t get_machine_state(){
      return machine_state;
    }

    int get_current_time(){
      if (start_time != 0){
        return (millis() - start_time)/1000;
      } else {
        return 0;
      }
    }

  private:
    machine_state_t machine_state;
    
    unsigned long start_time;
    unsigned long last_print_time;
    unsigned long soak_period;
    unsigned long soak_timer;
    unsigned long complete_start_time;
    
    float temperature_soak_max;
    float temperature_reflow_max;

    void interactive_handler(Heater &heater_interface){
      while (Serial.available()) {
        
        start_time = millis();

        char inChar = (char)Serial.read();

        switch (inChar) {
          case 'T':
            // Set Temperature
            float setpoint = Serial.parseFloat(SKIP_WHITESPACE);

            Serial.print("okay... T: ");
            Serial.println(setpoint);
            heater_interface.set_temperature(setpoint);
            break;

          case 'D':
            // Set Interactive Timer
            break;

          case 'F':
            // Disable Interactive Timer
          default:
            Serial.print("Invalid!");
        }
      }
    }

    void print_status(Heater &heater_interface){
      if (start_time > 0){
        if (1 + last_print_time < get_current_time()){
          last_print_time = get_current_time();

          Serial.print("State: ");
          Serial.print(message_machine_state[machine_state]);
          Serial.print(" ");
          Serial.print("Time:");
          Serial.print(get_current_time()); 
          Serial.print(" T:");
          Serial.print(heater_interface.get_temperature());
          Serial.print(" /");
          Serial.print(heater_interface.get_set_temperature());
          Serial.print(" E ");
          Serial.print(heater_interface.return_PID_effort());
          Serial.println();
        }
      }
    }
};

void setup_display(U8G2_SSD1306_128X32_UNIVISION_F_SW_I2C &oled) {
  oled.begin();
  oled.setFont(u8g2_font_8x13_mf);
  oled.clearDisplay();
  oled.clearBuffer();
  oled.firstPage();
  oled.enableUTF8Print();

  // Start-up splash screen
  oled.drawStr(0, 10, SPASH_LINE1);
  oled.drawStr(0, 30, SPASH_LINE2);
  oled.sendBuffer();
  delay(2500);
  oled.clearBuffer();
}

void update_display(U8G2_SSD1306_128X32_UNIVISION_F_SW_I2C &oled, State_Machine &machine_interface, Heater &heater_interface){
  oled.firstPage();
  do {
    oled.setDrawColor(2);
    oled.setFontMode(0);

    // Draw machine_state
    oled.drawStr(0, 10, message_machine_state[machine_interface.get_machine_state()]);

    if (machine_interface.get_machine_state() == ERROR){
      oled.drawStr(0, 30, message_error_status[heater_interface.get_first_error()]); 
    } else {
      oled.setCursor(0, 30);
      oled.print(heater_interface.get_temperature());
      if (heater_interface.get_set_temperature() > 0) {
        oled.print("->");
        oled.setDrawColor(0);
        oled.print(heater_interface.get_set_temperature());
      }
        oled.setDrawColor(2);
        oled.print("°C");
    }

    if (machine_interface.get_current_time() > 0) {
      char str[20];
      oled.drawStr(oled.getDisplayWidth() - oled.getStrWidth(itoa(machine_interface.get_current_time(), str, 10)), 10, itoa(machine_interface.get_current_time(), str, 3));
      // ^ i'll admit it, this is the most wasteful function in history. WHAT OF IT!
    }

  } while (oled.nextPage());
}

void setup_serial(){
  Serial.begin(9600);
  Serial.println(SPASH_LINE1);
  Serial.println(SPASH_LINE2);
}


U8G2_SSD1306_128X32_UNIVISION_F_SW_I2C oled(U8G2_R2, SCL, SDA, U8X8_PIN_NONE);

Switch switch_mode(SWITCH_MODE_PIN);
Switch switch_start_stop(SWITCH_START_STOP_PIN);
Heater heater_interface(SSR_PIN, THERMOCOUPLE_CLK_PIN, THERMOCOUPLE_CS_PIN, THERMOCOUPLE_SO_PIN);

State_Machine machine_interface;

void setup(){
  setup_serial();
  setup_display(oled);
}

void loop(){
  // Update Interfaces
  machine_interface.update_state(heater_interface, switch_mode, switch_start_stop);
  update_display(oled, machine_interface, heater_interface);
}