#ifndef BUTTON_H
#define BUTTON_H

#include "config.h"
#include <Preferences.h>  // esp32 nvs wrapper library

// button states for state machine
enum class ButtonState {
  IDLE,              // waiting for button press
  PRESSED,           // button is held down, waiting to see if short or long press
  RELEASED_ONCE,     // released after short press, waiting for possible double tap
  LONG_PRESS_ACTIVE, // long press threshold exceeded, waiting for release
  WAIT_RELEASE       // action taken, waiting for finger to lift before returning to idle
};

// color presets
enum class ColorPreset {
  // blues & cyans
  BLUE_1,   
  BLUE_2,   
  BLUE_3,   
  BLUE_4,    

  // greens
  GREEN_1,  
  GREEN_2,  

  // reds
  RED_1,    
  RED_2,    

  // oranges & yellows
  ORANGE_1, 
  YELLOW_1, 
  YELLOW_2, 

  // purples
  PURPLE_1, 
  PURPLE_2,
  PURPLE_3,

  // pinks & magentas
  PINK_1,   
  PINK_2,   

  // white
  WHITE,

  NUM_PRESETS // total
};



class ButtonHandler {
 private:
  // state machine
  ButtonState current_state;
  
  // timing variables for press detection
  unsigned long press_start_time;   // when button was first pressed
  unsigned long release_time;       // when button was released (for double-tap window)
  
  // debouncing variables
  bool last_raw_state;              // previous raw digitalRead value
  unsigned long last_debounce_time; // when last state change was detected
  bool debounced_state;             // current stable (debounced) button state
  bool previous_debounced;          // previous stable state for edge detection
  
  // current settings indices
  int current_color_index;
  int current_brightness_index;
  int current_foam_index;
  
  // preset arrays
  int brightness_presets[7];
  float foam_presets[5];
  
  // nvs storage object
  Preferences preferences;
  
  // debouncing - returns stable button state (true = pressed)
  bool getStableState();
  
  // action methods
  void cycleColor();
  void cycleBrightness();
  void cycleFoam();
  
  // nvs persistence methods
  void loadSettings();
  void saveSettings();
  
 public:
  ButtonHandler();
  
  // call once in setup()
  void init();
  
  // call every loop iteration
  void update();
  
  // getters for current settings
  ColorPreset getCurrentColor();
  int getCurrentBrightness();
  float getCurrentFoamThreshold();
  
  // get rgb values for current color preset
  void getCurrentColorRGB(uint8_t* r, uint8_t* g, uint8_t* b);
};

#endif