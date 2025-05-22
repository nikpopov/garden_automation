/*
 * Arduino Garden Controller - Enhanced Version
 * 
 * Components:
 * - Arduino Nano
 * - DS1307RTC real time clock module
 * - 16x2 LCD with I2C interface
 * - DHT22 temperature and humidity sensor
 * - Rotary encoder with push button
 * - 74HC154 4-to-16 line demultiplexer for device control
 * 
 * Features:
 * - Clock/temperature display switching every 5 seconds in sleep mode
 * - Menu system with time, date, interval, and device settings
 * - Device scheduling with ON/OFF times
 * - EEPROM storage for settings persistence
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <TimeLib.h>
#include <DS1307RTC.h>
#include <EEPROM.h>

// Pin definitions
#define DHT_PIN 2          // DHT22 data pin
#define ENCODER_CLK 3      // Encoder clock pin
#define ENCODER_DT 4       // Encoder data pin
#define ENCODER_SW 5       // Encoder switch pin
#define DEMUX_A 6          // 74HC154 input A (LSB)
#define DEMUX_B 7          // 74HC154 input B
#define DEMUX_C 8          // 74HC154 input C
#define DEMUX_D 9          // 74HC154 input D (MSB)
#define DEMUX_ENABLE 10    // 74HC154 enable pin (active LOW)

// Constants
#define DHTTYPE DHT22
#define LONG_PRESS_TIME 1000         // Time in ms for long press (1 second)
#define MAX_DEVICES 16              // Number of devices
#define MIN_INTERVAL 2              // Minimum display interval in seconds
#define MAX_INTERVAL 30             // Maximum display interval in seconds

// EEPROM addresses
#define EEPROM_INTERVAL_ADDR 0
#define EEPROM_DEVICES_ON_ADDR 1    // 16 bytes for ON times (hours)
#define EEPROM_DEVICES_ON_MIN_ADDR 17   // 16 bytes for ON times (minutes)
#define EEPROM_DEVICES_OFF_ADDR 33  // 16 bytes for OFF times (hours)
#define EEPROM_DEVICES_OFF_MIN_ADDR 49  // 16 bytes for OFF times (minutes)

// LCD setup (address might need adjustment)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// DHT setup
DHT dht(DHT_PIN, DHTTYPE);

// Encoder variables
volatile int encoderPos = 0;
int lastEncoderPos = 0;
int lastEncoderState = 0;
unsigned long buttonPressTime = 0;
bool buttonPressed = false;

// Menu variables
enum MenuState {
  SLEEP_MODE,
  MAIN_MENU,
  SET_TIME,
  SET_DATE,
  SET_INTERVAL,
  DEVICE_MENU,
  SET_DEVICE
};

MenuState currentState = SLEEP_MODE;
int menuPosition = 0;
int settingPosition = 0;
bool editingValue = false;

// Sleep mode variables
bool showingClock = true;  // true = clock, false = temp/humidity
unsigned long lastDisplaySwitch = 0;
int displayInterval = 5;   // seconds between display modes

// System variables
unsigned long lastActivityTime = 0;
bool backlightOn = true;

// Device schedule storage
struct DeviceSchedule {
  byte onHour;
  byte onMinute;
  byte offHour;
  byte offMinute;
};

DeviceSchedule deviceSchedules[MAX_DEVICES];
int currentDevice = 0;

// Setting values (temporary)
tmElements_t tempTime;
int tempInterval = 5;

// Function prototypes
void updateEncoder();
void updateDisplay();
void handleButton();
void controlDevice(int device, bool state);
void loadSettings();
void saveSettings();
void setDemuxAddress(byte address);
void checkDeviceSchedules();

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);
  Serial.println("Garden Controller Starting...");
  
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  
  // Initialize encoder pins with pull-up resistors
  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT, INPUT_PULLUP);
  pinMode(ENCODER_SW, INPUT_PULLUP);
  
  // Initialize demultiplexer pins
  pinMode(DEMUX_A, OUTPUT);
  pinMode(DEMUX_B, OUTPUT);
  pinMode(DEMUX_C, OUTPUT);
  pinMode(DEMUX_D, OUTPUT);
  pinMode(DEMUX_ENABLE, OUTPUT);
  digitalWrite(DEMUX_ENABLE, HIGH);  // Initially disabled
  
  // Initialize DHT sensor
  dht.begin();
  
  // Read last encoder state
  lastEncoderState = digitalRead(ENCODER_CLK);
  
  // Set up encoder interrupt
  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK), updateEncoder, CHANGE);
  
  // Initialize RTC
  setSyncProvider(RTC.get);
  if (timeStatus() != timeSet) {
    Serial.println("RTC sync failed!");
    // Set a default time if RTC is not working
    setTime(12, 0, 0, 1, 1, 2025);
  }
  
  // Load saved settings from EEPROM
  loadSettings();
  
  // Initial display update
  updateDisplay();
  
  // Update activity time
  lastActivityTime = millis();
  lastDisplaySwitch = millis();
  
  Serial.println("Setup complete");
}

void loop() {
  // Handle encoder button
  handleButton();
  
  // Check if encoder position has changed
  if (encoderPos != lastEncoderPos) {
    lastActivityTime = millis();
    
    // Wake up display if in sleep mode
    if (currentState == SLEEP_MODE) {
      if (!backlightOn) {
        lcd.backlight();
        backlightOn = true;
      }
    } else {
      // Handle encoder rotation based on current state
      int change = encoderPos - lastEncoderPos;
      
      switch (currentState) {
        case MAIN_MENU:
          menuPosition += change;
          if (menuPosition < 0) menuPosition = 4; // 5 menu items (0-4)
          if (menuPosition > 4) menuPosition = 0;
          break;
          
        case SET_TIME:
          if (editingValue) {
            handleTimeEditing(change);
          }
          break;
          
        case SET_DATE:
          if (editingValue) {
            handleDateEditing(change);
          }
          break;
          
        case SET_INTERVAL:
          tempInterval += change;
          if (tempInterval < MIN_INTERVAL) tempInterval = MIN_INTERVAL;
          if (tempInterval > MAX_INTERVAL) tempInterval = MAX_INTERVAL;
          break;
          
        case DEVICE_MENU:
          menuPosition += change;
          if (menuPosition < 0) menuPosition = 16; // 17 items (0-16, where 16 is exit)
          if (menuPosition > 16) menuPosition = 0;
          break;
          
        case SET_DEVICE:
          if (editingValue) {
            handleDeviceTimeEditing(change);
          }
          break;
      }
      
      updateDisplay();
    }
    
    lastEncoderPos = encoderPos;
  }
  
  // Handle sleep mode display switching
  if (currentState == SLEEP_MODE) {
    if (millis() - lastDisplaySwitch >= (displayInterval * 1000UL)) {
      showingClock = !showingClock;
      lastDisplaySwitch = millis();
      updateDisplay();
    }
    
    // Update clock display every second when showing clock
    static unsigned long lastClockUpdate = 0;
    if (showingClock && (millis() - lastClockUpdate > 1000)) {
      updateDisplay();
      lastClockUpdate = millis();
    }
    
    // Check device schedules
    static unsigned long lastScheduleCheck = 0;
    if (millis() - lastScheduleCheck > 60000) { // Check every minute
      checkDeviceSchedules();
      lastScheduleCheck = millis();
    }
  }
}

void updateEncoder() {
  int currentState = digitalRead(ENCODER_CLK);
  
  // If the states are different, encoder is rotating
  if (currentState != lastEncoderState && currentState == 1) {
    // If DT state is different from CLK state, rotating clockwise
    if (digitalRead(ENCODER_DT) != currentState) {
      encoderPos++;
    } else {
      encoderPos--;
    }
  }
  
  lastEncoderState = currentState;
}

void handleButton() {
  // Read the button state (active LOW with pull-up)
  int buttonState = digitalRead(ENCODER_SW);
  
  // Button press detected
  if (buttonState == LOW && !buttonPressed) {
    buttonPressed = true;
    buttonPressTime = millis();
    lastActivityTime = millis();
    
    // Wake up display if in sleep mode
    if (currentState == SLEEP_MODE && !backlightOn) {
      lcd.backlight();
      backlightOn = true;
      return;  // Just turn on backlight, don't process the button press further
    }
  }
  
  // Button release detected
  if (buttonState == HIGH && buttonPressed) {
    unsigned long pressDuration = millis() - buttonPressTime;
    buttonPressed = false;
    
    if (pressDuration > LONG_PRESS_TIME) {
      // Long press action
      handleLongPress();
    } else {
      // Short press action
      handleShortPress();
    }
    
    updateDisplay();
  }
}

void handleShortPress() {
  switch (currentState) {
    case SLEEP_MODE:
      // Wake up but stay in sleep mode
      if (!backlightOn) {
        lcd.backlight();
        backlightOn = true;
      }
      break;
      
    case MAIN_MENU:
      // Select menu item
      switch (menuPosition) {
        case 0: // Set Time
          currentState = SET_TIME;
          settingPosition = 0;
          editingValue = true;
          // Get current time as starting point
          tempTime.Hour = hour();
          tempTime.Minute = minute();
          tempTime.Second = second();
          tempTime.Day = day();
          tempTime.Month = month();
          tempTime.Year = year() - 1970;
          break;
        case 1: // Set Date
          currentState = SET_DATE;
          settingPosition = 0;
          editingValue = true;
          // Get current date as starting point
          tempTime.Hour = hour();
          tempTime.Minute = minute();
          tempTime.Second = second();
          tempTime.Day = day();
          tempTime.Month = month();
          tempTime.Year = year() - 1970;
          break;
        case 2: // Set Interval
          currentState = SET_INTERVAL;
          tempInterval = displayInterval;
          break;
        case 3: // Set Devices
          currentState = DEVICE_MENU;
          menuPosition = 0;
          break;
        case 4: // Exit
          currentState = SLEEP_MODE;
          break;
      }
      break;
      
    case SET_TIME:
      if (editingValue) {
        settingPosition++;
        if (settingPosition > 2) {
          // Finished editing time, save it
          RTC.write(tempTime);
          setTime(makeTime(tempTime));
          currentState = MAIN_MENU;
          menuPosition = 0;
          editingValue = false;
          settingPosition = 0;
        }
      }
      break;
      
    case SET_DATE:
      if (editingValue) {
        settingPosition++;
        if (settingPosition > 2) {
          // Finished editing date, save it
          RTC.write(tempTime);
          setTime(makeTime(tempTime));
          currentState = MAIN_MENU;
          menuPosition = 1;
          editingValue = false;
          settingPosition = 0;
        }
      }
      break;
      
    case SET_INTERVAL:
      // Save interval and return to menu
      displayInterval = tempInterval;
      EEPROM.write(EEPROM_INTERVAL_ADDR, displayInterval);
      currentState = MAIN_MENU;
      menuPosition = 2;
      break;
      
    case DEVICE_MENU:
      if (menuPosition < 16) {
        // Select device
        currentDevice = menuPosition;
        currentState = SET_DEVICE;
        settingPosition = 0; // 0=ON hour, 1=ON min, 2=OFF hour, 3=OFF min
        editingValue = true;
      } else {
        // Exit to main menu
        currentState = MAIN_MENU;
        menuPosition = 3;
      }
      break;
      
    case SET_DEVICE:
      if (editingValue) {
        settingPosition++;
        if (settingPosition > 3) {
          // Finished editing device schedule, save it
          saveSettings();
          currentState = DEVICE_MENU;
          menuPosition = currentDevice;
          editingValue = false;
          settingPosition = 0;
        }
      }
      break;
  }
}

void handleLongPress() {
  switch (currentState) {
    case SLEEP_MODE:
      // Enter main menu
      currentState = MAIN_MENU;
      menuPosition = 0;
      lcd.backlight();
      backlightOn = true;
      break;
      
    case MAIN_MENU:
    case SET_TIME:
    case SET_DATE:
    case SET_INTERVAL:
    case DEVICE_MENU:
    case SET_DEVICE:
      // Return to sleep mode
      currentState = SLEEP_MODE;
      showingClock = true;
      lastDisplaySwitch = millis();
      break;
  }
}

void handleTimeEditing(int change) {
  switch (settingPosition) {
    case 0: // Hours
      tempTime.Hour += change;
      if (tempTime.Hour > 23) tempTime.Hour = 0;
      if (tempTime.Hour < 0) tempTime.Hour = 23;
      break;
    case 1: // Minutes
      tempTime.Minute += change;
      if (tempTime.Minute > 59) tempTime.Minute = 0;
      if (tempTime.Minute < 0) tempTime.Minute = 59;
      break;
    case 2: // Seconds
      tempTime.Second += change;
      if (tempTime.Second > 59) tempTime.Second = 0;
      if (tempTime.Second < 0) tempTime.Second = 59;
      break;
  }
}

void handleDateEditing(int change) {
  switch (settingPosition) {
    case 0: // Day
      tempTime.Day += change;
      if (tempTime.Day < 1) tempTime.Day = 31;
      if (tempTime.Day > 31) tempTime.Day = 1;
      break;
    case 1: // Month
      tempTime.Month += change;
      if (tempTime.Month < 1) tempTime.Month = 12;
      if (tempTime.Month > 12) tempTime.Month = 1;
      break;
    case 2: // Year
      tempTime.Year += change;
      if (tempTime.Year < 50) tempTime.Year = 50; // 2020
      if (tempTime.Year > 80) tempTime.Year = 80; // 2050
      break;
  }
}

void handleDeviceTimeEditing(int change) {
  switch (settingPosition) {
    case 0: // ON hour
      deviceSchedules[currentDevice].onHour += change;
      if (deviceSchedules[currentDevice].onHour > 23) deviceSchedules[currentDevice].onHour = 0;
      if (deviceSchedules[currentDevice].onHour < 0) deviceSchedules[currentDevice].onHour = 23;
      break;
    case 1: // ON minute
      deviceSchedules[currentDevice].onMinute += change;
      if (deviceSchedules[currentDevice].onMinute > 59) deviceSchedules[currentDevice].onMinute = 0;
      if (deviceSchedules[currentDevice].onMinute < 0) deviceSchedules[currentDevice].onMinute = 59;
      break;
    case 2: // OFF hour
      deviceSchedules[currentDevice].offHour += change;
      if (deviceSchedules[currentDevice].offHour > 23) deviceSchedules[currentDevice].offHour = 0;
      if (deviceSchedules[currentDevice].offHour < 0) deviceSchedules[currentDevice].offHour = 23;
      break;
    case 3: // OFF minute
      deviceSchedules[currentDevice].offMinute += change;
      if (deviceSchedules[currentDevice].offMinute > 59) deviceSchedules[currentDevice].offMinute = 0;
      if (deviceSchedules[currentDevice].offMinute < 0) deviceSchedules[currentDevice].offMinute = 59;
      break;
  }
}

void updateDisplay() {
  lcd.clear();
  
  switch (currentState) {
    case SLEEP_MODE:
      if (showingClock) {
        // Display time on the first line
        lcd.setCursor(0, 0);
        if (hour() < 10) lcd.print("0");
        lcd.print(hour());
        lcd.print(":");
        if (minute() < 10) lcd.print("0");
        lcd.print(minute());
        lcd.print(":");
        if (second() < 10) lcd.print("0");
        lcd.print(second());
        
        // Display date on the second line
        lcd.setCursor(0, 1);
        if (day() < 10) lcd.print("0");
        lcd.print(day());
        lcd.print(":");
        if (month() < 10) lcd.print("0");
        lcd.print(month());
        lcd.print(":");
        lcd.print(year());
      } else {
        // Display temperature and humidity
        float t = dht.readTemperature();
        float h = dht.readHumidity();
        
        lcd.setCursor(0, 0);
        if (isnan(t)) {
          lcd.print("TEMP ERROR");
        } else {
          lcd.print("TEMP ");
          lcd.print((int)t);
          lcd.print(" C");
        }
        
        lcd.setCursor(0, 1);
        if (isnan(h)) {
          lcd.print("HUMID ERROR");
        } else {
          lcd.print("HUMID ");
          lcd.print((int)h);
          lcd.print(" %");
        }
      }
      break;
      
    case MAIN_MENU:
      lcd.setCursor(0, 0);
      lcd.print("MENU:");
      
      lcd.setCursor(0, 1);
      switch (menuPosition) {
        case 0: lcd.print("SET TIME"); break;
        case 1: lcd.print("SET DATE"); break;
        case 2: lcd.print("SET INTERVAL"); break;
        case 3: lcd.print("SET DEVICES"); break;
        case 4: lcd.print("EXIT"); break;
      }
      break;
      
    case SET_TIME:
      lcd.setCursor(0, 0);
      lcd.print("SET TIME:");
      
      lcd.setCursor(0, 1);
      // Display hours with cursor
      if (settingPosition == 0) lcd.print(">");
      else lcd.print(" ");
      if (tempTime.Hour < 10) lcd.print("0");
      lcd.print(tempTime.Hour);
      
      lcd.print(":");
      
      // Display minutes with cursor
      if (settingPosition == 1) lcd.print(">");
      else lcd.print(" ");
      if (tempTime.Minute < 10) lcd.print("0");
      lcd.print(tempTime.Minute);
      
      lcd.print(":");
      
      // Display seconds with cursor
      if (settingPosition == 2) lcd.print(">");
      else lcd.print(" ");
      if (tempTime.Second < 10) lcd.print("0");
      lcd.print(tempTime.Second);
      break;
      
    case SET_DATE:
      lcd.setCursor(0, 0);
      lcd.print("SET DATE:");
      
      lcd.setCursor(0, 1);
      // Display day with cursor
      if (settingPosition == 0) lcd.print(">");
      else lcd.print(" ");
      if (tempTime.Day < 10) lcd.print("0");
      lcd.print(tempTime.Day);
      
      lcd.print(":");
      
      // Display month with cursor
      if (settingPosition == 1) lcd.print(">");
      else lcd.print(" ");
      if (tempTime.Month < 10) lcd.print("0");
      lcd.print(tempTime.Month);
      
      lcd.print(":");
      
      // Display year with cursor
      if (settingPosition == 2) lcd.print(">");
      else lcd.print(" ");
      lcd.print(tempTime.Year + 1970);
      break;
      
    case SET_INTERVAL:
      lcd.setCursor(0, 0);
      lcd.print("SET INTERVAL:");
      
      lcd.setCursor(0, 1);
      lcd.print(tempInterval);
      lcd.print(" seconds");
      break;
      
    case DEVICE_MENU:
      lcd.setCursor(0, 0);
      lcd.print("DEVICES:");
      
      lcd.setCursor(0, 1);
      if (menuPosition < 16) {
        lcd.print("DEVICE ");
        lcd.print(menuPosition + 1);
      } else {
        lcd.print("EXIT");
      }
      break;
      
    case SET_DEVICE:
      lcd.setCursor(0, 0);
      lcd.print("TIME ON  ");
      if (settingPosition == 0) lcd.print(">");
      else lcd.print(" ");
      if (deviceSchedules[currentDevice].onHour < 10) lcd.print("0");
      lcd.print(deviceSchedules[currentDevice].onHour);
      lcd.print(":");
      if (settingPosition == 1) lcd.print(">");
      else lcd.print(" ");
      if (deviceSchedules[currentDevice].onMinute < 10) lcd.print("0");
      lcd.print(deviceSchedules[currentDevice].onMinute);
      
      lcd.setCursor(0, 1);
      lcd.print("TIME OFF ");
      if (settingPosition == 2) lcd.print(">");
      else lcd.print(" ");
      if (deviceSchedules[currentDevice].offHour < 10) lcd.print("0");
      lcd.print(deviceSchedules[currentDevice].offHour);
      lcd.print(":");
      if (settingPosition == 3) lcd.print(">");
      else lcd.print(" ");
      if (deviceSchedules[currentDevice].offMinute < 10) lcd.print("0");
      lcd.print(deviceSchedules[currentDevice].offMinute);
      break;
  }
}

void controlDevice(int device, bool state) {
  if (device >= 0 && device < MAX_DEVICES) {
    // Set the address lines for the demultiplexer
    setDemuxAddress(device);
    
    // Enable/disable the demultiplexer based on state
    digitalWrite(DEMUX_ENABLE, state ? LOW : HIGH);
    
    // Small delay to ensure the output is stable
    delayMicroseconds(10);
    
    Serial.print("Device ");
    Serial.print(device + 1);
    Serial.print(" set to ");
    Serial.println(state ? "ON" : "OFF");
  }
}

void setDemuxAddress(byte address) {
  // Set the 4 address lines for the 74HC154
  digitalWrite(DEMUX_A, address & 0x01);
  digitalWrite(DEMUX_B, (address >> 1) & 0x01);
  digitalWrite(DEMUX_C, (address >> 2) & 0x01);
  digitalWrite(DEMUX_D, (address >> 3) & 0x01);
}

void checkDeviceSchedules() {
  int currentHour = hour();
  int currentMinute = minute();
  
  for (int i = 0; i < MAX_DEVICES; i++) {
    // Check if it's time to turn ON
    if (currentHour == deviceSchedules[i].onHour && 
        currentMinute == deviceSchedules[i].onMinute) {
      controlDevice(i, true);
    }
    
    // Check if it's time to turn OFF
    if (currentHour == deviceSchedules[i].offHour && 
        currentMinute == deviceSchedules[i].offMinute) {
      controlDevice(i, false);
    }
  }
}

void loadSettings() {
  // Load display interval
  displayInterval = EEPROM.read(EEPROM_INTERVAL_ADDR);
  if (displayInterval < MIN_INTERVAL || displayInterval > MAX_INTERVAL) {
    displayInterval = 5; // Default value
  }
  
  // Load device schedules
  for (int i = 0; i < MAX_DEVICES; i++) {
    deviceSchedules[i].onHour = EEPROM.read(EEPROM_DEVICES_ON_ADDR + i);
    deviceSchedules[i].onMinute = EEPROM.read(EEPROM_DEVICES_ON_MIN_ADDR + i);
    deviceSchedules[i].offHour = EEPROM.read(EEPROM_DEVICES_OFF_ADDR + i);
    deviceSchedules[i].offMinute = EEPROM.read(EEPROM_DEVICES_OFF_MIN_ADDR + i);
    
    // Validate values
    if (deviceSchedules[i].onHour > 23) deviceSchedules[i].onHour = 0;
    if (deviceSchedules[i].onMinute > 59) deviceSchedules[i].onMinute = 0;
    if (deviceSchedules[i].offHour > 23) deviceSchedules[i].offHour = 0;
    if (deviceSchedules[i].offMinute > 59) deviceSchedules[i].offMinute = 0;
  }
}

void saveSettings() {
  // Save display interval
  EEPROM.write(EEPROM_INTERVAL_ADDR, displayInterval);
  
  // Save device schedules
  for (int i = 0; i < MAX_DEVICES; i++) {
    EEPROM.write(EEPROM_DEVICES_ON_ADDR + i, deviceSchedules[i].onHour);
    EEPROM.write(EEPROM_DEVICES_ON_MIN_ADDR + i, deviceSchedules[i].onMinute);
    EEPROM.write(EEPROM_DEVICES_OFF_ADDR + i, deviceSchedules[i].offHour);
    EEPROM.write(EEPROM_DEVICES_OFF_MIN_ADDR + i, deviceSchedules[i].offMinute);
  }
}