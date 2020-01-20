/*
Main Controller

OpenTransat
http://opentransat.com
http://github.com/opentransat
http://fb.com/opentransat
Released under the Creative Commons Attribution ShareAlike 4.0 International License
https://creativecommons.org/licenses/by-sa/4.0/
*/

#include <Wire.h>
#include <SPI.h>
#include <MPU9250.h>
#include <Adafruit_INA219.h>
#include <Adafruit_FRAM2Mbit_SPI.h>
#include <DataFlash.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <TinyGPS++.h>
#include <OneWire.h>
#include <IridiumSBD.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <avr/pgmspace.h>

#include "pins.h";
#include "rc.h";
#include "sensors.h";
#include "actuator.h";
#include "comms.h";
#include "log.h";
#include "declination.h";
#include "navigate.h";
#include "memory.h";
#include "camera.h";

//#define SERIAL_DEBUG //comment for real mission
//#define IGNORE_IRIDIUM //comment for real mission
//#define USE_RC //comment for real mission

#ifdef SERIAL_DEBUG
  #define debug(x) if (!sim.active) Serial.print(x)
  #define debugln(x) if (!sim.active) Serial.println(x)
  #define debug2(x, y) if (!sim.active) Serial.print((x), (y))
  #define debugln2(x, y) if (!sim.active) Serial.println((x), (y))
  #define debugWrite(x) if (!sim.active) Serial.write(x)
#else
  #define debug(x) noDebug()
  #define debugln(x) noDebug()
  #define debug2(x, y) noDebug()
  #define debugln2(x, y) noDebug()
  #define debugWrite(x) noDebug()
#endif

#define BAUD_SERIAL0 115200
#define BAUD_GPS 9600
#define BAUD_CAM 4800
#define BAUD_IR 19200
#define BAUD_RUDDER_HALL 9600
#define BAUD_AIS 38400 //or configure to 9600
#define BAUD_ANEMO 4800
#define BAUD_ACT 9600
#define BAUD_COMPASS 38400
#define FLOOD_THRESHOLD 700
#define GPS_HDOP_THRESHOLD 20

DataFlash dataflash;
Adafruit_FRAM_SPI fram = Adafruit_FRAM_SPI(FRAM_CS);  // use hardware SPI
SoftwareSerial camSerial(CAM_RX, CAM_TX); //RX, TX
SoftwareSerial rudderSerial(RUDDER_RX, RUDDER_TX); //RX, TX

#define MUX_GPS 0
#define MUX_IR 1
#define MUX_ACT 2
#define MUX_AIS 3

//ext led blinking
#define WORD(hb, lb) (((unsigned int)(hb) << 8) + (lb))
#define BLINK_STARTUP WORD(1, 1) //before GPS location
#define BLINK_GPS WORD(2, 2) //GPS location
#define BLINK_HEADING WORD(3, 3) //GPS location && correct heading 
#define BLINK_IRIDIUM WORD(4, 4) //transmitting via Iridium
#define BLINK_ACT WORD(2, 1) //actuator: waiting for response
#define BLINK_STOP_LNG_FROM -52.6
#define BLINK_STOP_LNG_TO -4.544

unsigned int blinkMode = BLINK_STARTUP;

//intervals
#define HEARTBEAT_INDEX 0
#define BLINK_INDEX 1
#define DEBUG_INDEX 2
#define VAMPS_INDEX 3
#define RUDDER_INDEX 4
#define IR_INDEX 5
#define LOG_INDEX 6
#define SIM_MOVE_INDEX 7
#define SIM_PRINT_INDEX 8
#define HEARTBEAT_INTERVAL 1000UL
#define BLINK_ON_INTERVAL 20UL
#define BLINK_PAUSE1_INTERVAL 500UL
#define BLINK_PAUSE2_INTERVAL 1500UL
#define DEBUG_INTERVAL 1000UL
#define VAMPS_INTERVAL 1000UL
#define RUDDER_INTERVAL 20UL
#define IR_STARTUP_INTERVAL 180000UL //3min after startup
#define IR_NORMAL_INTERVAL 3600000UL //60min normally
#define IR_URGENT_INTERVAL 600000UL //10min after being ignored or cancelled because of more important tasks (actuator)
#define LOG_INTERVAL 1800000UL //30min log
#define SIM_MOVE_INTERVAL 1UL
#define SIM_PRINT_INTERVAL 10UL
unsigned long intervals[] = {HEARTBEAT_INTERVAL, BLINK_PAUSE2_INTERVAL, DEBUG_INTERVAL, VAMPS_INTERVAL, RUDDER_INTERVAL, IR_STARTUP_INTERVAL, LOG_INTERVAL, SIM_MOVE_INTERVAL, SIM_PRINT_INTERVAL};
unsigned long setInterval_previousMillis[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

#define SAIL_STATE_IDLE 0
#define SAIL_STATE_WAIT_DATA 1
#define SAIL_STATE_WAIT_A 2
#define SAIL_STATE_WAIT_PE 3
#define SAIL_STATE_NEW_DATA 4
#define SAIL_STATE_NEW_A 5
#define SAIL_STATE_NEW_P 6
#define SAIL_STATE_NEW_E 7

#define SAIL_MAX_RETRIES 15
//#define SAIL_MAX_RETRIES 5 //testing

#define SAIL_TIMEOUT_DATA 10000UL
#define SAIL_TIMEOUT_A 7000UL
#define SAIL_TIMEOUT_PE 20000UL
#define SAIL_MAX_RETRIES_WAIT_PERIOD 1800000UL //30min
//#define SAIL_MAX_RETRIES_WAIT_PERIOD 60000UL //1min //testing

unsigned long sail_start_ms = 0;
char sail_state = SAIL_STATE_IDLE;
int sail_retries = 0;
bool sail_max_retries_wait = false;
unsigned long sail_max_retries_wait_ms = 0;

void (*saildata_callback_success)();
void (*saildata_callback_timeout)();

#define REL_WIND(SAIL_ANGLE, FLAP_ANGLE) mod360((int)(SAIL_ANGLE) + (int)(FLAP_ANGLE))

bool cancel_iridium = false;
char mux;

void setMUX(char x) {
  mux = x; // global var
  digitalWrite(MUX_A, bitRead(x, 0));
  digitalWrite(MUX_B, bitRead(x, 1));
}

struct {
  unsigned char pulse = 100; //100 +- angle
  char feedback;
  unsigned long last_update;
  unsigned long count = 0;
  unsigned long pause_start = 0;
} rudder;

void init_io() {
  pinMode(IR_SLEEP, OUTPUT);
  digitalWrite(IR_SLEEP, LOW); //Iridium sleeps @ LOW

  pinMode(HEARTBEAT, OUTPUT);
  digitalWrite(HEARTBEAT, LOW);
  
  pinMode(WARNING, INPUT); //active LOW
  
  pinMode(MPU_INT, INPUT);
  
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  
  pinMode(ESP_OFF, OUTPUT);
  digitalWrite(ESP_OFF, LOW); //ESP active @ LOW
  
  pinMode(GPS_OFF, OUTPUT);
  digitalWrite(GPS_OFF, LOW); //GPS active @ LOW

  pinMode(ACT_ON, OUTPUT);
  digitalWrite(ACT_ON, LOW); //active HIGH

  pinMode(HALL_ON, OUTPUT);
  digitalWrite(HALL_ON, HIGH); //active HIGH

  pinMode(LED_EXT, OUTPUT);
  digitalWrite(LED_EXT, LOW);

  pinMode(PWRSENSOR_ON, OUTPUT);
  digitalWrite(PWRSENSOR_ON, LOW); //active HIGH

  //mosfets
  pinMode(CAM_ON, OUTPUT);
  digitalWrite(CAM_ON, LOW); //active HIGH
  
  pinMode(SAT_ON, OUTPUT);
  digitalWrite(SAT_ON, LOW); //active HIGH
  
  pinMode(AUX_ON, OUTPUT);
  digitalWrite(AUX_ON, LOW); //active HIGH
  
  pinMode(PUMP1_ON, OUTPUT);
  digitalWrite(PUMP1_ON, LOW); //active HIGH
  
  pinMode(PUMP2_ON, OUTPUT);
  digitalWrite(PUMP2_ON, LOW); //active HIGH
  
  pinMode(AIS_OFF, OUTPUT);
  digitalWrite(AIS_OFF, HIGH); //active LOW
  
  pinMode(RC_OFF, OUTPUT);

  #ifdef USE_RC
    digitalWrite(RC_OFF, LOW); //active LOW
  #else
    digitalWrite(RC_OFF, HIGH); //active LOW
  #endif
  
  //MUX
  pinMode(MUX_A, OUTPUT);
  digitalWrite(MUX_A, LOW);
  
  pinMode(MUX_B, OUTPUT);
  digitalWrite(MUX_B, LOW);

  //onboard stuff
  pinMode(MPU_CS, OUTPUT);
  digitalWrite(MPU_CS, HIGH);

  pinMode(BME_CS, OUTPUT);
  digitalWrite(BME_CS, HIGH);

  pinMode(FRAM_CS, OUTPUT);
  digitalWrite(FRAM_CS, HIGH);
  
  pinMode(DF_CS, OUTPUT);
  digitalWrite(DF_CS, HIGH);

  //RC
  pinMode(RC_PWM0, INPUT_PULLUP);
  pinMode(RC_PWM1, INPUT_PULLUP);
  pinMode(RC_PWM2, INPUT_PULLUP);
  pinMode(RC_PWM3, INPUT_PULLUP);
}

void setup() {
  init_io();
  Serial.begin(BAUD_SERIAL0);
  Wire.begin();

  debugln(F("--------- start ---------------"));

  SPI.begin();

  //MPU
  MPU_init();

  //BME
  BME_init();

  //Iridium
  IR_init();

  //Remote Control
  #ifdef USE_RC
    RC_init();
  #endif

  //FRAM
  FRAM_init();
  
//  test_FRAM();

  //DataFlash

//  test_DataFlash();

  Serial1.begin(BAUD_RUDDER_HALL); //RUDDER TX and HALL RX
  Serial2.begin(BAUD_COMPASS); //compass
  Serial3.begin(BAUD_GPS);
  setMUX(MUX_GPS); //default: GPS whenever possible

  anemoSerial.begin(BAUD_ANEMO);
  
  rudderSerial.begin(BAUD_RUDDER_HALL);

  saildata_callback_success = printData2; //make sure it's initialized
  saildata_callback_timeout = printData2; //make sure it's initialized

  sail.hall_update = millis();
  rudder.pause_start = millis();

  eeLoad();
  ee.resets++;
  eeSave(SAVE_REASON_RESETS);
}

void loop() {
  setInterval(HEARTBEAT_INDEX);
  setInterval(BLINK_INDEX);
  setInterval(VAMPS_INDEX);
  setInterval(RUDDER_INDEX);
  setInterval(IR_INDEX);
  setInterval(DEBUG_INDEX);
  setInterval(LOG_INDEX);
  
  checkSailResponse();

  camera();
  countCapsize();
  checkCompass();
  checkHall();
  act.mustWait = actMustWait();

  navigate1();

  if (sim.active) {
    setInterval(SIM_MOVE_INDEX);
    setInterval(SIM_PRINT_INDEX);
  }
  
  smartDelay(0); //read and process all serial inputs

  #ifdef SERIAL_DEBUG
    readcmd();
  #endif
}

bool ISBDCallback() { //fires multiple times while Iridium is sending message, can be cancelled by cancel_iridium from switchAct()
  setInterval(HEARTBEAT_INDEX);
  setInterval(BLINK_INDEX);
  setInterval(VAMPS_INDEX);
  setInterval(RUDDER_INDEX);
  //setInterval(IR_INDEX); //do not call while transmitting
  setInterval(DEBUG_INDEX);
  setInterval(LOG_INDEX);
  
  //checkSailResponse(); //do not call, we are not expecting a response from the sailwing while sending a message (Serial3 is muxed to Iridium)

  camera();
  countCapsize();
  checkCompass();
  checkHall();
  act.mustWait = actMustWait();

  navigate1();

  if (sim.active) {
    setInterval(SIM_MOVE_INDEX);
    setInterval(SIM_PRINT_INDEX);
  }

  smartDelay(0); //read and process all serial inputs (except Iridium - it's read in the running function)

  #ifdef SERIAL_DEBUG
    readcmd();
  #endif

  if (cancel_iridium) {
    debugln(F(">>> CANCELLING IRIDIUM TRANSMISSION, SETTING NEXT INTERVAL TO URGENT"));
    intervals[IR_INDEX] = IR_URGENT_INTERVAL;
    return false;
  }
  else
    return true;
}

void navigate1() {
  if (warning()) { //watchdog shutdown warning => rudder straight and do not navigate
    rudder.pulse = 100;
    return;
  }
  
  if (sim.active) { //simulation
    nav.trueHeading = sim.trueHeading;
    nav.trueWind = sim.trueWind;
    nav.gps = sim.gps;
    navigate();
  }
  else { //real, not simulation
    //initialize magnetic declination if not initialized
    if (!nav.magDecInitialized && gps.location.isValid() && gps.hdop.isValid() && gps.hdop.value()/100 < GPS_HDOP_THRESHOLD) {
      updateMagDeclination({gps.location.lat(), gps.location.lng()}); //also updates nav.magDecInitialized

      //fix next waypoint if EEPROM checksum failed at startup
      if (ee.checksum_failed) {
        ee.checksum_failed = false;
        ee.nextWaypoint = failsafeWaypoint_West2East(); //valid GPS needed
        eeSave(SAVE_REASON_FAILSAFE_WAYPOINT);
      }
    }

    if (nav.mode != NAVMODE_MANUAL && sail.windUpdated && gps.location.isValid() && nav.magDecInitialized) {
      nav.trueHeading = mod360(round(compass.yaw + nav.magDec)); //degs
      nav.trueWind = mod360(nav.trueHeading + windFusion(sail.relWind, anemo.direction)); //degs
      nav.gps = {gps.location.lat(), gps.location.lng()};
      navigate();
    }
  }
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do {
    readAllSerial();
  } while (millis() - start < ms);
}

void readAllSerial() {
  while (Serial1.available()) { //hall sensor
    char c = Serial1.read();
    //debugWrite(c); //debug raw output
    hallObj.encode(c);
    if (hall.isValid() && hall.isUpdated())
      sail.hall_update = millis();
      sail.angle = (360 - atoi(hall.value()) + 360) % 360; //hall returns positive angle to the left, we want it like the compass
      sail.relWind = REL_WIND(sail.angle, (int)ee.last_A - 100);
      sail.windUpdated = true;
  }

  while (Serial2.available()) { //compass sensor
    char c = Serial2.read();
    //debugWrite(c); //debug raw output
    compassObj.encode(c);
    if (compass_ID.isValid() && compass_ID.isUpdated()
    && compass_yaw.isValid() && compass_yaw.isUpdated()) {
      compass.ID = atof(compass_ID.value());
      compass.yaw = atof(compass_yaw.value());
      compass.pitch = atof(compass_pitch.value());
      compass.roll = atof(compass_roll.value());
      compass.temp = atof(compass_temp.value());
    }
  }
  
  if(Serial3.available()) {
    switch (mux) {
      case MUX_GPS:
        while (Serial3.available())
          gps.encode(Serial3.read());
        break;
        
      case MUX_ACT:
        while (Serial3.available()) {
          char c = Serial3.read();
          //debugWrite(c); //debug raw output: ActuatorTX
          actObj.encode(c);
        }
        
        if (sail_A.isValid() && sail_A.isUpdated()) { //command to move the actuator was received
          if (sail_state == SAIL_STATE_WAIT_A) {
            sail_state = SAIL_STATE_NEW_A;
          }

          sail_start_ms = millis();
          //sail_A.value() is read when P/E is received
       
          debugln();
          debug(F("SAILWING RESPONSE: Actuator: "));
          debugln(sail_A.value());
        }
        
        if (sail_P.isValid() && sail_P.isUpdated()) {
          if (sail_state == SAIL_STATE_WAIT_PE) {
            sail_state = SAIL_STATE_NEW_P;
            actAddTime(millis(), false); //runs just once to avoid duplicates from multiple 'P' responses, do not save as eeSave() follows
          }
            
          sail_start_ms = millis();
          act.last_P = (unsigned int)atoi(sail_P.value());
          ee.last_A = (unsigned int)atoi(sail_A.value());
          eeSave(SAVE_REASON_LASTAP);
          sail.relWind = REL_WIND(sail.angle, (int)ee.last_A - 100);
          sail.windUpdated = true;
          
          debugln();
          debug(F("SAILWING RESPONSE: Potentiometer: "));
          debugln(sail_P.value());

        }
        
        if (sail_E.isValid() && sail_E.isUpdated()) {
          if (sail_state == SAIL_STATE_WAIT_PE) {
            sail_state = SAIL_STATE_NEW_E;
            actAddTime(millis(), false); //runs just once to avoid duplicates from multiple 'E' responses, do not save as eeSave() follows
          }
            
          sail_start_ms = millis();
          act.last_E = (unsigned int)atoi(sail_E.value());
          ee.last_A = (unsigned int)atoi(sail_A.value());
          eeSave(SAVE_REASON_LASTAE);
          sail.relWind = REL_WIND(sail.angle, (int)ee.last_A - 100);
          sail.windUpdated = true;
          
          debugln();
          debug(F("SAILWING RESPONSE: Encoder: "));
          debugln(sail_E.value());
        }
        
        if (sail_bat.isValid() && sail_bat.isUpdated()) {
          //new data ends with T
          sail_start_ms = millis();
          data2.sail_vaB = decodeVamps((unsigned int)atoi(sail_bat.value()), 10, 10);

          debugln();
          debug(F("SAILWING RESPONSE: Battery: "));
          debugln((unsigned int)atoi(sail_bat.value()));
        }
        
        if (sail_panelR.isValid() && sail_panelR.isUpdated()) {
          //new data ends with T
          sail_start_ms = millis();
          data2.sail_vaR = decodeVamps((unsigned int)atoi(sail_panelR.value()), 10, 1);

          debugln();
          debug(F("SAILWING RESPONSE: Right Panel: "));
          debugln((unsigned int)atoi(sail_panelR.value()));
        }
        
        if (sail_panelL.isValid() && sail_panelL.isUpdated()) {
          //new data ends with T
          sail_start_ms = millis();
          data2.sail_vaL = decodeVamps((unsigned int)atoi(sail_panelL.value()), 10, 1);

          debugln();
          debug(F("SAILWING RESPONSE: Left Panel: "));
          debugln((unsigned int)atoi(sail_panelL.value()));
        }
        
        if (sail_temp.isValid() && sail_temp.isUpdated()) {
          if (sail_state == SAIL_STATE_WAIT_DATA)
            sail_state = SAIL_STATE_NEW_DATA; //received T => that's all data
          data2.last_saildata_update = millis();
          
          sail_start_ms = millis();
          data2.sail_temp = atof(sail_temp.value());

          debugln();
          debug(F("SAILWING RESPONSE: Temp: "));
          debugln(sail_temp.value());
        }
        break;
        
      case MUX_AIS:
        while (Serial3.available()) {
          char c = Serial3.read();
          //debugWrite(c); //debug raw output: AIS
        }
        break;
        
      default:
        break;
    }
  }

  readCamCmd(); //camSerial

  while (anemoSerial && anemoSerial.available()) {
    char c = anemoSerial.read();
    //debugWrite(c); //debug raw output: anemo
    anemoObj.encode(c);

    if (anemoDirection.isValid() && anemoDirection.isUpdated())
      anemo.direction = atof(anemoDirection.value());

    if (anemoSpeed.isValid() && anemoSpeed.isUpdated())
      anemo.speed = atof(anemoSpeed.value());
  }
  
  while (rudderSerial && rudderSerial.available()) {
    char c = rudderSerial.read();
    rudder.feedback = c;
    rudder.count++;
    rudder.last_update = millis();

    debugWrite(c); //debug raw output: rudder feedback
  }
}

void checkSailResponse() {
  if (sail_state == SAIL_STATE_NEW_DATA) {
    debugln(F("SAILWING DATA RECEIVED."));
    sail_state = SAIL_STATE_IDLE;
    digitalWrite(ACT_ON, LOW);
    switchGPS();
    if (saildata_callback_success != NULL)
      saildata_callback_success();
  }
  else if (sail_state == SAIL_STATE_WAIT_DATA && millis() > sail_start_ms + SAIL_TIMEOUT_DATA) {
    debugln(F("SAILWING DATA TIMEOUT, turning off ActuatorTX..."));
    sail_state = SAIL_STATE_IDLE;
    digitalWrite(ACT_ON, LOW);
    switchGPS();
    if (saildata_callback_timeout != NULL)
      saildata_callback_timeout();
  }
  else if (sail_state == SAIL_STATE_NEW_A) {
    debugln(F("SAILWING 'A' RECEIVED. WAITING FOR PE..."));
    sail_state = SAIL_STATE_WAIT_PE;
    sail_start_ms = millis();
  }
  else if (sail_state == SAIL_STATE_WAIT_A && millis() > sail_start_ms + SAIL_TIMEOUT_A
    || sail_state == SAIL_STATE_WAIT_PE && millis() > sail_start_ms + SAIL_TIMEOUT_PE) {
      
    sail_retries++;
    
    if (sail_retries > SAIL_MAX_RETRIES) {
      debugln(F("SAILWING 'A'/'PE' TIMEOUT, MAX RETRIES REACHED, entering wait mode, turning off ActuatorTX..."));
      sail_state = SAIL_STATE_IDLE;
      sail_max_retries_wait = true;  //will change to false after SAIL_WAIT_MAX_RETRIES_PERIOD has passed
      sail_max_retries_wait_ms = millis();
      digitalWrite(ACT_ON, LOW);
      switchGPS();
      sail_retries = 0;
    }
    else { //retrying
      debug(F("SAILWING 'A' TIMEOUT, retrying... ")); debug(sail_retries); debug("/"); debugln(SAIL_MAX_RETRIES);
      retryActuator();
    }
  }
  else if (sail_state == SAIL_STATE_NEW_P) {
    debugln(F("SAILWING 'P' RECEIVED, turning off ActuatorTX..."));
    sail_state = SAIL_STATE_IDLE;
    act.confirmed_PE = true;
    digitalWrite(ACT_ON, LOW);
    switchGPS();
  }
  else if (sail_state == SAIL_STATE_NEW_E) {
    debugln(F("SAILWING 'E' RECEIVED, turning off ActuatorTX..."));
    sail_state = SAIL_STATE_IDLE;
    act.confirmed_PE = true;
    digitalWrite(ACT_ON, LOW);
    switchGPS();
  }
  else if (sail_max_retries_wait && millis() > sail_max_retries_wait_ms + SAIL_MAX_RETRIES_WAIT_PERIOD) {
    debugln(F("SAILWING WAIT MODE ENDED, going idle..."));
    sail_max_retries_wait = false;
    sail_retries = 0;
  }
}

void heartbeat() {
  static char hb = LOW;
  hb = (hb == LOW) ? HIGH : LOW;
  digitalWrite(HEARTBEAT, hb);   
}

void ledBlink() {
  if (gps.location.isValid() && gps.location.lng() > BLINK_STOP_LNG_FROM && gps.location.lng() < BLINK_STOP_LNG_TO) { //do not blink in this interval
    digitalWrite(LED, LOW);
    digitalWrite(LED_EXT, LOW);
    return;
  }

  bool gps_ok = (gps.location.isValid() && gps.hdop.isValid() && gps.hdop.value()/100 < GPS_HDOP_THRESHOLD);
  
  if (mux == MUX_IR)
    blinkMode = BLINK_IRIDIUM;
  else if (mux == MUX_ACT)
    blinkMode = BLINK_ACT;
  else if (gps_ok && nav.magDecInitialized && headingOK(nav.error))
    blinkMode = BLINK_HEADING;
  else if (gps_ok)
    blinkMode = BLINK_GPS;
  else
    blinkMode = BLINK_STARTUP;

  static unsigned char pos = 0;
  static unsigned char counter = 0;
  static unsigned char state = LOW;

  if (state == LOW) {
    digitalWrite(LED, HIGH);
    digitalWrite(LED_EXT, HIGH);
    state = HIGH;
    intervals[BLINK_INDEX] = BLINK_ON_INTERVAL;
  }
  else {
    digitalWrite(LED, LOW);
    digitalWrite(LED_EXT, LOW);
    state = LOW;

    counter++;

    if (pos == 0 && counter >= lowByte(blinkMode)) {
      counter = 0;
      pos = 1;
      intervals[BLINK_INDEX] = BLINK_PAUSE2_INTERVAL;
    }
    else if (pos == 1 && counter >= highByte(blinkMode)) {
      counter = 0;
      pos = 0;
      intervals[BLINK_INDEX] = BLINK_PAUSE2_INTERVAL;
    }
    else
      intervals[BLINK_INDEX] = BLINK_PAUSE1_INTERVAL;
  }
}

void printData() {
  debug(F("Running: "));
  debug(millis()/1000);
  debug("sec");
  debug(F("    Resets: "));
  debugln(ee.resets);
  
  debug("Nav: ");
  print_vamps(va);
  debug(" avg ");
  debug(avg_mamps);
  debugln("mA");

  debug(F("GPS HDOP:"));
  debug(gps.hdop.value()/100);
  debug(F(" Time:"));
  printDateTime(gps.date, gps.time);
  debug(" lat:");
  debug2(gps.location.lat(), 6);
  debug(" lng:");
  debugln2(gps.location.lng(), 6);
  
  debug(F("Compass ID:"));
  debug((uint8_t)compass.ID);
  debug(" Yaw:");
  debug2(compass.yaw, 4);
  debug(" Pitch:");
  debug(compass.pitch);
  debug(" Roll:");
  debug(compass.roll);
  debug(" Temp:");
  debugln(compass.temp);

  debug(F("Sail Angle: "));
  debugln(sail.angle);
  debug(F("Rel Wind: "));
  debugln(sail.relWind);
  
  debug(F("Actuator Last: A:"));
  debug(ee.last_A);
  debug(" pos:");
  debug(act.last_P);
  debug(" enc:");
  debug(act.last_E);
  debug(" count:");
  debug(act.count);
  debug("   total cycles:");
  debugln(ee.act_cycles);

  printActTimes();
  
  debug(F("    Must Wait?: "));
  debugln(act.mustWait);

  debug(F("Sail State: "));
  debug((uint8_t)sail_state);
  debug(F("   Confirmed PE: "));
  debugln(act.confirmed_PE);
  
  debug(F("Last Rudder Feedback: "));
  debug((uint8_t)rudder.feedback);
  debug(" @ ");
  debug(rudder.last_update/1000);
  debug("sec");
  debug(F("   Angle: "));
  debug(nav.rudder_angle);
  debug(F("   Count: "));
  debugln(rudder.count);
  
  debug(F("Next Iridium: "));
  debug((setInterval_previousMillis[IR_INDEX] + intervals[IR_INDEX])/1000);
  debug("sec");
  debug(F("   Interval: "));
  debug(intervals[IR_INDEX]/1000);
  debug(F("sec   Transmit Flag:"));
  debugln(nav.IR_transmit);
  
  debug(F("Navigation Mode: "));
  debug(nav.mode);
  debug(F("    MUX: "));
  debug((uint8_t)mux);
  debug(F("    Tack Mode: "));
  debugln(nav.tackmode);
  
  debug(F("Next Waypoint: "));
  debug(ee.override_flag ? ee.override_customID : ee.nextWaypoint);
  debug(F("    Mag Dec: "));
  debug(nav.magDec);
  debug(F("    True Heading: "));
  debug(nav.trueHeading);
  debug(F("    True Wind: "));
  debugln(nav.trueWind);
  
  debug(F("Ghost Heading: "));
  debug(nav.ghostHeading);
  debug(F("    Go Heading: "));
  debug(nav.goHeading);
  debug(F("    Warning?: "));
  debugln(warning() ? "true" : "false");
  
  if (RC_active) {
    debug("RC: ");
    debug(RC_pulse[0]);
    debug(" ");
    debugln(RC_pulse[1]);
  }

  if (ee.override_flag) {
    debug(F("override_customID: "));
    debugln(ee.override_customID);
    debug(F("override_prev_loc.lat: "));
    debugln(ee.override_prev_loc.lat);
    debug(F("override_prev_loc.lng: "));
    debugln(ee.override_prev_loc.lng);  
    debug(F("override_next_loc.lat: "));
    debugln(ee.override_next_loc.lat);
    debug(F("override_next_loc.lng: "));
    debugln(ee.override_next_loc.lng);
    debug(F("override_ghostDistance: "));
    debugln(ee.override_ghostDistance);
    debug(F("override_maxDev: "));
    debugln(ee.override_maxDev);
    debug(F("override_customVar: "));
    debugln(ee.override_customVar);
  }

  debugln(F("---------------------------------------------"));
}

void reqData2(void (*callback_success)(), void (*callback_timeout)()) {
  digitalWrite(PWRSENSOR_ON, HIGH); //turn on power sensor, charge capacitor min 2 sec

  data2.pressure_in = bme.readPressure() / 100.0F;
  data2.temp_in = bme.readTemperature();
  data2.humidity_in = bme.readHumidity();
  
  ds.reset_search();
  data2.temp_water = readTemp(); //takes min 1 sec
  data2.temp_bat = readTemp(); //takes min 1 sec

  //capacitor is charged (2 sec)
  
  ina219_sol1.begin();
  data2.hull_va1 = read_INA(ina219_sol1);

  ina219_sol2.begin();
  data2.hull_va2 = read_INA(ina219_sol2);
  
  ina219_sol3.begin();
  data2.hull_va3 = read_INA(ina219_sol3);

  ina219_bat.begin();
  ina219_bat.setCalibration_32V_2A(); //0.1 ohm resistor
  data2.backup_va = read_INA(ina219_bat);

  digitalWrite(PWRSENSOR_ON, LOW); //turn off power sensor

  //read sailwing data
  switchAct();
  transmitAct('S', 0);

  //in the main loop: ACT_ON is turned off and data is processed, then a callback function is called (printed or sent via Iridium, etc.)
  sail_state = SAIL_STATE_WAIT_DATA; //changed in smartDelay when the last expected saildata is received 
  sail_start_ms = millis();

  saildata_callback_success = callback_success;
  saildata_callback_timeout = callback_timeout;
}

void printData2() {
  debug(F("Last sail data update since start: "));
  debug(data2.last_saildata_update/1000);
  debugln("sec");
  
  debug(F("Pressure: in "));
  debug(data2.pressure_in);
  debugln("hPa");

  debug(F("Temp: in "));
  debug(data2.temp_in);
  debug(" water ");
  debug(data2.temp_water);
  debug(" bat ");
  debugln(data2.temp_bat);

  debug(F("Flood: middle:"));
  debug(data2.flood0);
  debug(" ");
  debug(data2.flood0_count);
  debug("/");
  debug(data2.flood0_total);
  debug(F(" stern:"));
  debug(data2.flood1);
  debug(" ");
  debug(data2.flood1_count);
  debug("/");
  debugln(data2.flood1_total);

  debug(F("Solar 1: "));
  print_vamps(data2.hull_va1);
  debugln();

  debug(F("Solar 2: "));
  print_vamps(data2.hull_va2);
  debugln();

  debug(F("Solar 3: "));
  print_vamps(data2.hull_va3);
  debugln();
  
  debug(F("Backup Bat: "));
  print_vamps(data2.backup_va);
  debugln();
  
  debug(F("Solar L: "));
  print_vamps(data2.sail_vaL);
  debugln();

  debug(F("Solar R: "));
  print_vamps(data2.sail_vaR);
  debugln();

  debug(F("Sail Bat: "));
  print_vamps(data2.sail_vaB);
  debugln();

  debug(F("Sail Temp: "));
  debugln(data2.sail_temp);
}

void setInterval(unsigned char index) {
  unsigned long currentMillis = millis();

  if (currentMillis < setInterval_previousMillis[index]) { //millis() overflow
    setInterval_previousMillis[index] = 0;
  }
  
  if (currentMillis - setInterval_previousMillis[index] >= intervals[index]) {
    setInterval_previousMillis[index] = currentMillis;

    switch(index) {
      case HEARTBEAT_INDEX:
        heartbeat();
        break;
        
      case BLINK_INDEX:
        ledBlink();
        break;
        
      case DEBUG_INDEX:
        printData();
        break;
        
      case VAMPS_INDEX:
        readVamps();
        readFlood();
        break;
        
      case RUDDER_INDEX:
        Serial1.write(rudder.pulse);
        break;
        
      case IR_INDEX:
        if (mux == MUX_ACT) { //ignore iridium if waiting for the actuator
          debugln(F(">>> IRIDIUM IGNORED BECAUSE ACTUATOR IS BUSY, SETTING NEXT INTERVAL TO URGENT..."));
          intervals[IR_INDEX] = IR_URGENT_INTERVAL;
        }
        else {
          if (ee.override_flag && ee.override_customVar / 2 > 10) {
            intervals[IR_INDEX] = (unsigned long)ee.override_customVar / 2 * 60000UL;
          }
          else
            intervals[IR_INDEX] = IR_NORMAL_INTERVAL; //switch from startup/urgent interval to normal interval

          if (nav.mode != NAVMODE_MANUAL) {
            if (nav.IR_transmit) //the navigate(...) function did not start transmitting in the previous Iridium interval => force transmitting
              reqData2(IR_sendData, IR_sendData);
            
            nav.IR_transmit = true; //the navigate(...) function will start transmission when the rudder pauses
          }
          else //manual mode
            reqData2(IR_sendData, IR_sendData);
        }
        break;

      case LOG_INDEX:
        if (gps.location.isValid() && gps.hdop.isValid() && gps.hdop.value()/100 < GPS_HDOP_THRESHOLD
        && !warning())
          logAdd(gps.date.value(), gps.time.value(), gps.location.lat(), gps.location.lng());
        break;

      case SIM_MOVE_INDEX:
          sim_move();
        break;

      case SIM_PRINT_INDEX:
          sim_print();
        break;
                
      default:
        break;
    }
  }
}

////////////////////////////////////////////////////////////

#ifdef SERIAL_DEBUG
  String cmd = "";
  int cmdlen;
  #define CMDMAX 200

  void readcmd() {
    while (Serial.available() > 0) {
      char ch = (char)Serial.read();
      if (ch == '\n' || ch == '\r') continue;
      if (ch == ';') { //end of command, process
        execmd();
        cmd = "";
        cmdlen = 0;
      }
      else if (ch == '$') { //start a new command (optional)
        cmd = "";
        cmdlen = 0;
      }
      else {
        cmd += ch;
        cmdlen++;
      }
      if (cmdlen > CMDMAX-2) { //too long command
        cmd = "";
        cmdlen = 0;      
      }
    }
  }
  
  void execmd() {
    debug(F("COMMAND: ")); debugln(cmd);
    //find first digit
    int i = 0;
    while(i < cmd.length() && !isDigit(cmd[i]) && cmd[i] != '-' && cmd[i] != '.') i++;
    //
    String cmd1 = cmd.substring(0, i);
    String cmd2_str = cmd.substring(i);
    int cmd2 = cmd2_str.toInt();
    double cmd2f = cmd2_str.toDouble();
    
    if (cmd1 == "DATA" && cmd2 == 2) {
      reqData2(printData2, printData2);
    }
    else if (cmd1 == "RUDDER" && cmd2 >= 0 && cmd2 <= 255) { //set rudder
      rudder.pulse = cmd2;
    }
    else if (cmd1.length() > 4 && cmd1.startsWith("FLAP") && cmd2 >= 80 && cmd2 <= 120) { //set flap
      //actuator already busy, ignore command
      if (mux == MUX_ACT || sail_state != SAIL_STATE_IDLE) {
        debugln(F(">>> ACTUATOR ALREADY BUSY, COMMAND IGNORED"));
        return;
      }
      
      //prevent the actuator from moving too many times
      if (cmd1[4] == 'A' && nav.mode != NAVMODE_MANUAL && act.mustWait) {
        debugln(F(">>> ACTUATOR MUST WAIT, COMMAND IGNORED"));
        return;
      }
  
      switchAct();
  
      if (cmd1[4] == 'S') {
        sail_state = SAIL_STATE_WAIT_DATA; //changed in smartDelay when update is received
        sail_start_ms = millis();
        saildata_callback_success = printData2;
        saildata_callback_timeout = printData2;
        transmitAct(cmd1[4], cmd2);
      }
      else if (cmd1[4] == 'P') {
        sail_state = SAIL_STATE_WAIT_PE; //changed in smartDelay when update is received
        sail_start_ms = millis();
        transmitAct(cmd1[4], cmd2);
      }
      else if (cmd1[4] == 'A') {
        startActuator(cmd2, true);
      }
    }
    else if (cmd1 == "SMARTA") { //set flap like in the "navigate" function
      if (act.mustWait) {
        debugln(">> Actuator must wait, ignoring");
      }
      else {
        smartStartActuator(cmd2 - 100, true, true);
      }    
    }
    else if (cmd1 == "IRIDIUM" && cmd2 == 0) { //test simple iridium with little data
      switchIR();
      IR_send("test", 4);
      if (mux != MUX_ACT) //if transmission cancelled by the actuator, do not switch to GPS
        switchGPS();
    }
    else if (cmd1 == "IRIDIUM" && cmd2 == 2) { //test requesting and sending all data via iridium
      reqData2(IR_sendData, IR_sendData);
    }
    else if (cmd1 == "IRIDIUMCSQ" && cmd2 == 0) { //test requesting and sending all data via iridium
      switchIR();
      IR_testCSQ();
      if (mux != MUX_ACT) //if transmission cancelled by the actuator, do not switch to GPS
        switchGPS();    
    }
    else if (cmd1 == "IRIDIUMINT" && cmd2 >= 10 && cmd2 <= 10000) { //test requesting and sending all data via iridium
      intervals[IR_INDEX] = (long)cmd2*1000;
    }
    else if (cmd1 == "LOGDUMP") { //dump log
      logDump(cmd2);
    }
    else if (cmd1 == "LOGRESET" && cmd2 == 0) { //reset log
      logReset();
    }
    else if (cmd1 == "LOGADD" && cmd2 == 0) { //test log
      if (gps.location.isValid() && gps.hdop.isValid() && gps.hdop.value()/100 < GPS_HDOP_THRESHOLD)
        logAdd(gps.date.value(), gps.time.value(), gps.location.lat(), gps.location.lng());
    }                  
    else if (cmd1 == "CLEAROVERRIDE" && cmd2 == 0) { //clear the override flag in EEPROM
      ee.override_flag = false;
      eeSave(SAVE_REASON_OVERRIDE_CLEAR);
    }
    else if (cmd1 == "LAUNCH" && cmd2 == 0) { //setup before the launch: clear log, set waypoint to 1 and other defaults, save
      logReset();
      ee = ee_default; //compiler takes care
      ee.last_A = 90; //current position
      ee.nextWaypoint = 1;
      eeSave(SAVE_REASON_LAUNCH);
    }
    else if (cmd1 == "MODE" && cmd2 >= 0 && cmd2 <= NAVMODE_FULLY_AUTO) { //change navigation mode
      nav.mode = cmd2;
    }
    else if (cmd1 == "PRINT" && cmd2 >= 1 && cmd2 <= 10000) { //change navigation mode
      intervals[DEBUG_INDEX] = (long)cmd2*1000;
    }
    else if (cmd1 == "COMPASSC" && cmd2 >= 1 && cmd2 <= 4) { //change compass (without saving)
      switchCompass(cmd2, false);
    }
    else if (cmd1 == "COMPASSE" && cmd2 >= 1 && cmd2 <= 4) { //change compass (and save it in compass' eeprom)
      switchCompass(cmd2, true);
    }
    else if (cmd1 == "CALIBSAVE" && cmd2 == 0) { //save calibration if compass is flashed with calibration mode
      Serial2.print("S");
    }
    else if (cmd1.length() > 2 && cmd1.startsWith("EEW") && cmd2 >= 0) { //change waypoint (and save to eeprom if 'S')
      debug(F("Setting ee.nextWaypoint to "));
      debugln(cmd2);
      ee.nextWaypoint = cmd2;
      if (cmd1.length() > 3 && cmd1[3] == 'S')
        eeSave(SAVE_REASON_TEST);
    }
    else if (cmd1.length() > 2 && cmd1.startsWith("EER") && cmd2 > 0) { //change rudder coeff (and save to eeprom if 'S')
      debug(F("Setting ee.rudder_coeff to "));
      debugln(cmd2);
      ee.rudder_coeff = cmd2;
      if (cmd1.length() > 3 && cmd1[3] == 'S')
        eeSave(SAVE_REASON_TEST);    
    }
    else if (cmd1 == "EELOAD" && cmd2 == 0) {
      eeLoad();
    }
    else if (cmd1 == "EEDEFAULT" && cmd2 == 0) {
      eeDefault(false);
    }
    else if (cmd1 == "EECORRUPT" && cmd2 == 0) {
      eeCorruptTest();
    }
    else if (cmd1 == "HALTTEST" && cmd2 == 0) { //no heartbeat for watchdog testing
      while(1);
    }
    else if (cmd1 == "CAMPIN") {
      debugln("CAM ON/OFF");
      if (cmd2) {
        digitalWrite(CAM_ON, HIGH); //active HIGH
        if (cmd2 == 115)
          camSerial.begin(115200);
        else
          camSerial.begin(BAUD_CAM);
      }
      else {
        digitalWrite(CAM_ON, LOW); //active HIGH
        camSerial.end();
      }
    }
    else if (cmd1 == "SATPIN") {
      digitalWrite(SAT_ON, cmd2 ? HIGH : LOW);
    }
    else if (cmd1 == "VID") {
      videoTask();
    }
    else if (cmd1 == "SAT") {
      satelliteTask();
    }
    else if (cmd1 == "ACTPIN") {
      debugln("Actuator pin ON/OFF");
      cancel_iridium = true;
      digitalWrite(ACT_ON, cmd2 ? HIGH : LOW);
    }
    else if (cmd1 == "SIMACTIVE") {
      sim.active = cmd2 ? true : false;
    }
    else if (cmd1 == "SIMWIND") {
      sim.trueWind = cmd2;
    }
    else if (cmd1 == "SIMGPSLAT") {
      sim.gps.lat = cmd2f;
    }
    else if (cmd1 == "SIMGPSLNG") {
      sim.gps.lng = cmd2f;
    }
    else if (cmd1 == "SIMMOVEINT") {
      intervals[SIM_MOVE_INDEX] = cmd2;
    }
    else if (cmd1 == "SIMPRINTINT") {
      intervals[SIM_PRINT_INDEX] = cmd2;    
    }
    else if (cmd1 == "SIMSPEED") {
      sim.max_speed = cmd2f;    
    }
    else if (cmd1 == "SIMRUDDERRESP") {
      sim.rudder_response = cmd2f;    
    }
    else if (cmd1 == "SIMPAUSE") {
      sim.pause = cmd2;    
    }
  }

#endif

void switchIR() {
  debugln(F("Switching to Iridium..."));
  debugln();
  cancel_iridium = false;
  Serial3.begin(BAUD_IR);
  setMUX(MUX_IR);
}

void switchGPS() {
  debugln(F("Switching to GPS..."));
  debugln();  
  Serial3.begin(BAUD_GPS);
  setMUX(MUX_GPS);
}

bool warning() { //watchdog shutdown warning
  return (digitalRead(WARNING) == LOW);
}

void noDebug() {
  delay(1); 
}

