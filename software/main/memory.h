#define EEPROM_ADDR 0

struct EEPROM_data {
  unsigned int resets;
  unsigned char nextWaypoint;
  int last_A;
  int rudder_coeff;
  unsigned char rudder_pause_sec; //seconds
  unsigned long act_cycles;
  unsigned int cam_transmit_sec;
  bool override_flag;
  unsigned char override_customID;
  Location override_prev_loc;
  Location override_next_loc;
  double override_ghostDistance;
  double override_maxDev;
  unsigned char override_customVar; //first 7 bits: iridium interval in minutes, last bit: checkMustWait
  bool checksum_failed;
  unsigned int checksum; //must be last 2 bytes of the struct
};

EEPROM_data ee;
EEPROM_data ee_default = {.resets = 0,
                          .nextWaypoint = FINISH_WAYPOINT,
                          .last_A = 100,
                          .rudder_coeff = DEFAULT_RUDDER_COEFF,
                          .rudder_pause_sec = DEFAULT_RUDDER_PAUSE_SEC,
                          .act_cycles = 0,
                          .cam_transmit_sec = 0,
                          .override_flag = false,
                          .override_customID = 100,
                          .override_prev_loc = {0.0, 0.0},
                          .override_next_loc = {0.0, 0.0},
                          .override_ghostDistance = 0.0,
                          .override_maxDev = 0.0,
                          .override_customVar = 1,
                          .checksum_failed = false,
                          .checksum = 0};

#define SAVE_REASON_RESETS 0
#define SAVE_REASON_LASTAE 1
#define SAVE_REASON_LASTAP 2
#define SAVE_REASON_TEST 3
#define SAVE_REASON_ACTADDTIME 4
#define SAVE_REASON_WAYPOINT 6
#define SAVE_REASON_DEFAULT 7
#define SAVE_REASON_CAM_SEC 8
#define SAVE_REASON_OVERRIDE_CLEAR 9
#define SAVE_REASON_LAUNCH 10
#define SAVE_REASON_OVERRIDE 11
#define SAVE_REASON_FAILSAFE_WAYPOINT 12

