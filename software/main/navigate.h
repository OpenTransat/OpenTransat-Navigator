#define NAVMODE_MANUAL 0
#define NAVMODE_RUDDER_AUTO 1
#define NAVMODE_FLAP_AUTO 2
#define NAVMODE_FULLY_AUTO 3

#define TACKMODE_DIRECTLY 0
#define TACKMODE_ADJ_POS 1
#define TACKMODE_ADJ_NEG 2
#define TACKMODE_MAXDEV_POS 3
#define TACKMODE_MAXDEV_NEG 4

#define DEFAULT_RUDDER_COEFF 40
#define DEFAULT_RUDDER_PAUSE_SEC 2 //in seconds
#define RUDDER_MIN_ANGLE -25
#define RUDDER_MAX_ANGLE 25

#define HEADING_OK_LIMIT 7 //in degs

#define FLAP_MAX 16
#define FLAP_NORMAL 10
#define TACK_SAIL_CRITICAL_ANGLE 7
#define WIND_CONFIRMATION_SEC 3

//Earth mean radius
#define R_MEAN 6371000.0 //meters

//how often the magnetic declination is updated
#define MAGDEC_UPDATE_DISTANCE 100000.0 //meters
#define INTENSIVE_TASKS_INTERVAL 30000UL //30sec (function lasts: 14ms)

//#define ACT_MUST_WAIT_FROM_LNG -52.8943 //prevent waiting during launch //not used, rather use LAUNCH_DATE
#define ACT_LAUNCH_DATE 80719UL //MMDDYYUL, no leading zero

struct Location {
  double lat;
  double lng;
};

struct {
  unsigned char mode = NAVMODE_FULLY_AUTO;
  unsigned char tackmode = TACKMODE_DIRECTLY;
  Location gps;
  float magDec; //in degs
  bool magDecInitialized = false; //magDec is initialized based on valid gps location before the boat can navigate
  bool intensiveTasks_initialized = false; //ghostPoint must be calculated before the boat can navigate
  bool ghostHeadingInitialized = false; //ghostHeading must be calculated before the boat can navigate
  Location magDec_loc; //last location where magDec was calculated
  bool IR_doTransmit = false; //this flag is set to true in Iridium interval and transmitting starts in the navigate(..) function when the rudder is idle for a moment
  int trueHeading; //compass yaw + magnetic declination
  int trueWind; //trueHeading + relative wind
  Location closestPoint;
  Location ghostPoint;
  double crossTrack;
  double alongTrack;
  int ghostHeading_instant; //heading to the ghostPoint
  int ghostHeading; //heading to the ghostPoint (if headingOK())
  int goHeading; //in degs, where to go
  int pathAngle; //path bearing at closestPoint
  bool isBehindPath;
  int error; //in degs
  int flap;
  int sail_angle;
  int rudder_angle = 0; //must be remembered after the navigate() iteration
  bool rudderAgainstWind; //if the default turning is against the wind
  bool IR_transmit = false;
  unsigned long intensive_ms = 0;
  unsigned long nonIntensive_micros = 0;
  char wind_confirmation_sec = WIND_CONFIRMATION_SEC;
} nav;

struct TadjustedAngles {
  bool canGoDirectly;
  int adjAngle1;
  int adjAngle2;
};

struct TbestAdjHeading {
  int heading;
  unsigned char tackmode;
};

//location with radius[meters]; waypoint is considered to be reached within the radius, boat is heading to the ghostPoint at ghostDistance and special consideration is taken if max deviation from the main path is reached
struct waypoint {
  Location loc;
  double radius;
  double ghostDistance;
  double maxDev;
};

#define MAXDEV_OK_FACTOR 0.75

//Newfoundland -> waypoint on finish line -> Brest
//set of waypoints
#define WAYPOINTS 4 //minimum 2: (start, finish}
#define FINISH_WAYPOINT 3

//loc, radius, ghostDistance, maxDev

waypoint path[WAYPOINTS] = {
  {{46.913520, -52.998886}, 0.0, 0.0, 0.0}, //Newfoundland
  {{46.75, -47.0}, 50000.0, 50000.0, 50000.0}, //on the start line, 139km below the north endpoint
  {{46.75, -46.34}, 50000.0, 50000.0, 50000.0}, //50km east from the previous point (to make sure to cross the start line)
  {{48.0, -13.0}, 20000.0, 75000.0, 75000.0}, //finish line (also update FINISH_WAYPOINT macro, indexing starts from 0)
};
/*
waypoint path[WAYPOINTS] = { //!!!ONLY FOR SIMULATION, COMMENT FOR REAL MISSION
  {{46.913520, -52.998886}, 0.0, 0.0, 0.0}, //Newfoundland
  {{46.75, -47.0}, 70000.0, 250000.0, 250000.0}, //on the start line, 139km below the north endpoint
  {{46.75, -46.34}, 70000.0, 250000.0, 250000.0}, //50km east from the previous point (to make sure to cross the start line)
  {{48.0, -13.0}, 70000.0, 500000.0, 500000.0}, //finish line (also update FINISH_WAYPOINT macro, indexing starts from 0)
};
*/
#define SIM_MAX_SPEED 2000000.0
#define SIM_RUDDER_RESPONSE 0.00005

struct {
  bool active = false;
  bool pause = false;
  int trueHeading = 0;
  double trueHeading_f = 0; //sim_move()
  int trueWind = 0;
  Location gps = path[0].loc;
  int rudder_angle = 0;
  int flap = FLAP_NORMAL;
  int adjAngle1;
  int adjAngle2;
  double max_speed = SIM_MAX_SPEED;
  double rudder_response = SIM_RUDDER_RESPONSE;
} sim;

struct {
} sim_debug;


