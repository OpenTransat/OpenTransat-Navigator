
void navigate() {
  
  checkWaypoint();

  //intensive tasks to calculate nav.ghostPoint, nav.ghostHeading_instant, etc.
  static unsigned long it_last = millis();
  if (sim.active || !nav.intensiveTasks_initialized || millis() > it_last + INTENSIVE_TASKS_INTERVAL) {
    intensiveTasks();
    it_last = millis();
  }

  unsigned long start_micros = micros();
  where2go();
  adjustRudder();
  adjustFlap();
  nav.nonIntensive_micros = micros() - start_micros;
}

void checkWaypoint() {
  if (ee.override_flag)
    return;
  
  //fix invalid waypoint (shouldn't happen)
  if (ee.nextWaypoint < 1)
    ee.nextWaypoint = FINISH_WAYPOINT;

  //check if waypoint is reached within the radius => go to the next waypoint index and save it to EEPROM
  //if it is the last waypoint, do not update NextWayPoint, just loop around it    
  if (distance(nav.gps, path[ee.nextWaypoint].loc) < path[ee.nextWaypoint].radius
  && ee.nextWaypoint < WAYPOINTS - 1) {

    if (ee.nextWaypoint == FINISH_WAYPOINT) { //target point hit!
      nav.IR_transmit = true; //make sure to transmit (it will transmit only once because ee.nextWaypoint gets incremented)
    }

    ee.nextWaypoint++;
    nav.isBehindPath = false;
        
    if (!sim.active)
      eeSave(SAVE_REASON_WAYPOINT);
      
    intensiveTasks();
    
    debug(F(">>> Going to the next waypoint: "));
    debugln(ee.nextWaypoint);
  }
}

void intensiveTasks() {
  debugln(F(">>> RUNNING INTENSIVE TASKS..."));
  unsigned long start_ms = millis();

  nav.intensiveTasks_initialized = true;

  if (distance(nav.gps, nav.magDec_loc) > MAGDEC_UPDATE_DISTANCE)
    updateMagDeclination(nav.gps);

  Location waypoint0 = ee.override_flag ? ee.override_prev_loc : path[ee.nextWaypoint-1].loc;
  Location waypoint1 = ee.override_flag ? ee.override_next_loc : path[ee.nextWaypoint].loc;
  double ghostDistance = ee.override_flag ? ee.override_ghostDistance : path[ee.nextWaypoint].ghostDistance;

  double startAngle = trueBearing(waypoint0, waypoint1);
  double pathLength = distance(waypoint0, waypoint1);
  double d13 = distance(waypoint0, nav.gps);
  
  nav.crossTrack = crossTrackDistance(waypoint0, waypoint1, nav.gps, d13);
  
  nav.alongTrack = 0;
  if (!isBeforePath(nav.gps, waypoint0, waypoint1))
    nav.alongTrack = alongTrackDistance(waypoint0, waypoint1, nav.gps, d13, nav.crossTrack);

  nav.ghostPoint = waypoint1;
  if (nav.alongTrack + ghostDistance < pathLength)
    nav.ghostPoint = destination(waypoint0, startAngle, nav.alongTrack + ghostDistance);
  
  nav.closestPoint = destination(waypoint0, startAngle, nav.alongTrack);

  nav.pathAngle = round(trueBearing(nav.closestPoint, waypoint1));
  if (nav.alongTrack > pathLength)
    nav.pathAngle = mod360(nav.pathAngle + 180); //pathAngle would be pointing the opposite way

  if (nav.alongTrack > pathLength + path[ee.nextWaypoint].radius) { //this prevents nav.isBehindPath from changing too often
    nav.isBehindPath = true;
  }
  else if (nav.alongTrack < pathLength - path[ee.nextWaypoint].radius) {
    nav.isBehindPath = false;
  }

  nav.ghostHeading_instant = round(trueBearing(nav.gps, nav.ghostPoint));

  nav.intensive_ms = millis() - start_ms;

  debug(F("Intensive tasks in millis: "));
  debugln(nav.intensive_ms);
  debug(F("Non-intensive tasks in micros: "));
  debugln(nav.nonIntensive_micros);
  debug(F("Magnetic Declination: "));
  debugln(nav.magDec);
}

void where2go() {
  //update ghostHeading only if heading is OK

  if (!nav.ghostHeadingInitialized || headingOK(nav.error))
    nav.ghostHeading = nav.ghostHeading_instant;

  nav.ghostHeadingInitialized = true;

  //change tackmode, crossTrack is positive on the right from the path, negative on the left

  double maxDev = ee.override_flag ? ee.override_maxDev : path[ee.nextWaypoint].maxDev;

  if (nav.crossTrack > maxDev)
      nav.tackmode = TACKMODE_MAXDEV_POS;
  else if (nav.crossTrack < -maxDev)
      nav.tackmode = TACKMODE_MAXDEV_NEG;

  if (nav.tackmode == TACKMODE_MAXDEV_POS && nav.crossTrack < maxDev*MAXDEV_OK_FACTOR
  || nav.tackmode == TACKMODE_MAXDEV_NEG && nav.crossTrack > -maxDev*MAXDEV_OK_FACTOR)
      nav.tackmode = TACKMODE_DIRECTLY;

  //if it can't go directly, calculate adjusted angles

  TadjustedAngles a = calcAdjustedAngles(nav.ghostHeading, nav.trueWind);
  sim.adjAngle1 = a.adjAngle1;
  sim.adjAngle2 = a.adjAngle2;
 
  if (nav.tackmode == TACKMODE_MAXDEV_NEG) {
    if (a.canGoDirectly)
      nav.goHeading = nav.ghostHeading;
    else {

      int angle_from = nav.pathAngle;
      int angle_to = nav.pathAngle + 140;
      
      if (nav.isBehindPath) {
        angle_from = nav.pathAngle + 180 - 140;
        angle_to = nav.pathAngle + 180;
      }
     
      if (!isBetweenOrientedAngles(a.adjAngle1, angle_from, angle_to)) {
        nav.goHeading = a.adjAngle2;
      }
      else if (!isBetweenOrientedAngles(a.adjAngle2, angle_from, angle_to)) {
        nav.goHeading = a.adjAngle1;
      }
      else {
        TbestAdjHeading bh = bestAdjHeading(a.adjAngle1, a.adjAngle2, nav.trueHeading, nav.trueWind);
        nav.goHeading = bh.heading;
      }
    }
  }
  else if (nav.tackmode == TACKMODE_MAXDEV_POS) {
    if (a.canGoDirectly)
      nav.goHeading = nav.ghostHeading;
    else {

      int angle_from = nav.pathAngle - 140;
      int angle_to = nav.pathAngle;
      
      if (nav.isBehindPath) {
        angle_from = nav.pathAngle + 180;
        angle_to = nav.pathAngle + 180 + 140;
      }
      
      if (!isBetweenOrientedAngles(a.adjAngle1, angle_from, angle_to)) {
        nav.goHeading = a.adjAngle2;
      }
      else if (!isBetweenOrientedAngles(a.adjAngle2, angle_from, angle_to)) {
        nav.goHeading = a.adjAngle1;
      }
      else {
        TbestAdjHeading bh = bestAdjHeading(a.adjAngle1, a.adjAngle2, nav.trueHeading, nav.trueWind);
        nav.goHeading = bh.heading;
      }
    }
  }
  else {
    if (a.canGoDirectly) {
        nav.goHeading = nav.ghostHeading;
        nav.tackmode = TACKMODE_DIRECTLY;
    }
    else {
      TbestAdjHeading bh = bestAdjHeading(a.adjAngle1, a.adjAngle2, nav.trueHeading, nav.trueWind);
      nav.goHeading = bh.heading;
      nav.tackmode = bh.tackmode;
    }
  }

  if (!sim.active) {
    nav.flap = (int)ee.last_A - 100;
  }
  else {
    nav.flap = sim.flap;
  }
  
  nav.sail_angle = mod360(nav.trueWind - nav.trueHeading - nav.flap);

}

TadjustedAngles calcAdjustedAngles(int ghostHeading, int trueWind) {
  #define FLAPFIX FLAP_NORMAL
  #define SMALLFIX 3
  int angle1 = mod360(nav.trueWind - 45 - SMALLFIX + FLAPFIX); //with the wind
  int angle2 = mod360(nav.trueWind + 45 + SMALLFIX - FLAPFIX); //with the wind
  int angle3 = mod360(nav.trueWind + 135 - SMALLFIX - FLAPFIX); //against the wind
  int angle4 = mod360(nav.trueWind - 135 + SMALLFIX + FLAPFIX); //against the wind
  
  bool canGoDirectly = false;
  int adjAngle1 = 0;
  int adjAngle2 = 0;
  
  if (isBetweenOrientedAngles(ghostHeading, angle1, angle2)) {
    canGoDirectly = false;
    adjAngle1 = angle1;
    adjAngle2 = angle2;
  }
  else if (isBetweenOrientedAngles(ghostHeading, angle2, angle3)) {
    canGoDirectly = true;
    adjAngle1 = angle2;
    adjAngle2 = angle3;
  }
  else if (isBetweenOrientedAngles(ghostHeading, angle3, angle4)) {
    canGoDirectly = false;
    adjAngle1 = angle3;
    adjAngle2 = angle4;
  }
  else if (isBetweenOrientedAngles(ghostHeading, angle4, angle1)) {
    canGoDirectly = true;
    adjAngle1 = angle4;
    adjAngle2 = angle1;
  }
      
  return {canGoDirectly, adjAngle1, adjAngle2};
}

TbestAdjHeading bestAdjHeading(int adjAngle1, int adjAngle2, int trueHeading, int trueWind) {
  int opWind = mod360(trueWind + 180);
  if ((isBetweenOrientedAngles(trueWind, adjAngle1, trueHeading)
  || isBetweenOrientedAngles(opWind, adjAngle1, trueHeading))
  && (isBetweenOrientedAngles(trueWind, trueHeading, adjAngle1)
  || isBetweenOrientedAngles(opWind, trueHeading, adjAngle1))) {
    return {adjAngle2, TACKMODE_ADJ_POS};
  }
  else if ((isBetweenOrientedAngles(trueWind, adjAngle2, trueHeading)
  || isBetweenOrientedAngles(opWind, adjAngle2, trueHeading))
  && (isBetweenOrientedAngles(trueWind, trueHeading, adjAngle2)
  || isBetweenOrientedAngles(opWind, trueHeading, adjAngle2))) {
      return {adjAngle1, TACKMODE_ADJ_NEG};
  }
  else {
    //shouldn't happen
  }
  return {0, 0}; //shouldn't happen
}

void adjustRudder() {

  //calculate error and adjust it based on the wind direction (boat must not turn against the wind)
  int angle_positive = orientedDiff(nav.trueHeading, nav.goHeading);
  
  if (nav.mode == NAVMODE_FULLY_AUTO && !isTurnAgainstWind(nav.trueWind, nav.trueHeading, nav.goHeading)
     || nav.mode != NAVMODE_FULLY_AUTO && angle_positive < 180) {
    nav.error = angle_positive;
  }
  else {
    nav.error = -(360 - angle_positive);
  }

  if (headingOK(nav.error)) { //if heading ok => pause the rudder for a few seconds (and start transmitting if the nav.IR_transmit flag is set)
    if (nav.mode != NAVMODE_FLAP_AUTO)
      nav.rudder_angle = 0; //rudder straight
      
    rudder.pause_start = millis();
    
    if (nav.IR_transmit/* && !sim.active*/) {
      nav.IR_transmit = false;
      reqData2(IR_sendData, IR_sendData);
    }
  }
  else if (sim.active || millis() > rudder.pause_start + (unsigned long)ee.rudder_pause_sec*1000UL) { //heading not ok => adjust the rudder after the pause
    //boat is against the wind and is reversed, do a special maneuver to safely switch forward (can't be the opposite way... proven)
  
    if (nav.sail_angle < 180 && nav.flap > 0 || nav.sail_angle > 180 && nav.flap < 0) { //reversed
      if (nav.sail_angle > 180 - 45 - 2*FLAP_NORMAL && nav.sail_angle < 180 && nav.flap > 0) { //against the wind
          nav.rudder_angle = RUDDER_MAX_ANGLE; //boat "turning left", but right backwards
      }
      else if (nav.sail_angle < 180 + 45 + 2*FLAP_NORMAL && nav.sail_angle > 180 && nav.flap < 0) {
          nav.rudder_angle = RUDDER_MIN_ANGLE;
      }
  //      else
  //        nav.rudder_angle = 0 //not needed, rather prevent rudder jitter
    }
    else { //not reversed
      nav.rudder_angle = limits(round(-(float)ee.rudder_coeff / 100 * nav.error), RUDDER_MIN_ANGLE, RUDDER_MAX_ANGLE);
    }
  }

  //generate pulse (pulse is sent in the main loop)
  if (!sim.active) {
    if (nav.mode != NAVMODE_FLAP_AUTO)
      rudder.pulse = 100 + nav.rudder_angle;
  }
  else {
    sim.rudder_angle = nav.rudder_angle;
  }
}

void adjustFlap() {
  bool cond1 = (nav.sail_angle < 180 && nav.flap >= 0);
  bool cond2 = (nav.error < 0 && (nav.sail_angle > 360 - TACK_SAIL_CRITICAL_ANGLE || nav.sail_angle < 0 + TACK_SAIL_CRITICAL_ANGLE) && nav.flap >= 0);
  bool cond3 = (nav.sail_angle > 180 && nav.flap <= 0);
  bool cond4 = (nav.error > 0 && (nav.sail_angle < 0 + TACK_SAIL_CRITICAL_ANGLE || nav.sail_angle > 360 - TACK_SAIL_CRITICAL_ANGLE) && nav.flap <= 0);
  
  if (validForSeconds(0, nav.wind_confirmation_sec, cond1 || cond2)) {
    if (cond2)
      smartStartActuator(-FLAP_MAX, true, true);
    else if (cond1)
      smartStartActuator(-FLAP_NORMAL, true, true);
  }
    
  else if (validForSeconds(1, nav.wind_confirmation_sec, cond3 || cond4)) {
    if (cond4)
      smartStartActuator(FLAP_MAX, true, true);
    else if (cond3)
      smartStartActuator(FLAP_NORMAL, true, true);
  }
    
  //FLAP FROM MAX BACK TO NORMAL

  else if (validForSeconds(2, nav.wind_confirmation_sec, nav.flap <= -FLAP_MAX && nav.sail_angle < 180 && nav.sail_angle > 50))
    smartStartActuator(-FLAP_NORMAL, false, false);
    
  else if (validForSeconds(3, nav.wind_confirmation_sec, nav.flap >= FLAP_MAX && nav.sail_angle > 180 && nav.sail_angle < 360 - 50))
    smartStartActuator(FLAP_NORMAL, false, false);
}

bool validForSeconds(char index, char sec, bool condition) {  
  static unsigned long valid_since_time[4] = {millis(), millis(), millis(), millis()};

  if (!condition)
    valid_since_time[index] = millis();
  
  if (millis() > valid_since_time[index] + (unsigned long)sec*1000UL)
    return true;
  else
    return false;
}

void smartStartActuator(int flap, bool checkMustWait, bool addTime) {
  if (sim.active) {
    sim.flap = flap;
    return;
  }
  
  if (mux == MUX_ACT) {
    //debugln(">> Actuator already busy, ignoring");
    return;
  }
  
  if (sail_state != SAIL_STATE_IDLE) {
    //debugln(">> Sail state not idle, ignoring");
    return;
  }

  if (sail_max_retries_wait) {
    //debugln(">> Waiting after max retries reached");
    return;
  }

  if (ee.override_flag && checkMustWait)
    checkMustWait = ee.override_customVar % 2 ? true : false;

  if (checkMustWait && act.mustWait /*&& nav.gps.lng > ACT_MUST_WAIT_FROM_LNG*/ && gps.date.value() != ACT_LAUNCH_DATE) {
    //debugln(">> Actuator must wait, ignoring");
    return;
  }

  if (/*nav.gps.lng < ACT_MUST_WAIT_FROM_LNG*/ gps.date.value() == ACT_LAUNCH_DATE) //launching the boat: don't add time
    addTime = false;

  int cmd = 100 + flap;
  if (ee.last_A == cmd) {
    debugln(">> Adjusting the flap to the same position, ignoring");
    return;
  }
  
  switchAct();
  startActuator(cmd, addTime);
}

bool headingOK(int error) {
  return (abs(error) < HEADING_OK_LIMIT);
}

int orientedDiff(int angle_from, int angle_to) {
  return mod360(angle_to - angle_from);
}

bool isBetweenOrientedAngles(int angle, int angle_from, int angle_to) {
  return (orientedDiff(angle_from, angle) < orientedDiff(angle_from, angle_to));
}

bool isTurnAgainstWind(int windAngle, int angle_from, int angle_to) {
  return isBetweenOrientedAngles(mod360(windAngle + 180), angle_from, angle_to);
}

//normalize angle to fall in interval <0, 360)
int mod360(int angle) {
  return ((angle % 360) + 360) % 360;
}

//normalize angle to fall in interval <0, 360)
double normAngle360_f(double angle) {
  while (angle >= 360)
    angle -= 360;
  while (angle < 0)
    angle += 360;
  return angle;
}

double deg2rad(double angle) {
  return angle * PI / 180;
}

double rad2deg_f(double angle) {
  return angle / PI * 180;
}

//distance in meters between two locations
double distance(Location loc1, Location loc2) {
  double lat1 = deg2rad(loc1.lat);
  double lng1 = deg2rad(loc1.lng);
  double lat2 = deg2rad(loc2.lat);
  double lng2 = deg2rad(loc2.lng);  
  double hav = sq(sin((lat2-lat1)/2)) + cos(lat1) * cos(lat2) * sq(sin((lng2-lng1)/2));
  double d = 2 * R_MEAN * atan2(sqrt_safe(hav), sqrt_safe(1-hav));
  return d;
}

//angle in degs between the vector (loc1, loc2)  and the vector pointing to the true north
double trueBearing(Location loc1, Location loc2) {
  double lat1 = deg2rad(loc1.lat);
  double lng1 = deg2rad(loc1.lng);
  double lat2 = deg2rad(loc2.lat);
  double lng2 = deg2rad(loc2.lng);
  double angle = atan2(sin(lng2 - lng1) * cos(lat2), cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lng2 - lng1));
  return normAngle360_f(rad2deg_f(angle));
}

Location destination(Location loc1, double bearing, double d) {
  double lat1 = deg2rad(loc1.lat);
  double lng1 = deg2rad(loc1.lng);
  bearing = deg2rad(bearing);
  double lat2 = asin_safe(sin(lat1)*cos(d/R_MEAN) + cos(lat1)*sin(d/R_MEAN)*cos(bearing));
  double lng2 = lng1 + atan2(sin(bearing)*sin(d/R_MEAN)*cos(lat1), cos(d/R_MEAN) - sin(lat1)*sin(lat2));
  Location loc2 = {rad2deg_f(lat2), rad2deg_f(lng2)};
  return loc2;
}

double sqrt_safe(double x) {
  if (x <= 0)
    return 0;
  return sqrt(x);
}

double asin_safe(double x) {
  if (x <= -1)
    return -PI/2;
  if (x >= 1)
    return PI/2;
  return asin(x);  
}

double acos_safe(double x) {
  if (x <= -1)
    return -PI;
  if (x >= 1)
    return 0;
  return acos(x);
}

double crossTrackDistance(Location loc1, Location loc2, Location loc3, double d13) {
  double b13 = deg2rad(trueBearing(loc1, loc3));
  double b12 = deg2rad(trueBearing(loc1, loc2));
  return R_MEAN*asin_safe(sin(d13/R_MEAN)*sin(b13 - b12));
}

double alongTrackDistance(Location loc1, Location loc2, Location loc3, double d13, double crossTrack) {
  return R_MEAN*acos_safe(cos(d13/R_MEAN)/cos(crossTrack/R_MEAN));
}

int limits(int x, int x_min, int x_max) {
  if (x < x_min)
    return x_min;
  else if (x > x_max)
    return x_max;
  return x;
}

bool isBehindPath(Location loc, Location loc1, Location loc2) {
  double finishAngle = normAngle360_f(trueBearing(loc2, loc1) - 180);
  double toBoat = normAngle360_f(trueBearing(loc2, loc));
  return (fabs(toBoat - finishAngle) < 90);
}

bool isBeforePath(Location loc, Location loc1, Location loc2) {
  return isBehindPath(loc, loc2, loc1);
}

void updateMagDeclination(Location gps_loc) {
  nav.magDec = magDeclination(gps_loc);
  nav.magDec_loc = gps_loc;
  nav.magDecInitialized = true;
}

unsigned char failsafeWaypoint_West2East() { //valid GPS needed!
  for (unsigned char i = WAYPOINTS-1; i >= 0; i--) {
    if (path[i].loc.lng < gps.location.lng()) {
      if (i < WAYPOINTS-1)
        return i+1;
      else
        return WAYPOINTS-1;
    }
  }
  return 1;
}

void sim_move() {
  if (!sim.active)
    return;

  if (sim.pause)
    return;

  double boat_speed = calcSpeed();
  double dt = (double)(intervals[SIM_MOVE_INDEX])/1000.0; //seconds

  sim.gps = destination(sim.gps, sim.trueHeading_f, boat_speed*dt);
  sim.trueHeading_f += -sim.rudder_response*sim.rudder_angle*boat_speed*dt;
  sim.trueHeading_f = normAngle360_f(sim.trueHeading_f);
  sim.trueHeading = mod360(round(sim.trueHeading_f));
}

double calcSpeed() {
  int sail_angle = mod360(sim.trueWind - sim.trueHeading - sim.flap);
  double ret = fabs(sim.max_speed*sin(deg2rad(sail_angle)));

  if (sail_angle < 180 and sim.flap > 0 or sail_angle > 180 and sim.flap < 0)
    ret = -ret;
    
  return ret;
}

void sim_print() {
  if (!sim.active)
    return;

  Serial.print("S ");
  Serial.print(sim.trueHeading);
  Serial.print(" ");
  Serial.print(sim.trueWind);
  Serial.print(" ");
  Serial.print(sim.gps.lat, 6);
  Serial.print(" ");
  Serial.print(sim.gps.lng, 6);
  Serial.print(" ");
  Serial.print(sim.rudder_angle);
  Serial.print(" ");
  Serial.print(sim.flap);
  Serial.print(" ");
  Serial.print(nav.closestPoint.lat, 6);
  Serial.print(" ");
  Serial.print(nav.closestPoint.lng, 6);
  Serial.print(" ");
  Serial.print(nav.ghostPoint.lat, 6);
  Serial.print(" ");
  Serial.print(nav.ghostPoint.lng, 6);
  Serial.print(" ");
  Serial.print(nav.ghostHeading);
  Serial.print(" ");
  Serial.print(nav.goHeading);
  Serial.print(" ");
  Serial.print(sim.adjAngle1);
  Serial.print(" ");
  Serial.print(sim.adjAngle2);
  Serial.print(" ");
  Serial.print(nav.isBehindPath ? 1 : 0);
  Serial.print(" ");
  Serial.print(nav.tackmode);
  Serial.print(" ");
  Serial.print(ee.override_flag ? ee.override_customID : ee.nextWaypoint);
  Serial.print(" ");
  Serial.print((uint8_t)mux);
  Serial.print(" ");
  Serial.print(nav.magDec);
  Serial.print(" E");
  Serial.print(" ");
    
  Serial.println();
}

