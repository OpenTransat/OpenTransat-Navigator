
void eeSave(unsigned char reason) {    
  debug(F("Saving EEPROM... reason:"));
  debugln(uint8_t(reason));
  ee.checksum = eeChecksum();
  EEPROM.put(EEPROM_ADDR, ee);
}

void eeDefault(bool checksum_failed) {
  debugln(F("Assigning default EEPROM values..."));
  ee = ee_default; //compiler takes care
  ee.nextWaypoint = FINISH_WAYPOINT; //problem with failsafeWaypoint: valid GPS needed at eeLoad(), fixed by checking ee.checksum_failed in main.ino
  ee.checksum_failed = checksum_failed;
  eeSave(SAVE_REASON_DEFAULT);
  eeDebug();
}

void eeLoad() {
  debugln(F("Loading EEPROM..."));
  EEPROM.get(EEPROM_ADDR, ee);
  
  if (ee.checksum != eeChecksum()) { //eeprom corrupted, load and save default values with ee.checksum_failed = true
    debugln(F(">> EEPROM CHECKSUM ERROR, LOADING DEFAULT VALUES AND SAVING"));
    eeDefault(true);
  }
  else
    debugln(F("EEPROM CHECKSUM OK"));

  eeDebug();
}

void eeDebug() {
  debug(F("resets: "));
  debugln(ee.resets);
  
  debug(F("nextWaypoint: "));
  debugln(ee.nextWaypoint);

  debug(F("last_A: "));
  debugln(ee.last_A);
  
  debug(F("rudder_coeff: "));
  debugln(ee.rudder_coeff);

  debug(F("rudder_pause_sec: "));
  debugln(ee.rudder_pause_sec);

  debug(F("act_cycles: "));
  debugln(ee.act_cycles);

  debug(F("cam_transmit_sec: "));
  debugln(ee.cam_transmit_sec);

  debug(F("override_flag: "));
  debugln(ee.override_flag);

  debug(F("override_customID: "));
  debugln(ee.override_customID);

  debug(F("override_prev_loc: "));
  debug(ee.override_prev_loc.lat);
  debug(" ");
  debugln(ee.override_prev_loc.lng);

  debug(F("override_next_loc: "));
  debug(ee.override_next_loc.lat);
  debug(" ");
  debugln(ee.override_next_loc.lng);
  
  debug(F("override_ghostDistance: "));
  debugln(ee.override_ghostDistance);

  debug(F("override_maxDev: "));
  debugln(ee.override_maxDev);

  debug(F("override_customVar: "));
  debugln(ee.override_customVar);

  debug(F("checksum_failed: "));
  debugln(ee.checksum_failed);
  
  debugln(F("-----------------------------------"));
}

unsigned int eeChecksum() {
  unsigned int ret = 0;
  unsigned char *p = (unsigned char *)&ee;
  for (int i = 0; i < sizeof(ee) - sizeof(ee.checksum); i++) {
    ret += *(p++);
  }
  return ret;
}

void eeCorruptTest() {
  debugln(F("Corruptin EEPROM..."));
  ee.checksum = 0; //wrong checksum
  EEPROM.put(EEPROM_ADDR, ee);
  eeLoad();
}

