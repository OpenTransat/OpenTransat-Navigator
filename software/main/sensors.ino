vamps read_INA(Adafruit_INA219 ina) {
  float shuntvoltage = ina.getShuntVoltage_mV();
  float busvoltage = ina.getBusVoltage_V();
  float volts = busvoltage + shuntvoltage / 1000;

  if (volts < 0 || volts > MAX_VALID_VOLTS)
    return {0, 0};
  
  float ma = ina.getCurrent_mA();
  return {volts, ma};
}

float readTemp(){
  byte data[12];
  byte addr[8];

  if (!ds.search(addr)) {
    //debug("No more addresses.\n");
    //no more sensors on chain, reset search
    ds.reset_search();
    return -1000;
  }
/*
  debug("R=");
  for( int i = 0; i < 8; i++) {
    debug(addr[i], HEX);
    debug(" ");
  }
*/
  if (OneWire::crc8( addr, 7) != addr[7]) {
    //debugln("CRC is not valid!");
    return -1000;
  }

  if (addr[0] != 0x10 && addr[0] != 0x28) {
    //debug("Device is not recognized");
    return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end
  smartDelay(1000); //sometimes garbage result if this delay is missing
  byte present = ds.reset();
  ds.select(addr);  
  ds.write(0xBE); // Read Scratchpad

 
  for (int i = 0; i < 9; i++) { // we need 9 bytes
   data[i] = ds.read();
  }
 
  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;
 
  return TemperatureSum;
}

void BME_init() {
  bool status = bme.begin();
  if (!status) {
    debugln(F("Could not find a valid BME280 sensor, check wiring!"));
  } else {
    debugln(F("BME found."));
  }
}

void MPU_init() {
  mpu.init(true);

  uint8_t wai = mpu.whoami();

  if (wai == 0x71){
    debugln(F("MPU Successful connection"));
  }
  else{
    debug(F("MPU Failed connection: "));
    debugln2(wai, HEX);
  }
  
  uint8_t wai_AK8963 = mpu.AK8963_whoami();
  if (wai_AK8963 == 0x48){
    debugln(F("MPU Successful connection to mag"));
  }
  else{
    debug(F("MPU Failed connection to mag: "));
    debugln2(wai_AK8963, HEX);
  }

  mpu.calib_acc();
  mpu.calib_mag(); 
}

void print_vamps(vamps va) {
  debug(va.volts);
  debug("V ");
  debug(va.mamps);
  debug("mA");
}

void readFlood() {
  #define MAX_MEASUREMENTS 10000

  if (data2.flood0_total > MAX_MEASUREMENTS)
    data2.flood0_total = data2.flood0_count = 0;
  
  data2.flood0 = analogRead(FLOOD0);

  if (data2.flood0 < FLOOD_THRESHOLD)
    data2.flood0_count++;

  data2.flood0_total++;

  if (data2.flood1_total > MAX_MEASUREMENTS)
    data2.flood1_total = data2.flood1_count = 0;
  
  data2.flood1 = analogRead(FLOOD1);

  if (data2.flood1 < FLOOD_THRESHOLD)
    data2.flood1_count++;

  data2.flood1_total++;
}

vamps decodeVamps(unsigned int val, float volts_factor, float mamps_factor) {
  vamps ret;
  ret.volts = ((val >> 8) & 0xFF) / volts_factor;
  ret.mamps = (val & 0xFF) / mamps_factor;
  return ret;
}

void readVamps() {
  static float avg_sum;
  static unsigned long count = 0;
  
  ina219_nav.begin();
  vamps va2 = read_INA(ina219_nav);
  ina219_nav.enterPowerSave();

  if (va2.volts > 1 && va2.volts < MAX_VALID_VOLTS && va2.mamps > 0 && va2.mamps < 4000) { //use only reasonable values for the Navigator
    va.volts = va2.volts;
    va.mamps = va2.mamps;
    avg_sum += va.mamps;
    count++;
    avg_mamps = avg_sum / count;
  }
}

void countCapsize() {
  static bool prev_capsized = false;
  bool capsized = (compass.ID > 0 && compass.ID < 5 && abs(compass.pitch) > 90 && abs(compass.roll) < 90);

  if (capsized && !prev_capsized)
    data2.capsize++;

  prev_capsized = capsized;
}

void checkCompass() { //to be checked in the main loop
  static float prev_yaw = 0;
  static unsigned long yaw_change_time = millis();
  
  if ((long)(compass.yaw*10000) != (long)(prev_yaw*10000)) { //compass updating
    compass.bad_resets = 0;
    yaw_change_time = millis();
    prev_yaw = compass.yaw;
  }

  if (millis() - yaw_change_time > COMPASS_TIMEOUT) {
    
    if (compass.bad_resets > MAX_COMPASS_BAD_RESETS) {
      debugln(F(">> COMPASS NOT UPDATING AFTER NUMEROUS RESETS, SWITCHING TO THE NEXT ONE..."));
      switchCompassNext(true);
      compass.bad_resets = 0;
    }
    else {
      debugln(F(">> COMPASS YAW NOT UPDATING, RESETTING..."));
      resetCompass();
      compass.bad_resets++;
    }
    
    yaw_change_time = millis();
  }
}

void switchCompass(unsigned char compassID, bool save) {
  if (warning())
    return;
  
  debug(F(">>> Switching compass to #"));
  debug(compassID);
  
  if (save) {
    Serial2.print("$E");
    debugln(F(" and saving"));
  }
  else {
    Serial2.print("$C");
    debugln(F(" and NOT saving"));
  }
  
  Serial2.print(compassID);
  Serial2.print(";");
}

void switchCompassNext(bool save) {
  static unsigned char nextID = 1;
  switchCompass(nextID, save);
  nextID++;
  if (nextID > 4)
    nextID = 1;
}

void resetCompass() {
  debugln(F(">>> Resetting compass"));
  Serial2.print(F("$RESET0;"));
}

void checkHall() { //to be checked in the main loop
  if (millis() - sail.hall_update > HALL_TIMEOUT) {
    debugln(F(">>> HALL NOT UPDATING FOR TOO LONG, TURNING OFF..."));
    digitalWrite(HALL_ON, LOW);
    smartDelay(HALL_RESET_DELAY);
    debugln(F(">>> HALL: TURNING ON..."));
    digitalWrite(HALL_ON, HIGH); //active HIGH
    sail.hall_update = millis();
  }
}

int windFusion(int sailWind, int anemoWind) {
  if (abs(anemoWind - sailWind) < 5) //anemometer is more accurate then sailwing position sensor
    return anemoWind;
  if (abs(anemoWind - sailWind) < 10)
    return (anemoWind + sailWind) / 2;
  return sailWind; //too much difference => anemometer is less accurate (heavy rain, high heel angle)
}

