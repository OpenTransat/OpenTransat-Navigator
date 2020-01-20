/*
void test_cam() {
  debugln(F("Booting in 5 sec..."));
  delay(5000);
  debugln(F("Booting now..."));
  camSerial.begin(BAUD_CAM);
  digitalWrite(CAM_ON, HIGH);
  smartDelay(30000); //do not send characters when booting
  while(1) {
    camSerial.write('b');    
    digitalWrite(LED, HIGH); 
    digitalWrite(HEARTBEAT, HIGH); 
    smartDelay(250);         
    digitalWrite(LED, LOW);  
    digitalWrite(HEARTBEAT, LOW); 
    smartDelay(250);                 
  }
  //camSerial.end();
  digitalWrite(CAM_ON, LOW);
}

void test_FRAM() {
  if (fram.begin()) {
    debugln(F("Found SPI FRAM"));
  } else {
    debugln(F("No SPI FRAM found ... check your connections\r\n"));
    while (1);
  }
  
  // Read the first byte
  uint8_t test = fram.read8(0x0);
  debug("Restarted "); debug(test); debugln(" times");

  // Test write ++
  fram.writeEnable(true);
  fram.write8(0x0, test+1);
  fram.writeEnable(false);

  fram.writeEnable(true);
  fram.write(0x1, (uint8_t *)"FTW!", 5);
  fram.writeEnable(false);

  // dump the entire memory!
  uint8_t value;
  for (uint16_t a = 0; a < 8192; a++) {
    value = fram.read8(a);
    if ((a % 32) == 0) {
      debug("\n 0x"); debug(a, HEX); debug(": ");
    }
    debug("0x"); 
    if (value < 0x1) 
      debug('0');
    debug(value, HEX); debug(" ");
  }
}


void test_MPU() {
    // various functions for reading
  // mpu.read_mag();
  // mpu.read_acc();
  // mpu.read_gyro();

  mpu.read_all();

  debug(mpu.gyro_data[0]);   debug('\t');
  debug(mpu.gyro_data[1]);   debug('\t');
  debug(mpu.gyro_data[2]);   debug('\t');
  debug(mpu.accel_data[0]);  debug('\t');
  debug(mpu.accel_data[1]);  debug('\t');
  debug(mpu.accel_data[2]);  debug('\t');
  debug(mpu.mag_data[0]);    debug('\t');
  debug(mpu.mag_data[1]);    debug('\t');
  debug(mpu.mag_data[2]);    debug('\t');
  debug(mpu.temperature);    debug('\t');
}

void test_BME() {
  debug("Temperature = ");
  debug(bme.readTemperature());
  debugln(" *C");

  debug("Pressure = ");

  debug(bme.readPressure() / 100.0F);
  debugln(" hPa");

  debug("Approx. Altitude = ");
  debug(bme.readAltitude(SEALEVELPRESSURE_HPA));
  debugln(" m");

  debug("Humidity = ");
  debug(bme.readHumidity());
  debugln(" %");

  debugln();
}

void test_DataFlash() {
  digitalWrite(DF_CS, HIGH);
  uint8_t status;
  DataFlash::ID id;

  const char* dummyMessage = "Hello world! Lorem ipsum";

  dataflash.setup(DF_CS);
  dataflash.begin();

  status = dataflash.status();
  dataflash.readID(id);
  
  debug("DataFlash ID: ");
  debug2(id.manufacturer, HEX);
  debug(" ");
  debug2(id.device[0], HEX);
  debug(" ");
  debug2(id.device[1], HEX);
  debug(" ");
  debugln2(id.extendedInfoLength, HEX);
  
  // For a brand new AT45DB161D dataflash
  //  status = BIN(00101100)
  //  id.manufacturer       = 0x1F;
  //  id.device[0]          = 0x26;
  //  id.device[1]          = 0x00;
  //  id.extendedInfoLength = 0x00;

  // Write "Hello world" to buffer 1.
  dataflash.bufferWrite(1, 0);
  for(int i=0; dummyMessage[i] != '\0'; i++)
  {
    SPI.transfer(dummyMessage[i]);
  }
  SPI.transfer('\0');
  
  // Transfer buffer 1 to page 7.
  dataflash.bufferToPage(1, 7);

  // Read page 7.
  pageread(7);
  digitalWrite(DF_CS, LOW);
}

void pageread(int i) {
  debug("PAGE");
  debugln(i);
  dataflash.pageToBuffer(i, 1);    
  dataflash.bufferRead(1, 0);
  uint8_t data;
  int j = 0;
  do
  {
    data = SPI.transfer(0xff);
    if(data != '\0')
      debugWrite(data);
    j++;
  } while((data != '\0') && (j < DF_45DB161_PAGESIZE));
  debugln();
  dataflash.disable(); //important to put CS LOW after reading
}


void test_GPS() {
  static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;
  debugln(F("Sats HDOP Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum"));
  debugln(F("          (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  ---- to London  ----  RX    RX        Fail"));
  debugln(F("---------------------------------------------------------------------------------------------------------------------------------------"));

  printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  printInt(gps.hdop.value(), gps.hdop.isValid(), 5);
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  printInt(gps.location.age(), gps.location.isValid(), 5);
  printDateTime(gps.date, gps.time);
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.value()) : "*** ", 6);

  unsigned long distanceKmToLondon =
    (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      LONDON_LAT, 
      LONDON_LON) / 1000;
  printInt(distanceKmToLondon, gps.location.isValid(), 9);

  double courseToLondon =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      LONDON_LAT, 
      LONDON_LON);

  printFloat(courseToLondon, gps.location.isValid(), 7, 2);

  const char *cardinalToLondon = TinyGPSPlus::cardinal(courseToLondon);

  printStr(gps.location.isValid() ? cardinalToLondon : "*** ", 6);

  printInt(gps.charsProcessed(), true, 6);
  printInt(gps.sentencesWithFix(), true, 10);
  printInt(gps.failedChecksum(), true, 9);
  debugln();
  
  smartDelay(1000);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    debugln(F("No GPS data received: check wiring"));
}

void test_mosfets() {
  digitalWrite(CAM_ON, HIGH);
  digitalWrite(SAT_ON, HIGH);
  digitalWrite(AUX_ON, HIGH);
  digitalWrite(PUMP1_ON, HIGH);
  digitalWrite(PUMP2_ON, HIGH);
  digitalWrite(AIS_OFF, LOW);
  digitalWrite(RC_OFF, LOW);
  delay(2000);
  digitalWrite(CAM_ON, LOW);
  digitalWrite(SAT_ON, LOW);
  digitalWrite(AUX_ON, LOW);
  digitalWrite(PUMP1_ON, LOW);
  digitalWrite(PUMP2_ON, LOW);
  digitalWrite(AIS_OFF, HIGH);
  digitalWrite(RC_OFF, HIGH);
  delay(2000);
}

void test_nav_INA() {
  debugln("nav");
  ina219_nav.begin();
  vamps va = read_INA(ina219_nav);
  ina219_nav.enterPowerSave();
  print_vamps(va);  
}

void test_all_INA() {
  test_nav_INA();

  digitalWrite(PWRSENSOR_ON, HIGH);
  smartDelay(2000); //charge capacitor

  debugln("sol1");
  ina219_sol1.begin();
  vamps va = read_INA(ina219_sol1);
  print_vamps(va);

  debugln("sol2");
  ina219_sol2.begin();
  va = read_INA(ina219_sol2);
  print_vamps(va);
  
  debugln("sol3");
  ina219_sol3.begin();
  va = read_INA(ina219_sol3);
  print_vamps(va);

  debugln("bat");
  ina219_bat.begin();
  ina219_bat.setCalibration_32V_2A(); //0.1 ohm resistor
  va = read_INA(ina219_bat);
  print_vamps(va);

  digitalWrite(PWRSENSOR_ON, LOW);
  debugln("-------------------------");
}

void test_flood() {
  debug("flood0:"); debugln(analogRead(FLOOD0));
  debug("flood1:"); debugln(analogRead(FLOOD1));
}

void test_temp() {
  debug("Water Sensor = "); debugln(readTemp());
  debug("Battery Sensor = "); debugln(readTemp());
  ds.reset_search();
}

void test_iridium() {
  if (isbd.begin() == ISBD_SUCCESS) { //wake up
    isbd.sendSBDText("test");
  }
  isbd.sleep(); //back to sleep
  smartDelay(5000);
}

void test_act() { //add to smartDelay for testing, also insert digitalWrite(ACT_ON, HIGH); in setup
  while (mux == MUX_ACT && Serial.available()) {
    Serial3.write(Serial.read());
  }
}

#define RUDDER_TEST_INTERVAL 5000

void pulse(char pulse_length) {
  Serial1.write(pulse_length);
  smartDelay(20);
}

void pulse_loop(char pulse_length, int ms) {
  unsigned long start = millis();
  while((unsigned int)(millis() - start) < ms) {
    pulse(pulse_length);
  }
}

void test_rudder() {
  digitalWrite(LED, HIGH);
  pulse_loop(100, RUDDER_TEST_INTERVAL);
  digitalWrite(LED, LOW);
  pulse_loop(100-35, RUDDER_TEST_INTERVAL);
  digitalWrite(LED, HIGH);
  pulse_loop(100+35, RUDDER_TEST_INTERVAL);
  digitalWrite(LED, LOW);
  pulse_loop(100, RUDDER_TEST_INTERVAL);
}
*/

/////////////////////////////// GPS print functions ////////////////////////////////////

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      debug('*');
    debug(' ');
  }
  else
  {
    debug2(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      debug(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  debug(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    debug(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    debug(sz);
  }
  
  if (!t.isValid())
  {
    debug(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    debug(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    debug(i<slen ? str[i] : ' ');
  smartDelay(0);
}

///////////////////////////////////////////

