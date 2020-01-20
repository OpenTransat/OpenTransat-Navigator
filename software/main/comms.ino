
void IR_init() {
  isbd.setPowerProfile(0); //high current
  isbd.adjustSendReceiveTimeout(IR_TIMEOUT);
}

void IR_sendData() { //send various data including data obtained from the sailwing
  msglen = 0;

  //the following commands update msg and msglen

  addlong(millis()/1000);
  addint(ee.resets);
  addchar(round(va.volts * 10)); //volts = 0..20
  addint(round(avg_mamps)); //mamps = 0..10000
  long hdop = gps.hdop.value()/100;
  addchar(min(hdop, 255)); //typ. 0..50
  addfloat(gps.location.lat());
  addfloat(gps.location.lng());
  addchar(compass.ID); //1..4
  addint(round(compass.yaw)); //0..360
  addint(round(compass.pitch)); //-360..360
  addint(round(compass.roll)); //-360..360
  addchar(round(compass.temp)); //-20..100
  addint(sail.angle); //0..360
  addint(sail.relWind); //0..360
  addchar(ee.last_A); //100 +- something
  addint(act.last_P); //0..1024
  addint(act.last_E); //0..1024
  addint(act.count); //?
  addlong(ee.act_cycles);
  addlong(act.times[act.times_i]/1000);
  addchar(act.mustWait); //bool => char
  addchar(act.confirmed_PE);
  addchar(rudder.feedback);
  addlong(rudder.last_update/1000);
  addlong(rudder.count);
  addlong(data2.last_saildata_update/1000);
  addint(round(data2.pressure_in)); //~1000hPa
  addchar(round(data2.temp_in)); //-20..100
  addchar(round(data2.humidity_in)); //0..100
  addchar(round(data2.temp_water)); //-20..100
  addchar(round(data2.temp_bat)); //-20..100
  addchar(round(((float)data2.flood0_count / data2.flood0_total) * 100)); //0-100
  addchar(round(((float)data2.flood1_count / data2.flood1_total) * 100)); //0-100
  addchar(round(data2.hull_va1.volts * 10)); //volts = 0..20
  addint(round(data2.hull_va1.mamps)); //mamps = 0..10000
  addchar(round(data2.hull_va2.volts * 10)); //volts = 0..20
  addint(round(data2.hull_va2.mamps)); //mamps = 0..10000
  addchar(round(data2.hull_va3.volts * 10)); //volts = 0..20
  addint(round(data2.hull_va3.mamps)); //mamps = 0..10000
  addchar(round(data2.backup_va.volts * 10)); //volts = 0..20
  addint(round(data2.backup_va.mamps)); //mamps = 0..10000
  addchar(round(data2.sail_vaL.volts * 10)); //volts = 0..20
  addint(round(data2.sail_vaL.mamps)); //mamps = 0..10000
  addchar(round(data2.sail_vaR.volts * 10)); //volts = 0..20
  addint(round(data2.sail_vaR.mamps)); //mamps = 0..10000
  addchar(round(data2.sail_vaB.volts * 10)); //volts = 0..20
  addint(round(data2.sail_vaB.mamps)); //mamps = 0..10000  
  addchar(round(data2.sail_temp)); //-20..100
  addchar(data2.capsize); //?
  addchar(ee.override_flag ? ee.override_customID : ee.nextWaypoint); //0..255
  addchar(nav.tackmode); //0..255
  addint(round(nav.trueHeading)); //0..360
  addint(round(nav.goHeading)); //0..360
  addchar((unsigned char)(ir_transmit_sec/2)); //0..500sec/2
  addchar((unsigned char)(ee.cam_transmit_sec/2)); //0..500sec/2
  
  //check if it fits the maximum of 338 chars (imposed by Iridium) AND IR_MSG_LENGTH (msg array size), currently used: 99 chars
  #ifdef SERIAL_DEBUG
    if (msglen > IR_MSG_LENGTH) {
      debugln(F("************ FATAL ERROR: IRIDIUM MESSAGE LENGTH EXCEEDED ****************"));
    }

    debug(F("IRIDIUM DATA TO BE SENT, LENGTH: "));
    debugln(msglen);
    print_msg(msg, msglen);
  #endif

  //mux to Iridium, then mux back to GPS after Iridium is finished
  switchIR();
  
  #ifndef IGNORE_IRIDIUM
    IR_send(msg, msglen);
  #endif
  
  if (mux != MUX_ACT) //if transmission cancelled by the actuator, do not switch to GPS
    switchGPS();
}

void IR_send(char* msg, int msglen) {
  unsigned long ir_transmit_millis = millis();
  int err;

  err = isbd.begin();
  if (err != ISBD_SUCCESS) {
    debug(F("IRIDIUM BEGIN FAILED: error "));
    debugln(err);
    if (err == ISBD_NO_MODEM_DETECTED)
      debugln(F("IRIDIUM: No modem detected, check wiring."));
    return;
  }

  debug(F("IRIDIUM: Sending binary message...\r\n"));

  //err = isbd.sendSBDBinary(msg, msglen);

  rxBufferSize = sizeof(rxBuffer);
  err = isbd.sendReceiveSBDBinary(msg, msglen, rxBuffer, rxBufferSize);
  
  processReceivedMessage(rxBuffer, rxBufferSize);

  if (err != ISBD_SUCCESS) {
    debug(F("IRIDIUM: sendSBDBinary failed: error "));
    debugln(err);
    if (err == ISBD_SENDRECEIVE_TIMEOUT)
      debugln(F("Try again with a better view of the sky."));
  }
  else {
    debug(F("IRIDIUM: MESSAGE SENT, going to sleep..."));
  }

  err = isbd.sleep();

  if (err != ISBD_SUCCESS) {
    debug(F(" SLEEP FAILED, error: "));
    debugln(err);
  }
  else
    debugln(F(" SLEEP SUCCESS."));

  ir_transmit_sec = (millis() - ir_transmit_millis)/1000;
}

void ISBDConsoleCallback(IridiumSBD *device, char c) { //do not remove this function! otherwise Iridium will fail
  debugWrite(c);
}

void ISBDDiagsCallback(IridiumSBD *device, char c) { //do not remove this function! otherwise Iridium will fail
  debugWrite(c);
}

void addchar(char x) {
  memcpy(msg + msglen, &x, sizeof(x));
  msglen += sizeof(x);
}

void addint(int x) {
  memcpy(msg + msglen, &x, sizeof(x));
  msglen += sizeof(x);
}

void addlong(long x) {
  memcpy(msg + msglen, &x, sizeof(x));
  msglen += sizeof(x);
}

void addfloat(float x) {
  memcpy(msg + msglen, &x, sizeof(x));
  msglen += sizeof(x);
}

void print_msg(char* msg, int msglen) {
  for (int i = 0; i < msglen; i++) {
    debug((uint8_t)msg[i]);
    debug(" ");
  }
  debugln();
}

void IR_testCSQ() {
  int err;

  err = isbd.begin();
  if (err != ISBD_SUCCESS) {
    debug(F("IRIDIUM BEGIN FAILED: error "));
    debugln(err);
    if (err == ISBD_NO_MODEM_DETECTED)
      debugln(F("IRIDIUM: No modem detected, check wiring."));
    return;
  }
     
  int signalQuality = -1;

  err = isbd.getSignalQuality(signalQuality);
  if (err != 0) {
    debug(F("SignalQuality failed: error "));
    debugln(err);
  }
  else {
    debug(F("Signal quality is "));
    debugln(signalQuality);
  }
  err = isbd.sleep();
}

void processReceivedMessage(char *rxBuffer, int rxBufferSize) {
  unsigned char buffer_pos = 0;
  
  if (rxBufferSize < 1)
    return;
  
  //rxBuffer = "c71af101640000000000000000000000000000000000000000000000000148"; //test
  //decodeHex(rxBuffer, &rxBufferSize); //received message is already in binary format

  #ifdef SERIAL_DEBUG
    debug(F("IRIDIUM RECEIVED MESSAGE: "));
    debugln(rxBufferSize);
    print_msg(rxBuffer, rxBufferSize);
  #endif 

  if (rxBufferSize != ACCEPTED_RX_BUFFER_SIZE) {
    debug(F(">> WRONG RECEIVED MESSAGE SIZE: "));
    debugln(rxBufferSize);
    return;
  }

  if (msg_checksum(rxBuffer, rxBufferSize - 1) != rxBuffer[rxBufferSize - 1]) { //last byte of the received message is checksum
    debugln(F(">> WRONG CHECKSUM"));
    return;
  }
 
  //secret code  
  
  memcpy(comm_uint32AsBytes.bytes, rxBuffer + buffer_pos, 4);
  if (comm_uint32AsBytes.lval != OVERRIDE_SECRET) {
    debugln(F(">> WRONG SECRET CODE. NOT OVERRIDEN."));
    return;
  }

  buffer_pos += 4;
  
  //customID
  
  if (rxBuffer[buffer_pos] == ee.override_customID) {
    debugln(F(">> SAME CUSTOM ID. NOT OVERRIDEN."));
    return;
  }

  if (OVERRIDE_ONLY_AFTER_FINISH && ee.nextWaypoint <= FINISH_WAYPOINT) {
    debugln(F(">> BEFORE THE FINISH POINT. NOT OVERRIDEN."));
    return;
  }
  
  ee.override_customID = rxBuffer[buffer_pos];
  debug(F("override_customID: "));
  debugln(ee.override_customID);

  buffer_pos += 1;

  //prev_loc

  memcpy(comm_floatAsBytes.bytes, rxBuffer + buffer_pos, 4);
  ee.override_prev_loc.lat = comm_floatAsBytes.fval;
  debug(F("override_prev_loc.lat: "));
  debugln(ee.override_prev_loc.lat);
  
  buffer_pos += 4;
  
  memcpy(comm_floatAsBytes.bytes, rxBuffer + buffer_pos, 4);
  ee.override_prev_loc.lng = comm_floatAsBytes.fval;
  debug(F("override_prev_loc.lng: "));
  debugln(ee.override_prev_loc.lng);
  
  buffer_pos += 4;
  
  //next_loc

  memcpy(comm_floatAsBytes.bytes, rxBuffer + buffer_pos, 4);
  ee.override_next_loc.lat = comm_floatAsBytes.fval;
  debug(F("override_next_loc.lat: "));
  debugln(ee.override_next_loc.lat);
  
  buffer_pos += 4;
  
  memcpy(comm_floatAsBytes.bytes, rxBuffer + buffer_pos, 4);
  ee.override_next_loc.lng = comm_floatAsBytes.fval;
  debug(F("override_next_loc.lng: "));
  debugln(ee.override_next_loc.lng);
  
  buffer_pos += 4;
  
  //ghostDistance

  memcpy(comm_floatAsBytes.bytes, rxBuffer + buffer_pos, 4);
  ee.override_ghostDistance = comm_floatAsBytes.fval;
  debug(F("override_ghostDistance: "));
  debugln(ee.override_ghostDistance);
  
  buffer_pos += 4;
  
  //maxDev
  
  memcpy(comm_floatAsBytes.bytes, rxBuffer + buffer_pos, 4);
  ee.override_maxDev = comm_floatAsBytes.fval;
  debug(F("override_maxDev: "));
  debugln(ee.override_maxDev);
  
  buffer_pos += 4;

  //checkMustWait

  ee.override_customVar = rxBuffer[buffer_pos];
  debug(F("override checkMustWait: "));
  debugln(ee.override_customVar % 2 ? 1 : 0);
  debug(F("override Iridium interval: "));
  debugln(ee.override_customVar / 2);
    
  buffer_pos += 1;
    
  ee.override_flag = true;
  
  if (!(warning()))
    eeSave(SAVE_REASON_OVERRIDE);

  debugln(F(">> WAYPOINT OVERRIDEN."));
}

char msg_checksum(char* s, int len) {
  unsigned char ret = 0;
  for (int i = 0; i < len; i++) {
    ret ^= *s;
    s++;
  }
  return ret;
}

void decodeHex(char *rxBuffer, int *rxBufferSize) {
  *rxBufferSize = *rxBufferSize / 2;
  for (int i = 0; i < *rxBufferSize; i++)  {
    rxBuffer[i] = (hex2dec(rxBuffer[2*i]) << 4) + hex2dec(rxBuffer[2*i+1]);
  }
}

char hex2dec(char c) { //character 0-F => decimal 0-16
  if (c >= '0' && c <= '9')
    return c - '0';
  if (c >= 'a' && c <= 'f')
    return c - 'a' + 10;
  if (c >= 'A' && c <= 'F')
    return c - 'A' + 10;
  return 0;
}

