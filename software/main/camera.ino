
void camera() { //call in the main loop
  if (!(gps.time.isValid() && gps.location.isValid() && gps.hdop.isValid() && gps.hdop.value()/100 < GPS_HDOP_THRESHOLD))
    return;

  static char prev_hour = -1;
  static char prev_minute = -1;
  
  //float pseudo_tz = gps.location.lat() / (-53.0) * (-3.5) + 1.0; //lng = 0: gmt+1, lng = -53: gmt-2.5
  //float pseudo_hour = (float)gps.time.hour() + pseudo_tz;

  if (gps.time.hour() == 12 && prev_hour == 11)
    videoTask();

  if (gps.time.hour() == 14 && prev_hour == 13)
    videoTask();
    
  if (gps.time.hour() == 16 && prev_hour == 15)
    videoTask();

  //if (gps.time.hour() == 13 && prev_hour == 12)
    //satelliteTask(); 

  //testing
  //if (gps.time.hour() == 23 && prev_hour == 23 && gps.time.minute() == 27 && prev_minute == 26)
  //  videoTask();

  prev_hour = gps.time.hour();
  prev_minute = gps.time.minute();

  repeatTask();
}

void videoTask() {
  //startTask("$VID120;", false, 135000UL);
  startTask("$VID150;", false, 165000UL);
}

void satelliteTask() {
  startTask("$SAT0;", true, 240000UL);  
}

void startTask(char* cmd, bool sat_on, unsigned long task_timeout) {
  if (cam.state != CAM_STATE_OFF) { //shouldn't happen
    debugln(F(">>> CAMERA: STARTING, BUT STATE IS NOT OFF! DISCONNECTING"));
    cam_off();
    cam.state = CAM_STATE_OFF;
    return;
  }

  debug(F(">>> CAMERA: STARTING WITH CMD: "));
  debugln(cmd);
  
  cam.send_cmd = cmd;
  cam.task_timeout = task_timeout;
  cam_on();

  if (sat_on) {
    debugln(F(">>> CAMERA: TURNING ON SATCOM"));
    digitalWrite(SAT_ON, HIGH);
  }
  
  cam.state = CAM_STATE_BOOTING;
  cam.start_boot = millis();
}

void repeatTask() {
  if (cam.state == CAM_STATE_OFF)  
    return;
        
  if (cam.state == CAM_STATE_READY && millis() > cam.repeat_cmd + CAM_REPEAT_INTERVAL) {
    debugln(F(">>> CAMERA: READY, SENDING COMMAND"));
    cam.repeat_cmd = millis();
    camSerial.print(cam.send_cmd);
  }
  else if (cam.state == CAM_STATE_DONE && millis() > cam.start_cmd + CAM_SHUTDOWN_INTERVAL) {
    debugln(F(">>> CAMERA: DONE, SHUTDOWN INTERVAL ELAPSED, DISCONNECTING"));
    cam_off();
    cam.state = CAM_STATE_OFF;
  }

  //timeout

  if (cam.state == CAM_STATE_BOOTING && millis() > cam.start_boot + BOOT_TIMEOUT) {
    debugln(F(">>> CAMERA: BOOT TIMEOUT, DISCONNECTING"));
    cam_off();
    cam.state = CAM_STATE_OFF;
  }
  else if (cam.state == CAM_STATE_READY && millis() > cam.start_cmd + READY_TIMEOUT) {
    debugln(F(">>> CAMERA: READY TIMEOUT (NOT BUSY), DISCONNECTING"));
    cam_off();
    cam.state = CAM_STATE_OFF;
  }
  else if (cam.state == CAM_STATE_BUSY && millis() > cam.start_cmd + cam.task_timeout) {
    debugln(F(">>> CAMERA: TASK TIMEOUT, DISCONNECTING"));
    cam_off();
    cam.state = CAM_STATE_OFF;
  }

  if (millis() > cam.start_boot + TOTAL_TIMEOUT) { //just to be sure
    debugln(F(">>> CAMERA: TOTAL TIMEOUT, DISCONNECTING"));
    cam_off();
    cam.state = CAM_STATE_OFF;
  }
}

void camExecmd() {
  int i = 0;
  while(i < cam.recv_cmd.length() && !isDigit(cam.recv_cmd[i]) && cam.recv_cmd[i] != '-' && cam.recv_cmd[i] != '.') i++;
  String cmd1 = cam.recv_cmd.substring(0, i);
  String cmd2_str = cam.recv_cmd.substring(i);
  int cmd2 = cmd2_str.toInt();
  
  if (cam.state == CAM_STATE_BOOTING && cmd1 == "RDY" && cmd2 == 0) {
    debugln(F(">>> CAMERA: READY STATE RECEIVED"));
    cam.state = CAM_STATE_READY;
    cam.start_cmd = millis();
    cam.repeat_cmd = 0;
  }
  else if (cam.state == CAM_STATE_READY && cmd1 == "BSY" && cmd2 == 0) {
    debugln(F(">>> CAMERA: BUSY STATE RECEIVED"));
    cam.state = CAM_STATE_BUSY;
    cam.start_cmd = millis();
  }
  else if (cam.state == CAM_STATE_BUSY && cmd1 == "DONE" && (cmd2 == 0 || cmd2 == 1)) {
    debugln(F(">>> CAMERA: DONE STATE RECEIVED"));
    cam.state = CAM_STATE_DONE;

    if (cam.send_cmd.substring(1, 4) == "SAT") {
      if (cmd2 == 0) //ok
        ee.cam_transmit_sec = (millis() - cam.start_cmd)/1000;
      else //error
        ee.cam_transmit_sec = 0;

      if (!(warning()))
        eeSave(SAVE_REASON_CAM_SEC);
    }
    
    cam.start_cmd = millis();
  }  
}

void readCamCmd() {
  while (camSerial && camSerial.available()) {
    char ch = (char)camSerial.read();
    
    debugWrite(ch); //debug raw output: linux PC serial output
    
    if (ch == '\n' || ch == '\r') continue;
    if (ch == ';') { //end of command, process
      camExecmd();
      cam.recv_cmd = "";
      cam.recv_cmdlen = 0;
    }
    else if (ch == '$') { //start a new command (optional)
      cam.recv_cmd = "";
      cam.recv_cmdlen = 0;
    }
    else {
      cam.recv_cmd += ch;
      cam.recv_cmdlen++;
    }
    if (cam.recv_cmdlen > CAM_CMDMAX-2) { //too long command
      cam.recv_cmd = "";
      cam.recv_cmdlen = 0;      
    }
  }
}

void cam_on() {
  debugln(F(">>> CAMERA: TURNING ON"));
  digitalWrite(CAM_ON, HIGH);
  camSerial.begin(BAUD_CAM);
}

void cam_off() {
  debugln(F(">>> CAMERA: TURNING OFF CAM AND SATCOM"));

//  return;//!!!
  
  digitalWrite(CAM_ON, LOW);
  digitalWrite(SAT_ON, LOW);
  camSerial.end();
}

