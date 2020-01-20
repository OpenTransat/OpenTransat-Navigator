/*
#define ACT_TIMES_THRESHOLD1 900000UL //15min
#define ACT_TIMES_THRESHOLD2 900000UL //15min
#define ACT_TIMES_THRESHOLD3 1800000UL //30min
#define ACT_TIMES_THRESHOLD4 3600000UL //60min
*/

#define ACT_TIMES_THRESHOLD1 3600000UL //60min
#define ACT_TIMES_THRESHOLD2 3600000UL //60min

bool actMustWait() {
  return (
       actMustWaitThreshold(10, ACT_TIMES_THRESHOLD1, ACT_TIMES_THRESHOLD2)
//    || actMustWaitThreshold(10, ACT_TIMES_THRESHOLD3, ACT_TIMES_THRESHOLD4)
  );
}

bool actMustWaitThreshold(char n, unsigned long threshold_within, unsigned long threshold_wait) { //n movements within threshold_within => must wait threshold_wait
  if (act.count < n) return false;
  for (char i = -1; i > -n; i--) {
    if (act.times[act.times_i] - act_time_rel(i) > threshold_within) return false;
  }
  return (millis() - act.times[act.times_i] < threshold_wait);
}

long act_time_rel(char diff) {
  diff += act.times_i;
  diff = (diff % ACT_TIMES_COUNT + ACT_TIMES_COUNT) % ACT_TIMES_COUNT; //go to interval 0..ACT_TIMES_COUNT-1
  return act.times[diff];
}

void switchAct() {
    debugln(F("Switching to ActuatorTX..."));
    debugln();
    cancel_iridium = true;
    setMUX(MUX_ACT);
    Serial3.begin(BAUD_ACT);
    digitalWrite(ACT_ON, HIGH);
    delay(500); //charge capacitor on ActuatorTX module
}

void transmitAct(char c, int num) {
  Serial3.print("$");
  Serial3.print(c);
  Serial3.print(num);
  Serial3.print(";");
}

void actAddTime(unsigned long t, bool save) {
  if (!act.addTime)
    return;
  
  act.times_i++;
  if (act.times_i > ACT_TIMES_COUNT - 1)
   act.times_i = 0;

  act.times[act.times_i] = t;

  act.count++;

  //save actuator cycles in EEPROM
  ee.act_cycles++;
  
  if (save)
    eeSave(SAVE_REASON_ACTADDTIME);
}

void retryActuator() {
  transmitAct('A', act.cmd);
  sail_state = SAIL_STATE_WAIT_A; //changed in smartDelay when update is received
  act.confirmed_PE = false;
  sail_start_ms = millis();  
}

void startActuator(int cmd, bool addTime) {
  act.cmd = cmd;
  act.addTime = addTime;
  retryActuator();
  sail_retries = 0;
}

void printActTimes() {
  debug(F("Actuator Times: "));
  for (int i = 0; i < ACT_TIMES_COUNT; i++) {
    if (i == act.times_i)
      debug("*");
    debug(act.times[i]/1000);
    if (i == act.times_i)
      debug("*");
    debug(" ");
  }
}

