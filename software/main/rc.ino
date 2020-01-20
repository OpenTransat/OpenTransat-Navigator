//using timer3

void RC_init() {
  //init timer3
  TCCR3A = 0; //normal counter mode
  TCCR3B = (TCCR3B & 0b11111000) | 0b010; //CS prescaler 1:8
  TCNT3 = 0;
  TIMSK3 = 0;
  
  //input pullups
  pinMode(RC_PWM0, INPUT_PULLUP);
  pinMode(RC_PWM1, INPUT_PULLUP);
  
  //interrupts
  attachInterrupt(digitalPinToInterrupt(RC_PWM0), calcRC0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_PWM1), calcRC1, CHANGE);
}

void calcRC(byte chan, int chan_pin) { //blocking max RC_TIMEOUT_MICROS
  TCNT3 = 0;
  if(digitalRead(chan_pin) == HIGH) {
    RC_last_interrupt[chan] = micros();    
    while(digitalRead(chan_pin) == HIGH) {
      RC_pulse[chan] = TCNT3 / 2;
      if (RC_pulse[chan] > RC_TIMEOUT_MICROS) //timeout
        return;
    }
  }
}

void calcRC0() {
  calcRC(0, RC_PWM0);
}

void calcRC1() {
  calcRC(1, RC_PWM1);
}

void print_RC() {
  debug("RC: ");
  debug(RC_pulse[0]);
  debug(" ");
  debug(RC_pulse[1]);
}

