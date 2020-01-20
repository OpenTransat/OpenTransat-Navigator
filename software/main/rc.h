#define RC_TIMEOUT_MICROS 2500
#define RC_INACTIVE_MICROS 1000000UL //1 second
#define RC_ACTIVE(chan) (RC_last_interrupt[chan] > 0 && micros() > RC_last_interrupt[chan] && RC_last_interrupt[chan] + RC_INACTIVE_MICROS > micros())

volatile unsigned long RC_last_interrupt[2] = {0, 0};
volatile unsigned long RC_pulse[2] = {0, 0};
bool RC_active = false;

