IridiumSBD isbd(Serial3, IR_SLEEP);

#define IR_TIMEOUT 200 //in seconds

//message to transmit
#define IR_MSG_LENGTH 150
#define IR_RXBUFFER_LENGTH 70 //hex message = 2 * binary message

char msg[IR_MSG_LENGTH];
unsigned int msglen = 0;

uint8_t rxBuffer[IR_RXBUFFER_LENGTH];
unsigned int rxBufferSize = 0;

#define ACCEPTED_RX_BUFFER_SIZE 31
#define OVERRIDE_SECRET 3251807UL
#define OVERRIDE_ONLY_AFTER_FINISH true //add at least one waypoint after the finish waypoint to override

unsigned int ir_transmit_sec = 0; //how long a message was transmitted (for reporting)


union {
    float fval;
    uint8_t bytes[4];
} comm_floatAsBytes;

union {
    uint16_t ival;
    uint8_t bytes[2];
} comm_uint16AsBytes;

union {
    uint32_t lval;
    uint8_t bytes[4];
} comm_uint32AsBytes;

