//262141 / 16 = 16383 lines possible
//eg every 30min for 340 days

#define FRAM_LAST_ADDR 262143UL
#define FRAM_START_ADDR 8 //the first 8 bytes are for count

union {
    float fval;
    uint8_t bytes[4];
} floatAsBytes;

union {
    uint16_t ival;
    uint8_t bytes[2];
} uint16AsBytes;

union {
    uint32_t lval;
    uint8_t bytes[4];
} uint32AsBytes;

