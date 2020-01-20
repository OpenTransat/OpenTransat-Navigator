//pin definitions
#define LED 13 //PB7
#define MPU_CS 53 //PB0
#define MPU_INT 70 //PE6
#define FRAM_CS 73 //PJ4
#define DF_CS 76 //PJ7
#define BME_CS 74 //PJ5
#define GPS_OFF 72 //PJ3
#define ESP_OFF 75 //PJ6
#define MUX_A 77 //PG4
#define MUX_B 78 //PG3
#define RC_PWM0 2 //PE4
#define RC_PWM1 3 //PE5
#define RC_PWM2 79 //PE7
#define RC_PWM3 62 //PK0
#define HEARTBEAT 32 //PC5
#define WARNING 33 //PC4
#define RC_OFF 34 //PC3
#define AIS_OFF 35 //PC2
#define IR_SLEEP 36 //PC1
#define CAM_ON 37 //PC0
#define AUX_ON 8 //PH5
#define ENC_CLK 9 //PH6
#define ENC_CS 10 //PB4
#define ENC_OUT 11 //PB5
#define TMP 12 //PB6
#define M3_REV 49 //PL0
#define M3_ON 48 //PL1
#define M2_ON 47 //PL2
#define M1_ON 46 //PL3
#define M2_PWM 45 //PL4
#define M1_PWM 44 //PL5
#define SAT_ON 42 //PL7
#define PWRSENSOR_ON 31 //PC6
#define FLOOD0 56 //PF2
#define FLOOD1 57 //PF3
#define PUMP1_ON 63 //PK1
#define PUMP2_ON 64 //PK2
#define PK3 65 //PK3
#define PIN_UNUSED0 41 //PG0, anemo TX
#define PIN_UNUSED1 40 //PG1, rudder TX

//pin modifications
#define ACT_ON M2_ON
#define LED_EXT M2_PWM
#define HALL_ON M1_ON
#define CAM_RX ENC_CS //has pin change interrupts
#define CAM_TX ENC_CLK
#define ANEMO_RX PK3 //has pin change interrupts
#define ANEMO_TX PIN_UNUSED0
#define RUDDER_RX ENC_OUT //has pin change interrupts
#define RUDDER_TX PIN_UNUSED1

