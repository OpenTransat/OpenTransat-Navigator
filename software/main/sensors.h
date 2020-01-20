
#define COMPASS_TIMEOUT 30000UL
#define MAX_COMPASS_BAD_RESETS 10
#define HALL_TIMEOUT 30000UL
#define HALL_RESET_DELAY 10000UL

#define INA219_NAV_ADDR 0x46 //Navigator: A0=SDA, A1=+5V
#define INA219_SOL1_ADDR 0x40 //PowerSensor: A0=GND2, A1=GND2
#define INA219_SOL2_ADDR 0x41 //PowerSensor: A0=GND2, A1=3V3-SOL
#define INA219_SOL3_ADDR 0x44 //PowerSensor: A0=3V3-SOL, A1=GND2
#define INA219_BAT_ADDR 0x43 //PowerSensor: A0=SCL-BAT, A1=GND3
#define MAX_VALID_VOLTS 25

//#define MPU_SPI_CLOCK 8000000
#define MPU_SPI_CLOCK 100000

Adafruit_INA219 ina219_nav(INA219_NAV_ADDR); //power sensor on Navigator
Adafruit_INA219 ina219_sol1(INA219_SOL1_ADDR); //PowerSensor: solar panel 1
Adafruit_INA219 ina219_sol2(INA219_SOL2_ADDR); //PowerSensor: solar panel 2
Adafruit_INA219 ina219_sol3(INA219_SOL3_ADDR); //PowerSensor: solar panel 3
Adafruit_INA219 ina219_bat(INA219_BAT_ADDR); //PowerSensor: backup battery
MPU9250 mpu(MPU_SPI_CLOCK, MPU_CS);

//use different objects for multiple serial inputs
TinyGPSPlus gps;
TinyGPSPlus actObj;
TinyGPSCustom sail_A(actObj, "A", 1);
TinyGPSCustom sail_P(actObj, "P", 1);
TinyGPSCustom sail_E(actObj, "E", 1);
TinyGPSCustom sail_bat(actObj, "B", 1);
TinyGPSCustom sail_panelR(actObj, "R", 1);
TinyGPSCustom sail_panelL(actObj, "L", 1);
TinyGPSCustom sail_temp(actObj, "T", 1);
TinyGPSPlus compassObj;
TinyGPSCustom compass_ID(compassObj, "C", 1);
TinyGPSCustom compass_yaw(compassObj, "C", 2);
TinyGPSCustom compass_pitch(compassObj, "C", 3);
TinyGPSCustom compass_roll(compassObj, "C", 4);
TinyGPSCustom compass_temp(compassObj, "C", 5);
TinyGPSPlus hallObj;
TinyGPSCustom hall(hallObj, "H", 1);

TinyGPSPlus anemoObj;
TinyGPSCustom anemoDirection(anemoObj, "WIMWV", 1);
TinyGPSCustom anemoSpeed(anemoObj, "WIMWV", 3);

OneWire ds(TMP);
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme(BME_CS); // hardware SPI

SoftwareSerial anemoSerial(ANEMO_RX, ANEMO_TX); //RX, TX

typedef struct {
  float volts;
  float mamps;
} vamps;

vamps va;
float avg_mamps = 0;

struct {
  long last_saildata_update = 0;
  float pressure_in;
  float temp_in;
  float humidity_in;
  float temp_water;
  float temp_bat;
  unsigned int flood0;
  unsigned int flood1;
  unsigned int flood0_count = 0;
  unsigned int flood1_count = 0;
  unsigned int flood0_total = 0;
  unsigned int flood1_total = 0;
  vamps hull_va1;
  vamps hull_va2;
  vamps hull_va3;
  vamps backup_va;
  vamps sail_vaL;
  vamps sail_vaR;
  vamps sail_vaB;
  float sail_temp;
  unsigned char capsize;
} data2;

struct {
  char ID = 0; //0=none, compass returns 1-4
  float yaw;
  float pitch;
  float roll;
  float temp;
  unsigned char bad_resets = 0;
} compass;

struct {
  unsigned long hall_update;
  int angle;
  int relWind;
  bool windUpdated = false;
} sail;

struct {
  float direction;
  float speed;
} anemo;

