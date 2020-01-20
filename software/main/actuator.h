#define ACT_TIMES_COUNT 11

struct {
  int cmd; //command to execute
  int count = 0;
  long times[ACT_TIMES_COUNT]; //millis of the last action
  bool mustWait = false;
  char times_i = 0;
//  int last_A; //use ee.last_A instead
  int last_P;
  int last_E;
  bool confirmed_PE = true;
  bool addTime = true;
} act;

