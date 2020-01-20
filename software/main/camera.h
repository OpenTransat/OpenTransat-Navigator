#define CAM_STATE_OFF 0
#define CAM_STATE_BOOTING 1
#define CAM_STATE_READY 2
#define CAM_STATE_BUSY 3
#define CAM_STATE_DONE 4

#define CAM_CMDMAX 10

struct {
  char state = CAM_STATE_OFF;
  String recv_cmd;
  int recv_cmdlen;
  String send_cmd;
  unsigned long start_boot;
  unsigned long start_cmd;
  unsigned long repeat_cmd;  
  unsigned long task_timeout;
  char sat_retcode = 0;
} cam;

#define CAM_REPEAT_INTERVAL 2000UL //2sec
#define CAM_SHUTDOWN_INTERVAL 15000UL //15sec
#define BOOT_TIMEOUT 45000UL //45sec
#define READY_TIMEOUT 10000UL //10sec
#define TOTAL_TIMEOUT 300000UL //5min

