#ifdef __cplusplus
extern "C" {
#endif

#define BUMPER_FL   97
#define BUMPER_FR   98
#define BUMPER_RL   99
#define BUMPER_RR   100
#define BATTERY     101
#define READ_ANALOG 102
#define GPIO_SET    103
#define BEEP        104
#define ESTOP_BTN   105
#define ESTOP       106

	extern int portOpen (char *device, int baud);
	extern void portFlush (void);
	extern void portClose (void);
	extern int portDataAvail (void);
	extern int portGetchar (void);
	extern void portPutchar (int data);
	extern void sendCmd(char cmd, char data1, char data2);
	extern void lockSerial(bool lock);
	extern bool getSerial(void);

#ifdef __cplusplus
}
#endif