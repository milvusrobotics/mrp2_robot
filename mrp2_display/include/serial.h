#ifdef __cplusplus
extern "C" {
#endif

	extern int portOpen (char *device, int baud);
	extern void portFlush (void);
	extern void portClose (void);
	extern int portDataAvail (void);
	extern int portGetchar (void);
	extern void portPutchar (int data);

#ifdef __cplusplus
}
#endif