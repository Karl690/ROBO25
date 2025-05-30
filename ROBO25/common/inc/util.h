#ifndef util_HEADER // prevent double dipping
#define util_HEADER
// inline macros... work with any data types... careful using where one of the params is an equation, as
// the equation will execute twice and occupy the code space 3 times....IF not compiling with optimization

//#define MIN(a,b) (((a)<(b))?(a):(b))
//#define MAX(a,b) (((a)>(b))?(a):(b))
//#define ABS(a) (((a)>(0))?(a):(-(a)))
extern uint64_t tickCount;
typedef struct {
	byte		size8;
	uint16_t	size16;
	uint32_t	size32;
	float		sizeFlt;
//	enum		enummed;
	uint64_t	size64;
} oneOfEachSizeStruct;

// compiler will GREATLY reduce this for each call, just & and == for right sized value
#define IS_ERASED(a) 		((sizeof(a)==1) ? ((*(uint8_t  *)&a & 0xff) == 0xff) : 								\
							((sizeof(a)==2) ? ((*(uint16_t *)&a & 0xffff) == 0xffff) : 							\
							((sizeof(a)==4) ? ((*(uint32_t *)&a & 0xffffffff) == 0xffffffff) :					\
											  ((*(uint64_t *)&a & 0xffffffffffffffff) == 0xffffffffffffffff))))

extern void memZero(byte *addr, uint32_t qty);

extern void deinitClkAndResetAPB1(uint32_t periph);
extern void deinitClkAndResetAPB2(uint32_t periph);
extern void initClkAndResetAPB1(uint32_t periph);
extern void initClkAndResetAPB2(uint32_t periph);
extern void initClkAndResetAHB1(uint32_t periph);
extern void initClkAndResetAHB2(uint32_t periph);
extern void initClkAndResetAHB3(uint32_t periph);

extern void interruptSetupAndDisable(uint8_t channel, uint8_t priority);
extern void interruptSetupAndEnable(uint8_t channel, uint8_t priority);

extern int32_t imin(int32_t a, int32_t b);
extern int32_t imax(int32_t a, int32_t b);
extern int32_t iabs(int32_t a);
extern int32_t iFitWithinRange(int32_t value, int32_t low, int32_t high);
extern uint32_t umin(uint32_t a, uint32_t b);
extern uint32_t umax(uint32_t a, uint32_t b);
extern uint32_t uFitWithinRange(uint32_t value, uint32_t low, uint32_t high);

extern int64_t imin64(int64_t a, int64_t b);
extern int64_t imax64(int64_t a, int64_t b);
extern int64_t iabs64(int64_t a);
extern int64_t iFitWithinRange64(int64_t value, int64_t low, int64_t high);
extern uint64_t umin64(uint64_t a, uint64_t b);
extern uint64_t umax64(uint64_t a, uint64_t b);
extern uint64_t uFitWithinRange64(uint64_t value, uint64_t low, uint64_t high);

extern float fFitWithinRange(float value, float low, float high);
extern double dFitWithinRange(double value, double low, double high);

extern void delayUsec(uint32_t us);
extern void delayMsec(uint32_t ms);
extern void delaySec(uint32_t sec);

extern uint32_t interruptsOff(void);
extern void interruptsOn(uint32_t);

extern float fpu_sqrtf(float op);

extern char *getPidMethodStr(char *s, byte method);

#endif // #ifndef util_HEADER // prevent double dipping - MUST BE LAST LINE OF FILE
