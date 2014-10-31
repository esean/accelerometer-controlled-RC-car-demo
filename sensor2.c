//******************************************************************************
// PIC18F45K20 PICkit 3 Accelerometer & Gryometer sensor module interface
//
// Read values from accel and gyro sensors for RC car control.
//
// History:
// 01-23-2010 - copied from uChip example code
//            - added SPI interface to Parallax sensor modules:
//              gyrometer #27922, accelerometer #28526
//
// *******************************************************************

// DEFAULT: NOT defined
// Turns on some debugging features
//  eg, send232(IAgS)
//#define DEBUGBLD

// DEFAULT: defined
// if defined, changes to ADC var.pot read affect performance of car
// from very slow to very fast and erratic
#define ADC_VAR_PERFORMANCE

//==========================================
// MOTION-CONTROL defines:
//==========================================
// DEFAULT: NOT defined
//    Adjust the PWM1_MULTiplier based on PWM2_drive, such that
//    faster speed means less steering ability.
//    NOTE: so far this makes the car really not able to steer
//          when it gets moving, which is when you want to steer
//          to avoid obstacles
//#define SPEED_ADJUST_PWM1_MULT
// DEFAULT: NOT defined
//    Adjust the Ayz_motion_off_min threshold based on speed.
//#define SPEED_ADJUST_AYZ_MIN
// DEFAULT: defined
//    Adjust both the Ayz_motion_off_min _AND_ the PWM1_MULTiplier
//    based on PWM2 drive
#define SPEED_ADJUST_AYZ_PWM1_MIN
//==========================================

// DEFAULT: defined
#define ALLOW_GYRO_STEER

// DEFAULT: defined
// enable/disable USART
#define ALLOW_USART

// DEFAULT: NOT defined
// if defined, replaces existing LED_mode>2 modes for sending sensor data w/
// sending out 232 the movement cfg constants that can be adjusted via ADC,
// one at a time as they are selected (LED_mode advanced)
//#define DBG_CONST_ADC_ADJUST

// DEFAULT: NOT defined
// CLOCK output
//#define CLKOUT // DEBUG: used to verify clock speed on RA6

// DEFAULT: NOT defined
//#define NIGHT	// turns off buzzer for night-time working

// DEFAULT: NOT defined (so RD5 can display uP processing time)
// for timing/performance measurements:
//    1. if defined, make pulse on RD5 at each top of main to measure main() cycle time
//    2. if undefined, it will be turned on when updating brain and taking samples (for measuring processing time)
//
// Also, RD6 is now used to toggle every 24ms sensor sampling cycle for scope triggers
//  
//#define TOP_OF_MAIN_DEBUG_PULSE

// DEFAULT: NOT defined
// don't do calib on each power up (TODO: need these stored in EEPROM!)
//#define BYPASS_CALIB

// DEFAULT: NOT defined
//#define ACCEL_THRESH		// TODO: using INT1 requires turning off meas.mode 

// DEFAULT: NOT defined
//#define ALLOW_EEPROM


/** CHECKS ******************************/
#ifdef DBG_CONST_ADC_ADJUST
 #if !defined(ALLOW_USART)
  #error "DBG_CONST_ADC_ADJUST requires ALLOW_USART defined!"
 #endif
#endif


/** C O N F I G U R A T I O N   B I T S ******************************/

#ifdef CLKOUT
#pragma config FOSC = INTIO7, FCMEN = OFF, IESO = OFF                       // CONFIG1H
#else
#pragma config FOSC = INTIO67, FCMEN = OFF, IESO = OFF                       // CONFIG1H
#endif
#pragma config PWRT = OFF, BOREN = OFF, BORV = 30                        // CONFIG2L
#pragma config WDTEN = OFF, WDTPS = 32768                                     // CONFIG2H
#pragma config MCLRE = ON, LPT1OSC = OFF, PBADEN = OFF, CCP2MX = PORTC       // CONFIG3H
#pragma config STVREN = ON, LVP = OFF, XINST = OFF                          // CONFIG4L
#pragma config CP0 = OFF, CP1 = OFF, CP2 = OFF, CP3 = OFF                   // CONFIG5L
#pragma config CPB = OFF, CPD = OFF                                         // CONFIG5H
#pragma config WRT0 = OFF, WRT1 = OFF, WRT2 = OFF, WRT3 = OFF               // CONFIG6L
#pragma config WRTB = OFF, WRTC = OFF, WRTD = OFF                           // CONFIG6H
#pragma config EBTR0 = OFF, EBTR1 = OFF, EBTR2 = OFF, EBTR3 = OFF           // CONFIG7L
#pragma config EBTRB = OFF                                                  // CONFIG7H

/** I N C L U D E S **************************************************/
#include "p18f45k20.h"
#include "sensor2.h"
#include "mma7455l.h"
#include "delays.h"
/* Functions:
 *		 Delay1TCY()		// delay one clock ('nop')
 *       Delay10TCY()  		// same as Delay10TCYx(1)
 *       Delay10TCYx(uchar)	// 10 clock * 'x' delay
 *       Delay100TCYx(uchar)
 *       Delay1KTCYx(uchar)
 *       Delay10KTCYx(uchar)
 */


// Fosc setting
//--------------
//#define CLK_1MHZ	// default clk (shows 250kHz on RA6)
//#define CLK_2MHZ	// shows 500kHz on RA6
#define CLK_4MHZ	// shows 1mhz on RA6

// RS232 max baud rates/clock rates:
// 		1MHz	19.2k
//		2MHz	57.6k
//		4MHz	115.2k
#define BAUD_115
//#define BAUD_57
//#define BAUD_19
//#define BAUD_9

//++++++++++++++++++++++++++++++++++
// TODO:
// WARNING: The TMR0_refill and BUZZER_TIME defines below are not correct
//          for 8MHZ clock
//++++++++++++++++++++++++++++++++++
//#define CLK_8MHZ	// NOTE: timer3(buzzer) times are funny since timer3 prescales max=8, so not much resolution
//++++++++++++++++++++++++++++++++++
#ifdef CLK_1MHZ
 #define GYRO_POWER_ON_TIME	15	// 1MHz, ie = 150ms (*10k clk cycles)
 #define GYRO_15MS_UPDATE	15	// 1MHz, ie = 15ms (*1k clk cycles)
 #define ACCEL_10MS_UPDATE	1	// 1MHz, ie = 10ms (*10k clk cycles)
 #define OSCCON_IRCF 0x30
 #define CLK		250000	// this is Fosc/4 
 #define TIMER0_PRESCALE	4
 #define TIMER3_PRESCALE	2
 #define T0CON_DEF 0x01  // prescale 1:4 - about 1 second maximum delay.
 #define T3CON_DEF 0x10	// prescale 1:2 - about 1 second maximum delay.
#elif defined(CLK_2MHZ)
 #define GYRO_POWER_ON_TIME	30	// 2MHz, ie = 150ms (*10k clk cycles)
 #define GYRO_15MS_UPDATE	30	// 2MHz, ie = 15ms (*1k clk cycles)
 #define ACCEL_10MS_UPDATE	2	// 2MHz, ie = 10ms (*10k clk cycles)
 #define OSCCON_IRCF 0x40
 #define CLK		500000	// this is Fosc/4
 #define TIMER0_PRESCALE	4
 #define TIMER3_PRESCALE	4
 #define T0CON_DEF 0x01  // prescale 1:4 - about 1 second maximum delay.
 #define T3CON_DEF 0x20	// prescale 1:4 - about 1 second maximum delay.
#elif defined(CLK_4MHZ)
 #define GYRO_POWER_ON_TIME	60	// 4MHz, ie = 150ms (*10k clk cycles)
 #define GYRO_15MS_UPDATE	60	// 4MHz, ie = 15ms (*1k clk cycles)
 #define ACCEL_10MS_UPDATE	4	// 4MHz, ie = 10ms (*10k clk cycles)
 #define OSCCON_IRCF 0x50
 #define CLK		1000000	// this is Fosc/4
 #define TIMER0_PRESCALE	4
 #define TIMER3_PRESCALE	8
 #define T0CON_DEF 0x01  // prescale 1:4 - about 1 second maximum delay.
 #define T3CON_DEF 0x30	// prescale 1:8 - about 1 second maximum delay.
#if 0	// 0
#elif defined(CLK_8MHZ)
 #define OSCCON_IRCF 0x60 
 #define CLK		2000000	// this is Fosc/4
 #define TIMER0_PRESCALE	4
 #define TIMER3_PRESCALE	8
 #define T0CON_DEF 0x01  // prescale 1:4 - about 1 second maximum delay.
 #define T3CON_DEF 0x30	// prescale 1:8 - about 1 second maximum delay.
#endif	// 0
#else
 #error "NO CLOCK defined!"
#endif

//--------------------------------------------------------------------
// TIMER0 - provides timing for taking sensor modules samples
// TIMER1 - 
// TIMER2 - reserverd for possible future SPI clk
// TIMER3 - for buzzer alerts, can also be used for general time requests
//--------------------------------------------------------------------
// TIMER0:
//
// config for querying from gyro and accel
// accel updates @ 125Hz (8ms) for 62.5BW and gyro has 88Hz update.
// set timer0=4ms so we can take an accel meas. every 2nd time (8ms)
// and gyro meas. every 3rd time (12ms)
#define TIMER0_WRAP		256*256	// same as 2^16
// gyro has 88Hz update, so we poll every 12ms
//#define GYRO_POLL_MS	12	// poll for new data every 12ms
// now we get a timer every 4ms and maintain a count
#define GYRO_POLL_MS	4	// poll for new data every 4ms
// just to be clear that above GYRO_POLL_MS is in millisec
#define MS_PER_SEC		1000
// ((CLK/4)*12ms) = 750, 2^16-750 = 64786
// ((CLK/4)*4ms) = 250 => 65286
#define TMR0_refill 	(TIMER0_WRAP-((CLK/TIMER0_PRESCALE)*GYRO_POLL_MS/MS_PER_SEC))	// 4ms
#define TMR0_1ms_refill 	(TIMER0_WRAP-((CLK/TIMER0_PRESCALE)*1/MS_PER_SEC))	// 1ms
#define TMR0_3ms_refill 	(TIMER0_WRAP-((CLK/TIMER0_PRESCALE)*3/MS_PER_SEC))	// 3ms
// 
// TIMER3:
// used to time buzzer events
#define TIMER3_WRAP			256*256	// same as 2^16
#define BUZZER_TIME_1_8		(TIMER3_WRAP-((CLK/TIMER3_PRESCALE)*125/MS_PER_SEC)) // 0.125sec
#define BUZZER_TIME(x)		(TIMER3_WRAP-((CLK/TIMER3_PRESCALE)*(x)/MS_PER_SEC))
#define BUZZER_TIME_MIN		BUZZER_TIME(125)	// this is minimum time required to make a beep
#ifdef NIGHT
 #define B_ON 0	// no buzzers at night!
#else
 #define B_ON 1
#endif
#define B_OFF 0

// SPI mode - DO NOT CHANGE !
#define MODE3	// default mode, needs to be mode3 for gyro & accel to work (and SMP=0)

// gyro self-test
#define GYRO_CALIB_OFFSET	(300.0)	// +mV offset in self-test
#define GYRO_ADC_RESOLUTION	(3.22)	// mV/deg/sec gyro resolution
#define GYRO_CALIB_DELTA	(GYRO_CALIB_OFFSET/GYRO_ADC_RESOLUTION)
#define GYRO_ST_LOW			(GYRO_CALIB_DELTA * 0.9)
#define GYRO_ST_HIGH		(GYRO_CALIB_DELTA * 1.1)

// accel self-test
// TODO: keeps failing at +/-10%, expanded to 15%, should this self-test be used for scaling values from accel?
// TODO: I keep getting 57. If this is really 64(+1g), then should I scale all reads by (64/57)?
#define STEPS_PER_G		64	// for +/-2g @ 8bit
#define ACCEL_ST_LOW	(STEPS_PER_G * 0.8)
#define ACCEL_ST_HIGH	(STEPS_PER_G * 1.176)

// calibration constants
#define NUM_CALIB_CYCLES	4	// how many times we will take a calib cycle, iterative 0g offset procedure
#define CAL_SZ				4	// how many avg samples to sum during 1 calib cycle to compute an offset
#define CAL_SZ_SHIFT		2	// for gyro (unsigned) divides

#define tABS(x) ((x)<0 ? (-1*(x)) : (x))
#define tMIN(x,y) ((x)<(y)?(x):(y))


//
// ---<> DERIVATIVE 
//
// !! NOTE: these are thresholds (abs()) and _WONT_ work for negative nums
//
// LT & GT (abs value is less than/greater than)
//-----------------------------------------------------------------
#define Dx_lt(x) (tABS(b.SYS_AVGDERIVx) < (x) ? TRUE : FALSE)
#define Dy_lt(x) (tABS(b.SYS_AVGDERIVy) < (x) ? TRUE : FALSE)
#define Dz_lt(x) (tABS(b.SYS_AVGDERIVz) < (x) ? TRUE : FALSE)
#define Dg_lt(x) (tABS(b.SYS_AVGDERIVg) < (x) ? TRUE : FALSE)
#define Dx_gt(x) (tABS(b.SYS_AVGDERIVx) > (x) ? TRUE : FALSE)
#define Dy_gt(x) (tABS(b.SYS_AVGDERIVy) > (x) ? TRUE : FALSE)
#define Dz_gt(x) (tABS(b.SYS_AVGDERIVz) > (x) ? TRUE : FALSE)
#define Dg_gt(x) (tABS(b.SYS_AVGDERIVg) > (x) ? TRUE : FALSE)
// LE & GE (abs value is less than or equal to/greater than or equal to)
//-----------------------------------------------------------------
#define Dx_le(x) (tABS(b.SYS_AVGDERIVx) <= (x) ? TRUE : FALSE)
#define Dy_le(x) (tABS(b.SYS_AVGDERIVy) <= (x) ? TRUE : FALSE)
#define Dz_le(x) (tABS(b.SYS_AVGDERIVz) <= (x) ? TRUE : FALSE)
#define Dg_le(x) (tABS(b.SYS_AVGDERIVg) <= (x) ? TRUE : FALSE)
#define Dx_ge(x) (tABS(b.SYS_AVGDERIVx) >= (x) ? TRUE : FALSE)
#define Dy_ge(x) (tABS(b.SYS_AVGDERIVy) >= (x) ? TRUE : FALSE)
#define Dz_ge(x) (tABS(b.SYS_AVGDERIVz) >= (x) ? TRUE : FALSE)
#define Dg_ge(x) (tABS(b.SYS_AVGDERIVg) >= (x) ? TRUE : FALSE)
// RANGE (abs value is greater than and less than)
//-----------------------------------------------------------------
#define Dx_gtlt(gt,lt) ((Dx_gt((gt)) && Dx_lt((lt))) ? TRUE : FALSE)
#define Dy_gtlt(gt,lt) ((Dy_gt((gt)) && Dy_lt((lt))) ? TRUE : FALSE)
#define Dz_gtlt(gt,lt) ((Dz_gt((gt)) && Dz_lt((lt))) ? TRUE : FALSE)
#define Dg_gtlt(gt,lt) ((Dg_gt((gt)) && Dg_lt((lt))) ? TRUE : FALSE)
// RANGE (abs value is greater than and less than or equal to)
//-----------------------------------------------------------------
#define Dx_gtle(gt,le) ((Dx_gt((gt)) && Dx_le((le))) ? TRUE : FALSE)
#define Dy_gtle(gt,le) ((Dy_gt((gt)) && Dy_le((le))) ? TRUE : FALSE)
#define Dz_gtle(gt,le) ((Dz_gt((gt)) && Dz_le((le))) ? TRUE : FALSE)
#define Dg_gtle(gt,le) ((Dg_gt((gt)) && Dg_le((le))) ? TRUE : FALSE)

#ifdef DBG_CONST_ADC_ADJUST
// RD5 & RD6 are not used in this debug mode so we can see ADC values on LEDs
#define DBG_IO_ON	NOP
#define DBG_IO_OFF	NOP
#define DBG_TIMER_ON	NOP
#define DBG_TIMER_OFF	NOP
#else
// RD5 is used for general scope info
#define DBG_IO_ON	DBG_IO = 1
#define DBG_IO_OFF	DBG_IO = 0
// RD6 toggles every 24ms to indicate sensor timing sync pulse for triggers
#define DBG_TIMER_ON	DBG_TIMER = 1
#define DBG_TIMER_OFF	DBG_TIMER = 0
#endif

/** V A R I A B L E S *************************************************/
#pragma udata   // declare statically allocated uinitialized variables

// FALSE means the IO line is low
BOOL sensor_sampling_24ms_cycle_state = FALSE;	// toggles on sensor sampling sequence for triggers

// latest values
signed int gyro_curr = 0;	// ADC value from board is 10bit unsigned, but we remove the CALgF to produce a signed value
signed char accelX_curr = 0;
signed char accelY_curr = 0;
signed char accelZ_curr = 0;
BOOL new_accel = 0;
BOOL new_gyro = 0;
BOOL delayed_accel2_sample = FALSE;

// timing measurements
char timer_meas_cnt = 1;	// wraps at 7, allows timing measurements

// Controls the current sensor data mode (for display/storage/calibration,etc)
//
// Basic modes:
//     0 = basic operation (controlling car, etc). while(1)
//     1 = calibration (countdown beeps, start, calc, store, end beep, if mode is
//         changed during this operation, mode goes to #2). At end of calib,
//         code returns to mode #0 for normal operation
// Special modes (RS232 xmit for postprocessing on PC):
//     2 = send raw sensor data
//     3 = send averaged sensor data
//     4 = send averaged & derivative sensor data
//
// NOTE:
// ===============
// If 'DBG_CONST_ADC_ADJUST' is defined, the above list is changed to the following
// where all modes are "basic operation" and calibration is only performed at boot.
// Then the movement cfg constants are Tx'd on 232 in all modes. In each mode, one
// cfg constant may be adjusted with the var.resistor ADC reading.
//
//	0	send values out 232
//  1   calib
//	2..13	alter variables described in movement_cfg
//
unsigned char LED_mode = 0;
#ifdef DBG_CONST_ADC_ADJUST
#define MAX_LED_mode 14
#else
#define MAX_LED_mode 5
#endif
unsigned char main_calib_stage = 0;

// self-test constant
signed int gyro_calib = 0;	// should be 91 ~= +300mV during self-test
signed int accel_calib = 0;	// shoudl be 64 steps for 1g applied to z-axis

// calibration constants
signed int CALx;
signed int CALxF;
signed int CALy;
signed int CALyF;
signed int CALz;
signed int CALzF;
unsigned long CALg;	// gyro sits at 1.65V at rest, div.by 3.22mV/step ~= 512 steps
unsigned long CALgF;
BOOL did_calib = FALSE;
BOOL completed_calib = FALSE;

#ifdef ACCEL_THRESH
// for thresholds
unsigned char levelX_detect = 0x5;
unsigned char levelY_detect = 0;
unsigned char levelZ_detect = 0;
#endif
// was in ACCEL_THRESH only, but now used for DBG_CONST_ADC_ADJUST also
unsigned int cntr = 0;	// TODO: for debugging why we get interrupt after ~32 passes...
						// don't see anything on scope
#ifdef DBG_CONST_ADC_ADJUST
unsigned char cntr2 = 0;
#endif

//  boolean to indicate data ready for acquire
BOOL RDY_accel = FALSE;
BOOL RDY_accel_THRESH = FALSE;
BOOL RDY_gyro = FALSE;

// buzzer
char Rstate = 0;
unsigned int Rtime;
BOOL BUZZ_STATE;
#define BUZZ_OFF	{ BUZZER = B_OFF; BUZZ_STATE = FALSE; }
#define BUZZ_ON		{ BUZZER = B_ON; BUZZ_STATE = TRUE; }

// if needed, comes from dial on dev.board
unsigned char ADC;
unsigned char ADC_last_update = 0;
BOOL ADC_update_first = 1;

// RS232 vars
unsigned int rs232_xmit_bytes = 0;
// A = 40bytes, G = 30bytes, for one cycle [A G A A&G] = 3*40+2*30 = > 180bytes/24ms
// sent a CR every 200ms to keep File::Tail happy (on PC),
// thus 100ms/24ms*180bytes = 750bytes
//#define RS232_BYTES_BREAK 750
#define RS232_BYTES_BREAK 250	// works better

// for debugging
unsigned char FATAL_ERROR_CODE = 0;	// for basic fatal backtrace

// SYSTEM variables used for global decision making
// 'b' for brain
typedef struct {
	signed char SYS_AVGx;
	signed char SYS_AVGy;
	signed char SYS_AVGz;
	signed int SYS_AVGg;
	// delta values are all signed
	signed char SYS_AVGDERIVx;
	signed char SYS_AVGDERIVy;
	signed char SYS_AVGDERIVz;
	signed int SYS_AVGDERIVg;
	// computed vars for use in rest of program
	signed int Axz_slope;
	signed int Ayz_slope;
	BOOL motion_any;
	BOOL motion_large;
	BOOL rot_left;
	BOOL rot_right;
	BOOL rot_hard_left;
	BOOL rot_hard_right;
	BOOL sensors_up;
	BOOL steer_reverse;
	BOOL just_updated;
	BOOL valid;
} brain;
brain b;

// These used to be in brain struct
//===============================================
#if 0
	signed int Axy_slope;
	signed int Dxz_slope;
	signed int Dyz_slope;
	signed int Dxy_slope;
	BOOL motion_none;
	BOOL motion_move;
#endif
// These used to be in brain struct
//===============================================


typedef struct {
	unsigned char SLOPE_MULTXZ;					// 2	// multiply by 5 when used (ADC returns 256 max giving 0-1280 range
	unsigned char SLOPE_MULTYZ;					// 3	//   "            "
	unsigned char SLOPE_MULTXZ_orig;					// 2	// multiply by 5 when used (ADC returns 256 max giving 0-1280 range
	unsigned char SLOPE_MULTYZ_orig;					// 3	//   "            "
	unsigned char AXZ_MOTION_OFF_MIN;			// 4
	unsigned char AYZ_MOTION_OFF_MIN;			// 5
	unsigned char GYRO_ROT_MIN_TURN_STEERING;	// 6
	unsigned char ROT_MULT;						// 7
	unsigned char GYRO_ROT_HARD_STEER;			// 8
	unsigned char ROT_MULT_HARD;				// 9
	unsigned char PWM1_MULT;					// 10	
	unsigned char PWM2_MULT;					// 11
	unsigned char PR2;							// 12
	unsigned char PR2_orig;							// 12
	unsigned char PWM1_DYN_THRES;				// 13	// as car goes faster (b.Axz_slope increases), we decrease steering mult,
									// this value is the threshold over which we start reducing PWM1_MULT
	unsigned char PWM2_DYN_THRES;				// 14
	unsigned char PWM1_DYN_MULT;				// 15
	unsigned char PWM2_DYN_MULT;				// 16
} movement_cfg;
movement_cfg m;


#ifdef DBG_CONST_ADC_ADJUST
 #define SLOPE_MULT_MULT	5
#else
 //#define SLOPE_MULT_MULT	1	// can't do this b/c SLOPE_MULT[XY]Z is unsigned char => won't fit
  #define SLOPE_MULT_MULT	5
#endif
#define cSLOPE_MULTXZ 350	// used for xz,yz,xy slope calc division
#define cSLOPE_MULTYZ 475	// used for xz,yz,xy slope calc division
#define cSLOPE_MULTXZ_DEF (cSLOPE_MULTXZ / SLOPE_MULT_MULT)
#define cSLOPE_MULTYZ_DEF (cSLOPE_MULTYZ / SLOPE_MULT_MULT)
#define cAXZ_MOTION_OFF_MIN 30
#define cAYZ_MOTION_OFF_MIN 20
#define GYRO_STEP_DEG_MULT	200	// for each degree, in integral, we get 200
#define cGYRO_ROT_MIN_TURN_STEERING	10
#define cROT_MULT	3		// twisting ctrlr makes it turn
#define cGYRO_ROT_HARD_STEER	18	// more than this twist and we turn!
#define cROT_MULT_HARD	10

// Motion control
// turn outputs off
#define LEFT_RIGHT_OFF	{SET_PWM1_DUTY(0);dbg_left=dbg_right=FALSE;LATAbits.LATA4 = 0; LATAbits.LATA5 = 0;}
#define FORWARD_BACKWARD_OFF {SET_PWM2_DUTY(0);dbg_forw=dbg_back=FALSE;LATBbits.LATB2 = 0; LATBbits.LATB3 = 0;}
#define ALL_MOTION_OFF {LEFT_RIGHT_OFF; FORWARD_BACKWARD_OFF;}

// set direction
BOOL dbg_left = FALSE;
BOOL dbg_right = FALSE;
BOOL dbg_forw = FALSE;
BOOL dbg_back = FALSE;
// (turn off other IO first, then turn on desired IO)
#define TURN_LEFT {dbg_right=FALSE;dbg_left=TRUE;LATAbits.LATA5 = 0; LATAbits.LATA4 = 1;}
#define TURN_RIGHT {dbg_right=TRUE;dbg_left=FALSE;LATAbits.LATA4 = 0; LATAbits.LATA5 = 1;}
#define GO_FORWARD {dbg_back=FALSE;dbg_forw=TRUE;LATBbits.LATB3 = 0; LATBbits.LATB2 = 1;}
#define GO_BACKWARD {dbg_back=TRUE;dbg_forw=FALSE;LATBbits.LATB2 = 0; LATBbits.LATB3 = 1;}

unsigned int PWM1_val;
unsigned int PWM2_val;

#define PWM1_MODE 0x0f	// CCP1CON:CCP1M<0:3>	// PWM1 is active high, but we want always low
#define PWM2_MODE 0x0f	// CCP2CON:CCP2M<0:3>
#define PWM1_drive_max 1022		// setting any higher sets line high, we want it normally low to keep transistor drives off, setting high connects switch
#define PWM2_drive_max 1023

#define cPWM1_MULT	10	// left-right
#define cPWM2_MULT	2	// foward-backward
#define PWM1_MULT_MIN	1
#define PWM2_MULT_MIN	1
#define cPWM1_DYN_MULT	0	// subtracted from PWM1_MULT to make steering less sensitive as car travels faster
#define cPWM2_DYN_MULT	0	// TODO: not used
#define PR2_DEF		0x78

signed long IAgS;
signed long IAgS_base;
unsigned char gyro_int_reset;
#define GYRO_INT_RESET_MAX	20	// 10*24ms = 240ms we update IAgS to the current value, kinda an avg
#define GYRO_CURR_MIN_INT_RESET 3	// if gyro_curr is less than this, we consider it "still"
#define GYRO_STEERING_ADJUST	((GYRO_STEP_DEG_MULT * cGYRO_ROT_MIN_TURN_STEERING)/2)
unsigned char gyro_left_reset_cntr = 0;
#define GYRO_LEFT_RESET_CNTR_MAX 3
unsigned char gyro_right_reset_cntr = 0;
#define GYRO_RIGHT_RESET_CNTR_MAX 3

//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
#ifdef SPEED_ADJUST_PWM1_MULT
#define vAYZ_MOTION_OFF_MIN	(m.AYZ_MOTION_OFF_MIN)
#define PWM1_MULT_VAL	((m.PWM1_DYN_MULT==0) ? \
							(unsigned int)m.PWM1_MULT : \
								((m.PWM1_DYN_MULT>=m.PWM1_MULT) ? \
								(unsigned int)PWM1_MULT_MIN : \
								(unsigned int)(m.PWM1_MULT - m.PWM1_DYN_MULT)))
#define PWM2_MULT_VAL	((m.PWM2_DYN_MULT==0) ? \
							(unsigned int)m.PWM2_MULT : \
								((m.PWM2_DYN_MULT>=m.PWM2_MULT) ? \
								(unsigned int)PWM2_MULT_MIN : \
								(unsigned int)(m.PWM2_MULT - m.PWM2_DYN_MULT)))
// setting PWM1 to 0 sets level to 0v
// setting PWM1 above PWM1_drive_max sets to 0v (TODO...)
#define SET_PWM1_DUTY(duty) {PWM1_val = PWM1_drive_max+1-(duty); \
								CCPR1L = PWM1_val >> 2; \
								CCP1CON = PWM1_MODE | ((PWM1_val & 0x03) << 4);}
// setting PWM2 to 1024 sets level to 0v
// when setting forward/back PWM duty, we also compute the L/R scaling factor based on speed
								// scale left/right drive inversly by forward/backward drive
#define SET_PWM2_DUTY(duty) {PWM2_val = (duty); \
								m.PWM1_DYN_MULT = ((duty) < m.PWM1_DYN_THRES) ? 0 : (unsigned char)((duty)/m.PWM1_DYN_THRES); \
								CCPR2L = (duty) >> 2; \
								CCP2CON = PWM2_MODE | (((duty) & 0x03) << 4);}
#else	// !SPEED_ADJUST_PWM1_MULT :::::::::::::::::::::::::::::::::::::::::::::::::::
#ifdef SPEED_ADJUST_AYZ_MIN //#########################################################
#define vAYZ_MOTION_OFF_MIN	(m.AYZ_MOTION_OFF_MIN+m.PWM1_DYN_MULT * 2)
#define PWM1_MULT_VAL	((unsigned int)m.PWM1_MULT)
#define PWM2_MULT_VAL	((unsigned int)m.PWM2_MULT)
// setting PWM1 to 0 sets level to 0v
// setting PWM1 above PWM1_drive_max sets to 0v (TODO...)
#define SET_PWM1_DUTY(duty) {PWM1_val = PWM1_drive_max+1-(duty); \
								CCPR1L = PWM1_val >> 2; \
								CCP1CON = PWM1_MODE | ((PWM1_val & 0x03) << 4);}
// setting PWM2 to 1024 sets level to 0v
// when setting forward/back PWM duty, we also compute the L/R scaling factor based on speed
								// scale left/right drive inversly by forward/backward drive
#define SET_PWM2_DUTY(duty) {PWM2_val = (duty); \
								m.PWM1_DYN_MULT = ((duty) < m.PWM1_DYN_THRES) ? 0 : (unsigned char)((duty)/m.PWM1_DYN_THRES); \
								CCPR2L = (duty) >> 2; \
								CCP2CON = PWM2_MODE | (((duty) & 0x03) << 4);}
#else	// SPEED_ADJUST_AYZ_MIN	//#######################################################
#ifdef SPEED_ADJUST_AYZ_PWM1_MIN //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
#define vAYZ_MOTION_OFF_MIN	(m.AYZ_MOTION_OFF_MIN+m.PWM1_DYN_MULT * 2)
#define PWM1_MULT_VAL	((m.PWM1_DYN_MULT==0) ? \
							(unsigned int)m.PWM1_MULT : \
								((m.PWM1_DYN_MULT>=m.PWM1_MULT) ? \
								(unsigned int)PWM1_MULT_MIN : \
								(unsigned int)(m.PWM1_MULT - m.PWM1_DYN_MULT)))
#define PWM2_MULT_VAL	((unsigned int)m.PWM2_MULT)
// setting PWM1 to 0 sets level to 0v
// setting PWM1 above PWM1_drive_max sets to 0v (TODO...)
#define SET_PWM1_DUTY(duty) {PWM1_val = PWM1_drive_max+1-(duty); \
								CCPR1L = PWM1_val >> 2; \
								CCP1CON = PWM1_MODE | ((PWM1_val & 0x03) << 4);}
// setting PWM2 to 1024 sets level to 0v
// when setting forward/back PWM duty, we also compute the L/R scaling factor based on speed
								// scale left/right drive inversly by forward/backward drive
#define SET_PWM2_DUTY(duty) {PWM2_val = (duty); \
								m.PWM1_DYN_MULT = ((duty) < m.PWM1_DYN_THRES) ? 0 : (unsigned char)((duty)/m.PWM1_DYN_THRES); \
								CCPR2L = (duty) >> 2; \
								CCP2CON = PWM2_MODE | (((duty) & 0x03) << 4);}
#else // SPEED_ADJUST_AYZ_PWM1_MIN //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
#define vAYZ_MOTION_OFF_MIN	(m.AYZ_MOTION_OFF_MIN)
#define PWM1_MULT_VAL	((unsigned int)m.PWM1_MULT)
#define PWM2_MULT_VAL	((unsigned int)m.PWM2_MULT)
// setting PWM1 to 0 sets level to 0v
// setting PWM1 above PWM1_drive_max sets to 0v (TODO...)
#define SET_PWM1_DUTY(duty) {PWM1_val = PWM1_drive_max+1-(duty); \
								CCPR1L = PWM1_val >> 2; \
								CCP1CON = PWM1_MODE | ((PWM1_val & 0x03) << 4);}
// setting PWM2 to 1024 sets level to 0v
// when setting forward/back PWM duty, we also compute the L/R scaling factor based on speed
#define SET_PWM2_DUTY(duty) {PWM2_val = (duty); \
								CCPR2L = (duty) >> 2; \
								CCP2CON = PWM2_MODE | (((duty) & 0x03) << 4);}
#endif  // SPEED_ADJUST_AYZ_PWM1_MIN //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
#endif	// SPEED_ADJUST_AYZ_MIN	//#######################################################
#endif	// SPEED_ADJUST_PWM1_MULT :::::::::::::::::::::::::::::::::::::::::::::::::::


#define cPWM1_DYN_THRES		(PWM1_drive_max/cPWM1_MULT)	// TODO: these values need to be revisited, just hacks right now
#define cPWM2_DYN_THRES		(PWM2_drive_max/cPWM2_MULT)	// TODO: ditto

// sensor delta motion - min to trigger 'motion' sensed
// accel
#define MTHRESH_none 	1	// if delta is equal to or less than this, considered "at rest"
#define MTHRESH_small 	3	// a gentle nudge
#define MTHRESH_move 	5	// a move, being picked up, etc
#define MTHRESH_large	40	// TODO:any thing this high is extreme movement
// gyro
#define GTHRESH_none 	4	// if delta is equal to or less than this, considered "at rest"
#define GTHRESH_small 	8	// a gentle nudge
#define GTHRESH_move 	12	// a move, being picked up, etc
#define GTHRESH_large	100	// any thing this high is extreme movement

#ifdef ALLOW_EEPROM
BOOL tried_eecalib = FALSE;
#endif

#ifdef ADC_VAR_PERFORMANCE
unsigned char pv_pre;
unsigned char performance_var;
#endif

// hold sensor data for post-processing
#pragma udata
#define SZ_AVG 25	// moving avg window size
//#define SZ_AVG_SHIFT 6	// use instead of div.
#define SZ_DER 10	// moving sample deriv.avg window size
#pragma udata gpr1
signed char Ax[SZ_AVG];
unsigned char pAx;
signed int AxS;
BOOL AxWrap;
BOOL DAxL_first;
signed char DAxL;
signed char DAx[SZ_DER];
unsigned char DpAx;
signed int DAxS;
BOOL DAxWrap;
#pragma udata gpr2
signed char Ay[SZ_AVG];
unsigned char pAy;
signed int AyS;
BOOL AyWrap;
BOOL DAyL_first;
signed char DAyL;
signed char DAy[SZ_DER];
unsigned char DpAy;
signed int DAyS;
BOOL DAyWrap;
#pragma udata gpr3
signed char Az[SZ_AVG];
unsigned char pAz;
signed int AzS;
BOOL AzWrap;
BOOL DAzL_first;
signed char DAzL;
signed char DAz[SZ_DER];
unsigned char DpAz;
signed int DAzS;
BOOL DAzWrap;
#pragma udata gpr4
signed int Ag[SZ_AVG];	// raw values are 10bit unsigned, but we offset 1.65V = 0 reading, so becomes signed
unsigned char pAg;
signed long AgS;			// raw sum is unsigned
BOOL AgWrap;
BOOL DAgL_first;
signed int DAgL;			// last sample is unsigned (raw)
signed int DAg[SZ_DER];		// deltas from last are signed
unsigned char DpAg;
signed long DAgS;			// delta sum is signed
BOOL DAgWrap;


#if 0
/** I N T E R R U P T (Compat-mode for PIC16x) ************************/
// the name says high priority, but we are not using priorities to
// maintain back-compat with PIC16X uP, just in case of future change...
#pragma code InterruptVectorHigh = 0x08
void InterruptVectorHigh (void)
{
  _asm
    goto InterruptServiceCompat //jump to interrupt routine
  _endasm
}
#pragma code

// -------------------- Iterrupt Service Routines --------------------------
#pragma interrupt InterruptServiceCompat  // "interrupt" pragma also for high priority
void InterruptServiceCompat(void)
{
	if (INTCONbits.TMR0IF == 1) {
		// Timer expired
		INTCONbits.TMR0IF = 0;          // Reset Timer flag
		// TODO
	}
}
#endif

/** D E C L A R A T I O N S *******************************************/
#pragma code    // declare executable instructions
void main(void)
{
    BOOL SwitchPressed = FALSE;
	char switch_count = 0;
    BOOL RevSwitchPressed = FALSE;
	char Revswitch_count = 0;
#ifdef DBG_CONST_ADC_ADJUST
	unsigned char pr2_change;
#endif
#ifdef DEBUGBLD
	signed long IAgS_tmp;
	BOOL pkt_2_send = FALSE;
#endif
	BOOL new_timer0_cycle = FALSE;

    TRISD = 0b00000000;     	// PORTD bits 7:0 are all outputs (0)
	LATD = 0xff;	// do a countdown on boot to show progress

	clear_brain();
	init_movement_cfg();
	LATD = 0xfe;

	// these are the calibration values in use by system
	//  (TODO: actually, if the accel hasn't been powered off, it will maintain
	//       the calib values from last time, but we set 0 here, this is fine because
	//       we just use these for programming and iterative calibration)
	// but we do clear these also at the start of calib cycle
	CALxF = CALyF = CALzF = 0;
	CALgF = 0;
#ifdef BYPASS_CALIB
	did_calib = TRUE;
	completed_calib = TRUE;
#else
	did_calib = FALSE;
	completed_calib = FALSE;
#endif

	// init Tx unit switches
	TRISAbits.TRISA4 = 0;		// LEFT output Tx switch
	TRISAbits.TRISA5 = 0;		// RIGHT output Tx switch
	TRISBbits.TRISB2 = 0;		// FORWARD output Tx switch
	TRISBbits.TRISB3 = 0;		// BACKWARD output Tx switch
	ALL_MOTION_OFF;

    // Init I/O for switch (B0)
	TRISAbits.TRISA0 = 1;		// TRISA0 input, connected to board potentiometer
	TRISAbits.TRISA3 = 0;		// TRISA3 output for jokie
	LATAbits.LATA3 = 0;
	TRISAbits.TRISA2 = 1;		// TRISA2 input, connected to old "speaker" switch, for reversing steering direction
	INTCON2bits.RBPU = 0;		// enable PORTB internal pullups
	WPUBbits.WPUB0 = 1;			// enable pull up on RB0
    TRISBbits.TRISB0 = 1;       // PORTB bit 0 (connected to switch) is input (1)
	WPUBbits.WPUB1 = 1;			// enable pull up on RB1
	TRISEbits.TRISE1 = 0;		// RE1 is output for busy/idle indication

	// buzzer init
	TRISAbits.TRISA1 = 0;		// output for driving buzzer
	BUZZ_OFF;
	Timer3_Init();	// for buzzer sounds

	// clear Sdata[] sensor circular buffer
	clear_buffers();
	LATD = 0xfc;

	// clear IRCF[0..2] bits and set clock mode
	// 0x03 - sysclock uses internal osc.block
	OSCCON = 0x03 | OSCCON_IRCF;
	// wait 1ms for it to become active
	Delay1KTCYx(10);	// if it's more that's ok, so select longest possible time,
						// at 8MHz clk is 0.125us so 1ms is 8000 cycles
	LATD = 0xf8;
	// bring up SPI 
	SPI_Init();

#ifdef ALLOW_USART
	// uart init
	usart_Init();
#endif

	// pwm init for driving RC car
	pwm_init();
	LATD = 0xf0;

	// init sensor modules
//#ifdef ACCEL_THRESH
	// NOTE:even though not using this accel INT mode now, this is an input and should
	// not be connected as default output
    TRISBbits.TRISB1 = 1;       // PORTB bit 1 (connected to accel INT1) is input (1)
//#endif
	accel_Init();
	LATD = 0xe0;

	gyro_Init();	// NOTE:takes 300ms from power up
	LATD = 0xc0;

    // Init ADC
    ADC_init();
	LATD = 0x80;

    // Init Timer0 for sampling module data, don't turn on until modules are init
	// and space for their data has been init
    Timer0_Init();
	LATD = 0;

	BUSYIO = 0;		// set when we do something
	while (1)
	{
#if defined(TOP_OF_MAIN_DEBUG_PULSE)
		debug_pulse();	// toggle RD5 so we can time the loop
#endif

#ifdef ACCEL_THRESH
		++cntr;
		if (RDY_accel_thresh == TRUE) {
			RDY_accel_thresh = FALSE;
			// the level threshold or pulse trigger was detected by accel chip
			// read $0A - find source of interrupt
			// write to $17 to clear interrupt
			write_accel(INTRST,0x03);
			write_accel(INTRST,0x00);
			fatal(2);
		}
#endif

		// do "main()" stuff
#ifdef DBG_CONST_ADC_ADJUST
		if (LED_mode == 0) {
			if (!completed_calib) {
				// go do calib, via main_rc_car as normal code does
				LED_mode = 2;	// could select any mode from 2-12
			}
			// sending data, don't move
			ALL_MOTION_OFF;
		} else if (LED_mode == 1) {
			main_calib();
		} else {
			// one special mode that just xmits current movement cfg values
			// all other modes act as a car


			////////////////////////////////////
			// NOTE: CAREFUL HERE 
			if (timer_meas_cnt == 4) {
				// the time to process main_rc_car() varies from 180us at rest to 
				// almost 1.5ms during movement. We used to run this after each sample
				// since we had just updated the brain, but we can't meet timing
				// requirements like that, now we just do once a 24ms cycle
				//
				// NOTE: code will exec here every loop cycle when timer0=4
				// but main_rc_car() will only process when we have a new sample
				// so it really only exec'd once at timer0=4
				main_rc_car();
			}
			////////////////////////////////////


		}
#else
		if (LED_mode == 0) {


			////////////////////////////////////
			// NOTE: CAREFUL HERE 
			if (timer_meas_cnt == 4) {
				// the time to process main_rc_car() varies from 180us at rest to 
				// almost 1.5ms during movement. We used to run this after each sample
				// since we had just updated the brain, but we can't meet timing
				// requirements like that, now we just do once a 24ms cycle
				//
				// NOTE: code will exec here every loop cycle when timer0=4
				// but main_rc_car() will only process when we have a new sample
				// so it really only exec'd once at timer0=4
				main_rc_car();
			}
#ifdef ADC_VAR_PERFORMANCE
			////////////////////////////////////
			// when timer0=6 (it was just 5 in prev.loop meaning we have
			// a free 4ms), update the ADC var_performance vars, but only
			// once in that time, ie. when we have a new sample
			else if ((timer_meas_cnt == 6) && b.valid && completed_calib) {
				if (new_timer0_cycle) {
					pv_pre = performance_var;
					performance_var = ADC_update(performance_var);
					if (pv_pre != performance_var) {
						pv_pre = performance_var / 7;	// scale down for use and to allow min range -18 to 18
						m.SLOPE_MULTXZ = m.SLOPE_MULTXZ_orig + pv_pre - 18;
						m.SLOPE_MULTYZ = m.SLOPE_MULTYZ_orig + pv_pre - 18;
						pv_pre = performance_var / 2;	// scale down for use and to allow range -63 to +63
						m.PR2 = m.PR2_orig + 63 - pv_pre;
						PR2 = m.PR2;
					}
				}
			}
			////////////////////////////////////
#endif

		} else {
			// if we are doing calib, or sending 232 data, turn off car ctrl
			ALL_MOTION_OFF;
		}
		if (LED_mode == 1) {
			main_calib();
		}
#endif

#ifdef DEBUGBLD
	// if timer0=5(actually 6, we just updated below), we have 4ms of free time, do we have anything else to do
	if (timer_meas_cnt == 6) {
		if (pkt_2_send) {
			pkt_2_send = FALSE;
			send232('#');
			IAgS_tmp = IAgS - IAgS_base;
			send232_char((IAgS_tmp & 0xff0000) >> 16);
		    send232_char((IAgS_tmp & 0x00ff00) >> 8);
			send232_char(IAgS_tmp & 0x0000ff);	
			send232(0x0a);
			send232(0x0c);
		}
	}
#endif

		//-------------------------------------------------------------------
		// SWITCH PRESSED ?
		//-------------------------------------------------------------------
		// check external "speaker" switch 
        if (Switch_SteerRev == 1)
        { // look for switch released
			if (RevSwitchPressed == TRUE) {
				b.steer_reverse = FALSE;
			}
			RevSwitchPressed = FALSE;
        }
        else if (RevSwitchPressed == FALSE) // && (Switch_SteerRev == 0) due to if-else
        { // switch was just pressed
			++Revswitch_count;
			if (Revswitch_count == DetectsInARow) {
				Revswitch_count = 0;
            	RevSwitchPressed = TRUE;
				
				// reverse steering direction, useful when driving back towards oneself
				b.steer_reverse = TRUE;
			}
		}
		// check on-board switch (SW1)
        if (Switch_Pin == 1)
        { // look for switch released
			SwitchPressed = FALSE;
        }
        else if (SwitchPressed == FALSE) // && (Switch_Pin == 0) due to if-else
        { // switch was just pressed
			++switch_count;
			if (switch_count == DetectsInARow) {
				switch_count = 0;
            	SwitchPressed = TRUE;

				if (LED_mode == 1) {
					// if we were in mode '1' (calib) we may need to turn some
					// things off before continuing
					BUZZ_OFF;
					reset_timer3();
					main_calib_stage = 0;
				}
				LATD = 0;
				++LED_mode;
				if (LED_mode >= MAX_LED_mode) LED_mode = 0; 	// reset
#ifdef DBG_CONST_ADC_ADJUST
				LATD = LED_mode;
				send232('@');
				send232(dec2ascii(LED_mode));
				send232(0x0a);
				send232(0x0c);
				if (LED_mode == 13) {
					LATD = 0xff;	// we flash it quickly
					buzzer_on(BUZZER_TIME(200));
					cntr = 0;
				}
#else
				// just for general 'alive' indication
				if (LED_mode == 0) {
					//LATD = 0x00;		// TODO: main_... handles itself, for now turn off for better power readings (turned off above)
				} else if (LED_mode == 1) {
					//LATD = 0x02;		// main_... handles itself
				} else if (LED_mode == 2) {
					LATD = 0x04;
				} else if (LED_mode == 3) {
					LATD = 0x08;
				} else if (LED_mode == 4) {
					LATD = 0x10;
				} else {
					fatal(5);
				}
#endif
			}
        }	// if (switchpressed == false)

		//-------------------------------------------------------------------
		// TIMER0
		//-------------------------------------------------------------------
		// check for timer0 fire
		if (INTCONbits.TMR0IF == 1) {
			// Timer expired
        	INTCONbits.TMR0IF = 0;          // Reset Timer flag

#if !defined(TOP_OF_MAIN_DEBUG_PULSE)
			DBG_IO_ON;
#endif
			// 2-10-10
			// The sensor sampling is now a little different - when both samples are
			// taken at timer_meas_cnt==6, first we take gyro, then accel so that gyro
			// can stay on it's 12ms timing, but now this pushes out the next accel
			// at timer_meas_cnt==2 by 1ms. So on '2', we set the timer for another 1ms,
			// come back in and take the measurement.
			if ((timer_meas_cnt==2)||(timer_meas_cnt==4)) {
				if (delayed_accel2_sample) {
					// we are back after 1ms to take the accel sample
					delayed_accel2_sample = FALSE;
					TMR0H = (unsigned char)((TMR0_3ms_refill & 0xFF00) >> 8);	// always write upper first
					TMR0L = (unsigned char)(TMR0_3ms_refill & 0x00FF);
				} else {
					// don't take accel sample now, wait 1ms
					delayed_accel2_sample = TRUE;
					TMR0H = (unsigned char)((TMR0_1ms_refill & 0xFF00) >> 8);	// always write upper first
					TMR0L = (unsigned char)(TMR0_1ms_refill & 0x00FF);
				}
			} else {

				TMR0H = (unsigned char)((TMR0_refill & 0xFF00) >> 8);	// always write upper first
				TMR0L = (unsigned char)(TMR0_refill & 0x00FF);
			}

			if (delayed_accel2_sample == FALSE) {
	
				// accel updates @ 125Hz (8ms) for 62.5BW and gyro has 88Hz update.
				// set timer0=4ms so we can take an accel meas. every 2nd time (8ms)
				// and gyro meas. every 3rd time (12ms)
				// To make this fast and easy, make lookup table:
				//
				//    timer_meas_cnt   time(ms)  accel    gyro
				//         1              4        0       0
				//         2              8        1       0
				//         3              12       0       1
				//         4              16       1       0
				//         5              20       0       0
				//         6              24       1       1
				//         		(... pattern repeats...)
				//
				// NOTE: don't use 'mod' as it brings in fixed point libraries, faster
				// to just use if else
				//
				// NOTE: _NEED_ to take gyro sample first, then accel so that gyro does
				//       not get delayed next read and possibly have incorrect data.
				//
				if ((timer_meas_cnt==3)||(timer_meas_cnt==6)) {
					// read Z rotation
					BUSYIO = 1;
					read_gyro();
					new_gyro = TRUE;
					BUSYIO = 0;
					store_sampleG();
					BUSYIO = 1;
#ifdef ALLOW_USART
#if !defined(DBG_CONST_ADC_ADJUST)
					if (LED_mode >= 2) {	// main code doesn't care about all these, speed it up
						if (LED_mode==2) send232_raw_gyro();
						if (LED_mode==3) send232_avg_gyro();
						if (LED_mode==4) send232_avgderiv_gyro();
					}
#endif
#endif
				} else {
					new_gyro = FALSE;
				}
				BUSYIO = 0;
				if ((timer_meas_cnt==2)||(timer_meas_cnt==4)||(timer_meas_cnt==6)) {
					// read X & Y & Z accel
					BUSYIO = 1;
					read_accel();
					new_accel = TRUE;
					BUSYIO = 0;
					store_sampleX();
					BUSYIO = 0;
					store_sampleY();
					BUSYIO = 0;
					store_sampleZ();
					BUSYIO = 0;
#ifdef ALLOW_USART
#if !defined(DBG_CONST_ADC_ADJUST)
					if (LED_mode >= 2) {	// main code doesn't care about all these, speed it up
						if (LED_mode==2) send232_raw_accel();
						if (LED_mode==3) send232_avg_accel();
						if (LED_mode==4) send232_avgderiv_accel();
					}
#endif
#endif
				} else {
					new_accel = FALSE;
				}
				BUSYIO = 0;
	
				// recompute our motion detection numbers if new samples arrived
				if (new_accel || new_gyro) {


					////////////////////////////////////
					// NOTE: CAREFUL HERE 
					////////////////////////////////////
					if (!completed_calib || (timer_meas_cnt == 2)) {
						// the time to process update_brain() varies from 770us at rest to 
						// almost 1ms during movement. We used to run this after each sample
						// but we can't meet timing
						// requirements like that, now we just do once a 24ms cycle
#if !defined(TOP_OF_MAIN_DEBUG_PULSE)
						DBG_IO_OFF;
#endif
						// NOTE: now update_brain is only done at timer0=2 and we
						// only get here if there is a new sample which will only
						// happen during timer0=2 one time (when processing accel sample)
						BUSYIO = 1;
						update_brain();
						b.just_updated = TRUE;
						BUSYIO = 0;
#if !defined(TOP_OF_MAIN_DEBUG_PULSE)
						DBG_IO_ON;
#endif
					}	// timer_meas_cnt
					////////////////////////////////////
					else {
						b.just_updated = FALSE;
					}
	

#ifdef DBG_CONST_ADC_ADJUST
					// 30 cycles =~ 0.25sec, i.e, 8ms cycle time
					++cntr2;
					if (cntr2 > 15) {	// 1/8sec update for ADC reads
						cntr2 = 0;
						if (LED_mode == 0) {
							send_movement_cfg_232();
						} else if (LED_mode == 1) {
							// calibration
							BREAKPT;
						} else if (LED_mode == 2) {
							m.SLOPE_MULTXZ = ADC_update(m.SLOPE_MULTXZ);
						} else if (LED_mode == 3) {
							m.SLOPE_MULTYZ = ADC_update(m.SLOPE_MULTYZ);
						} else if (LED_mode == 4) {
							m.AXZ_MOTION_OFF_MIN = ADC_update(m.AXZ_MOTION_OFF_MIN);
						} else if (LED_mode == 5) {
							m.AYZ_MOTION_OFF_MIN = ADC_update(m.AYZ_MOTION_OFF_MIN);
						} else if (LED_mode == 6) {
							m.GYRO_ROT_MIN_TURN_STEERING = ADC_update(m.GYRO_ROT_MIN_TURN_STEERING);
						} else if (LED_mode == 7) {
							m.ROT_MULT = ADC_update(m.ROT_MULT);
						} else if (LED_mode == 8) {
							m.GYRO_ROT_HARD_STEER = ADC_update(m.GYRO_ROT_HARD_STEER);
						} else if (LED_mode == 9) {
							m.ROT_MULT_HARD = ADC_update(m.ROT_MULT_HARD);
						} else if (LED_mode == 10) {
							m.PWM1_MULT = ADC_update(m.PWM1_MULT);
						} else if (LED_mode == 11) {
							m.PWM2_MULT = ADC_update(m.PWM2_MULT);
						} else if (LED_mode == 12) {
							pr2_change = m.PR2;
							m.PR2 = ADC_update(m.PR2);
							if (m.PR2 != pr2_change) {
								PR2 = m.PR2;
							}
						} else if (LED_mode == 13) {
							m.PWM1_DYN_THRES = ADC_update(m.PWM1_DYN_THRES);
						} else if (LED_mode == 14) {
							m.PWM2_DYN_THRES = ADC_update(m.PWM2_DYN_THRES);
						} else if (LED_mode == 15) {
							m.PWM1_DYN_MULT = ADC_update(m.PWM1_DYN_MULT);
						} else if (LED_mode == 16) {
							m.PWM2_DYN_MULT = ADC_update(m.PWM2_DYN_MULT);
						} else if (LED_mode == 17) {
							// this mode used to indicate we are about to wrap back to mode 0 = RS232 xmit mode
							++cntr;
							if (cntr > 2) {	// flash every ~1/4sec
								LATD ^= 0xff;
								cntr = 0;
							}
						} else {
							fatal(20);
						}
					}
#endif
				} else {
					b.just_updated = FALSE;
				}


				////////////////////////////////////
				// NOTE: CAREFUL HERE
				////////////////////////////////////
				if (delayed_accel2_sample == FALSE) {
					new_timer0_cycle = TRUE;
#ifdef DEBUGBLD
					pkt_2_send = TRUE;
#endif
					++timer_meas_cnt;
					if (timer_meas_cnt >= 7) {
						timer_meas_cnt = 1;
						// update 24ms sync pulse state
						if (sensor_sampling_24ms_cycle_state == FALSE) {
							DBG_TIMER_ON;
							sensor_sampling_24ms_cycle_state = TRUE;
						} else {
							DBG_TIMER_OFF;
							sensor_sampling_24ms_cycle_state = FALSE;
						}
					}
				}
				////////////////////////////////////


			}	// 1ms/3ms delayed 1st accel reading (timer_meas_cnt==2)
	
#if !defined(TOP_OF_MAIN_DEBUG_PULSE)
			DBG_IO_OFF;
#endif
		}	// if (timer1 fired)
		else {
			new_accel = FALSE;
			new_gyro = FALSE;
`			new_timer0_cycle = FALSE;
		}

#ifdef ACCEL_THRESH
		// check for interrupt from accel
		if (INTCON3bits.INT1IF == 1) {
			INTCON3bits.INT1IF = 0;
			RDY_accel_thresh = TRUE;
		}
#endif

		//-------------------------------------------------------------------
		// TIMER3
		//-------------------------------------------------------------------
		// check for buzzer timer fire, turn off buzzer
		if (PIR2bits.TMR3IF == 1) {
			// TODO: confusing but if we are starting calib (LED_mode=1, main_calib_stage=1),
			//       we will handle the buzzer in that piece of code, so ignore here
			//       in other parts of calib code (main_calib_stage!=1), we _do_ want
			//       the buzzer to be handled here
			if ((LED_mode != 1) && (main_calib_stage != 1)) {
				(void)handle_buzzer_event(0xFFFF,0x80);	// timer fired, do whatever...
			}
		}	// if (timer3 fired)

		if (BUSYIO == 1) {
			// this should never happen!
			fatal(3);
		}
    }	// while (1)
}


//========================================================================
// ADC
//========================================================================
void ADC_init(void)
{ // initialize the Analog-To-Digital converter.
    // First, we need to make sure the AN0 pin is enabled as an analog input
    // as the demo board potentiometer is connected to RA0/AN0
    // Don't forget that RB0/AN12 must be digital!
	ANSEL = 0;	//turn off all other analog inputs
	ANSELH = 0;
 	ANSELbits.ANS0 = 1;	// turn on RA0 analog

    // Sets bits VCFG1 and VCFG0 in ADCON1 so the ADC voltage reference is VSS to VDD

    ADCON1 = 0;

    // The ADC clock must as short as possible but still greater than the
    // minimum TAD time, datasheet parameter 130.  At the time this lesson was
    // written TAD minimum for the PIC18F45K20 is 1.4us.
    // At 1MHz clock, selecting ADCS = FOSC/2 = 500kHz.  One clock period
    // 1 / 500kHz = 2us, which greater than minimum required 1.4us.
    // So ADCON2 bits ADCS2-0 = 000
    //
    // The ACQT aquisition time should take into accound the internal aquisition
    // time TACQ of the ADC, datasheet paramter 130, and the settling time of
    // of the application circuit connected to the ADC pin.  Since the actual
    // settling time of the RC circuit with the demo board potentiometer is very
    // long but accuracy is not very important to this demo, we'll set ACQT2-0 to
    // 20TAD = 111
    //
    // ADFM = 0 so we can easily read the 8 Most Significant bits from the ADRESH
    // Special Function Register
	//
	// sbh - ACDS = 001 => Fosc/8 for 500khz @ 4MHz 
    //ADCON2 = 0b00111000;
	ADCON2 = 0b00111001;

    // Select channel 0 (AN0) to read the potentiometer voltage and turn on ADC
    ADCON0 = 0b00000001;
}
unsigned char ADC_convert(void)
{ // start an ADC conversion and return the 8 most-significant bits of the result
    ADCON0bits.GO_DONE = 1;             // start conversion
    while (ADCON0bits.GO_DONE == 1);    // wait for it to complete
    return ADRESH;                      // return high byte of result
}
//
// PASS:
//      in - value that will be returned if the ADC read has not changed since last fcn call,
//             if ADC has been updated since, return the new ADC value
//
// Usage:
//     adjVar = 5;
//     adjVar = ADC_upate(adjVar);
//
// This is useful when switching modes where we want to maintain a value
// as it was before, unless there has been a change on the ADC.
// For example, switch mode to 'changeX' and X is 15 and ADC value has
// been 100 for a while. This function will return always return 15 unless
// the user moves the var.resistor and changes the ADC value. Then that new
// value would be returned and adjVar would be updated. Moving to the next mode
// though, again the original value will be returned as this function will be
// called every 4ms.
//
unsigned char ADC_update(unsigned char in) {
	unsigned char au = ADC_convert();
	if (ADC_update_first == 1) {
		ADC_update_first = 0;
		// first time thru, any reads get stuffed into ADC_update
		ADC_last_update = au;
		// first time, just store, we will look for deltas in the future
#ifdef DBG_CONST_ADC_ADJUST
		LATD = in;
#endif
		return(in);															// RETURN
	} else {
		// compare to last
		if (tABS(au-ADC_last_update) > 10) {
			ADC_last_update = au;
#ifdef DBG_CONST_ADC_ADJUST
			// it was updated, send out RS232
			send232('U');
			send232_char(LED_mode);
			send232_char(ADC_last_update);
			send232(0x0a);
			send232(0x0c);
			//send232_break();
			LATD = ADC_last_update;
#endif
			return(ADC_last_update);										// RETURN
		} else {
#ifdef DBG_CONST_ADC_ADJUST
			LATD = in;
#endif
			return(in);														// RETURN
		}
	}
}


//========================================================================
// debug and error checking
//========================================================================
// toggle I/O line for misc. scope triggers (timing, etc)
void debug_pulse(void) {
	// calling set_debug_LED takes a lot of time, just set bits ourself
	DBG_IO_ON;
	DBG_IO_OFF;
}
// fatal error codes:
//		1 = test
//		2 = ifdef ACCEL_THRESH, dies in main (TODO: unimplemented)
//		3 = if BUSYIO==1 at end of main while(1) loop
//      4 = UART Tx register was always full (TXSTAbits.TRMT==0 for too long)
//      5 = unexpected value for LED_mode
//      6 = unexpected value for main_calib_stage
//      7 = tried to write timer3 value but timer3 was already active
//      ###8 = cnvt_s16_2_s10: 16bit signed int was too large to fit into 10bit signed (NEGATIVE)
//      ###9 = cnvt_s10_2_s16: 10bit signed int contained some bits out of range for a 10bit value
//      10 = gyro calibration outside 10% of correct value
//      ###11 = cnvt_s16_2_s10: 16bit signed int was too large to fit into 10bit signed (POSITIVE)
//		12 - accelerometer self-test results are out of bounds
//      20 - DBG_CONST_ADC_ADJUST - unexpected LED_mode
//      21 - dec2ascii value > 0xf
void fatal(char code) {
	unsigned char pk;
	unsigned int test = BUZZER_TIME(250);
	FATAL_ERROR_CODE = code;	// basic backtrace
	BUSYIO = 0;
	ALL_MOTION_OFF;
	T2CONbits.TMR2ON = 0;
	T0CONbits.TMR0ON = 0;
	LATD = code;	// overwrites debug_pulse#5
	(void)handle_buzzer_event(BUZZER_TIME(250),4);	// for ~2sec of buzzes
	while(1) {
		// check for buzzer timer fire, turn off buzzer
		if (PIR2bits.TMR3IF == 1) {
			pk = handle_buzzer_event(0xFFFF,0x80);	// timer fired, do whatever...
			if (pk == 0) {
				LATD = LATD ^ code;	// flash code
				(void)handle_buzzer_event(BUZZER_TIME(250),4);	// so every 2sec we flash LEDs
			}
		}	// if (timer3 fired)
	}

	// never gets here
}


//========================================================================
// buzzer functions
//========================================================================
void set_timer3_time(unsigned int time) {
	if (T3CONbits.TMR3ON == 1) {
		reset_timer3();	// or else fatal will not be able to use buzzer!
		fatal(7);
	}
	TMR3H = (unsigned char)((time & 0xFF00) >> 8);	// always write upper first
	TMR3L = (unsigned char)(time & 0x00FF);
    T3CONbits.TMR3ON = 1;           // start timer
} 
// Turn buzzer on for 'time'. It will be turned off in the main() spin loop
void buzzer_on(unsigned int time) {
	if (T3CONbits.TMR3ON == 1) {
		// if the buzzer is already being used, just return
		return;																// RETURN
	} else {
		BUZZ_ON;
		set_timer3_time(time);
	}
}
// var 'req': set bit#7(0x80) to indicate a timer has fired (set time=0xffff in this case)
// otherwise, if bit#7 is clear, this is considered a request to
// start a new buzzer event.
// 'req' indicates the number of buzzes to make each for 'time'
// seperated by another time.
// 'Rstate' remembers the current remaining count, 'Rtime' remembers how much
// the original time was for repeating buzzer events.
// returns the number of remaining buzzes to make, can then be used to fire
// other things when the buzzing sequence is complete.
//
// NOTE: timer3 is also used to activate the jokie unit in the RC car. To activate:
//		...
//		JOKIE=1;
//		buzzer_on(BUZZER_TIME(200));
//		BUZZ_OFF;
//		...
unsigned char handle_buzzer_event(unsigned int time, unsigned char req) {
	if (req & 0x80) {
		PIR2bits.TMR3IF = 0;          // Reset Timer flag
		reset_timer3();
		if (JOKIE == 1) {
			JOKIE = 0;	// timer3 is also used to fire jokes, so turn off
			BUZZ_OFF;
			Rstate = 0;
			Rtime = 0;
			return 0;															// RETURN
		}
		// timer event fired (ignore 'time' here)
		if (BUZZ_STATE == FALSE) {
			// currently off
			BUZZ_ON;
		} else {
			// currently on
			BUZZ_OFF;
		}
		--Rstate;
		if (Rstate > 0) {
			// reshedule 
			// makes no distinction between on or off buzzer times, all the same
			set_timer3_time(Rtime);
		} else {
			// make sure it's off when we're done
			BUZZ_OFF;
			Rtime = 0;
			reset_timer3();
		}
	} else {
		Rstate = req;
		Rtime = time;
		buzzer_on(time);
	}
	return Rstate;
}


//========================================================================
// uP SPI xmit & Rx
//========================================================================
unsigned char SPI_IN(void) {
	do {
	} while (SSPSTATbits.BF == 0);
	return SSPBUF;
}
// all writes show up in SSPBUF, make sure to clear
unsigned char SPI_OUT(unsigned char byte) {
	SSPBUF = byte;
	return SPI_IN();
}


//========================================================================
// uP init
//========================================================================
static void set_mode3(void) {
	SSPCON1bits.CKP = 0;	// SCLK idles low
	SSPSTATbits.CKE = 1;	// xmit on 1->0 sclk
}
#if 0
// NOTE: don't use these, put in if 0 so will not compile if turned on
static void set_mode1(void) {
	SSPCON1bits.CKP = 0;	// SCLK idles low
	SSPSTATbits.CKE = 0;	// xmit on 0->1 sclk
}
static void set_mode2(void) {
	SSPCON1bits.CKP = 1;	// SCLK idles high
	SSPSTATbits.CKE = 0;	// xmit on 0->1 sclk
}
static void set_mode4(void) {
	SSPCON1bits.CKP = 1;	// SCLK idles high
	SSPSTATbits.CKE = 1;	// xmit on 1->0 sclk
}
#endif	//0
void SPI_Init(void) {
	SSPCON1 = 0;	// set all back to defaults

	// gyro setup
	TRISBbits.TRISB4 = 0;		// PORTB bit 4 for gyro ST output
	GYRO_ST = 0;				// turn off self-test
	TRISEbits.TRISE0 = 0;		// PORTE bit 0 for gyro SPI ~CS output
	GYRO_CS = 1;				// not selected (active-low)
	// accel setup
	TRISCbits.TRISC0 = 0;		// PORTC bit 0 for accel SPI ~CS output
	ACCEL_CS = 1;				// not selected (active-low)

	// set in Master mode
	//  - set MSSP enable bit SSPEN in SSPCON1
	//  - set SDO output - clear TRISC<5>
	//  - set SCK output - clean TRISC<3>
	TRISCbits.TRISC5 = 0;	// SDO = output
	TRISCbits.TRISC3 = 0;	// SCK = output
#if defined(MODE1)
	set_mode1();
#elif defined(MODE2)
	set_mode2();
#elif defined(MODE3)
	set_mode3();
#elif defined(MODE4)
	set_mode4();
#else
	#error "WTF! - no SPI mode selected! you want to define MODE3 to make sensor modules work!"
#endif
	SSPSTATbits.SMP = 0;	// inRX sampled in middle of data output time
	SSPCON1bits.SSPEN = 1;	// enable MSSP
}
void SPI_reset(void) {
	// reset SSPEN
	// write SSPCON1 reg
	// set SSPEN
	SSPCON1bits.SSPEN = 0;	// turn off enable
	SPI_Init();
}
#ifdef ALLOW_USART
// RS232 USART
//----------------------
void usart_Init(void) {
	// according to DS, set both to input, EUSART module then takes over
	TRISCbits.TRISC6 = 1;	// input
	TRISCbits.TRISC7 = 1;	// input

	// for highest baud possible w/ 8bit async, set
	//		SYNC=0(default), BRG16=BRGH=1
	TXSTAbits.BRGH = 1;	// fast mode
	BAUDCONbits.BRG16 = 1;	// fast mode
	BAUDCONbits.RCIDL = 1;	// Rx is idle
	// formula here is ((Fosc/baud)/[16|64])-1
#ifdef CLK_1MHZ
 #if defined(BAUD_19)
	SPBRGH = 0;
	SPBRG = 12;	// 19.2k @ 1MHz
 #elif defined(BAUD_9)
	SPBRGH = 0;
	SPBRG = 25;	// 9.6k @ 1MHz
 #else
  #error "UNSUPPORTED BAUD RATE!"
 #endif	// BAUD_19
#elif defined(CLK_2MHZ)
 #if defined(BAUD_57)
	SPBRGH = 0;
	SPBRG = 8;	// 57.6k @ 2MHz
 #elif defined(BAUD_19)
	SPBRGH = 0;
	SPBRG = 25;	// 19.2k @ 2MHz
 #elif defined(BAUD_9)
	SPBRGH = 0;
	SPBRG = 51;	// 9.6k @ 2MHz
 #else
  #error "UNSUPPORTED BAUD RATE!"
 #endif	// BAUD_57
#elif defined(CLK_4MHZ)
 #if defined(BAUD_115)
	SPBRGH = 0;
	SPBRG = 8;	// max = 115200baud @ 4MHz
 #elif defined(BAUD_57)
	SPBRGH = 0;
	SPBRG = 16;	// 57.6k @ 4MHz
 #elif defined(BAUD_19)
	SPBRGH = 0;
	SPBRG = 51;	// 19.2k @ 4MHz
 #elif defined(BAUD_9)
	SPBRGH = 0;
	SPBRG = 103;	// 9.6k @ 4MHz
 #else
  #error "UNSUPPORTED BAUD RATE!"
 #endif	// BAUD_115
#else
 #error "UNSUPPORTED CLK RATE!"
#endif

	RCSTAbits.SPEN = 1;		// enabled serial port
	TXSTAbits.TXEN = 1;
	// now ready to go, load 8bit data to txreg
}
#endif
// PWM init for driving (2) PWMs to RC car
void pwm_init(void) {

	PR2 = PR2_DEF;

	// PWM1 @ RC2
	CCPR1L = 0xFF;
	CCP1CON |= 0x30;	// set DC1B1 & DC1B0 = 1 (LSB of duty)
	TRISCbits.TRISC1 = 0;	// output
	
	// PWM2 @ RC1
	CCPR2L = 0xFF;
	CCP2CON |= 0x30;	// set DC2B1 & DC2B0 = 1 (LSB of duty)
	TRISCbits.TRISC2 = 0;	// output

	// prescaler 1:16
	T2CON |= 0x03;
	T2CONbits.TMR2ON = 1;	// enable
	
	// set mode
	CCP1CON |= PWM1_MODE;
	CCP2CON |= PWM2_MODE;

	// redundant
	SET_PWM1_DUTY(0);
	SET_PWM2_DUTY(0);
}

//========================================================================
// sensor module init
//========================================================================
// GYRO
//----------------------
void gyro_Init(void) {
	signed int pre = 0, post = 0;
	unsigned char retry = 5;	// if self-test doesn't work first, retry a few times
	BOOL passed = FALSE;

	// just in case, give it some more time to turn on and stabilize
	Delay10KTCYx(GYRO_POWER_ON_TIME);

	// do self-test:
	//  - read baseline voltage (ST=0)
	//  - set ST(RB4)=1 to init self-test
	//  - delay for EM field to influence MEMs parts
	//  - read voltage many times. delta from baseline should be ~300mV higher
	//  - set ST(RB4)=0

	// just in case, turn off self-test and wait
	GYRO_ST = 0;
	Delay1KTCYx(GYRO_15MS_UPDATE);	// wait 15ms

	// read a few times to get data flowing
	read_gyro(); Delay1KTCYx(GYRO_15MS_UPDATE);
	read_gyro(); Delay1KTCYx(GYRO_15MS_UPDATE);
	read_gyro(); Delay1KTCYx(GYRO_15MS_UPDATE);

	while ((passed == FALSE) && (retry >0)) {
		// ready to take measurements, take two and avg
		read_gyro(); Delay1KTCYx(GYRO_15MS_UPDATE);
		pre = gyro_curr;
		read_gyro(); Delay1KTCYx(GYRO_15MS_UPDATE);
		pre += gyro_curr;
		pre = pre /2;
	
		// turn on self-test mode and wait
		GYRO_ST = 1;
		Delay1KTCYx(GYRO_15MS_UPDATE);	// wait 15ms
	
		// now in self-test, take two readings and avg
		read_gyro(); Delay1KTCYx(GYRO_15MS_UPDATE);
		post = gyro_curr;
		read_gyro(); Delay1KTCYx(GYRO_15MS_UPDATE);
		post += gyro_curr;
		post = post /2;
	
		// the self-test offset is ~300mV, 300mV/3.22mV/step = 93 steps
		gyro_calib = post - pre;

		if ((gyro_calib < GYRO_ST_LOW) || (gyro_calib > GYRO_ST_HIGH)) {
			passed = FALSE;
			--retry;
			LATD ^= 0x2;
			if (retry == 0) {
				fatal(10);
			}
			gyro_calib = 0;
			// turn off self-test for next retry
			GYRO_ST = 0;
			// just a little delay
			Delay1KTCYx(GYRO_15MS_UPDATE);
		} else {
			passed = TRUE;
		}
	}

	// turn off self-test and wait
	GYRO_ST = 0;
	Delay1KTCYx(GYRO_15MS_UPDATE);
}
// ACCEL
//----------------------
void accel_Init(void) {
	signed st1, st2;
	unsigned char retry = 5;	// if self-test doesn't work first, retry a few times
	BOOL passed = FALSE;

	// need to turn off I2C mode in accel chip b/c to it, ~CS high
	// indicates to use I2C, ~CS low says use SPI. to disable I2S in accel
	// chip, set bit 7 in $OD (I2CDIS) via SPI
	write_accel(I2CAD,0x80);

	// write to MCTL(0x16)
	//	SPI3W = 1 	// 3-wire SPI mode
	//	STON = 0	// self test off
	//	GLVL = 01	// 2g mode
	//	MODE = 01	// measurement mode
#if !defined(ACCEL_THRESH)
	//	DRPD = 1	// do not drive data ready on INT1/DRDY pin
	write_accel(MCTL,0x65);	// NOTE: any changes here should be made also below in retry code
#else
	//	DRPD = 0	// drive data ready on INT1/DRDY pin
	write_accel(MCTL,0x25);

	// write level detection level (absolute THOPT=0)
	write_accel(LDTH,levelX_detect);
#endif
	
	// read a few times to get data flowing
	read_accel(); Delay10KTCYx(ACCEL_10MS_UPDATE);
	read_accel(); Delay10KTCYx(ACCEL_10MS_UPDATE);
	read_accel(); Delay10KTCYx(ACCEL_10MS_UPDATE);

	while ((passed == FALSE) && (retry > 0)) { 
		// for Zout correction, do self-test, set STON bit MCTL<4>
		st1 = read_one_accel(ZOUT8); Delay10KTCYx(ACCEL_10MS_UPDATE);	// wait 10ms
		st1 += read_one_accel(ZOUT8);Delay10KTCYx(ACCEL_10MS_UPDATE);	// wait 10ms
		st1 = st1 / 2;
	
		write_accel(MCTL,0x75);	// turn on self-test (only affect z-axis for manuf. calib)
		Delay1KTCYx(GYRO_15MS_UPDATE);	// wait 15ms, 10ms for update + 5ms self-test settle
		st2 = read_one_accel(ZOUT8);
		Delay10KTCYx(ACCEL_10MS_UPDATE);	// wait 10ms
		st2 += read_one_accel(ZOUT8);
		st2 = st2 / 2;
	
		accel_calib = st2 - st1;
	
		// check response, fail if !ok
		if ((accel_calib < ACCEL_ST_LOW) || (accel_calib > ACCEL_ST_HIGH)) {
			passed = FALSE;
			--retry;
			LATD ^= 0x4;
			if (retry == 0) {
				fatal(12);
			}
			accel_calib = 0;
			// turn off self-test for next retry
#if !defined(ACCEL_THRESH)
			write_accel(MCTL,0x65);
#else
			write_accel(MCTL,0x25);
#endif
			// just a little delay
			Delay10KTCYx(ACCEL_10MS_UPDATE);
		} else {
			passed = TRUE;
		}
	}

#if !defined(ACCEL_THRESH)
	//	DRPD = 1	// do not drive data ready on INT1/DRDY pin
	write_accel(MCTL,0x65);
#else
	//	DRPD = 0	// drive data ready on INT1/DRDY pin
	write_accel(MCTL,0x25);
#endif	
	Delay1KTCYx(GYRO_15MS_UPDATE);	// wait 15ms to let self-test settle off
}


//========================================================================
// gyro API - signed 10bits from gyro ADC, returns signed 16bit int
//========================================================================
void read_gyro(void) {
	unsigned int tmp, tmp2;
	GYRO_CS = 0;	// select for SPI comm.
	tmp = (unsigned int)SPI_OUT(0x00);	// provide SCLK so slave can respond
	tmp2 = (unsigned int)SPI_OUT(0x00);	// provide SCLK so slave can respond
	GYRO_CS = 1;

	// ADC101S021 sends back four zeros followed by 10bit reading, then 2 zeros
	// above, as the uchar is converted to uint, byte is placed in upper portion
	// of int. 
	// here it's 10bit unsigned value
	gyro_curr = ((tmp<<6)&0x03C0) | ((tmp2>>2)&0x003f);

	if (did_calib == TRUE) {
		gyro_curr -= (unsigned int)CALgF;
	}

#ifdef GYRO_MISREAD_BUG
	if ((last_gyro!=0) && (gyro_calib != 0)) {
		if (tABS(gyro_curr - last_gyro) > 50) {
			BREAKPT;
			NOP;
		}
	}
	last_gyro = gyro_curr;
#endif
	// after the offset subtract, it's now a signed value
}


//========================================================================
// accel API - each 8bits
//========================================================================
void read_accel(void) {
	accelX_curr = read_one_accel(XOUT8);
	accelY_curr = read_one_accel(YOUT8);
	accelZ_curr = read_one_accel(ZOUT8);
}
// addr is 6-bit
// reads are indicated w/ MSB=0 (writes MSB=1), then 6bit addr followed by 0
signed char read_one_accel(unsigned char addr) {
	unsigned char tmp;
	ACCEL_CS = 0;	// select for SPI comm.
	tmp = SPI_OUT((addr << 1) & 0x7e);	// bit8 = 0 for reads
	tmp = SPI_OUT(0x00);	// provide SCLK so slave can respond
	ACCEL_CS = 1;
	return (signed char)tmp;
}
// addr is 6-bit
// reads are indicated w/ MSB=0 (writes MSB=1), then 6bit addr followed by 0
void write_accel(unsigned char addr, unsigned char cmd) {
	ACCEL_CS = 0;	// select for SPI comm.
	(void)SPI_OUT(0x80 | ((addr << 1) & 0x7e));
	(void)SPI_OUT(cmd);
	ACCEL_CS = 1;	// de-select	
}
void write_accel_calibs(void) {
	signed int gg;
	gg = CALxF * 2;
	write_accel(XOFFL,(unsigned int)gg & 0x00FF);
	write_accel(XOFFH,((unsigned int)gg & 0xFF00) >> 8);
	gg = CALyF * 2;
	write_accel(YOFFL,(unsigned int)gg & 0x00FF);
	write_accel(YOFFH,((unsigned int)gg & 0xFF00) >> 8);
	gg = CALzF * 2;
	write_accel(ZOFFL,(unsigned int)gg & 0x00FF);
	write_accel(ZOFFH,((unsigned int)gg & 0xFF00) >> 8);
}


//========================================================================
// sensor data - we have memory and not a lot of time, so copy code
//
// below is moving average window code
//========================================================================
// init circ.buffer
// this buffer maintains a list of the last 'SZ_AVG' samples and a sum of their total
void clear_SdataX(void) {
	for(pAx = 0 ; pAx<SZ_AVG ; ++pAx) {
		Ax[pAx] = 0;
	}
	pAx = 0;
	AxS = 0;
	AxWrap = FALSE;
}
// stores samples in circular buffer
void store_sampleX(void) {
	if (AxWrap == TRUE) {
		AxS -= Ax[pAx];	// pAx points to next that would be overwritten
		Ax[pAx] = accelX_curr;
		++pAx;
		AxS += accelX_curr;
		// already wrapped, we are now removing one for each addition
		// remove 'next'
		if (pAx >= SZ_AVG) pAx = 0;
		// now push the current avg to the deriv filter
		BUSYIO = 1;
		store_DsampleX(get_avgX());
	} else {
		Ax[pAx] = accelX_curr;
		++pAx;
		AxS += accelX_curr;
		// !wrapped yet
		if (pAx >= SZ_AVG) {
			AxWrap = TRUE;
			pAx = 0;
		}
	}
}
signed char get_avgX(void) {
	if (AxWrap == TRUE) {
		b.SYS_AVGx = (signed char)(AxS/SZ_AVG);
		return b.SYS_AVGx;
	} else {
		return(0);
		//return (AxS/pAx);
	}
}
// init circ.buffer
// this buffer maintains a list of the last 'SZ_AVG' samples and a sum of their total
void clear_SdataY(void) {
	for(pAy = 0 ; pAy<SZ_AVG ; ++pAy) {
		Ay[pAy] = 0;
	}
	pAy = 0;
	AyS = 0;
	AyWrap = FALSE;
}
// stores samples in circular buffer
void store_sampleY(void) {
	if (AyWrap == TRUE) {
		AyS -= Ay[pAy];	// pAx points to next that would be overwritten
		Ay[pAy] = accelY_curr;
		++pAy;
		AyS += accelY_curr;
		// already wrapped, we are now removing one for each addition
		// remove 'next'
		if (pAy >= SZ_AVG) pAy = 0;
		// now push the current avg to the deriv filter
		BUSYIO = 1;
		store_DsampleY(get_avgY());
	} else {
		Ay[pAy] = accelY_curr;
		++pAy;
		AyS += accelY_curr;
		// !wrapped yet
		if (pAy >= SZ_AVG) {
			AyWrap = TRUE;
			pAy = 0;
		}
	}
}
signed char get_avgY(void) {
	if (AyWrap == TRUE) {
		b.SYS_AVGy = (signed char)(AyS/SZ_AVG);
		return b.SYS_AVGy;
	} else {
		return(0);
		//return (AyS/pAy);
	}
}
// init circ.buffer
// this buffer maintains a list of the last 'SZ_AVG' samples and a sum of their total
void clear_SdataZ(void) {
	for(pAz = 0 ; pAz<SZ_AVG ; ++pAz) {
		Az[pAz] = 0;
	}
	pAz = 0;
	AzS = 0;
	AzWrap = FALSE;
}
// stores samples in circular buffer
void store_sampleZ(void) {
	if (AzWrap == TRUE) {
		AzS -= Az[pAz];	// pAx points to next that would be overwritten
		Az[pAz] = accelZ_curr;
		++pAz;
		AzS += accelZ_curr;
		// already wrapped, we are now removing one for each addition
		// remove 'next'
		if (pAz >= SZ_AVG) pAz = 0;
		// now push the current avg to the deriv filter
		BUSYIO = 1;
		store_DsampleZ(get_avgZ());
	} else {
		Az[pAz] = accelZ_curr;
		++pAz;
		AzS += accelZ_curr;
		// !wrapped yet
		if (pAz >= SZ_AVG) {
			AzWrap = TRUE;
			pAz = 0;
		}
	}
}
signed char get_avgZ(void) {
	if (AzWrap == TRUE) {
		b.SYS_AVGz = (signed char)(AzS/SZ_AVG);
		return b.SYS_AVGz;
	} else {
		return(0);
		//return (AzS/pAz);
	}
}
// init circ.buffer
// this buffer maintains a list of the last 'SZ_AVG' samples and a sum of their total
void clear_SdataG(void) {
	for(pAg = 0 ; pAg<SZ_AVG ; ++pAg) {
		Ag[pAg] = 0;
	}
	pAg = 0;
	AgS = 0;
	AgWrap = FALSE;
}
// stores samples in circular buffer
void store_sampleG(void) {
	signed int tk;
	if (AgWrap == TRUE) {
		AgS -= Ag[pAg];	// pAx points to next that would be overwritten
		Ag[pAg] = gyro_curr;	// 16bit signed int
		++pAg;
		AgS += gyro_curr;
		// already wrapped, we are now removing one for each addition
		// remove 'next'
		if (pAg >= SZ_AVG) pAg = 0;
		// now push the current avg to the deriv filter
		BUSYIO = 1;
		tk = get_avgG();
		IAgS += (signed long)tk;
		BUSYIO = 0;
		store_DsampleG(tk);
	} else {
		Ag[pAg] = gyro_curr;
		++pAg;
		AgS += gyro_curr;
		// !wrapped yet
		if (pAg >= SZ_AVG) {
			AgWrap = TRUE;
			pAg = 0;
		}
	}
}
signed int get_avgG(void) {
	if (AgWrap == TRUE) {
		b.SYS_AVGg = (signed int)(AgS/SZ_AVG);
		return b.SYS_AVGg;
	} else {
		return(0);
		//return (AgS/pAg);
	}
}
//========================================================================
// below is deriv code (actually also contains moving window avg)
//========================================================================
// init circ.buffer
// this buffer maintains a list of the last 'SZ_AVG' samples and a sum of their total
void clear_DSdataX(void) {
	for(DpAx = 0 ; DpAx<SZ_DER ; ++DpAx) {
		DAx[DpAx] = 0;
	}
	DpAx = 0;
	DAxS = 0;
	DAxWrap = FALSE;
	DAxL_first = TRUE;
	DAxL = 0;
}
// stores samples in circular buffer
void store_DsampleX(signed char samp) {
	signed char tp;
	if (DAxL_first == TRUE) {
		DAxL = samp;
		DAxL_first = FALSE;
	}
	tp = DAxL - samp;	// find delta from last sample
	DAxL = samp;		// last = this sample

	if (DAxWrap == TRUE) {
		DAxS -= DAx[DpAx];	// DpAx points to next that would be overwritten
		DAx[DpAx] = tp;
		++DpAx;
		DAxS += tp;
		// already wrapped, we are now removing one for each addition
		// remove 'next'
		if (DpAx >= SZ_DER) DpAx = 0;
		BUSYIO=0;
		(void)get_DavgX();	// update the b.SYS_ var
		BUSYIO=1;
	} else {
		DAx[DpAx] = tp;
		++DpAx;
		DAxS += tp;
		// !wrapped yet
		if (DpAx >= SZ_DER) {
			DAxWrap = TRUE;
			DpAx = 0;
		}
	}
}
signed char get_DavgX(void) {
	if (DAxWrap == TRUE) {
		b.SYS_AVGDERIVx = (signed char)(DAxS);
		return b.SYS_AVGDERIVx;
	} else {
		return(0);
		//return (DAxS/DpAx);
	}
}
// init circ.buffer
// this buffer maintains a list of the last 'SZ_AVG' samples and a sum of their total
void clear_DSdataY(void) {
	for(DpAy = 0 ; DpAy<SZ_DER ; ++DpAy) {
		DAy[DpAy] = 0;
	}
	DpAy = 0;
	DAyS = 0;
	DAyWrap = FALSE;
	DAyL_first = TRUE;
	DAyL = 0;
}
// stores samples in circular buffer
void store_DsampleY(signed char samp) {
	signed char tp;
	if (DAyL_first == TRUE) {
		DAyL = samp;
		DAyL_first = FALSE;
	}
	tp = DAyL - samp;
	DAyL = samp;

	if (DAyWrap == TRUE) {
		DAyS -= DAy[DpAy];	// pAx points to next that would be overwritten
		DAy[DpAy] = tp;
		++DpAy;
		DAyS += tp;
		// already wrapped, we are now removing one for each addition
		// remove 'next'
		if (DpAy >= SZ_DER) DpAy = 0;
		BUSYIO=0;
		(void)get_DavgY();	// update the b.SYS_ var
		BUSYIO=1;
	} else {
		DAy[DpAy] = tp;
		++DpAy;
		DAyS += tp;
		// !wrapped yet
		if (DpAy >= SZ_DER) {
			DAyWrap = TRUE;
			DpAy = 0;
		}
	}
}
signed char get_DavgY(void) {
	if (DAyWrap == TRUE) {
		b.SYS_AVGDERIVy = (signed char)(DAyS);
		return b.SYS_AVGDERIVy;
	} else {
		return(0);
		//return (DAyS/DpAy);
	}
}
// init circ.buffer
// this buffer maintains a list of the last 'SZ_AVG' samples and a sum of their total
void clear_DSdataZ(void) {
	for(DpAz = 0 ; DpAz<SZ_DER ; ++DpAz) {
		DAz[DpAz] = 0;
	}
	DpAz = 0;
	DAzS = 0;
	DAzWrap = FALSE;
	DAzL_first = TRUE;
	DAzL = 0;
}
// stores samples in circular buffer
void store_DsampleZ(signed char samp) {
	signed char tp;
	if (DAzL_first == TRUE) {
		DAzL = samp;
		DAzL_first = FALSE;
	}
	tp = DAzL - samp;
	DAzL = samp;

	if (DAzWrap == TRUE) {
		DAzS -= DAz[DpAz];	// pAx points to next that would be overwritten
		DAz[DpAz] = tp;
		++DpAz;
		DAzS += tp;
		// already wrapped, we are now removing one for each addition
		// remove 'next'
		if (DpAz >= SZ_DER) DpAz = 0;
		BUSYIO=0;
		(void)get_DavgZ();	// update the b.SYS_ var
		BUSYIO=1;
	} else {
		DAz[DpAz] = tp;
		++DpAz;
		DAzS += tp;
		// !wrapped yet
		if (DpAz >= SZ_DER) {
			DAzWrap = TRUE;
			DpAz = 0;
		}
	}
}
signed char get_DavgZ(void) {
	if (DAzWrap == TRUE) {
		b.SYS_AVGDERIVz = (signed char)(DAzS);
		return b.SYS_AVGDERIVz;
	} else {
		return(0);
		//return (DAzS/DpAz);
	}
}
// init circ.buffer
// this buffer maintains a list of the last 'SZ_AVG' samples and a sum of their total
void clear_DSdataG(void) {
	for(DpAg = 0 ; DpAg<SZ_DER ; ++DpAg) {
		DAg[DpAg] = 0;
	}
	DpAg = 0;
	DAgS = 0;
	DAgWrap = FALSE;
	DAgL_first = TRUE;
	DAgL = 0;
	IAgS = 0;
	IAgS_base = 0;
	gyro_int_reset = 0;
}
// stores samples in circular buffer
// values passed in is 10bit ADC minus the offset, giving signed 16bit
void store_DsampleG(signed int samp) {
	signed int tp;
	if (DAgL_first == TRUE) {
		DAgL = samp;
		DAgL_first = FALSE;
	}
	tp = DAgL - samp;
	DAgL = samp;

	if (DAgWrap == TRUE) {
		DAgS -= (signed long)DAg[DpAg];	// pAx points to next that would be overwritten
		DAg[DpAg] = tp;
		++DpAg;
		DAgS += (signed long)tp;
		// already wrapped, we are now removing one for each addition
		// remove 'next'
		if (DpAg >= SZ_DER) DpAg = 0;
		BUSYIO = 1;
		(void)get_DavgG();	// update the b.SYS_ var
		BUSYIO = 0;
	} else {
		DAg[DpAg] = tp;
		++DpAg;
		DAgS += (signed long)tp;
		// !wrapped yet
		if (DpAg >= SZ_DER) {
			DAgWrap = TRUE;
			DpAg = 0;

			//
			// once we have wrapped the gyro (takes
			// longer than accel) deriv, then the avg filter
			// is full of data and now deriv is full of data, it should
			// be valid now!
			//
			b.valid = TRUE;
			// 
		}
	}
}
signed int get_DavgG(void) {
	if (DAgWrap == TRUE) {
		b.SYS_AVGDERIVg = (signed int)(DAgS);
		return b.SYS_AVGDERIVg;
	} else {
		return(0);
		//return (DAgS/DpAg);
	}
}
void clear_buffers(void) {
#if 0	// yes... but...
	b.valid = FALSE;	// need to wait for data to pipe thru filters
#endif
	clear_SdataX();
	clear_SdataY();
	clear_SdataZ();
	clear_SdataG();
	clear_DSdataX();
	clear_DSdataY();
	clear_DSdataZ();
	clear_DSdataG();
}


#ifdef ALLOW_USART
//========================================================================
// EUSART API
//========================================================================
void send232_break(void) {
	if (rs232_xmit_bytes > RS232_BYTES_BREAK) {
		//send232(0x0d);	// CR
		send232(0x0a);	// LF
		send232(0x0c);	// 12d=FF to remind Tail::Fail we are updating
		rs232_xmit_bytes = 0;
#if !defined(DBG_CONST_ADC_ADJUST)
		LATD = LATD ^ 0xdf;	// don't toggle bit5, this is debug_pulse output
#endif
	}
}
void send232(unsigned char data) {
	// die if something is already being sent
	if (TXSTAbits.TRMT == 0)
		fatal(4);
	TXREG = data;
	++rs232_xmit_bytes;		// we have some time here, no penalty to do this here
	while(TXSTAbits.TRMT == 0)
		;
}
// sends HL
// 0x32 is send '3' first, then '2'
void send232_char(unsigned char data) {
	send232(dec2ascii((data & 0xf0) >> 4));
	send232(dec2ascii(data & 0x0f));
}
// 0x32 is send '3' first, then '2'
void send232_int(unsigned int data) {
	send232(dec2ascii((data & 0xf000) >> 12));
	send232(dec2ascii((data & 0x0f00) >> 8));
	send232(dec2ascii((data & 0x00f0) >> 4));
	send232(dec2ascii(data & 0x000f));
}
// sends:
//		'A'
//		accelX_curr
//		accelY_curr
//		accelZ_curr
// Uses: 4 bytes = 40bits
void send232_raw_accel(void) {
	send232(0x41);			// 'A'
	send232((unsigned char)accelX_curr);
	send232((unsigned char)accelY_curr);
	send232((unsigned char)accelZ_curr);
	send232_break();	// need to send a CR?
}
void send232_avg_accel(void) {
	send232(0x41);			// 'A'
	send232(b.SYS_AVGx);
	send232(b.SYS_AVGy);
	send232(b.SYS_AVGz);
	send232_break();	// need to send a CR?
}
void send232_avgderiv_accel(void) {
	send232(0x41);			// 'A'
	send232(b.SYS_AVGDERIVx);
	send232(b.SYS_AVGDERIVy);
	send232(b.SYS_AVGDERIVz);
	send232_break();	// need to send a CR?
}
// sends:
//		'G'
//		gyro_curr (msb)
//		gyro_curr (lsb)
// Uses: 3 bytes = 30bits
void send232_raw_gyro(void) {
	send232(0x47);			// 'G'
	send232((unsigned char)((gyro_curr&0xff00)>>8));
	send232((unsigned char)(gyro_curr&0x00ff));
	send232_break();	// need to send a CR?
}
void send232_avg_gyro(void) {
	send232(0x47);			// 'G'
	send232((unsigned char)((b.SYS_AVGg&0xff00)>>8));
	send232((unsigned char)(b.SYS_AVGg&0x00ff));
	send232_break();	// need to send a CR?
}
void send232_avgderiv_gyro(void) {
	send232(0x47);			// 'G'
	send232((unsigned char)((b.SYS_AVGDERIVg&0xff00)>>8));
	send232((unsigned char)(b.SYS_AVGDERIVg&0x00ff));
	send232_break();	// need to send a CR?
}
// 0=0x30
// 1=0x31
// ...
// 9=0x39
// a=0x41
// ...
// f=0x46
unsigned char dec2ascii(unsigned char byte) {
	if (byte > 0xF) {
		fatal(21);
	}
	if (byte <= 9) {
		return (byte+0x30);
	} else {
		return (byte-10+0x41);
	}
}
#endif


//========================================================================
// DBG_CONST_ADC_ADJUST
//========================================================================
#ifdef DBG_CONST_ADC_ADJUST
void send_movement_cfg_232(void) {
	send232('+');
	send232('1');
	send232_char(m.SLOPE_MULTXZ);
	send232('2');
	send232_char(m.SLOPE_MULTYZ);
	send232('3');
	send232_char(m.AXZ_MOTION_OFF_MIN);
	send232('4');
	send232_char(m.AYZ_MOTION_OFF_MIN);
	send232('5');
	send232_char(m.GYRO_ROT_MIN_TURN_STEERING);
	send232('6');
	send232_char(m.ROT_MULT);
	send232('7');
	send232_char(m.GYRO_ROT_HARD_STEER);
	send232('8');
	send232_char(m.ROT_MULT_HARD);
	send232('9');
	send232_char(m.PWM1_MULT);
	send232('A');
	send232_char(m.PWM2_MULT);
	send232('B');
	send232_char(m.PR2);
	send232('C');
	send232_char(m.PWM1_DYN_THRES);
	send232('D');
	send232_char(m.PWM2_DYN_THRES);
	send232('E');
	send232_char(m.PWM1_DYN_MULT);
	send232('F');
	send232_char(m.PWM2_DYN_MULT);
	send232(0x0a);
	send232(0x0c);
}
#endif


//========================================================================
// 	Misc init
//========================================================================
void Timer0_Init(void)
{
    INTCONbits.TMR0IF = 0;          // clear roll-over interrupt flag
    T0CON = T0CON_DEF;
	TMR0H = (unsigned char)((TMR0_refill & 0xFF00) >> 8);	// always write upper first
	TMR0L = (unsigned char)(TMR0_refill & 0x00FF);
    T0CONbits.TMR0ON = 1;           // start timer
}
void Timer3_Init(void)
{
    PIR2bits.TMR3IF = 0;          // clear roll-over interrupt flag
    T3CON = T3CON_DEF;
	reset_timer3();
}
void reset_timer3(void) {
    PIR2bits.TMR3IF = 0;          // clear roll-over interrupt flag
	TMR3H = 0;
	TMR3L = 0;
    T3CONbits.TMR3ON = 0;           // make sure timer turned off
}


//========================================================================
// Movement cfg constants
//========================================================================
void init_movement_cfg(void) {
	m.SLOPE_MULTXZ = cSLOPE_MULTXZ_DEF;					// 2	// mult by 5 when used
	m.SLOPE_MULTXZ_orig = m.SLOPE_MULTXZ;
	m.SLOPE_MULTYZ = cSLOPE_MULTYZ_DEF;					// 3	// mult by 5 when used
	m.SLOPE_MULTYZ_orig = m.SLOPE_MULTYZ;
	m.AXZ_MOTION_OFF_MIN = cAXZ_MOTION_OFF_MIN;			// 4
	m.AYZ_MOTION_OFF_MIN = cAYZ_MOTION_OFF_MIN;			// 5
	m.GYRO_ROT_MIN_TURN_STEERING = cGYRO_ROT_MIN_TURN_STEERING;	// 6
	m.ROT_MULT = cROT_MULT;						// 7
	m.GYRO_ROT_HARD_STEER = cGYRO_ROT_HARD_STEER;			// 8
	m.ROT_MULT_HARD = cROT_MULT_HARD;				// 9
	m.PWM1_MULT = cPWM1_MULT;					// 10
	m.PWM2_MULT = cPWM2_MULT;					// 11
	m.PR2 = PR2_DEF;							// 12
	m.PR2_orig = m.PR2;
	m.PWM1_DYN_THRES = cPWM1_DYN_THRES;			// 13
	m.PWM2_DYN_THRES = cPWM2_DYN_THRES;			// 14
	m.PWM1_DYN_MULT = cPWM1_DYN_MULT;			// 15
	m.PWM2_DYN_MULT = cPWM2_DYN_MULT;			// 16
}


//========================================================================
// Brain functions
//========================================================================
void clear_brain(void) {
	b.SYS_AVGx = 0;
	b.SYS_AVGy = 0;
	b.SYS_AVGz = 0;
	b.SYS_AVGg = 0;
	b.SYS_AVGDERIVx = 0;
	b.SYS_AVGDERIVy = 0;
	b.SYS_AVGDERIVz = 0;
	b.SYS_AVGDERIVg = 0;
	b.Axz_slope = 0;
	b.Ayz_slope = 0;
	b.rot_left = FALSE;
	b.rot_right = FALSE;
	b.rot_hard_left = FALSE;
	b.rot_hard_right = FALSE;
	b.motion_any = FALSE;
	b.motion_large = FALSE;
#if 0
	b.Axy_slope = 0;
	b.Dxz_slope = 0;
	b.Dyz_slope = 0;
	b.Dxy_slope = 0;
	b.motion_none = FALSE;
	b.motion_move = FALSE;
#endif
	b.sensors_up = FALSE;	// set in calib
	b.steer_reverse = FALSE;
	b.just_updated = FALSE;
	b.valid = FALSE;
}
//
// do all numeric crunch calcs needed for determining user intent
//
void update_brain(void) {
	signed long IAgS_tmp;

	// wait for all our data to pipe thru filters, but we still do
	// brain updates during calib so we can detect user movement and
	// restart calib if needed.
	if (!b.valid) {
		return;
	}

	//============================================================================
	// Used to determine an approximate angle the controller is being held.
	//
	// These values range from:
	//		0							- if [2ndVar] is at max (and [1stVar] is small)
	//      (SLOPE_MULT * [1stVar]) 	- if [2ndVar] is 0 (and [1stVar] is large)
	//
	// The output is exponential since being fed to PWM for RC drive control
	//============================================================================

	// average calcs
	// if 'z' is negative, we are being held up or down and past 90deg (pointing down and back, up and pointing back)
	//    or the ctrlr is being held to the side but starting to go upside down. In this case, just stay at max on
	if (b.sensors_up) {
		b.Axz_slope = (b.SYS_AVGz <= 0) ? ((signed int)m.SLOPE_MULTXZ * SLOPE_MULT_MULT * b.SYS_AVGx) : ((signed int)m.SLOPE_MULTXZ * SLOPE_MULT_MULT * b.SYS_AVGx) / b.SYS_AVGz;
		b.Ayz_slope = (b.SYS_AVGz <= 0) ? ((signed int)m.SLOPE_MULTYZ * SLOPE_MULT_MULT * b.SYS_AVGy) : ((signed int)m.SLOPE_MULTYZ * SLOPE_MULT_MULT * b.SYS_AVGy) / b.SYS_AVGz;
	} else {
		b.Axz_slope = (b.SYS_AVGz >= 0) ? ((signed int)m.SLOPE_MULTXZ * SLOPE_MULT_MULT * b.SYS_AVGx) : ((signed int)m.SLOPE_MULTXZ * SLOPE_MULT_MULT * b.SYS_AVGx) / b.SYS_AVGz;
		b.Ayz_slope = (b.SYS_AVGz >= 0) ? ((signed int)m.SLOPE_MULTYZ * SLOPE_MULT_MULT * b.SYS_AVGy) : ((signed int)m.SLOPE_MULTYZ * SLOPE_MULT_MULT * b.SYS_AVGy) / b.SYS_AVGz;
	}

#if 0
	// NOTE: these formula are OLD, don't use
	b.Axy_slope = (b.SYS_AVGy <= 0) ? (SLOPE_MULT * b.SYS_AVGx) : (SLOPE_MULT * b.SYS_AVGx) / b.SYS_AVGy;
	// derivative(average) calcs:
	b.Dxz_slope = (b.SYS_AVGDERIVz == 0) ? (SLOPE_MULT * b.SYS_AVGDERIVx) : (SLOPE_MULT * b.SYS_AVGDERIVx) / b.SYS_AVGDERIVz;
	b.Dyz_slope = (b.SYS_AVGDERIVz == 0) ? (SLOPE_MULT * b.SYS_AVGDERIVy) : (SLOPE_MULT * b.SYS_AVGDERIVy) / b.SYS_AVGDERIVz;
	b.Dxy_slope = (b.SYS_AVGDERIVy == 0) ? (SLOPE_MULT * b.SYS_AVGDERIVx) : (SLOPE_MULT * b.SYS_AVGDERIVx) / b.SYS_AVGDERIVy;
#endif

#ifdef ALLOW_GYRO_STEER

	//
	// NOTE:
	//  these two 'if 0' cases are THE SAME but the variable being
	//  compared is different, and SYS_AVGg is int, IAgS is long
	//
#if 0
	// negative SYS_AVGg means turning right
	if (tABS(b.SYS_AVGg) > m.GYRO_ROT_MIN_TURN_STEERING) {
		b.rot_left = (b.SYS_AVGg > 0);
		b.rot_right = (b.SYS_AVGg < 0);
		if (tABS(b.SYS_AVGg) > m.GYRO_ROT_HARD_STEER) {
			b.rot_hard_left = b.rot_left;
			b.rot_hard_right = b.rot_right;
		}
	} else {
		b.rot_hard_left = b.rot_hard_right = b.rot_left = b.rot_right = FALSE;
	}
#else	// 0
	IAgS_tmp = IAgS - IAgS_base;
	if (tABS(IAgS_tmp) > (GYRO_STEP_DEG_MULT * m.GYRO_ROT_MIN_TURN_STEERING)) {
		b.rot_left = (IAgS_tmp > 0);
		b.rot_right = (IAgS_tmp < 0);
		if (tABS(IAgS_tmp) > (GYRO_STEP_DEG_MULT * m.GYRO_ROT_HARD_STEER)) {
			b.rot_hard_left = b.rot_left;
			b.rot_hard_right = b.rot_right;
		}
	} else {
		b.rot_hard_left = b.rot_hard_right = b.rot_left = b.rot_right = FALSE;
	}	
#endif		// 0
#endif

	// motion indicators
	b.motion_any = (Dx_ge(MTHRESH_small) || Dy_ge(MTHRESH_small) || Dz_ge(MTHRESH_small) || Dg_ge(GTHRESH_small));
	b.motion_large = (Dx_ge(MTHRESH_large) || Dy_ge(MTHRESH_large) || Dz_ge(MTHRESH_large) || Dg_ge(GTHRESH_large));
#if 0
	// b.motion_none should be basically the opposite of b.motion_any
	b.motion_none = (Dx_le(MTHRESH_none) && Dy_le(MTHRESH_none) && Dz_le(MTHRESH_none) && Dg_le(GTHRESH_none));
	b.motion_move = (Dx_ge(MTHRESH_move) || Dy_ge(MTHRESH_move) || Dz_ge(MTHRESH_move) || Dg_ge(GTHRESH_move));
#endif
}


//========================================================================
// "main" handler functions
//========================================================================
void main_rc_car(void) {
	static unsigned int calib_cntr = 0;
	signed int tmp2 = 1;
	BOOL prev_result = FALSE;

	if (b.valid && completed_calib) {
		if (b.motion_large) {
			// a trick to hold JOKIE on for 200ms (min.required to activate joke master in RC car)
			JOKIE = 1;
			buzzer_on(BUZZER_TIME(200));
			BUZZ_OFF;
		}
		// set PWM, I/O, etc
		if (new_accel || new_gyro) {
			BUSYIO = 1;
			//
			// forward-backward
			//
			if (tABS(b.Axz_slope) < m.AXZ_MOTION_OFF_MIN) {
				FORWARD_BACKWARD_OFF;
#ifdef DEBUGBLD
LATDbits.LATD2 = 0;
LATDbits.LATD3 = 0;
#endif
			} else if (b.Axz_slope > 0) {
#ifdef DEBUGBLD
LATDbits.LATD2 = 1;
LATDbits.LATD3 = 0;
#endif
				GO_FORWARD;
				SET_PWM2_DUTY(((unsigned int)tABS(b.Axz_slope)*(unsigned int)m.PWM2_MULT) > PWM2_drive_max ? \
					PWM2_drive_max : \
					(unsigned int)tABS(b.Axz_slope)*(unsigned int)m.PWM2_MULT);
			} else {
#ifdef DEBUGBLD
LATDbits.LATD2 = 0;
LATDbits.LATD3 = 1;
#endif
				GO_BACKWARD;
				SET_PWM2_DUTY(((unsigned int)(tABS(b.Axz_slope ^ 0xffff) + 1)*(unsigned int)m.PWM2_MULT) > PWM2_drive_max ? \
					PWM2_drive_max : \
					(unsigned int)tABS((b.Axz_slope ^ 0xffff) + 1)*(unsigned int)m.PWM2_MULT);
			}
			//
			// left-right
			//
			// if we are less than the movement threshold and the gyro didn't detect
			// a twist, turn things off, otherwise, let the gyro participate also
			//
			if (tABS(b.Ayz_slope) < vAYZ_MOTION_OFF_MIN) {
				prev_result = TRUE;
				// we set IAgS to the current value if the controller
				// has been "still" for a few consecutive samples
				++gyro_int_reset;
				if (tABS(gyro_curr)<=GYRO_CURR_MIN_INT_RESET) {
					if (gyro_int_reset >= GYRO_INT_RESET_MAX) {
						gyro_int_reset = 0;
						// it has been still for a while, think about resetting the gyro_curr integral
						IAgS_base = IAgS;
						// NOTE: cheat here... (should not be modifying brain vars here)
						b.rot_hard_left = b.rot_hard_right = b.rot_left = b.rot_right = FALSE;
					}
				} else {
					gyro_int_reset = 0;
				}
			} else {
				gyro_int_reset = 0;
			}
			if (prev_result && \
				!b.rot_left && !b.rot_right && !b.rot_hard_left && !b.rot_hard_right)
			{
				// here we turn off PWM1 so if user jerks ctrlr then sits it still,
				// the wheels won't continue doing a dance
				LEFT_RIGHT_OFF;
#ifdef DEBUGBLD
LATDbits.LATD0 = 0;
LATDbits.LATD7 = 0;
#endif
			} else if ( (!b.steer_reverse && ((b.sensors_up && (b.Ayz_slope > 0)) || (!b.sensors_up && (b.Ayz_slope < 0)))) \
					|| (b.steer_reverse && ((b.sensors_up && (b.Ayz_slope < 0)) || (!b.sensors_up && (b.Ayz_slope > 0))))) {
				TURN_LEFT;
#ifdef DEBUGBLD
LATDbits.LATD0 = 0;
LATDbits.LATD7 = 1;
#endif
#ifdef ALLOW_GYRO_STEER
				// if we are turning left, but gyro vars say right, reset them
				if ((IAgS-IAgS_base) < (-1 * (signed long)GYRO_STEERING_ADJUST)) {
					++gyro_left_reset_cntr;
					if (gyro_left_reset_cntr > GYRO_LEFT_RESET_CNTR_MAX) {
						gyro_left_reset_cntr = 0;
						IAgS = (signed long)(GYRO_STEERING_ADJUST);		// default value
						b.rot_left = b.rot_hard_left = FALSE;
					}
				} else {
					gyro_left_reset_cntr = 0;
				}
				if (b.rot_left) {
					tmp2 = m.ROT_MULT;
				} else if (b.rot_hard_left) {
					tmp2 = m.ROT_MULT_HARD;
				}
#endif
				SET_PWM1_DUTY(((unsigned int)tABS(b.Ayz_slope)*(unsigned int)PWM1_MULT_VAL*tmp2) > PWM1_drive_max ? \
					PWM1_drive_max : \
					(unsigned int)tABS(b.Ayz_slope)*(unsigned int)PWM1_MULT_VAL*tmp2);
			} else {
				TURN_RIGHT;
#ifdef DEBUGBLD
LATDbits.LATD0 = 1;
LATDbits.LATD7 = 0;
#endif
#ifdef ALLOW_GYRO_STEER
				// if we are turning right, but gyro vars say left, reset them
				if ((IAgS-IAgS_base) > (signed long)GYRO_STEERING_ADJUST) {
					++gyro_right_reset_cntr;
					if (gyro_right_reset_cntr > GYRO_RIGHT_RESET_CNTR_MAX) {
						gyro_right_reset_cntr = 0;
						IAgS = (signed long)(-1) * (signed long)(GYRO_STEERING_ADJUST);		// default value
						b.rot_right = b.rot_hard_right = FALSE;
					}
				} else {
					gyro_right_reset_cntr = 0;
				}
				if (b.rot_right) {
					tmp2 = m.ROT_MULT;
				} else if (b.rot_hard_right) {
					tmp2 = m.ROT_MULT_HARD;
				}
#endif
				SET_PWM1_DUTY(((unsigned int)(tABS(b.Ayz_slope ^ 0xffff) + 1)*(unsigned int)PWM1_MULT_VAL*tmp2) > PWM1_drive_max ? \
					PWM1_drive_max : \
					(unsigned int)tABS((b.Ayz_slope ^ 0xffff) + 1)*(unsigned int)PWM1_MULT_VAL*tmp2);
			}
			BUSYIO = 0;
		}
	} else {
		// we haven't done calib yet, annoy the user with beeps if there's any motion
		// otherwise, if there is no activity for ~2sec, go ahead and do the calib
		if (b.valid) {

#ifdef ALLOW_EEPROM
			// do we have valid calib in eeprom?
			if ((!tried_eecalib) && (read_eeprom_calib() == 0)) {
				// eeprom values look correct, program and goto main code
				did_calib = TRUE;
				completed_calib = TRUE;
				write_accel_calibs();
				(void)handle_buzzer_event(BUZZER_TIME(200),4);
				tried_eecalib = TRUE;
				return;														// RETURN
			}
			tried_eecalib = TRUE;
#endif

			if (b.motion_any) {
				calib_cntr = 0;
				//buzzer_on(BUZZER_TIME(200));
			} else {
				// no motion... and we haven't done calib, if we count long enough
				// go ahead and do cal
				++calib_cntr;
				// assuming a 40us main loop cycle, 50000 would be about 2 seconds of no activity
				// but now we only get here every 24ms
				if (calib_cntr > 2000) {
					LED_mode = 1;
					calib_cntr = 0;
				}
			}
		}
	}
}


//---------------------------------
// calib stages:
//
//  * initial beeping (lay me flat!) at least 3 seconds, or keep beeping until 3seconds after moving stops
//    0 = init beeper regs
//    1 = maintain beeper and advance when ready to start
//
//  * start
//    2 = start calib
//    3 = find baseline avg value(beep if deltas are noticed during)
//
//  * store & end
//    4 = store calibs to accel (TODO: gyro needs to use PIC eeprom), when done, set LED_mode = 0
//
// if at any time switch is pressed again, set LED_mode++
// if user touches or moves unit during calib, restart test
// does not honor LATD#5 for debug pulse
void main_calib(void) {
	static unsigned char temp;
	static unsigned char temp2;
	static unsigned char num_calib;
	unsigned char pk;

	// if user pressed mode button to calib mode, but we don't have good data, skip
	if (b.valid == FALSE) {
		LED_mode = 2;	// can't do this until we have good data, jump over this mode until the data is valid
		buzzer_on(BUZZER_TIME(200));	// give some confirmation 
		return;
	}

	if (main_calib_stage == 0) {
		// init beeping and vars needed for calib cycle
		temp = 0;
		temp2 = 0;
		num_calib = NUM_CALIB_CYCLES;
		LATD = 0xff;
		(void)handle_buzzer_event(BUZZER_TIME(200),2);	// for ~1sec of buzzes
		temp = 3;	// sets up initial beeping waiting time
		main_calib_stage = 1;	// advance 

	} else if (main_calib_stage == 1) {
		// check for buzzer timer fire, turn off buzzer
		// here is where we handle the buzzer instead of being handled in main()
		if (PIR2bits.TMR3IF == 1) {
			pk = handle_buzzer_event(0xFFFF,0x80);	// timer fired, do whatever...
			if (pk == 0) {
				// if reached countdown, bounce to next calib stage
				if (temp == 0) {
					LATD = 0;
					main_calib_stage = 2;
					// just in case
					BUZZ_OFF;
					// it is safe to clear these here as we _are_ going into calib cycle
					CALxF = CALyF = CALzF = 0;
					CALgF = 0;	// we clear this one before starting cal
				} else {
					// otherwise, flash&beep some more
					--temp;
					LATD = LATD ^ 0xff;	// flash
					(void)handle_buzzer_event(BUZZER_TIME(200),2);
				}
			}
		}

	} else if (main_calib_stage == 2) {
		LATD = 0x55;
		clear_buffers();
#ifdef ALLOW_EEPROM
		invalidate_eeprom_calib();
#endif
		completed_calib = FALSE;
#if 0
			// not certain why.... but sometimes in calib it would think there
			// was activity b/c gyro AgS would be 11d at the end but it appeared 
			// this was b/c we seeing data from the previous Ag buffer fill...
			// even though we clear them between cal scans... setting this
			// to one less than the buffer size fixes it !!!????!
		temp = SZ_AVG-1;	// whenever we fill up the moving avg window, grab one avg reading
#else
		temp = SZ_AVG;
#endif
		temp2 = CAL_SZ+1;	// take this many avg samples during calibration measurements, skip first b/c gyro not ready (12ms)
		main_calib_stage = 3;
		CALx = CALy = CALz = 0;
		CALg = 0;

	} else if (main_calib_stage == 3) {
		// now sit here counting down samples
		// complain if we get moved, be more picky, user should _NOT_ touch during calib
		if (b.valid && b.motion_any) {
			main_calib_stage = 0;
			return;								// RETURN: restart calib
		}
		if (new_accel) {
			--temp;
			if (temp == 0) {
				LATD = LATD ^ 0xFFFF;	// flash 0x55, 0xaa...
				temp = SZ_AVG;	// reset so after we fill moving avg window again, we take another sample
				--temp2;
				// skip the first reading since gyro won't be ready with
				// a valid avg (otherwise would report as 0)
				// first time thru temp2=CAL_SZ
				if (temp2 != CAL_SZ) {
					CALx += (signed int)b.SYS_AVGx;
					CALy += (signed int)b.SYS_AVGy;
					CALz += (signed int)b.SYS_AVGz;
					CALg += (unsigned long)b.SYS_AVGg;
				}
				if (temp2 == 0) {
					main_calib_stage = 4;
				}
			}
		}
	} else if (main_calib_stage == 4) {
		// all done, store the calib values and return to normal mode
		// TODO: can make these faster by using shift and OR'ing in
		//       upper bits that shifted in 0's (if it was negative to start with).
		//       multiply by -1 to find the value that would need to be added to a
		//       measurement to make it zero (or +64 = 1g for CALz)
		CALx = -1 * (CALx / CAL_SZ);
		CALy = -1 * (CALy / CAL_SZ);
		CALz = 64 - (CALz / CAL_SZ);
		CALg = CALg >> CAL_SZ_SHIFT;	// unsigned
		if (num_calib == NUM_CALIB_CYCLES) {
			// first cycle thru, so CAL{xyzg}F are not init
			CALxF = CALx;
			CALyF = CALy;
			CALzF = CALz;
			CALgF = CALg;
		} else {
			CALxF += CALx;
			CALyF += CALy;
			CALzF += CALz;
			CALgF -= CALg;
		}

		write_accel_calibs();
		did_calib = TRUE;
		--num_calib;
		if (num_calib == 0) {
			// done with all the iterative cycles
			LATD = 0;
			LED_mode = 0;		// back to normal RC mode
			main_calib_stage = 0;
			completed_calib = TRUE;
			// if z is positive, the sensors are facing up
			b.sensors_up = (b.SYS_AVGz > 0);
			(void)handle_buzzer_event(BUZZER_TIME(200),2);
#ifdef ALLOW_EEPROM
			write_calib_2_eeprom();
#endif
		} else {
			main_calib_stage = 2;	// go back and run it again now that we've stored the offset values
		}
	} else {
		fatal(6);
	}
}


//========================================================================
// EEPROM api
//========================================================================
#ifdef ALLOW_EEPROM
// EEPROM fcns
void write_calib_2_eeprom(void) {
	eewrite(EEA_CALIB_XH,((unsigned int)CALxF & 0xff00) >> 8);
	eewrite(EEA_CALIB_XL,(unsigned int)CALxF & 0x00ff);
	eewrite(EEA_CALIB_YH,((unsigned int)CALyF & 0xff00) >> 8);
	eewrite(EEA_CALIB_YL,(unsigned int)CALyF & 0x00ff);
	eewrite(EEA_CALIB_ZH,((unsigned int)CALzF & 0xff00) >> 8);
	eewrite(EEA_CALIB_ZL,(unsigned int)CALzF & 0x00ff);
	eewrite(EEA_CALIB_GHU,((unsigned long)CALgF & 0xff000000) >> 24);
	eewrite(EEA_CALIB_GLU,((unsigned long)CALgF & 0x00ff0000) >> 16);
	eewrite(EEA_CALIB_GHL,((unsigned long)CALgF & 0x0000ff00) >> 8);
	eewrite(EEA_CALIB_GLL,((unsigned long)CALgF & 0x000000ff));
	eewrite(EEA_CALIB_OK,1);
}
// EE[addr] = high byte
// EE[addr+1] = low byte
unsigned int read_eeprom_int(unsigned char addr) {
	unsigned int x1,x2;
	x1 = (unsigned int)eeread(addr);
	x2 = (unsigned int)eeread(addr+1);
	return(((x1<<8)&0xff00) | (0x00ff & x2));
}
unsigned char read_eeprom_calib(void) {
	unsigned long t1, t2;
	if (eeread(EEA_CALIB_OK) == 1) {
		CALxF = (signed int)read_eeprom_int(EEA_CALIB_XH);
		CALyF = (signed int)read_eeprom_int(EEA_CALIB_YH);
		CALzF = (signed int)read_eeprom_int(EEA_CALIB_ZH);
		t1 = (unsigned long)read_eeprom_int(EEA_CALIB_GHU);
		t2 = (unsigned long)read_eeprom_int(EEA_CALIB_GHL);
		CALgF = (signed long)(((t1 << 16)&0xffff0000) | (0x0000ffff & t2));
		return 0;
	} else {
		return 1;
	}
}
void invalidate_eeprom_calib(void) {
	eewrite(EEA_CALIB_OK,0);
	eewrite(EEA_CALIB_XH,0);
	eewrite(EEA_CALIB_XL,0);
	eewrite(EEA_CALIB_YH,0);
	eewrite(EEA_CALIB_YL,0);
	eewrite(EEA_CALIB_ZH,0);
	eewrite(EEA_CALIB_ZL,0);
	eewrite(EEA_CALIB_GHU,0);
	eewrite(EEA_CALIB_GLU,0);
	eewrite(EEA_CALIB_GHL,0);
	eewrite(EEA_CALIB_GLL,0);
}


//========================================================================
// EEPROM read/write
//========================================================================
void eewrite(unsigned char address, unsigned char databyte)
{ // writes the "databyte" value to EEPROM at the address given
  // location in "address".
    EECON1bits.EEPGD = 0;   // Set to access EEPROM memory
    EECON1bits.CFGS = 0;    // Do not access Config registers

    EEDATA = databyte;      // Load EEDATA with byte to be written
    EEADR = address;        // Load EEADR with address of location to write.

    EECON1bits.WREN = 1;    // Enable writing
    
    // NOTE: as a protective measure to prevent accidental writes, the following
    // sequence must be completed without interruption to write a byte to EEPROM.
    // Therefore, if interrupts are used they should be disabled during the sequence
    // as it would not work if an interrupt was received during the sequence.

    //INTCONbits.GIE = 0;   // Disable interrupts
    EECON2 = 0x55;          // Begin Write sequence
    EECON2 = 0xAA;
    EECON1bits.WR = 1;      // Set WR bit to begin EEPROM write
    //INTCONbits.GIE = 1;   // re-enable interrupts
    
    while (EECON1bits.WR == 1)
    {   // wait for write to complete.  WR bit will be cleared when write finishes
        // EEPROM write completion will also set the EEIF flag bit in PIR2, which
        // may be used to generate an interrupt when the EEPROM write completes.
    };

    EECON1bits.WREN = 0;    // Disable writing as a precaution.
}

unsigned char eeread(unsigned char address)
{ // reads and returns the EEPROM byte value at the address given
  // given in "address".
	unsigned char t;

    EECON1bits.EEPGD = 0;   // Set to access EEPROM memory
    EECON1bits.CFGS = 0;    // Do not access Config registers

    EEADR = address;        // Load EEADR with address of location to write.

    // execute the read
    EECON1bits.RD = 1;      // Set the RD bit to execute the EEPROM read

    // The value read is ready the next instruction cycle in EEDATA.  No wait is
    // needed.
	NOP;
	NOP;
	NOP;
	NOP;

	t = EEDATA;
    return t;
}
#endif	// ALLOW_EEPROM






#if 0
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
//========================================================================
// Math helpers
//========================================================================
// convert s10 -> s16
// pass in signed 10bit as unsigned int so we can deal without any signed funniness
signed int cnvt_s10_2_s16(unsigned int s10) {
	signed int s16;
	if ((s10 & 0xfc00) > 0) {
		fatal(9);
	}
	if ((s10 & 0x0200) > 0) {
		// NEGATIVE
		s16 = (((s10 ^ 0x03ff) + 1) ^ 0xffff) + 1;
	} else {
		// POSITIVE
		s16 = (signed int)(s10 & 0x01ff);
	}
	return (s16);
}
// convert s16 -> s10
// assumes s16 info will fit into s10, should in this app where we are just storing 10bit
// values in 16bit vars
unsigned int cnvt_s16_2_s10(unsigned int s16) {
	unsigned int s10;
	if ((s16 & 0x8000) > 0) {
		// NEGATIVE
		s10 = (s16 ^ 0xffff) + 1;	// get positive from negative 16bit twos
		// 10bit twos from 511 -> -512
		// if there are any bits set that would be outside range for 10bit signed, fail
		if (s10 > 512) {
			fatal(8);
		}
		s10 = (s10 ^ 0x3ff) + 1;	// convert to 10bit
	} else {
		// POSITIVE
		// if there is any bits set that would be outside range for 10bit, fail
		if ((s16 & 0x7e00) > 0) {
			fatal(11);
		}
		s10 = (signed int)(s16 & 0x01ff);
	}
	return (s10);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
#endif	// 0

