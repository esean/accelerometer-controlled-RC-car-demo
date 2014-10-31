// PICkit 2 Lesson file header

#ifndef PICKIT_H
#define PICKIT_H

/** D E C L A R A T I O N S **************************************************/
typedef enum {FALSE, TRUE} BOOL;

/** D E F I N I T I O N S ****************************************************/
#define Switch_Pin      PORTBbits.RB0
#define Switch_SteerRev	PORTAbits.RA2
#define DetectsInARow   5

#define GYRO_CS			LATEbits.LATE0
#define ACCEL_CS		LATCbits.LATC0
#define GYRO_ST			LATBbits.LATB4	// gyro self-test IO
#define BUZZER			LATAbits.LATA1	// drives buzzer on
#define BUSYIO			LATEbits.LATE1	// toggles on processing cycles for timing meas.
#define JOKIE			LATAbits.LATA3	// sends ON 200ms pulse to jokie unit
#define DBG_IO			LATDbits.LATD5	// set high during any sample processing
#define DBG_TIMER		LATDbits.LATD6	// toggles on 24ms sampling sequence

#define BREAKPT _asm nop _endasm;
#define NOP		_asm nop _endasm;

/** E X T E R N S ************************************************************/
// declare variables accessible by other files.

/** P R O T O T Y P E S ******************************************************/

void InterruptServiceCompat(void);
void write_accel_calibs(void);
void write_calib_2_eeprom(void);
unsigned int read_eeprom_int(unsigned char);
unsigned char read_eeprom_calib(void);
void invalidate_eeprom_calib(void);
#define EEA_CALIB_OK		0
#define EEA_CALIB_XH		1
#define EEA_CALIB_XL		2
#define EEA_CALIB_YH		3
#define EEA_CALIB_YL		4
#define EEA_CALIB_ZH		5
#define EEA_CALIB_ZL		6
#define EEA_CALIB_GHU		7	// long
#define EEA_CALIB_GLU		8
#define EEA_CALIB_GHL		9
#define EEA_CALIB_GLL		10
void eewrite(unsigned char address, unsigned char databyte);
unsigned char eeread(unsigned char address);

void init_movement_cfg(void);
void send_movement_cfg_232(void);

void Timer0_Init(void);
void ADC_Init(void);
unsigned char ADC_Convert(void);
unsigned char ADC_update(unsigned char);

unsigned int cnvt_s16_2_s10(unsigned int);
signed int cnvt_s10_2_s16(unsigned int);

// AVG
void clear_SdataX(void);
void store_sampleX(void);
signed char get_avgX(void);
void clear_SdataY(void);
void store_sampleY(void);
signed char get_avgY(void);
void clear_SdataZ(void);
void store_sampleZ(void);
signed char get_avgZ(void);
void clear_SdataG(void);
void store_sampleG(void);
signed int get_avgG(void);

// DERIV
void clear_DSdataX(void);
void store_DsampleX(signed char);
signed char get_DavgX(void);
void clear_DSdataY(void);
void store_DsampleY(signed char);
signed char get_DavgY(void);
void clear_DSdataZ(void);
void store_DsampleZ(signed char);
signed char get_DavgZ(void);
void clear_DSdataG(void);
void store_DsampleG(signed int);
signed int get_DavgG(void);

void clear_buffers(void);

void send232_break(void);
void send232(unsigned char);
void send232_char(unsigned char);
void send232_int(unsigned int);
void send232_raw_accel(void);
void send232_avg_accel(void);
void send232_avgderiv_accel(void);
void send232_raw_gyro(void);
void send232_avg_gyro(void);
void send232_avgderiv_gyro(void);

unsigned char dec2ascii(unsigned char);

void SPI_Init(void);
void gyro_Init(void);
void accel_Init(void);
void SPI_reset(void);
void usart_Init(void);
void pwm_init(void);

void read_gyro(void);
void read_accel(void);
void write_accel(unsigned char, unsigned char);
signed char read_one_accel(unsigned char);

void debug_pulse(void);
void fatal(char);
void check_fail(char);
void set_debug_LED(char,char);

void ADC_init(void);
unsigned char ADC_convert(void);

void Timer3_Init(void);
void reset_timer3(void);
void set_timer3_time(unsigned int);
void buzzer_on(unsigned int);
unsigned char handle_buzzer_event(unsigned int,unsigned char);

unsigned char SPI_IN(void);
unsigned char SPI_OUT(unsigned char);

void main_rc_car(void);
void main_calib(void);
void update_brain(void);
void clear_brain(void);

#endif // PICKIT_H
