/*
 * 7135x8 2.8A Nanjg 105C LED Driver
 *
 * Created: 23.01.2012 20:52:39
 * Last modified: 26.07.2013 02:57:00
 *
 * FIRMWARE VERSION: 2.6.5
 *
 * Fcpu = 4.8MHz
 *
 * This code is distributed under the GNU Public License
 * which can be found at http://www.gnu.org/licenses/gpl.txt
 *
 *  Author: Tamagotchi http://tamagotchi-007.livejournal.com/
 *                     http://avr.tamatronix.com
 *  Optimization: DooMmen http://doommen.blogspot.com/
 *				  vdavid http://forum.fonarevka.ru/member.php?u=20922
 *
 *  Download current version: http://109.87.62.61/uploads/LD_7135_v_2.6.5.zip
 *
 */

#define F_CPU 4800000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

#define MAX_MODES				3			// �� 0 �� 3 �������� 4 ������
#define START_MODE				0			// ����� ������, � �������� �������� ������� (������� �� REVERSE_SW)
#define REVERSE_SW							// ������������� ������� �� �������� �� �������� DOWNTOUP
//#define NOMEM								// ������� ��� ������.
#define WEAR_LEWELING						// ���������� ��������� ������� ������
#define MODE_SET_LEVEL						// ������������� ������� � ���. �������
#define	RST_LEV_MOON						// ����� ������� ��� ������ �� �������� (� ������ NOMEM - ����������� �������)
#define	FLASH_OFF							// ���������� ��������� ��������� (������ �������������)
//#define FREQMOON_OFF						// ��������� ��������� ������� � �������� (�������� ������� ������� ��� ���-�� �����������)
#define ONECLICKMOON						// ����� �� MODELINECHG �� ���������� �����
//#define CAPACITOR							// ����������� �� reset (��������������� ���� ���)
//#define THERMVD								// ��c�������������� ���� ����������� �������������� ����� ����

//#define TURBO_TIME 				120		    // ���������� ������������� ������ �� ������� � �������� MAX=500 (��������� �������������)

//#define THEMPERATURE_CONTROL				// �������� �������������
#define CALIBRATE_CLICKS		12			// ���������� ������ ��� ����� � ����� ���������������

#define BATTERY_CLICKS			4			// ���������� ������ ��� ����� � ����� ��������� �������

#define MODELINECHG
#define MODELINECHG_CLICKS		3

#define RAMPING
#define RAMPING_CLICKS			8

#define POLICE_MODE
#define POLICE_MODE_CLICKS		5			//

#define SLOW_PULSE_MODE
#define SLOW_PULSE_MODE_CLICKS	6			//

#define SOS_MODE
#define SOS_MODE_CLICKS			7			//

//#define PULSE_MODE
#define PULSE_MODE_CLICKS		8			//


//#define ALPINE_MODE
#define ALPINE_MODE_CLICKS		8			//


//#define VELO_STROBE
#define VELO_STROBE_CLICKS		8			//


#ifdef	NOMEM
	#define	RST_LEV_MOON
#endif

#ifdef TURBO_TIME
	#undef	THEMPERATURE_CONTROL
	#if ((TURBO_TIME<16) || (TURBO_TIME>500))
	//	#undef	TURBO_TIME
		#define	TURBO_TIME 120
		#warning "Invalid value for TURBO_TIME. Set default value is 120"
	#endif
#endif

/*��������� ������ ������� � ������� (����������� ��� MODE_SET_LEVEL)*/
#define PulseMod		2
#define SlowPulseMod	1
#define AlpineMod		2
#define PoliceMod		2
#define SOSMod			2
/*��������� ������ ������� � ������� (�� ����������� ��� MODE_SET_LEVEL)*/
#define VeloMod			0
#define VeloPulse		MAX_MODES
/*����� ����� � ���� ������*/
#define	VeloOFF			2					// 1 = 1 sec


/*����������� ����� ��� ���������� ������*/
#define PULSE_ON			125				// 1 = 2mc
#define PULSE_OFF			1				// 1 = 1 sec

#define CFG_CURRENTMODE		0x3A			// ������� ����� ������ ������
#define CFG_MOONMODE		0x3B			// ���� ���������� ���������� ����� 0- ���������� 1- �������
#define CFG_CALIBRATE		0x3D			// ���� ���������� ����������� ����������
#define CFG_RAMPING			0x3F			// ���� ���������� �������� ��������

//������ ���������� ���������
//val = ((V_bat - V_diode) * R2 * 255) / ((R1 + R2) * V_ref)
//V_diode = 0.28V; R1 = 19.1k; R2 = 4.7k
#define U1					0x98			// 3.6
#define U2 					0xA0			// 3.7
#define U3					0xA4			// 3.8
#define U4					0xAB			// 3.95
#define OFF_VOLTAGE			0x7D			// 2.90V ;3.00V = 7F

#define PB_CONFIG			0x30;			// PB4, PB5 ����������� � +
#define DDRB_CONFIG			0x0B;			// PB0, PB1, PB3 - ������

//#define MoonMod	do {leds_off(); set_moon_mode();} while (0);
#define Mod0001	do {leds_off(); OCR0A  = 0x01;} while (0);
#define Mod0020	do {leds_off(); OCR0A  = 0x12;} while (0);
#define Mod0035	do {leds_off(); OCR0A  = 0x20;} while (0);
#define Mod0050	do {leds_off(); OCR0A  = 0x40;} while (0);
#define Mod0075	do {leds_off(); OCR0A  = 0x55;} while (0);
#define Mod0130	do {leds_off(); OCR0A  = 0x64;} while (0);
#define Mod0175	do {leds_off(); OCR0A  = 0x80;} while (0);
#define Mod0350	do {leds_off(); OCR0A  = 0xFF;} while (0);
#define Mod0700	do {LED_PORT = LED_MASK_2 | PB_CONFIG; OCR0A  = 0x00;} while (0);
#define Mod1050	do {LED_PORT = LED_MASK_2 | PB_CONFIG; OCR0A  = 0xFF;} while (0);
#define Mod1750	do {LED_PORT = LED_MASK_5 | PB_CONFIG; OCR0A  = 0x00;} while (0);
#define Mod2100	do {LED_PORT = LED_MASK_5 | PB_CONFIG; OCR0A  = 0xFF;} while (0);
#define Mod2450	do {LED_PORT = LED_MASK_5 | LED_MASK_2 | PB_CONFIG; OCR0A  = 0x00;} while (0);
#define Mod2800	do {LED_PORT = LED_MASK_5 | LED_MASK_2 | PB_CONFIG; OCR0A  = 0xFF;} while (0);

#ifdef REVERSE_SW
#define SetMod0()	Mod0075
#define SetMod1()	Mod0350
#define SetMod2()	Mod1050
#define SetMod3()	Mod2800
#define SetMod4()	Mod2800		// 5-� �����
#else
#define SetMod0()	Mod2800
#define SetMod1()	Mod1050
#define SetMod2()	Mod0350
#define SetMod3()	Mod0075
#define SetMod4()	Mod0075		// 5-� �����
#endif

#define LED_PORT		PORTB
#define LED_MASK_5		(1<<1)				// ������ �� 5-�� 7135
#define LED_MASK_2		(1<<3)				// ������ �� 2-�  7135

#define EEMPE		0x02
#define EEPE		0x01

#define WDTO_120MS	0x03
#define WDTO_250MS	0x04
#define WDTO_500MS	0x05
#define WDTO_1S		0x06
#define WDTO_2S		0x07
#define WDTO_4S		0x08
#define WDTO_8S		0x09
#define wdt_reset() asm volatile("wdr")

#define SIZE				0x20
#define	CMODE				0x00
#define CMODE_ADR			CMODE * SIZE

//#define all_off()	do {LED_PORT = PB_CONFIG; OCR0A  = 0x00;} while (0); 	// ��������� ������ ���
#define leds_off()	do {LED_PORT = PB_CONFIG;} while (0); 					// ��������� ��� �� ��� ������
#define adc_off()	do {ADCSRA &=~ (1<<ADIE | (1<<ADSC));} while (0);		// ��������� ����������, ���������
#define adc_on()	do {ADCSRA |= (1<<ADIE | (1<<ADSC));} while (0);		// ��������� ����������, ���������

#define OS_MAIN		__attribute__((OS_main))
#define INLINE		inline __attribute__((__always_inline__))
#define NOINLINE	__attribute__((__noinline__))


/* ���������� ������� ������ */
#define SETBIT(x,y)		(x |= (y))		/* ��������� ���� y � ����� x	*/
#define CLEARBIT(x,y)	(x &= (~y))		/* ����� ���� y � ����� x		*/
#define CHECKBIT(x,y)	(x & (y))		/* �������� ���� y � ����� x	*/

/* ���������� ����� */
#define FLAG_MEASURE	_BV(0)			/* ������ 0 : */
#define FLAG_NEW_MOD	_BV(1)			/* ������ 1 : */
#define FLAG_OFF_MOD	_BV(2)			/* ������ 2 : */
//		00000000		flags register
//		     |||
//		     |||______ measure_mode ���� 1 - �������� ����������� ���� 0- �������
//		     ||_______ newmode		���� 1 - ���� �������� ���������� �������, ���� 0 - �������
//		     |________ offmode		���� 1 - ����������� ��������

#define ClearFlags()		flag_reg = 0

#define SetMeasureThemp()	SETBIT(flag_reg, FLAG_MEASURE)
#define SetMeasureVolt()	CLEARBIT(flag_reg, FLAG_MEASURE)
#define IsMeasureThemp()	CHECKBIT(flag_reg, FLAG_MEASURE)
#define InvMeasureMode()	flag_reg ^= FLAG_MEASURE

#define SetNewMode()		SETBIT(flag_reg, FLAG_NEW_MOD)
#define ClearNewMode()		CLEARBIT(flag_reg, FLAG_NEW_MOD)
#define IsNewMode()			CHECKBIT(flag_reg, FLAG_NEW_MOD)

#define SetOffMode()		SETBIT(flag_reg, FLAG_OFF_MOD)
#define ClearOffMode()		CLEARBIT(flag_reg, FLAG_OFF_MOD)
#define IsOffMode()			CHECKBIT(flag_reg, FLAG_OFF_MOD)
