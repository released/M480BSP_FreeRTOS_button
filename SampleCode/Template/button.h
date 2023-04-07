/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include "NuMicro.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/

/*_____ D E F I N I T I O N S ______________________________________________*/

/*
					  Timeout ON
                _______|_____
 P             |             |		  Timeout OFF
 R  ___________|             |________|____
		^      ^  ^    ^  ^  ^   ^    ^
 S		0	   1  2    3  4  5	 6    7

P - pressed, R - released, S - BTN_STATE
*/

#define BTN_ACTIVE                          0		                //when pressed, switch to GND

typedef enum 
{
	TIME_OK = 0,
	TIME_OUT
} TIMER_STATE;

typedef struct
{
    uint32_t timeInit;
    uint32_t timeToCount;
    uint8_t on;
    uint8_t timeOut;
} TIMER_TYPE;

typedef enum 
{
	PORTA = 0,
	PORTB ,
	PORTC ,
	PORTD ,
	PORTE ,
	PORTF ,
	PORTG ,
	PORTH
} GPIO_TypeDef;

typedef struct
{
	// Public 

	GPIO_TypeDef    GPIO_Port;
	uint16_t        GPIO_Pin;
	uint8_t		    u8ActiveState;	// button is pressed if (io_getDigital(u8Button)==bActiveState)
	uint16_t		u16TimeOutON;	// time the button has to be pressed to be recognized as pressed
	uint16_t		u16TimeOutOFF;	// time the button has to be pressed to be recognized as released
	
	// Private
	TIMER_TYPE	tTimer;
 	uint8_t		u8State;

} BTN_STRUCT;

typedef enum
{
	BTN_IDLE = 0,
	BTN_EDGE1,
	BTN_TRIGGERED,
	BTN_PRESSED,	//< most important
	BTN_PRESS_HOLD,
	BTN_EDGE2,
	BTN_RELEASE_HOLD,
	BTN_RELEASED	
} BTN_STATE;


BTN_STATE btn_getState(BTN_STRUCT *pBtn);
void time_setTimerCount(TIMER_TYPE *pu8timer,uint32_t u32timeToCount); 
void io_getDigital(GPIO_TypeDef GPIOx, uint16_t GPIO_Pin,uint8_t *pu8Value);
TIMER_STATE time_getTimeOut(TIMER_TYPE *pu8timer);

void btn_task(void);
void btn_Init(void);

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/

