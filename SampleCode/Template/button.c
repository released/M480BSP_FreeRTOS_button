/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include "NuMicro.h"

#include "FreeRTOS.h"
#include "task.h"

#include "button.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/

/*_____ D E F I N I T I O N S ______________________________________________*/


#define BTN1_GPIO_Port              (PORTG)
#define BTN1_Pin                    (15)

#define BTN2_GPIO_Port              (PORTF)
#define BTN2_Pin                    (11)

BTN_STRUCT	BTN1_short ={BTN1_GPIO_Port,	
    					BTN1_Pin,
						BTN_ACTIVE,			                  // Active state
						50,		                          // Time the button has to be in 'Active state' to be recognized as pressed
						50 };	
BTN_STRUCT	BTN1_long = { BTN1_GPIO_Port,
						BTN1_Pin,
						BTN_ACTIVE,			                  // Active state
						2000,		                          // Time the button has to be in 'Active state' to be recognized as pressed
						50	 } ;

BTN_STRUCT	BTN2_200mS ={BTN2_GPIO_Port,	
    					BTN2_Pin,
						BTN_ACTIVE,			                  // Active state
						200,		                          // Time the button has to be in 'Active state' to be recognized as pressed
						50 };	
BTN_STRUCT	BTN2_3S = { BTN2_GPIO_Port,
						BTN2_Pin,
						BTN_ACTIVE,			                  // Active state
						3000,		                          // Time the button has to be in 'Active state' to be recognized as pressed
						50	 } ;

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
extern uint32_t get_tick3(void);

void io_getDigital(GPIO_TypeDef GPIOx, uint16_t GPIO_Pin,uint8_t *pu8Value)
{	
	*pu8Value = GPIO_PIN_DATA(GPIOx,GPIO_Pin);
}

void time_setTimerCount(TIMER_TYPE *pu8timer,uint32_t u32timeToCount)  
{
    TIMER_Start(TIMER3);

	if(pu8timer->on == 0)
		pu8timer->on = 1;
	if(pu8timer->on == 1)
		pu8timer->timeInit = get_tick3();
	pu8timer->timeOut = 0;
	pu8timer->timeToCount = u32timeToCount;
}

TIMER_STATE time_getTimeOut(TIMER_TYPE *pu8timer)
{
	uint32_t Temp_Val;
	if(get_tick3() > pu8timer->timeInit)
		Temp_Val = get_tick3() - pu8timer->timeInit;
	else
		Temp_Val = (0xFFFFFFFF-pu8timer->timeInit)+get_tick3();
	if(Temp_Val >= pu8timer->timeToCount)
	{
		pu8timer->timeOut = 1;
		pu8timer->on = 0;
		pu8timer->timeToCount = 0;
		pu8timer->timeInit = 0;
	}	
	else 
		pu8timer->timeOut = 0;
	return (pu8timer->timeOut == 1) ? TIME_OUT : TIME_OK;
}

BTN_STATE btn_getState(BTN_STRUCT *pBtn)
{
    const uint8_t transition_table[8][4]={	0,	1,	0,	1,
										5,	2,	5,	1,
										7,	2,	5,	3,
										5,	4,	5,	4,
										5,	4,	5,	4,
										6,	1,	0,	1,
										6,	1,	7,	1,
										0,	1,	0,	1 };
	
	//register uint8_t u8Input;
	uint8_t u8Input;
	// Get button state
	io_getDigital(pBtn->GPIO_Port,pBtn->GPIO_Pin ,&u8Input);
	u8Input = (u8Input == pBtn->u8ActiveState)?1:0;
	
	// Get timeout state
	u8Input |= ((time_getTimeOut(&(pBtn->tTimer))==TIME_OUT)?2:0);

	// Get new state
	pBtn->u8State = transition_table[pBtn->u8State][u8Input]; // we want only the state, not action

	// Perform action 
	switch (pBtn->u8State)
	{
		case 1:
			time_setTimerCount(&(pBtn->tTimer), pBtn->u16TimeOutON);
			break;
		case 5:
			time_setTimerCount(&(pBtn->tTimer), pBtn->u16TimeOutOFF);
			break;
	}
	// return pBtn->u8State;
	return (BTN_STATE)pBtn->u8State;	
} 

void btn_task(void)
{
    // under loop
    if(btn_getState(&BTN1_short) == BTN_PRESSED)
    {
        taskENTER_CRITICAL();
        printf( "BTN1_short\r\n");          
        taskEXIT_CRITICAL();
    }//BTN_EDGE2 : pressed then released

    if(btn_getState(&BTN1_long) == BTN_PRESSED)
    {
        taskENTER_CRITICAL();
        printf( "BTN1_long\r\n");          
        taskEXIT_CRITICAL();
    } //BTN_PRESSED : pressed and hold

    if(btn_getState(&BTN2_3S) == BTN_PRESSED)
    {
        taskENTER_CRITICAL();
        printf( "BTN2_3S\r\n");          
        taskEXIT_CRITICAL();

        while(btn_getState(&BTN2_200mS));// prevent pressed long time then release to trigger short release operation
    }
}

void btn_Init(void)
{
	SYS->GPG_MFPH = (SYS->GPG_MFPH & ~(SYS_GPG_MFPH_PG15MFP_Msk)) | (SYS_GPG_MFPH_PG15MFP_GPIO);
	SYS->GPF_MFPH = (SYS->GPF_MFPH & ~(SYS_GPF_MFPH_PF11MFP_Msk)) | (SYS_GPF_MFPH_PF11MFP_GPIO);

	GPIO_SetMode(PG,BIT15,GPIO_MODE_INPUT);
	GPIO_SetMode(PF,BIT11,GPIO_MODE_INPUT);
	
}


