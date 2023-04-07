# M480BSP_FreeRTOS_button
 M480BSP_FreeRTOS_button

update @ 2023/04/07

1. initial FreeRTOS , create Semaphore and button task for test 

2. timer1 will trigger Semaphore flag (SemaphoreTrigger) per 2 sec , and start to get Semaphore from vTask_Semaphore_flow 

3. use PG15 , PF11 as button , to test button pressed short / long

4. below is log capture 

![image](https://github.com/released/M480BSP_FreeRTOS_button/blob/main/log.jpg)	

