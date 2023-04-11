#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <x_axis.h>
#include <y_axis.h>
#include <z_axis.h>

#include "FreeRTOS.h"
#include "task.h"

/* LWIP and ENET phy */
#include "arch/lpc18xx_43xx_emac.h"
#include "arch/lpc_arch.h"
#include "arch/sys_arch.h"							  
#include "lpc_phy.h" /* For the PHY monitor support */

/* LPC */
#include "board.h"

#include "debug.h"
#include "temperature.h"
#include "temperature_ds18b20.h"
#include "relay.h"
#include "lwip_init.h"
#include "settings.h"
#include "lwip/ip_addr.h"
#include "tcp_server.h"
#include "mem_check.h"
#include "encoders.h"
#include "x_zs.h"

extern struct gpio_entry relay_1;

/* GPa 201117 1850 Iss2: agregado de Heap_4.c*/
uint8_t __attribute__((section ("." "data" ".$" "RamLoc40"))) ucHeap[configTOTAL_HEAP_SIZE];

/* Sets up system hardware */
static void prvSetupHardware(void)
{
#if defined(__FPU_PRESENT) && __FPU_PRESENT == 1
	fpuInit();
#endif

	Board_SystemInit();
	SystemCoreClockUpdate();

	// Without the following command the use of DWT makes debugging impossible
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	// Enable Cycle Counter, used to create precision delays in wait.c
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	debugInit();

	Board_Init();
	settings_init();
	settings_erase();
	relay_init();
	x_zs_init();
	x_axis_init();
	//y_axis_init();
	z_axis_init();

	//temperature_init();
	//temperature_ds18b20_init();
	encoders_init();
	mem_check_init();


}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	MilliSecond delay function based on FreeRTOS
 * @param	ms	: Number of milliSeconds to delay
 * @returns	nothing
 * Needed for some functions, do not use prior to FreeRTOS running
 */
void msDelay(uint32_t ms)
{
	vTaskDelay((configTICK_RATE_HZ * ms) / 1000);
}

/**
 * @brief	main routine for example_lwip_tcpec+ho_freertos_18xx43xx
 * @returns	function should not exit
 */
int main(void)
{	debugLocalSetLevel(Info);
	debugNetSetLevel(Info);

	prvSetupHardware();

	/* Task - Ethernet PHY Initialization  */
	xTaskCreate(vStackIpSetup, "StackIpSetup",
	configMINIMAL_STACK_SIZE * 4, NULL, (tskIDLE_PRIORITY + 1UL),
			(xTaskHandle*) NULL);

	/* Start the scheduler itself. */
	vTaskStartScheduler();

	return 0;
}

#if (configCHECK_FOR_STACK_OVERFLOW > 0)
void vApplicationStackOverflowHook( xTaskHandle *pxTask,
		signed char *pcTaskName)
{
	volatile signed char *name;
	volatile xTaskHandle *pxT;

	name = pcTaskName;
	pxT = pxTask;

	(void) name;
	(void) pxT;

	while (1)
		;
}
#endif

/*-----------------------------------------------------------*/
/**
 * @brief	configASSERT callback function
 * @param 	ulLine		: line where configASSERT was called
 * @param 	pcFileName	: file where configASSERT was called
 */
void vAssertCalled(unsigned long ulLine, const char *const pcFileName)
{
	volatile uint32_t ulSetToNonZeroInDebuggerToContinue = 0;

	taskENTER_CRITICAL();
	{
		printf("[ASSERT] %s:%lu\n", pcFileName, ulLine);
		/* You can step out of this function to debug the assertion by using
		 the debugger to set ulSetToNonZeroInDebuggerToContinue to a non-zero
		 value. */
		while (ulSetToNonZeroInDebuggerToContinue == 0) {
		}
	}
	taskEXIT_CRITICAL();
}
/*-----------------------------------------------------------*/

void prvGetRegistersFromStack(uint32_t *pulFaultStackAddress)
{
	/* These are volatile to try and prevent the compiler/linker optimising them
	 away as the variables never actually get used.  If the debugger won't show the
	 values of the variables, make them global my moving their declaration outside
	 of this function. */
	volatile uint32_t r0;
	volatile uint32_t r1;
	volatile uint32_t r2;
	volatile uint32_t r3;
	volatile uint32_t r12;
	volatile uint32_t lr; /* Link register. */
	volatile uint32_t pc; /* Program counter. */
	volatile uint32_t psr;/* Program status register. */

	r0 = pulFaultStackAddress[0];
	r1 = pulFaultStackAddress[1];
	r2 = pulFaultStackAddress[2];
	r3 = pulFaultStackAddress[3];

	r12 = pulFaultStackAddress[4];
	lr = pulFaultStackAddress[5];
	pc = pulFaultStackAddress[6];
	psr = pulFaultStackAddress[7];

	(void) r0;
	(void) r1;
	(void) r2;
	(void) r3;
	(void) r12;
	(void) lr;
	(void) pc;
	(void) psr;

	/* When the following line is hit, the variables contain the register values. */
	for (;;)
		;
}



