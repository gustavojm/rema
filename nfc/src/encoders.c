#include <stdint.h>

#include "board.h"
#include "gpio.h"

#include "encoders.h"
#include "mot_pap.h"

extern struct mot_pap x_axis;
extern struct mot_pap y_axis;
extern struct mot_pap z_axis;

/**
* @brief	Handle interrupt from GPIO pin or GPIO pin mapped to PININT
* @return	Nothing
*/
void GPIO5_IRQHandler(void)
{
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(5));
	mot_pap_update_position(&x_axis);
}

void GPIO6_IRQHandler(void)
{
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(6));
    mot_pap_update_position(&y_axis);
}

void GPIO7_IRQHandler(void)
{
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(7));
    mot_pap_update_position(&z_axis);
}

/**
 * @brief	Main program body
 * @return	Does not return
 */
void encoders_init(void) {
	//Chip_Clock_Enable(CLK_MX_GPIO);
	gpio_init_input((struct gpio_entry) {7, 4, (SCU_MODE_INBUFF_EN | SCU_MODE_PULLUP | SCU_MODE_FUNC0), 3, 12});
	gpio_init_input((struct gpio_entry) {7, 5, (SCU_MODE_INBUFF_EN | SCU_MODE_PULLUP | SCU_MODE_FUNC0), 3, 13});
	gpio_init_input((struct gpio_entry) {7, 6, (SCU_MODE_INBUFF_EN | SCU_MODE_PULLUP | SCU_MODE_FUNC0), 3, 14});

	/* Configure interrupt channel for the GPIO pin in SysCon block */
	Chip_SCU_GPIOIntPinSel(5, 3, 12);
	Chip_SCU_GPIOIntPinSel(6, 3, 13);
	Chip_SCU_GPIOIntPinSel(7, 3, 14);

	/* Configure channel interrupt as edge sensitive and falling edge interrupt */
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(5));
	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH(5));
	Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH(5));
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(6));
	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH(6));
	Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH(6));
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(7));
	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH(7));
	Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH(7));

	/* Enable interrupt in the NVIC */
	NVIC_ClearPendingIRQ(PIN_INT5_IRQn);
	NVIC_EnableIRQ(PIN_INT5_IRQn);
	NVIC_ClearPendingIRQ(PIN_INT6_IRQn);
	NVIC_EnableIRQ(PIN_INT6_IRQn);
	NVIC_ClearPendingIRQ(PIN_INT7_IRQn);
	NVIC_EnableIRQ(PIN_INT7_IRQn);

}
