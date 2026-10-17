#include "boot.h"

#include "main.h"
#include "usbd_core.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

#define BOOT_MAGIC_DFU 0xDF00B007u
#define SYSMEM_BASE 0x1FFF0000u /* system memory: vector table of the ST bootloader */

void boot_check(void) {
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_RCC_RTCAPB_CLK_ENABLE(); /* TAMP registers */
	if (TAMP->BKP0R != BOOT_MAGIC_DFU) return;
	PWR->CR1 |= PWR_CR1_DBP;
	TAMP->BKP0R = 0; /* one shot: the next reset boots the firmware */
	PWR->CR1 &= ~PWR_CR1_DBP;
	__HAL_RCC_SYSCFG_CLK_ENABLE();
	__HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();
	SCB->VTOR = SYSMEM_BASE;
	const uint32_t sp = ((const uint32_t*)SYSMEM_BASE)[0], pc = ((const uint32_t*)SYSMEM_BASE)[1];
	/* in asm: after the MSP changes, nothing may be read from the old stack */
	__asm volatile(
		"msr msp, %0\n"
		"bx %1\n" ::"r"(sp),
		"r"(pc));
	for (;;) {
	}
}

void boot_reboot(int dfu) {
	HAL_Delay(20); /* the reply leaves: UART DMA ~0.4 ms, USB queue <= 2 KB */
	HAL_NVIC_DisableIRQ(USB_LP_IRQn);
	USBD_Stop(&hUsbDeviceFS); /* D+ pull-up off: the host drops the port now, not on a timeout */
	HAL_Delay(50);
	if (dfu) {
		PWR->CR1 |= PWR_CR1_DBP;
		TAMP->BKP0R = BOOT_MAGIC_DFU;
	}
	NVIC_SystemReset();
}
