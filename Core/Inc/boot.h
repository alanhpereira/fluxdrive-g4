#ifndef BOOT_H
#define BOOT_H

/* Reboot, plain or into the ST system bootloader (USB DFU on PA11/PA12, also USART/I2C/SPI/FDCAN, AN2606).
 * The DFU request survives the reset in TAMP backup register 0 */

/* first thing in main(), on the reset clock with nothing set up: jumps to the bootloader if requested */
void boot_check(void);

/* drops the USB connection so the host sees the device leave, then resets. Does not return.
 * The caller makes sure the motor is off and the reply has been queued */
void boot_reboot(int dfu);

#endif
