/*
 * PCA9685.c
 *
 *  Created on: Sep 10, 2024
 *      Author: KU
 */
#include "PCA9685.h"

PCA9685_status PCA9685_set_bit(PCA9685_Handler* pca, uint8_t register_addr, uint8_t bit, uint8_t value){
	uint8_t read_value;
	int ret_read;
	int ret_write;

	/* Read 8 bit from mem and set it to only one bit (0/1) */
	ret_read = pca->read_mem(pca->addr, register_addr, 1, &read_value, 1, 100, NULL);
	if(ret_read != PCA9685_OK){
		return PCA9685_ERROR_READ;
	}

	// Modify the bit
	if(value == 0){
		read_value &= ~(1 << bit);
	}
	else {
		read_value |= (1 << bit);
	}

	/* write all 8 bits to mem back */
	ret_write = pca->write_mem(pca->addr, register_addr, 1, &read_value, 1, 100, NULL);

	if(ret_write != PCA9685_OK) {
		return PCA9685_ERROR_WRITE;
	}
	return PCA9685_OK;
}

PCA9685_status PCA9685_set_PWM_frequency(PCA9685_Handler* pca, uint16_t frequency){
	uint8_t prescale;
	int ret_fre;
	if(frequency >= 1526){
		prescale = 0x03;
	}
	else if(frequency <= 24) {
		prescale = 0xFF;
	}
	// internal 25 MHz oscillator as in the datasheet page no 1/52
	else {
		prescale = (25000000 / (4096 * frequency));
	}

	// prescale changes 3 to 255 for 1526Hz to 24Hz as in the datasheet page no 1/52
	PCA9685_set_bit(pca, PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, 1);
	ret_fre = pca->write_mem(pca->addr, PCA9685_PRE_SCALE, 1, &prescale, 1, 100, NULL);
	PCA9685_set_bit(pca, PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, 0);	// normal sleep mode
	PCA9685_set_bit(pca, PCA9685_MODE1, PCA9685_MODE1_RESTART_BIT, 1);	// enable restart bit

	if(ret_fre != PCA9685_OK){
		return PCA9685_ERROR;
	}

	return PCA9685_OK;
}

void PCA9685_init(PCA9685_Handler* pca, uint16_t frequency){
	PCA9685_set_PWM_frequency(pca, frequency);	// 1 ==> OK
	PCA9685_set_bit(pca, PCA9685_MODE1, PCA9685_MODE1_AI_BIT, 1);
}

PCA9685_status PCA9685_set_PWM_LED(PCA9685_Handler* pca, uint8_t channel, uint16_t on_time, uint16_t off_time){
	uint8_t register_address;
	register_address = PCA9685_LED0_ON_L + (4 * channel);

	// See example 1 in the datasheet page no 18/52
	uint8_t pwm[4];
	pwm[0] = on_time & 0xFF;
	pwm[1] = (on_time>>8);
	pwm[2] = off_time & 0xFF;
	pwm[3] = (off_time>>8);

	int pwm_err = pca->write_mem(pca->addr, register_address, 1, pwm, 4, 100, NULL);
	if(pwm_err != PCA9685_OK){
		return PCA9685_ERROR_WRITE;
	}

	return PCA9685_OK;
}

PCA9685_status PCA9685_LED_on(PCA9685_Handler* pca, uint8_t channel){
	int err = PCA9685_set_PWM_LED(pca, channel, 0, (uint16_t)4095);  // turn on LED (on at 0 and off at 4095)
	if (err != PCA9685_OK){
		return PCA9685_ERROR;
	}
	return PCA9685_OK;
}

PCA9685_status PCA9685_LED_off(PCA9685_Handler* pca, uint8_t channel){
	int err = PCA9685_set_PWM_LED(pca, channel, 0, (uint16_t)0);  // turn on LED (on at 0 and off at 4095)
	if (err != PCA9685_OK){
		return PCA9685_ERROR;
	}
	return PCA9685_OK;
}

void PCA9685_LED_toggle(PCA9685_Handler* pca, uint8_t channel){
	PCA9685_LED_on(pca, channel);
	PCA9685_LED_off(pca, channel);
}

PCA9685_status PCA9685_LED_brightness(PCA9685_Handler* pca, uint8_t channel, uint16_t brightness){
	// brightness value 0-100%
	float value = (float)((brightness/100.0)*4096.0);
	PCA9685_set_PWM_LED(pca, channel, 0, (uint16_t)value);
	return PCA9685_OK;
}

PCA9685_status PCA9685_LED_dim(PCA9685_Handler* pca, uint8_t channel){
	for (int i = 0; i<=4095; i+=5){
		PCA9685_set_PWM_LED(pca, channel, 0, (uint16_t)i);
	}
	for (int i = 4095; i>=0; i-=5){
		PCA9685_set_PWM_LED(pca, channel, 0, (uint16_t)i);
	}
	return PCA9685_OK;
}
