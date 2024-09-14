/*
 * PCA9685.h
 *
 *  Created on: Sep 10, 2024
 *      Author: KU
 */

#ifndef INC_PCA9685_H_
#define INC_PCA9685_H_

#include <stdio.h>
#include <stdint.h>

// Datasheet link --> https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf
#define PCA9685_MODE1_SLEEP_BIT      4    // as in the datasheet page no 14/52 (4 is number bit for sleep mode)
#define PCA9685_MODE1_AI_BIT         5    // as in the datasheet page no 14/52 (5 is number bit for mode1 AI)
#define PCA9685_MODE1_RESTART_BIT    7    // as in the datasheet page no 14/52 (7 is number bit for restart)

typedef enum {
	PCA9685_MODE1 = 0x00, 		/**< Mode Register 1 */
	PCA9685_MODE2 = 0x01, 		/**< Mode Register 2 */

	PCA9685_SUBADR1 = 0x02, 	/**< I2C-bus subaddress 1 */
	PCA9685_SUBADR2 = 0x03, 	/**< I2C-bus subaddress 2 */
	PCA9685_SUBADR3 = 0x04, 	/**< I2C-bus subaddress 3 */
	PCA9685_ALL_CALLADR = 0x05, 	/**< LED All Call I2C-bus address */

	PCA9685_LED0_ON_L = 0x06, 	/**< LED0 on tick, low byte*/
	PCA9685_LED0_ON_H = 0x07, 	/**< LED0 on tick, high byte*/
	PCA9685_LED0_OFF_L = 0x08, 	/**< LED0 off tick, low byte */
	PCA9685_LED0_OFF_H = 0x09, 	/**< LED0 off tick, high byte */

	PCA9685_LED1_ON_L = 0xA,
	PCA9685_LED1_ON_H = 0xB,
	PCA9685_LED1_OFF_L = 0xC,
	PCA9685_LED1_OFF_H = 0xD,

	PCA9685_LED2_ON_L = 0xE,
	PCA9685_LED2_ON_H = 0x10,
	PCA9685_LED2_OFF_L = 0x11,
	PCA9685_LED2_OFF_H = 0x12,

	PCA9685_LED3_ON_L = 0x13,
	PCA9685_LED3_ON_H = 0x14,
	PCA9685_LED4_OFF_L = 0x15,
	PCA9685_LED4_OFF_H = 0x16,
	// etc all 16:  LED15_OFF_H 0x45

	PCA9685_ALL_LED_ON_L= 0xFA, 	/**< load all the LEDn_ON registers, low */
	PCA9685_ALL_LED_ON_H = 0xFB, /**< load all the LEDn_ON registers, high */
	PCA9685_ALL_LED_OFF_L = 0xFC, /**< load all the LEDn_OFF registers, low */
	PCA9685_ALL_LED_OFF_H = 0xFD, /**< load all the LEDn_OFF registers,high */

	PCA9685_PRE_SCALE = 0xFE, 	/**< Prescaler for PWM output frequency */
	PCA9685_TEST_MODE = 0xFF, 	/**< defines the test mode to be entered */

	// MODE1 bits
	MODE1_ALLCAL = 0x01,  /**< respond to LED All Call I2C-bus address */
	MODE1_SUB3 = 0x02,    /**< respond to I2C-bus subaddress 3 */
	MODE1_SUB2 = 0x04,    /**< respond to I2C-bus subaddress 2 */
	MODE1_SUB1 = 0x08,    /**< respond to I2C-bus subaddress 1 */
	MODE1_SLEEP = 0x10,   /**< Low power mode. Oscillator off */
	MODE1_AI = 0x20,      /**< Auto-Increment enabled */
	MODE1_EXTCLK = 0x40,  /**< Use EXTCLK pin clock */
	MODE1_RESTART = 0x80, /**< Restart enabled */

	// MODE2 bits
	MODE2_OUTNE_0 = 0x01, /**< Active LOW output enable input */
	MODE2_OUTNE_1 = 0x02, /**< Active LOW output enable input - high impedience */
	MODE2_OUTDRV = 0x04,  /**< totem pole structure vs open-drain */
	MODE2_OCH = 0x08,     /**< Outputs change on ACK vs STOP */
	MODE2_INVRT = 0x10,   /**< Output logic state inverted */
} PCA9685_Register;

typedef enum {
	PCA9685_OK,
	PCA9685_ERROR,
	PCA9685_ERROR_READ,
	PCA9685_ERROR_WRITE,
} PCA9685_status;
typedef struct {
	uint8_t addr; // I2C address (PCA9685)
	PCA9685_Register reg;

//	int (*write)(uint16_t addr, uint8_t* data, uint16_t size, uint32_t timeout);
	int (*read_mem)(uint8_t addr, uint16_t reg_mem_address, uint16_t mem_size,
			const uint8_t* data, uint16_t size_data, uint32_t timeout, void* arg);
	int (*write_mem)(uint8_t addr, uint16_t reg_mem_address, uint16_t mem_size,
			const uint8_t* data, uint16_t size_data, uint32_t timeout, void* arg);

} PCA9685_Handler;

PCA9685_status PCA9685_set_bit(PCA9685_Handler* pca, uint8_t register_addr, uint8_t bit, uint8_t value);
PCA9685_status PCA9685_set_PWM_frequency(PCA9685_Handler* pca, uint16_t frequency);

void PCA9685_init(PCA9685_Handler* pca, uint16_t frequency);

PCA9685_status PCA9685_set_PWM_LED(PCA9685_Handler* pca, uint8_t channel, uint16_t on_time, uint16_t off_time);

PCA9685_status PCA9685_LED_on(PCA9685_Handler* pca, uint8_t channel);

PCA9685_status PCA9685_LED_off(PCA9685_Handler* pca, uint8_t channel);

void PCA9685_LED_toggle(PCA9685_Handler* pca, uint8_t channel);

PCA9685_status PCA9685_LED_dim(PCA9685_Handler* pca, uint8_t channel);

PCA9685_status PCA9685_LED_brightness(PCA9685_Handler* pca, uint8_t channel, uint16_t brightness);


#endif /* INC_PCA9685_H_ */
