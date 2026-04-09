#ifndef I2C_HW_H_
#define I2C_HW_H_

#include "ch.h"
#include "hal.h"
#include "stdint.h"
#include "stdbool.h"
#include "stddef.h"

/*
 * Hardware I2C shim exposing the original i2c_bb_* API so higher-level
 * code does not need to change. Internally this uses ChibiOS I2CDriver
 * instances so multiple hardware peripherals are supported.
 *
 * NOTE: the struct name and field layout are intentionally compatible
 * with the original bit-banged implementation (so code that references
 * these fields still compiles). We add driver / buffering fields at the
 * tail so the original offsets are preserved for existing code that
 * directly reads the first fields.
 */

typedef enum {
	I2C_BB_RATE_100K = 0,
	I2C_BB_RATE_200K,
	I2C_BB_RATE_400K,
	I2C_BB_RATE_700K
} I2C_BB_RATE;

typedef struct {
	/* kept for compatibility with older code (ignored by HW driver) */
	stm32_gpio_t *sda_gpio; int sda_pin;
	stm32_gpio_t *scl_gpio; int scl_pin;

	/* original fields */
	I2C_BB_RATE rate;
	bool has_started;
	bool has_error;
	mutex_t mutex;

	/* hardware-specific additions (safe to use by this driver only) */
	I2CDriver *i2c;           /* selected ChibiOS I2C driver (e.g. &HW_I2C_DEV) */

	/* simple per-transaction buffer to support legacy per-byte API */
	uint8_t addr;             /* 7-bit address currently in transaction */
	bool rw;                  /* 0 = write, 1 = read */
	bool transaction_open;    /* send_start / send_stop emulation */
	uint8_t txbuf[256];
	size_t txlen;
} i2c_bb_state;

/* Public API (same names as the previous bitbang driver) */
void i2c_bb_init(i2c_bb_state *s);
void i2c_bb_init_with_driver(i2c_bb_state *s, I2CDriver *driver);
void i2c_bb_set_driver(i2c_bb_state *s, I2CDriver *driver);
I2CDriver *i2c_bb_get_driver(i2c_bb_state *s);

void i2c_bb_restore_bus(i2c_bb_state *s);
bool i2c_bb_tx_rx(i2c_bb_state *s, uint16_t addr,
                  uint8_t *txbuf, size_t txbytes,
                  uint8_t *rxbuf, size_t rxbytes);

bool i2c_bb_write_byte(i2c_bb_state *s, bool send_start, bool send_stop, unsigned char byte);
unsigned char i2c_bb_read_byte(i2c_bb_state *s, bool nack, bool send_stop);

#endif /* I2C_HW_H_ */
