/*
 * lis2de12 - ST LIS2DE12 accelerometer driver
 *
 * Copyright (c) 2018, Archos S.A.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * * Neither the name of Archos nor the names of its contributors may be
 *   used to endorse or promote products derived from this software without
 *   specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS'' AND
 * AND EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ARCHOS S.A. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <os.h>
#include <i2c.h>
#include <xprintf.h>
#include "lis2de12.h"

/* LIS2DE12 I2C address */
#define LIS2DE12_ADDR			(0x19 << 1)

/*
 * LIS2DE12 registers
 */

/* Reserved				0x00 - 0x06 */
#define REG_STATUS_REG_AUX		0x07		/* RO */
/* Reserved				0x08 - 0x0B */
#define REG_OUT_TEMP_L			0x0C		/* RO */
#define REG_OUT_TEMP_H			0x0D		/* RO */
/* Reserved				0x0E */
#define REG_WHO_AM_I			0x0F		/* RO */
/* Reserved				0x10 - 0x1E */
#define REG_TEMP_CFG_REG		0x1F		/* RW */
#define REG_CTRL_REG1			0x20		/* RW */
#define REG_CTRL_REG2			0x21		/* RW */
#define REG_CTRL_REG3			0x22		/* RW */
#define REG_CTRL_REG4			0x23		/* RW */
#define REG_CTRL_REG5			0x24		/* RW */
#define REG_CTRL_REG6			0x25		/* RW */
#define REG_REFERENCE_DATACAPTURE	0x26		/* RW */
#define REG_STATUS_REG			0x27		/* RO */
#define REG_OUT_X_RES			0x28		/* RO */
#define REG_OUT_X			0x29		/* RO */
#define REG_OUT_Y_RES			0x2A		/* RO */
#define REG_OUT_Y			0x2B		/* RO */
#define REG_OUT_Z_RES			0x2C		/* RO */
#define REG_OUT_Z			0x2D		/* RO */
#define REG_FIFO_CTRL_REG		0x2E		/* RW */
#define REG_FIFO_SRC_REG		0x2F		/* RO */
#define REG_INT1_CFG			0x30		/* RW */
#define REG_INT1_SRC			0x31		/* RO */
#define REG_INT1_THS			0x32		/* RW */
#define REG_INT1_DURATION		0x33		/* RW */
#define REG_INT2_CFG			0x34		/* RW */
#define REG_INT2_SRC			0x35		/* RO */
#define REG_INT2_THS			0x36		/* RW */
#define REG_INT2_DURATION		0x37		/* RW */
#define REG_CLICK_CFG			0x38		/* RW */
#define REG_CLICK_SRC			0x39		/* RO */
#define REG_CLICK_THS			0x3A		/* RW */
#define REG_TIME_LIMIT			0x3B		/* RW */
#define REG_TIME_LATENCY		0x3C		/* RW */
#define REG_TIME_WINDOW			0x3D		/* RW */
#define REG_ACT_THS			0x3E		/* RW */
#define REG_ACT_DUR			0x3F		/* RW */


/*
 * Mask and bit definitions of LIS2DE12 registers
 */

#define STATUS_REG_AUX_TOR		0x40
#define STATUS_REG_AUX_TDA		0x04

#define WHO_AM_I_ID_MASK		0xFF
#define WHO_AM_I_ID			0x33

#define TEMP_CFG_REG_TEMP_DIS		0x00
#define TEMP_CFG_REG_TEMP_EN		0xC0

#define CTRL_REG1_XEN			0x01
#define CTRL_REG1_YEN			0x02
#define CTRL_REG1_ZEN			0x04
#define CTRL_REG1_LPEN			0x08
#define CTRL_REG1_ODR_MASK		0xF0
#define		ODR_OFF			0x00
#define		ODR_1HZ			0x10
#define		ODR_10HZ		0x20
#define		ODR_25HZ		0x30
#define		ODR_50HZ		0x40
#define		ODR_100HZ		0x50
#define		ODR_200HZ		0x60
#define		ODR_400HZ		0x70
#define		ODR_1620HZ		0x80
#define		ODR_5378HZ		0x90

#define CTRL_REG2_HPF_AOI_INT1_EN	0x01
#define CTRL_REG2_HPF_AOI_INT2_EN	0x02
#define CTRL_REG2_HPCLICK_EN		0x04
#define CTRL_REG2_FDS			0x08
#define CTRL_REG2_HPF_CUTOFF_FREQ_MASK	0x30
#define		HPF_MODE_MASK		0xC0
#define		HPF_MODE_NORMAL		0x00
#define		HPF_MODE_REF_SIG	0x40
#define		HPF_MODE_NORMAL2	0x80
#define		HPF_MODE_AUTORESET	0xC0

#define CTRL_REG3_INT1_OVERRUN_EN	0x02
#define CTRL_REG3_INT1_WTM_EN		0x04
#define CTRL_REG3_INT1_DRDY2_EN		0x08
#define CTRL_REG3_INT1_DRDY1_EN		0x10
#define CTRL_REG3_INT1_AOI2_EN		0x20
#define CTRL_REG3_INT1_AOI1_EN		0x40
#define CTRL_REG3_INT1_CLICK_EN		0x80

#define CTRL_REG4_SIM_3WIRE		0x00
#define CTRL_REG4_SIM_4WIRE		0x01
#define CTRL_REG4_SELF_TEST_MASK	0x06
#define		SELF_TEST_OFF		0x00
#define		SELF_TEST_0		0x02
#define		SELF_TEST_1		0x04
#define CTRL_REG4_FULL_SCALE_MASK	0x30
#define		FULL_SCALE_2G		0x00
#define		FULL_SCALE_4G		0x10
#define		FULL_SCALE_8G		0x20
#define		FULL_SCALE_16G		0x30
#define CTRL_REG4_BDU_CONTINUOUS	0x00
#define CTRL_REG4_BDU_SYNC_ON_READ	0x80

#define CTRL_REG5_D4D_INT2_EN		0x01
#define CTRL_REG5_LIR_INT2_EN		0x02
#define CTRL_REG5_D4D_INT1_EN		0x04
#define CTRL_REG5_LIR_INT1_EN		0x08
#define CTRL_REG5_FIFO_EN		0x40
#define CTRL_REG5_REBOOT		0x80

#define CTRL_REG6_INT_ACTIVE_HIGH	0x00
#define CTRL_REG6_INT_ACTIVE_LOW	0x02
#define CTRL_REG6_I2PIN_ACT_EN		0x08
#define CTRL_REG6_I2PIN_BOOT_EN		0x10
#define CTRL_REG6_INT2_ON_I2PIN		0x20
#define CTRL_REG6_INT1_ON_I2PIN		0x40
#define CTRL_REG6_CLICKINT_ON_I2PIN	0x80

#define STATUS_REG_XDA			0x01
#define STATUS_REG_YDA			0x02
#define STATUS_REG_ZDA			0x04
#define STATUS_REG_XYZDA		0x08
#define STATUS_REG_XOR			0x10
#define STATUS_REG_YOR			0x20
#define STATUS_REG_ZOR			0x40
#define STATUS_REG_XYZOR		0x80

#define FIFO_CTRL_REG_TR_INT1		0x00
#define FIFO_CTRL_REG_TR_INT2		0x20
#define FIFO_CTRL_REG_MODE_MASK		0xC0
#define		MODE_BYPASS		0x00
#define		MODE_FIFO		0x40
#define		MODE_STREAM		0x80
#define		MODE_STREAM_FIFO	0xC0

#define INTX_CFG_XLIE			0x01
#define INTX_CFG_XHIE			0x02
#define INTX_CFG_YLIE			0x04
#define INTX_CFG_YHIE			0x08
#define INTX_CFG_ZLIE			0x10
#define INTX_CFG_ZHIE			0x20
#define INTX_CFG_6D			0x40
#define INTX_CFG_AOI			0x80

#define INTX_THS_MASK			0x7F

#define INTX_DURATION_MASK		0x7F

#define I2C_NOT_INITIALIZED		255

static uint8_t sensor_i2c = I2C_NOT_INITIALIZED;

static uint8_t curent_odr = ODR_1HZ;


/*
 * I2C Stuff
 */

static void lis2de12_write(uint8_t reg, uint8_t value)
{
	uint8_t data[2];

	data[0] = reg;
	data[1] = value;

	i2c_write(sensor_i2c, LIS2DE12_ADDR, data, 2);
}

static uint8_t lis2de12_read(uint8_t reg)
{
	uint8_t value = 0xFF;

	i2c_write(sensor_i2c, LIS2DE12_ADDR, &reg, 1);
	i2c_read(sensor_i2c, LIS2DE12_ADDR, &value, 1);

	return value;
}


/*
 * LIS2DE12 stuff
 */

static uint8_t lis2de12_get_id(void)
{
	uint8_t data;

	data = lis2de12_read(REG_WHO_AM_I) & WHO_AM_I_ID_MASK;
	return data;
}

static void lis2de12_enable(uint8_t en)
{
	uint8_t data;

	data = lis2de12_read(REG_CTRL_REG1) & ~CTRL_REG1_ODR_MASK;
	lis2de12_write(REG_CTRL_REG1, data | ((en ? curent_odr : ODR_OFF) & CTRL_REG1_ODR_MASK));
}

uint8_t lis2de12_is_enabled(void)
{
	return ((lis2de12_read(REG_CTRL_REG1) & CTRL_REG1_ODR_MASK) != 0);
}

static void lis2de12_set_odr(uint8_t odr)
{
	curent_odr = odr;
}

static uint32_t lis2de12_get_odr_num(void)
{
	/* Return the corresponding samplerate value in Hz */
	switch (curent_odr) {
		case ODR_5378HZ:
			return 5378;

		case ODR_1620HZ:
			return 1620;

		case ODR_400HZ:
			return 400;

		case ODR_200HZ:
			return 200;

		case ODR_100HZ:
			return 100;

		case ODR_50HZ:
			return 50;

		case ODR_25HZ:
			return 25;

		case ODR_10HZ:
			return 10;

		case ODR_1HZ:
			return 1;
	}

	return 0;
}

static void lis2de12_set_scale(uint8_t scale)
{
	uint8_t data;

	data = lis2de12_read(REG_CTRL_REG4) & ~CTRL_REG4_FULL_SCALE_MASK;
	lis2de12_write(REG_CTRL_REG4, data | (scale & CTRL_REG4_FULL_SCALE_MASK));
}

uint8_t lis2de12_get_scale(void)
{
	return lis2de12_read(REG_CTRL_REG4) & CTRL_REG4_FULL_SCALE_MASK;
}

/* Threshold is expected in mg */
static void lis2de12_set_move_threshold(uint8_t int_num, uint32_t thershold)
{
	switch (lis2de12_get_scale()) {
		case FULL_SCALE_2G:
			thershold /= 16;
			break;
		case FULL_SCALE_4G:
			thershold /= 32;
			break;
		case FULL_SCALE_8G:
			thershold /= 62;
			break;
		case FULL_SCALE_16G:
			thershold /= 186;
			break;
	}

	lis2de12_write((int_num == LIS2DE12_INT1) ? REG_INT1_THS : REG_INT2_THS, ((uint8_t) (thershold) &INTX_THS_MASK));
}

/* Transient duration is expected in ms, and
 * has to be set after the samplerate !
 */
static void lis2de12_set_move_duration(uint8_t int_num, uint32_t duration)
{
	uint32_t odr = lis2de12_get_odr_num();

	lis2de12_write((int_num == LIS2DE12_INT1) ? REG_INT1_DURATION : REG_INT2_DURATION, ((uint8_t) ((duration * odr) / 1000) & INTX_DURATION_MASK));
}

void lis2de12_get_accel(struct lis2de12_accel_data *data)
{
	while (!(lis2de12_read(REG_STATUS_REG) & STATUS_REG_XYZDA));

	data->x = (int8_t) lis2de12_read(REG_OUT_X);
	data->y = (int8_t) lis2de12_read(REG_OUT_Y);
	data->z = (int8_t) lis2de12_read(REG_OUT_Z);
}

static void lis2de12_configure(void)
{
	/*
	 * Set Standby mode, Measurement mode will
	 * be set when the device will be open
	 */
	lis2de12_enable(0);

	/* Enable X, Y and Z axis + low-power/8bit mode */
	lis2de12_write(REG_CTRL_REG1, CTRL_REG1_XEN | CTRL_REG1_YEN | CTRL_REG1_ZEN | CTRL_REG1_LPEN);

	/* Set range to 2G */
	lis2de12_set_scale(FULL_SCALE_2G);

	/* Set samplerate to 50Hz */
	lis2de12_set_odr(ODR_50HZ);

	/* High pass filter (INT1 only), cut-off freq = 0.1Hz */
	lis2de12_write(REG_CTRL_REG2, 0x31);

	/* Set movement detection parameters */
	lis2de12_set_move_threshold(LIS2DE12_INT1, 64); // Threshold is 0,064g
	lis2de12_set_move_duration(LIS2DE12_INT1, 140); // 150ms

	/* Enable movement detection */
	lis2de12_write(REG_INT1_CFG, INTX_CFG_XHIE | INTX_CFG_YHIE | INTX_CFG_ZHIE);

	/* Configure INT1 */
	lis2de12_write(REG_CTRL_REG3, 0x40);
	lis2de12_write(REG_CTRL_REG5, 0x08);

	/* Clear the INTs in case some are set (seems to happen from time to time) */
	lis2de12_read(REG_INT1_SRC);
	lis2de12_read(REG_INT2_SRC);
}

void lis2de12_handle_int(uint8_t pin)
{
	if (pin & LIS2DE12_INT1) {
		lis2de12_read(REG_INT1_SRC);
	} else if (pin & LIS2DE12_INT2) {
		lis2de12_read(REG_INT2_SRC);
	}
}

uint8_t lis2de12_init(uint8_t i2c_port)
{
	uint8_t id;

	if (sensor_i2c != i2c_port) {
		sensor_i2c = i2c_port;
		i2c_init(sensor_i2c, 100000);
	}

	id = lis2de12_get_id();
	if (id != WHO_AM_I_ID) {
		xprintf("LIS2DE12 accelerometer not found\n");
		return 1;
	}

	/* Reset the accelerometer */
	lis2de12_write(REG_CTRL_REG5, lis2de12_read(REG_CTRL_REG5) | CTRL_REG5_REBOOT);
	os_delay(ms2ostime(50));
	lis2de12_write(REG_CTRL_REG5, lis2de12_read(REG_CTRL_REG5) & ~CTRL_REG5_REBOOT);

	lis2de12_configure();

	lis2de12_enable(1);

	return 0;
}

void lis2de12_deinit(void)
{
	lis2de12_enable(0);

	os_delay(ms2ostime(100));

	i2c_deinit(sensor_i2c);

	sensor_i2c = I2C_NOT_INITIALIZED;
}
