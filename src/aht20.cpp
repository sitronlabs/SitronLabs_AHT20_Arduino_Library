/* Self header */
#include <aht20.h>

/* Arduino libraries */
#include "crc8.h"

/* Config */
#define CONFIG_AHT20_I2C_ADDRESS 0x38
#define CONFIG_AHT20_CALIBRATION_DURATION 10
#define CONFIG_AHT20_CALIBRATION_TIMEOUT 30
#define CONFIG_AHT20_MEASUREMENT_DURATION 80
#define CONFIG_AHT20_MEASUREMENT_RETRY 1
#define CONFIG_AHT20_MEASUREMENT_TIMEOUT 100
#define CONFIG_AHT20_DEBUG_ENABLED 1

/* Macros */
#if CONFIG_AHT20_DEBUG_ENABLED
#define CONFIG_AHT20_DEBUG_FUNCTION(x) Serial.println(x)
#else
#define CONFIG_AHT20_DEBUG_FUNCTION(x)
#endif

/**
 * List of commands supported by the sensor.
 */
enum aht20_commands {
	AHT20_COMMAND_CALIBRATION = 0xBE,
	AHT20_COMMAND_MEASUREMENT_TRIGGER = 0xAC,
	AHT20_COMMAND_RESET_SOFT = 0xBA,
};

/**
 * Default constructor.
 */
aht20::aht20() {
	m_i2c_library = NULL;
}

/**
 *
 * @param[in] i2C_library
 */
void aht20::setup(TwoWire &i2C_library) {
	m_i2c_library = &i2C_library;
}

/**
 * Tries to detect the sensor.
 * @note It takes a maximum of 20 ms after power up for the sensor to be ready to receive commands.
 * @param[in] wait If set to false the library will not make sure 20 ms have elapsed since boot.
 * @return true if the sensor has been detected, or false otherwise.
 */
bool aht20::detect(bool wait) {
	int res;

	/* Wait for power up */
	if (wait) {
		while (millis() < 20)
			;
	}

	/* Ping device */
	m_i2c_library->beginTransmission(CONFIG_AHT20_I2C_ADDRESS);
	res = m_i2c_library->endTransmission(true);
	if (res != 0) {
		return false;
	}

	/* Return success */
	return true;
}

/**
 * Request the sensor to trigger a measurement and returns immediately.
 * After that, make repeated calls to measurement_async_get until the functions return anything else than -EINPROGRESS.
 * @return 0 in case of success, or a negative error code otherwise.
 */
int aht20::measurement_async_trigger(void) {
	int res;

	/* Remember start time to detect timeout */
	uint32_t time_start = millis();

	/* Ensure calibration has been performed */
	while (1) {

		/* Read calibration status bit */
		uint8_t reg_status;
		res = m_register_read(&reg_status, 1);
		if (res < 1) {
			CONFIG_AHT20_DEBUG_FUNCTION(" [e] Failed to read status register!");
			return res;
		}

		/* If calibration has been done, we can stop here */
		if ((reg_status & (1 << 3))) {
			break;
		}

		/* Request to start calibration */
		uint8_t cmd[] = { AHT20_COMMAND_CALIBRATION, 0x08, 0x00 };
		res = m_register_write(cmd, 3);
		if (res < 0) {
			CONFIG_AHT20_DEBUG_FUNCTION(" [e] Failed to send calibration command!");
			return res;
		}

		/* Wait enough time for calibration to be done */
		delay(CONFIG_AHT20_CALIBRATION_DURATION);

		/* Detect timeout */
		if (millis() - time_start > CONFIG_AHT20_CALIBRATION_TIMEOUT) {
			CONFIG_AHT20_DEBUG_FUNCTION(" [e] Timed out waiting for calibration!");
			return -ETIMEDOUT;
		}
	}

	/* Request to trigger measurement */
	uint8_t cmd[] = { AHT20_COMMAND_MEASUREMENT_TRIGGER, 0x33, 0x00 };
	res = m_register_write(cmd, 3);
	if (res < 0) {
		CONFIG_AHT20_DEBUG_FUNCTION(" [e] Failed to send calibration command!");
		return res;
	}

	/* Return success */
	return 0;
}

/**
 * Retrieves the measurement results if available.
 * Call measurement_async_trigger first and then call this function repeatedly as long as it returns -EINPROGRESS.
 * @param[out] temperature A pointer to a variable that will be updated with the temperature in degrees Celsius.
 * @param[out] humidity A pointer to a variable that will be updated with the humidity in percents.
 * @return 0 in case of success, -EINPROGRESS if a measurement is still in progress or a negative error code otherwise.
 */
int aht20::measurement_async_get(float * const temperature, float * const humidity) {
	int res;

	/* Read all registers */
	uint8_t regs[7];
	res = m_register_read(regs, 7);
	if (res < 0) {
		CONFIG_AHT20_DEBUG_FUNCTION(" [e] Failed to read registers!");
		return res;
	}

	/* Wait for busy bit to be cleared */
	if (regs[0] & (1 << 7)) {
		return -EINPROGRESS;
	}

	/* Extract 20-bits of humidity data */
	uint32_t reg_humidity = regs[1];
	reg_humidity <<= 8;
	reg_humidity |= regs[2];
	reg_humidity <<= 4;
	reg_humidity |= (regs[3] >> 4);
	*humidity = (reg_humidity / 1048576.0);

	/* Extract 20-bits of temperature data */
	uint32_t reg_temperature = (regs[3] & 0x0F);
	reg_temperature <<= 8;
	reg_temperature |= regs[4];
	reg_temperature <<= 8;
	reg_temperature |= regs[5];
	*temperature = (reg_temperature / 1048576.0) * 200.0 - 50.0;

	/* Check crc */
	crc8 crc8(0x31, 0xFF);
	uint8_t crc8_computed = crc8.process(regs, 6);
	if (crc8_computed != regs[6]) {
		CONFIG_AHT20_DEBUG_FUNCTION(" [e] Checksum mismatch!");
		return -EIO;
	}

	/* Return success */
	return 0;
}

/**
 *
 * @param temperature
 * @param humidity
 * @return
 */
int aht20::measurement_sync_get(float * const temperature, float * const humidity) {
	int res;

	/* Trigger measurement */
	res = measurement_async_trigger();
	if (res < 0) {
		CONFIG_AHT20_DEBUG_FUNCTION(" [e] Failed to trigger measurement!");
		return res;
	}

	/* Remember start time to detect timeout */
	uint32_t time_start = millis();

	/* Wait enough time for measurement to be done and try to request results */
	delay(CONFIG_AHT20_MEASUREMENT_DURATION);
	while (1) {

		/* Detect timeout */
		if (millis() - time_start > CONFIG_AHT20_MEASUREMENT_TIMEOUT) {
			CONFIG_AHT20_DEBUG_FUNCTION(" [e] Timed out waiting for measurement!");
			return -ETIMEDOUT;
		}

		/* Request measurement results,
		 * But if they are not ready yet, wait a bit more */
		res = measurement_async_get(temperature, humidity);
		if (res == -EINPROGRESS) {
			delay(CONFIG_AHT20_MEASUREMENT_RETRY);
			continue;
		} else if (res < 0) {
			CONFIG_AHT20_DEBUG_FUNCTION(" [e] Failed to retrieve measurement!");
			return res;
		} else if (res == 0) {
			break;
		}
	}

	/* Return success */
	return 0;
}

/**
 *
 * @param[out] bytes
 * @param[in] length
 * @return the number of bytes read from the sensor in case of success, or a negative error code otherwise.
 */
int aht20::m_register_read(uint8_t * const bytes, const size_t length) {
	int res;

	/* Ensure library has been set */
	if (m_i2c_library == NULL) {
		return -EINVAL;
	}

	/* Request to read */
	res = m_i2c_library->requestFrom(CONFIG_AHT20_I2C_ADDRESS, length, true);
	if (res < 0) {
		return -EIO;
	}

	/* Return bytes */
	for (size_t i = 0; i < length && i < (size_t) res; i++) {
		bytes[i] = m_i2c_library->read();
	}
	return res;
}

/**
 *
 * @param[int] bytes
 * @param[in] length
 * @return the number of bytes written to the sensor in case of success, or a negative error code otherwise.
 */
int aht20::m_register_write(const uint8_t * const bytes, const size_t length) {
	int res;

	/* Ensure library has been set */
	if (m_i2c_library == NULL) {
		return -EINVAL;
	}

	/* Request to write */
	m_i2c_library->beginTransmission(CONFIG_AHT20_I2C_ADDRESS);
	m_i2c_library->write(bytes, length);
	res = m_i2c_library->endTransmission(true);
	if (res != 0) {
		return -EIO;
	}

	/* Return bytes */
	return res;
}
