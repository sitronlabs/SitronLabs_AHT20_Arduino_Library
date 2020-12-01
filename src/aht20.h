#ifndef AHT20_H
#define AHT20_H

/* Arduino libraries */
#include <Arduino.h>
#include <Wire.h>

/* C liraries */
#include <errno.h>

class aht20 {

public:
	aht20();
	void setup(TwoWire &i2c_library = Wire);
	bool detect(bool wait = true);
	int measurement_async_trigger(void);
	int measurement_async_get(float * const temperature, float * const humidity);
	int measurement_sync_get(float * const temperature, float * const humidity);

private:
	TwoWire *m_i2c_library;
	int m_register_read(uint8_t * const bytes, const size_t length);
	int m_register_write(const uint8_t * const bytes, const size_t length);

};

#endif
