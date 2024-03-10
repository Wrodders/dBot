#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>
#include "../common/common.h"

typedef struct  GPIO {
	uint32_t pin;
	uint32_t port;
}GPIO;

static GPIO initGPIO(uint32_t pin, uint32_t port, uint32_t mode, uint32_t pupd) {
	GPIO p;
	p.pin = pin;
	p.port = port;

	gpio_mode_setup(p.port, mode, pupd, p.pin);
	return p;
}

#endif // GPIO_H