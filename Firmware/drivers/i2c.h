#ifndef I2C_H
#define I2C_H

#include "../common/common.h"
#include <libopencm3/stm32/i2c.h>

static void i2cReset(uint32_t i2cPerif){
    I2C_CR1(i2cPerif) |= I2C_CR1_SWRST;
    I2C_CR1(i2cPerif) &= ~I2C_CR1_SWRST;
}

static uint32_t i2cInit(uint32_t i2cPerif, uint32_t gpioPort, uint32_t sclPin, uint32_t sdaPin){
    //@Brief: Configures I2C Peripheral on GPIO pins
    //@Note: Requires Clocks to be enabled for GPIO's & I2C
    // ENSURE PUPD_PULLUP IS ENABLED !!!!!!
    gpio_mode_setup(gpioPort, GPIO_MODE_AF, GPIO_PUPD_PULLUP,  sclPin| sdaPin); 
    gpio_set_output_options(gpioPort, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, sclPin | sdaPin);
    gpio_set_af(gpioPort, GPIO_AF4, sclPin | sdaPin);

    i2c_peripheral_disable(i2cPerif);
    i2cReset(i2cPerif);

    i2c_set_clock_frequency(i2cPerif, 42); // Set the Peripheral Clock Freq
    i2c_set_fast_mode(i2cPerif); // 400Khz

    i2c_set_ccr(i2cPerif, 0x8019); // Set the bus clock frequency

    i2c_set_trise(i2cPerif, 43); // Max Rise time
    i2c_peripheral_enable(i2cPerif);
    return i2cPerif;
}


static uint16_t i2cReadSeq(uint32_t i2c, uint16_t addr, uint8_t reg, uint8_t *data, uint16_t len) {
    //@Brief: Reads array of bytes from I2C Device
    //@Note: Assumes Device has Auto Increment Registers

    while ((I2C_SR2(i2c) & (I2C_SR2_BUSY))); // Block if bus is busy

    i2c_enable_ack(i2c); 
    i2c_send_start(i2c); // Begin Select start register command

    // I2C_EVENT_MASTER_MODE_SELECT EV5: BUSY, MSL and SB 
    while (!((I2C_SR1(i2c) & I2C_SR1_SB) & 
             (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

    i2c_send_7bit_address(i2c, addr, I2C_WRITE);
    // I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED EV6: BUSY, MSL, ADDR, TXE and TRA 
    while (!(  (I2C_SR1(i2c) & (I2C_SR1_TxE | I2C_SR1_ADDR)) & 
               (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY | I2C_SR2_TRA)) ));
    (void)I2C_SR2(i2c); // clear

    i2c_send_data(i2c, reg);
    // I2C_EVENT_MASTER_BYTE_TRANSMITTED EV8_2: TRA, BUSY, MSL, TXE and BTF 
    while (!((I2C_SR1(i2c) & (I2C_SR1_TxE | I2C_SR1_BTF)) & 
             (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY | I2C_SR2_TRA))));


    i2c_send_start(i2c); // Begin read command
    // I2C_EVENT_MASTER_MODE_SELECT EV5: BUSY, MSL and SB 
    while (!((I2C_SR1(i2c) & I2C_SR1_SB) & 
             (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

    i2c_send_7bit_address(i2c, addr, I2C_READ);
    // I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED EV6: BUSY, MSL and ADDR 
    while (!(  (I2C_SR1(i2c) & (I2C_SR1_ADDR)) &
               (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY)) ));

    uint16_t i;
    for (i = 0; i < len; i++) {
        if (i == len - 1) {
            i2c_disable_ack(i2c);
            i2c_send_stop(i2c);
        }
        /* I2C_EVENT_MASTER_BYTE_RECEIVED EV7: BUSY, MSL and RXNE */
        while (!(I2C_SR1(i2c) & I2C_SR1_RxNE));
        data[i] = i2c_get_data(i2c);
    }

    while ((I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))); // Block if bus is busy

    return ++i; // return number of bytes read
}


static uint8_t i2cReadReg(uint32_t i2c, uint16_t addr, uint8_t reg) {
    //@Brief: Reads one Byte from a I2C Device Register
    uint8_t data = 0;

    i2c_enable_ack(i2c);
    i2c_send_start(i2c);

    // I2C_EVENT_MASTER_MODE_SELECT EV5: BUSY, MSL and SB 
    while (!((I2C_SR1(i2c) & I2C_SR1_SB) & 
             (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

    i2c_send_7bit_address(i2c, addr, I2C_WRITE);
    // I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED EV6: BUSY, MSL, ADDR, TXE and TRA 
    while (!(  (I2C_SR1(i2c) & (I2C_SR1_TxE | I2C_SR1_ADDR)) & 
               (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY | I2C_SR2_TRA)) ));
    (void)I2C_SR2(i2c);

    i2c_send_data(i2c, reg);
    // I2C_EVENT_MASTER_BYTE_TRANSMITTED EV8_2: TRA, BUSY, MSL, TXE and BTF 
    while (!((I2C_SR1(i2c) & (I2C_SR1_TxE | I2C_SR1_BTF)) & 
             (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY | I2C_SR2_TRA))));

    i2c_send_start(i2c);
    // I2C_EVENT_MASTER_MODE_SELECT EV5: BUSY, MSL and SB 
    while (!((I2C_SR1(i2c) & I2C_SR1_SB) & 
             (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

    i2c_send_7bit_address(i2c, addr, I2C_READ);
    // I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED EV6: BUSY, MSL and ADDR 
    while (!(  (I2C_SR1(i2c) & (I2C_SR1_ADDR)) &
               (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY)) ));

    i2c_disable_ack(i2c);
    i2c_send_stop(i2c);
    // I2C_EVENT_MASTER_BYTE_RECEIVED EV7: BUSY, MSL and RXNE 
    while (!( (I2C_SR1(i2c) & I2C_SR1_RxNE )));
    data = i2c_get_data(i2c);

    while ((I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY)));

    return data;
}


static void i2cWriteReg(uint32_t i2c, uint16_t addr, uint8_t reg, uint8_t data) {
    //@Brief: write Byte to I2C Device Register

    i2c_enable_ack(i2c);
    i2c_send_start(i2c);

    // I2C_EVENT_MASTER_MODE_SELECT EV5: BUSY, MSL and SB 
    while (!((I2C_SR1(i2c) & I2C_SR1_SB) & 
             (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

    i2c_send_7bit_address(i2c, addr, I2C_WRITE);
    // I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED EV6: BUSY, MSL, ADDR, TXE and TRA 
    while (!(  (I2C_SR1(i2c) & (I2C_SR1_TxE | I2C_SR1_ADDR)) & 
               (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY | I2C_SR2_TRA)) ));
    (void)I2C_SR2(i2c);

    i2c_send_data(i2c, reg);
    // I2C_EVENT_MASTER_BYTE_TRANSMITTED EV8_2: TRA, BUSY, MSL, TXE and BTF 
    while (!((I2C_SR1(i2c) & (I2C_SR1_TxE | I2C_SR1_BTF)) & 
             (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY | I2C_SR2_TRA))));

    i2c_send_data(i2c, data);
    // I2C_EVENT_MASTER_BYTE_TRANSMITTED EV8_2: TRA, BUSY, MSL, TXE and BTF 
    while (!((I2C_SR1(i2c) & (I2C_SR1_TxE | I2C_SR1_BTF)) & 
             (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY | I2C_SR2_TRA))));

    i2c_send_stop(i2c);

    while ((I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY)));
}

#endif // I2C_H