#ifndef SERIAL_H
#define SERIAL_H

#include "../common/common.h"
#include <libopencm3/stm32/usart.h>


typedef struct Serial{
    uint32_t perif;
}Serial;

static Serial serial_init( uint32_t uartPerif, uint32_t gpioPort, uint32_t rxPin, uint32_t txPin, uint32_t gpioAF, uint8_t irq){
    //@Breif: Sets up USART Periferal on GPIOS
    //@Note: Requires GPIO and USART Clocks to be enabled
    gpio_mode_setup(gpioPort, GPIO_MODE_AF, GPIO_PUPD_NONE, rxPin | txPin); // tx RX
    gpio_set_af(gpioPort, gpioAF, rxPin | txPin);

    USART_CR1(uartPerif) &= ~USART_CR1_UE;  // Disable UART for configuration

    if(irq != 0){
        USART_CR1(uartPerif) |= USART_CR1_RXNEIE;   // enable interupt receive data Shift Register not empty                           
        
        USART_SR(uartPerif) &= ~USART_SR_TXE; // clear TX Empty flag before enabling interupt
        USART_CR1(uartPerif) |= USART_CR1_TXEIE; // enable TX inteupt
        
        nvic_enable_irq(irq); // enable interupt in NVIC
    }

    Serial ser;
    ser.perif = uartPerif;
    return ser;
}

static void serial_config(Serial *serial, uint32_t baud, uint32_t dataBits, uint32_t stopBits, uint32_t parity, uint32_t flowcontrol){
    //@Breif: Configures USART parameteres
    uint32_t usart = serial->perif;

    USART_CR1(usart) = (USART_CR1(usart) & ~USART_MODE_MASK) | USART_MODE_TX_RX; // enable TX and RX
    uint32_t clock = rcc_apb2_frequency;
    if(usart == USART2){
        clock = rcc_apb1_frequency;
    }
    USART_BRR(usart) = (clock + baud / 2) / baud; // set baudrate     
    if (dataBits == 8) { 
		USART_CR1(usart) &= ~USART_CR1_M; // * 8 bit word
	} else {
		USART_CR1(usart) |= USART_CR1_M;  // 9 bit word
	}

    USART_CR2(usart) = (USART_CR2(usart) & ~USART_CR2_STOPBITS_MASK) | stopBits;     // clear and set stop bits
    USART_CR1(usart) = (USART_CR1(usart) & ~USART_PARITY_MASK) | parity;            // clear and set parity bits
    USART_CR3(usart) = (USART_CR3(usart) & ~USART_FLOWCONTROL_MASK) | flowcontrol;  // clear and set flow controll bits
    return;
}

static void serial_begin(Serial *ser){
    //@Breif: Enables USART
    USART_CR1(ser->perif) |= USART_CR1_UE;  // Enable
    return; 
}


static void serial_writeByte(Serial *ser, uint8_t byte){
    //@breif: Blocking write byte to USART TX FIFO
    while(!(USART_SR(ser->perif) & USART_SR_TXE)){}; // wait for shift registe to be empty
    USART_DR(ser->perif) = byte;
    while(!(USART_SR(ser->perif) & USART_SR_TC)){}; // wait for transmistion to complete
}

static void serial_write(Serial *ser, uint8_t *data, uint16_t size){
    //@breif: blocking send array to USART FIFO
    for(uint16_t i = 0; i < size; i++){
        serial_writeByte(ser, data[i]);
    }
}



#endif // SERIAL_H