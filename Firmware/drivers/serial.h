#ifndef SERIAL_H
#define SERIAL_H

#include "../common/common.h"
#include <libopencm3/stm32/usart.h>


typedef struct Serial{
    uint32_t perif;
}Serial;


#define UART_BUFFER_SIZE (16)

static uint8_t uartRB[UART_BUFFER_SIZE];
static Ring_Buffer tx_buffer = {.buffer = uartRB, .size = sizeof(uartRB)}; // Place holder for class


void usart1_isr(void){
    const bool overrun_occurred = usart_get_flag(USART1, USART_FLAG_ORE) == 1;
    const bool received_data = usart_get_flag(USART1, USART_FLAG_RXNE) == 1;
    const bool transmit_empty = usart_get_flag(USART1, USART_FLAG_TXE) == 1;

    if (transmit_empty) {  // Transmit buffer is empty - send next byte
        if (!ringbuffer_empty(&tx_buffer)) { // Data available in RingBuffer
            // send element from ring buffer
            USART_DR(USART1) = ringbuffer_get(&tx_buffer);

        } else {
            // Buffer is empty, disable TXE interrupt
            USART_CR1(USART1) &= ~USART_CR1_TXEIE;
        }
    }
}



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

static void serial_sendByte(Serial *ser, uint8_t byte){
    //@Breif: Interupt Driven write byte to USARt FIFO
    while (ringbuffer_full(&tx_buffer)) {} ; // If  blokc often, increase rb sieze
    ringbuffer_put(&tx_buffer, byte); // input ring buffer
    if ((USART_CR1(ser->perif) & USART_CR1_TXEIE) == 0) {
        usart_enable_tx_interrupt(ser->perif);
    }
    return;
}

static void serial_send(Serial *ser, uint8_t *data, uint16_t size){
    //@Breif: Intrupt Driven write array to USART FIFO
    for (uint16_t i = 0; i < size; i++) {
        serial_sendByte(ser, data[i]);
    }
    return;
}

#endif // SERIAL_H