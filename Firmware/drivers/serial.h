#ifndef SERIAL_H
#define SERIAL_H

#include "../common/common.h"
#include <libopencm3/stm32/usart.h>



/******* USART Serial Driver **********
Usage: 
1. serial_init()
2. serial_config()
 
Write (blocking) serial_write()
Write TX ISR    serial_send()
Read (Blocking) serial_read()          
Read (RX ISR)   serial_get()
**************************************/

//*** NOTE USART2 DOESNT SEEM TO WORK I THINK DUE TO RCC Clock Configuration

typedef struct Serial{
    uint32_t perif;
    RingBuffer *rxBuf;
    RingBuffer *txBuf;
}Serial;


// ***** GLOBAL USART RING BUFFERS for ISR *****
RingBuffer rx1_rb  = {.size = 64};
RingBuffer tx1_rb =  {.size = 64};



void usart1_isr(void){
    const bool overrun_occurred = usart_get_flag(USART1, USART_FLAG_ORE) == 1;
    const bool received_data = usart_get_flag(USART1, USART_FLAG_RXNE) == 1;
    const bool transmit_empty = usart_get_flag(USART1, USART_FLAG_TXE) == 1;

    volatile uint8_t data = 0;

    if(received_data){
        if(ringbuffer_full(&rx1_rb) == 0){
            data = (USART_DR(USART1) & USART_DR_MASK);
            //gpio_toggle(GPIOA, GPIO5);
            ringbuffer_put(&rx1_rb, data);
        } // IF Ring bufffer full susequent data will be discared 
    }
    if(transmit_empty){
        if(ringbuffer_empty(&tx1_rb) == 0){
            data = ringbuffer_get(&tx1_rb);// get byte from ring buffer
            USART_DR(USART1) = data & USART_DR_MASK; // write byte
        }
        else{
            usart_disable_tx_interrupt(USART1); // Full contents of buffer have been written
        }
    }
}


static Serial serialInit(uint32_t perif, uint32_t port, uint32_t rxPin, uint32_t txPin, uint32_t pinMode, uint32_t irq ){
    //@Breif: Initializes the USART Peripheral Hardware
    gpio_mode_setup(port, GPIO_MODE_AF, GPIO_PUPD_NONE, rxPin | txPin);
    gpio_set_af(port, pinMode, rxPin| txPin);

    usart_disable(perif);

    if(irq != 0){
        usart_enable_rx_interrupt(perif);
        nvic_enable_irq(irq); // enable irq in nvic
    }

    Serial ser;
    ser.perif = perif;
    // Assign Global Ring Buffer 
    switch(perif){
        case USART1:
            ser.rxBuf = &rx1_rb;
            ser.txBuf = &tx1_rb;
            break;
        default:
            break;
    }
    return ser;    
}


static void serialConfig(Serial *ser, uint32_t baud, uint8_t databits, uint8_t stopBits, uint32_t parity, uint32_t flowcontroll ){
    //@Breif: Configures USART Parameters

    uint32_t usart = ser->perif;
    usart_set_mode(usart, USART_MODE_TX_RX);
    usart_set_baudrate(usart, baud );
    usart_set_databits(usart, databits);
    usart_set_stopbits(usart, stopBits);
    usart_set_parity(usart, parity);
    usart_set_flow_control(usart, flowcontroll);
    usart_enable(usart);
}

static void serialWrite(Serial *ser, uint8_t *data, uint16_t size){
    //@Breif: Writes Bytes to USART Trasmit Data Register 
    //@Note: Blocking, waits on transmit complete

    for(int i =0; i<size; i++){
        while(!(USART_SR(ser->perif) & USART_SR_TXE)){}; // wait for shift registe to be empty
        USART_DR(ser->perif) = data[i] & USART_DR_MASK; // write byte
        while(!(USART_SR(ser->perif) & USART_SR_TC)){}; // wait for transmistion to complete
    }
}

static void serialRead(Serial *ser, uint8_t *buf, uint16_t size){
    for(int i =0; i< size; i++){
        while((USART_SR(ser->perif) & USART_SR_RXNE)){}; // wait for data to be avalibal in receive shift resgter
        buf[i] = (USART_DR(ser->perif) & USART_DR_MASK) + 1;// test
    }
}

static bool serialAvailable(Serial *ser){
    return (USART_SR(ser->perif) & USART_SR_RXNE); 
}


static void serialSend(Serial *ser, uint8_t *data, uint16_t size){
    //@Breif: adds data to ring buffer and sets up ISR trasmit

    for(int i =0; i < size; i++){
        while(ringbuffer_full(&tx1_rb) == 1){}; // if blcoks here often increare size
        ringbuffer_put(&tx1_rb, data[i]); // add to ring buffer
    }
    usart_enable_tx_interrupt(ser->perif); // Set up ISR 
}

static uint8_t serialReceive(Serial *ser, uint8_t *buf, uint16_t size){
    //@Breif: Reads size bytes from ring buffer 
    int i = 0;
    for(i; i < size; i++){
        if(ringbuffer_empty(&rx1_rb) ==  1){
            gpio_toggle(GPIOA, GPIO5);
            return i; // exit
        }
        buf[i] = ringbuffer_get(&rx1_rb) + 1; //test
    }
    return i;
}
    


#endif // SERIAL_H