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
Read (RX ISR)   serial_receive()
**************************************/

typedef struct Serial{
    uint32_t perif;
    RingBuffer *rxBuf;
    RingBuffer *txBuf;
    bool recvFlag;
}Serial;

// ***** GLOBAL USART RING BUFFERS for ISR *****
RingBuffer rx1_rb = {.size = RING_BUFFER_SIZE};
RingBuffer tx1_rb = {.size = RING_BUFFER_SIZE};

// ************** ISR's ************************** // 
static RingBuffer *getRingBuffer(uint32_t usart){
    RingBuffer *rb;
    switch(usart){ // assign global ring buffers
        case USART1:
            rb = &tx1_rb;
            break;
        default:
            return NULL; // exit 
    }
    return rb;
}

static void usartTX_ISR(uint32_t usart){
    //@Brief: Generic ISR Handler for TX USART
    RingBuffer *rb = getRingBuffer(usart);
    volatile uint8_t data = 0;
    if(rb_empty(rb) == 0){
        data = rb_get(rb); 
        USART_DR(usart) = data & USART_DR_MASK; // write byte
    }
    else{
        usart_disable_tx_interrupt(usart); // Full contents of buffer have been written
    }
}

static void usartRX_ISR(uint32_t usart){
    //@Brief: Generic ISR Handler for RX USART
    RingBuffer *rb = getRingBuffer(usart);
    volatile uint8_t data = 0;
    if(rb_full(&rx1_rb) == 0){
        data = (USART_DR(USART1) & USART_DR_MASK);
        rb_put(&rx1_rb, data);
    } // If ring buffer full subsequent data will be discared 
}

void usart1_isr(void){
    const bool overrun_occurred = usart_get_flag(USART1, USART_FLAG_ORE) == 1;
    const bool received_data = usart_get_flag(USART1, USART_FLAG_RXNE) == 1;
    const bool transmit_empty = usart_get_flag(USART1, USART_FLAG_TXE) == 1;

    if(received_data){
        usartRX_ISR(USART1);
    }
    if(transmit_empty){
       usartTX_ISR(USART1);
    }
}

// ************ SETUP ***************************** // 
static Serial serialInit(uint32_t perif, uint32_t port, uint32_t rxPin, uint32_t txPin, uint32_t pinMode, uint32_t irq ){
    //@Brief: Initializes the USART Peripheral Hardware
    //@Brief: If irc == NULL serial device will be confirmed without ISR 
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

    rb_init(ser.rxBuf, RING_BUFFER_SIZE);
    rb_init(ser.txBuf, RING_BUFFER_SIZE);

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

// *********** POLLING *************************** //
//@Note: This doesent fully work when the ISR is enabled ???? gives some wired values on read

static void serialWrite(Serial *ser, uint8_t *data, uint16_t size){
    //@Brief: Writes Bytes to USART Trasmit Data Register 
    //@Note: Blocking, waits on transmit complete
    for(int i =0; i<size; i++){
        while((USART_SR(ser->perif) & USART_SR_TXE) == 0){}; // wait for shift registe to be empty
        USART_DR(ser->perif) = data[i] & USART_DR_MASK; // write byte
    }
}

static void serialRead(Serial *ser, uint8_t *buf, uint16_t size){
    //@Brief: Reads from USART Receive Data Register
    //@Note: Blocking, waits on data available
    for(int i =0; i< size; i++){
        while((USART_SR(ser->perif) & USART_SR_RXNE) == 0){}; // wait for data to be avalibal in receive shift resgter
        buf[i] = (USART_DR(ser->perif) & USART_DR_MASK);
    }
}

static uint8_t serialReadLine(Serial *ser, uint8_t *buf, uint8_t size){
    //@Brief: Reads USART Data Register until \n 
    //@Note: Blocking, waits on data available
    uint8_t c, i;
    for(i = 0; i < size; i++){
        while((USART_SR(ser->perif) & USART_SR_RXNE) == 0){}; // wait for data
        c = (USART_DR(ser->perif) & USART_DR_MASK);
        if(c == '\n'){
            buf[i] = '\0'; // null terminate
            return i;
        }
        buf[i] = c;
    }
    return i;
}

static bool serialAvailable(Serial *ser){
    //Brief: Checks if Data available in USART Receive Register
    return (USART_SR(ser->perif) & USART_SR_RXNE); 
}

// *********** ISR DRIVEN *********************** //

static void serialSend(Serial *ser, uint8_t *data, uint16_t size){
    //@Breif: adds data to ring buffer and sets up ISR trasmit
    //@Note: Blocks if ringbuffer full

    for(int i =0; i < size; i++){
        while(rb_full(ser->txBuf) == 1){}; 
        rb_put(ser->txBuf, data[i]);
    }
    usart_enable_tx_interrupt(ser->perif); // Set up ISR 
}

static uint8_t serialReceive(Serial *ser, uint8_t *buf, uint16_t size){
    //@Brief: reads size bytes from ring buffer 
    //@Note: Blocks, Returns number of bytes read
    int i = 0;
    for(; i < size; i++){
        while(rb_empty(ser->rxBuf) == 1){};
        buf[i] = rb_get(ser->rxBuf);
    }
    return i;
}

static uint8_t serialGrab(Serial *ser, uint8_t *buf, uint16_t size){
    //@Brief: Attempts to read size bytes from ring buffer
    //@Note: If less bytes available returns num of bytes read
    int i = 0;
    for(; i < size; i++){
        if(rb_empty(ser->rxBuf) == 1){return i;}
        buf[i] = rb_get(ser->rxBuf);
    }
    return i;

}
    

#endif // SERIAL_H