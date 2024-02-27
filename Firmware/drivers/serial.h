#ifndef SERIAL_H
#define SERIAL_H

#include "../common/common.h"
#include <libopencm3/stm32/usart.h>

/******* USART Serial Driver **********
 
This should generally be used as part of a higher level communication functionality 

Polling - Blocks on waiting for byte Received, byte Transmitted & byte Match
        - serialRead() serialWrite() serialReadLine() 

ISR     - Reads/Writes data byte by byte into allocated RingBuffer, new data ignored if full 
        - Must be serviced by main loop fast enough to ensure buffer doesn't fill up.
        - serialSend() blocks if buffer full. TX ISR flushes Ringbuffer by the TXE interrupt
        - serialReceive() reads n bytes from RX ringbuffer, blocks if empty 
        - serialGrab() non-blocking reads n bytes from ringbuffer, if empty returns num bytes read
**************************************/

typedef struct Serial{
    uint32_t perif;
    RingBuffer rxRB; 
    RingBuffer txRB;
}Serial;

// ************** ISR's ************************** // 
// Code below is executed from ISRs

/*----- GLOBAL RING BUFFERS Pointers for ISR -------
* STM32F401 has USART1 USART2 USART6
* These are set to point to the alloced RB by serialInit()
* This way users can simply access the buffer through serial->rb
* The application will access the correct buffer through the global pointer
---------------------------------------------------*/
RingBuffer *rx1_rb_ = NULL;
RingBuffer *tx1_rb_ = NULL;
RingBuffer *rx2_rb_ = NULL;
RingBuffer *tx2_rb_ = NULL;
RingBuffer *rx6_rb_ = NULL;
RingBuffer *tx6_rb_ = NULL;

static RingBuffer *getRB_TX(uint32_t usart){
    RingBuffer *rb;
    switch(usart){ // assign global ring buffers
        case USART1:
            rb = tx1_rb_;
            break;
        case USART2:
            rb = tx2_rb_;
            break;
        case USART6:
            rb = tx6_rb_;
            break;
        default:
            return NULL; // exit 
    }
    return rb;
}
static RingBuffer *getRB_RX(uint32_t usart){
    RingBuffer *rb;
    switch(usart){ // assign global ring buffers
        case USART1:
            rb = rx1_rb_;
            break;
        case USART2:
            rb = rx2_rb_;
            break;
        case USART6:
            rb = rx6_rb_;
            break;
        default:
            return NULL; // exit 
    }
    return rb;
}

static void usartTX_ISR(uint32_t usart){
    //@Brief: Generic ISR Handler for TX USART
    //@Description: Gets data available from TX RingBuffer writes to USART Data Registers
    /*@Note: Disables TX Interrupts once no data is in RingBuffer
            this is enabled again by the SerialSend Cmd */
    RingBuffer *  rb = getRB_TX(usart);
    if(rb == NULL){return;} // exit ensure appropriate USART is serialInit()
    uint8_t data = 0;
    
    if(rb_empty(rb) == 0){
        rb_get(rb, &data); // only write if there is data in the rb 
        USART_DR(usart) = data & USART_DR_MASK; // write byte
    }
    else{
        usart_disable_tx_interrupt(usart); // Full contents of buffer have been written
    }
}

static void usartRX_ISR(uint32_t usart){
    //@Brief: Generic ISR Handler for RX USART
    //@Description: Puts Data to RX Ring Buffer if not full
    //@Note: Discards Data if RB is full 
    RingBuffer * const rb = getRB_RX(usart);
    if(rb == NULL){return;} // ensure appropriate USART is serialInit()
   
    uint8_t data = 0;
    data = (USART_DR(usart) & USART_DR_MASK); // read byte
    if(rb_full(rb) == 0){
        rb_put(rb, data); // add byte to buffer    
    }
}

void usart1_isr(void){ 
    const bool overrun_occurred = usart_get_flag(USART1, USART_FLAG_ORE) == 1;
    const bool received_data = usart_get_flag(USART1, USART_FLAG_RXNE) == 1;
    const bool transmit_empty = usart_get_flag(USART1, USART_FLAG_TXE) == 1;

    if(received_data){usartRX_ISR(USART1);}
    if(transmit_empty){usartTX_ISR(USART1);}
}

void usart2_isr(void){
    const bool overrun_occurred = usart_get_flag(USART2, USART_FLAG_ORE) == 1;
    const bool received_data = usart_get_flag(USART2, USART_FLAG_RXNE) == 1;
    const bool transmit_empty = usart_get_flag(USART2, USART_FLAG_TXE) == 1;

    if(received_data){usartRX_ISR(USART2);}
    if(transmit_empty){usartTX_ISR(USART2);}
}


// ************ SETUP ***************************** // 

static Serial serialInit(uint32_t perif, uint32_t port, uint32_t rxPin, uint32_t txPin, uint32_t pinMode, uint32_t irq, uint8_t * const rxbuf, size_t rxSize, uint8_t * const txbuf, size_t txSize){
    //@Brief: Initializes the USART Peripheral Hardware
    //@Brief: If irc == NULL serial device will be confirmed without ISR 
    gpio_mode_setup(port, GPIO_MODE_AF, GPIO_PUPD_NONE, rxPin | txPin);
    gpio_set_af(port, pinMode, rxPin| txPin);

    usart_disable(perif);

    if(irq != 0){
        usart_enable_rx_interrupt(perif);
        nvic_enable_irq(irq); // enable irq in nvic
    }
    // Create Serial Device 
    Serial ser = {
        .perif = perif,
        .rxRB = rb_init(rxbuf, rxSize),
        .txRB = rb_init(txbuf, txSize),
    };
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

    //Maps the global ringbuffer pointers to a Serial's ringbuffer
    //ISR's access the ring buffer based on which USART peripheral is used. 
    switch (ser->perif){ // assign global ring buffer pointers
        case USART1:
            rx1_rb_ = &(ser->rxRB);
            tx1_rb_ = &(ser->txRB);
            break;
        case USART2:
            rx2_rb_ = &(ser->rxRB);
            tx2_rb_ = &(ser->txRB);
            break;
        case USART6:
            rx6_rb_ = &(ser->rxRB);
            tx6_rb_ = &(ser->txRB);
            break;
        default:
            break;
    }   

    usart_enable(usart);
}


// *********** POLLING *************************** //
//@Note: This doesent fully work when the ISR is enabled ???? gives some wired values on read

static void serialWrite(Serial *ser, uint8_t *data, uint16_t size){
    //@Brief: Writes Bytes to USART Trasmit Data Register 
    //@Note: Blocking, waits on transmit complete
    for(int i =0; i<size; i++){
        while((USART_SR(ser->perif) & USART_SR_TXE) == 0){}; // wait for shift register to be empty
        USART_DR(ser->perif) = data[i] & USART_DR_MASK; // write byte
    }
}

static void serialRead(Serial *ser, uint8_t *buf, uint16_t size){
    //@Brief: Reads from USART Receive Data Register
    //@Note: Blocking, waits on data available
    for(int i =0; i< size; i++){
        while((USART_SR(ser->perif) & USART_SR_RXNE) == 0){}; // wait for data to be available in receive shift register
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
    //@Brief: adds data to ring buffer and sets up ISR transmit
    //@Note: Blocks if ringbuffer full

    for(int i =0; i < size; i++){
        while(rb_full(&ser->txRB) == 1){}; // block while full
        rb_put(&ser->txRB, data[i]);
    }
    usart_enable_tx_interrupt(ser->perif); // Set up ISR 
}

static uint8_t serialReceive(Serial *ser, uint8_t *buf, uint16_t size){
    //@Brief: reads size bytes from ring buffer 
    //@Note: Blocks, Returns number of bytes read
    int i = 0;
    for(; i < size; i++){
        while(rb_empty(&ser->rxRB) == 1){};
        rb_get(&ser->rxRB, &buf[i]);
    }
    return i;
}

static uint8_t serialGrab(Serial *ser, uint8_t *buf, uint16_t size){
    //@Brief: Attempts to read size bytes from ring buffer
    //@Note: If less bytes available returns num of bytes read
    int i = 0;
    for(; i < size; i++){
        if(rb_empty(&ser->rxRB) == 1){return i;}
        rb_get(&ser->rxRB, &buf[i]);
    }
    return i;
}

#endif // SERIAL_H