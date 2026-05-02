#include "UART.h"
#include "hardware/UART.h"
#include "pico/assert.h"


#ifndef UART_RESET_NUM 
#include <string> 
#include <iostream>
#include <stdio.h>

static unsigned int uart_get_index (uart_inst_t *uart){
    
   //Gets the instance numer of UART (0,1)
   if (uart != 0 && uart != 1){
    return -1 ; 
   }
   if(uart == 0 ){
    return 0 ;
   }
   return 1; 

}

static uart_inst_t * uart_get_instance (uint num){
    //grabs the instance as a pointer 
    //First lets ensure that the num is 0 or 1 
    if (num != 0 && num != 1){
        return 0  ;
    }
    return UART_INSTANCE(num);
    
}
//Note: incline for used for small function:
static incline uart_hw_t * uart_get_hw (uart_inst_t *uart){
     if (!uart) {
        return NULL;
     }
     //THis is the pointer value 
    return uart->hw;
    

}
uint uart_init (uart_inst_t *uart, uint baudrate){
    if (!uart || baudrate == 0){
     return 0;
    }
    //First lets get the hardware pointer 
    uart_hw_t *hw = uart_get_hw(uart);

    //Now lets set up the baudrate 
    uint actual_baud = uart_set_baudrate(uart, baudrate);

    //lets set it up 
    hw->cr = 1;
    
    return actual_baud ; 
}

void uart_deinit(uart_inst_t *uart) {
    // We get the pointer so we can shut it down ( BLACKPINK refernce)
    uart_hw_t *hw = uart_get_hw(uart);

    // this disables the UART 
    hw->cr = 0;
}

uint uart_set_baudrate(uart_inst_t *uart, uint baudrate) {
    //Always get the pointer, it makes it simple 
    uart_hw_t *hw = uart_get_hw(uart);

    
    uint clk = UART_CLOCK_NUM(uart);
    //I just researched this by we need the 16x which 
    uint div = clk / (16 * baudrate);

    // We deference it to get the baudrate 
    hw->ibrd = div;

    return baudrate;
}





