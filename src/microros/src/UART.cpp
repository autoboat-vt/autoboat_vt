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
    //Gets HARDWARE number as an reference number 
    

}
uint uart_init (uart_inst_t *uart, uint baudrate){
    //Installs the UART file 
}


