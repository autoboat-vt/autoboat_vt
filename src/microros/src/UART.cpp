#include "UART.h"

#include <string> 
#include <iostream>
#include <stdio.h>
static unsigned int uart_get_index (uart_inst_t *uart){
   //Gets the instance numer of UART  
   
}
static uart_inst_t * uart_get_instance (uint num){
    //grabs the instance as a pointer 
}
static uart_hw_t * uart_get_hw (uart_inst_t *uart){
    //Gets HARDWARE number as an reference number 
}
uint uart_init (uart_inst_t *uart, uint baudrate){
    //Installs the UART file 
}

