#ifndef UART.h
#define UARH.h
#define UART_NUM(uart)
#define UART_INSTANCE(num)

#define UART_DREQ_NUM(uart, is_tx)

#define UART_CLOCK_NUM(uart)

#define UART_FUNCSEL_NUM(uart, gpio)

#define UART_IRQ_NUM(uart)

#define UART_RESET_NUM(uart)


#include <string> 
#include <iostream> 

//For the UART file we will be focusing on making sure the communcation between the reciever and the transmitter 

//First lets make the Enum class for parity 
//The parity enum class determines if we have an error: Ensures integrity 

enum uart_parity_t { UART_PARITY_NONE, UART_PARITY_EVEN, UART_PARITY_ODD };

//Functions: 

// Note: uint = unsigned int 
static unsigned int uart_get_index (uart_inst_t *uart);

// Gets UART instance num as pointer 

static uart_inst_t * uart_get_instance (uint num);

// gets the UART HARDWARE instance as pointer

static uart_hw_t * uart_get_hw (uart_inst_t *uart);

//Installs the UART connection: 

uint uart_init (uart_inst_t *uart, uint baudrate);

//Deletes the UART Connection: 

void uart_deinit (uart_inst_t *uart);