

#include "main.h"
#include <assert.h>

#define GPIO_ADDR_BASE 0x48000000
#define GPIO_MODER_OFFSET 0x00
#define GPIO__TYPER_OFFSET 0x04

const uint8_t BIO_ADDRESS = 0x55;

//enum IO { IN, OUT };

// direction 1 (input), direction 0, output
void config_gpio( const char port, const int pin_num, const enum IO direction )
    {
    // Invariants
    assert( port >= 'A' && port <= 'F' ); 
    assert( pin_num >= 0 && pin_num <= 15 );
    assert( direction == IN || direction == OUT );


    // Port address differ by 0x400 bytes
    const uint32_t port_offset = ( ( uint32_t ) ( port - 'A' ) ) * 0x400;

    uint32_t io_mode = direction == IN ? 0b00 : 0b01;
    uint32_t mask = 0b11;

    // Shift to the pin_num's relevant bits9 (in moder register).
    io_mode <<= ( pin_num << 1 ); // Shift by 2*pin_num
    mask    <<= ( pin_num << 1 );

    uint32_t * const gpio_moder = (uint32_t * )( GPIO_ADDR_BASE + port_offset + GPIO_MODER_OFFSET );

    *gpio_moder &= ~mask; // Clear the mode bits
    *gpio_moder |=  io_mode; // Write to it
    } // end config_gpio( )

void set_pin_mode( struct GPIO * const gpio, const enum IO direction )
    {
    gpio->io_state = direction;
    config_gpio( gpio->port, gpio->pin_num, direction );
    } // end set_pin_mode( )
