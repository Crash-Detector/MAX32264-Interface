

#include "main.h"
#include <assert.h>

// GPIO Configuration
#define RCC_ADDR 0x40021000
#define RCC_AHB2ENR_OFFSET 0x4C
#define GPIO_ADDR_BASE 0x48000000
#define GPIO_MODER_OFFSET 0x00
#define GPIO__TYPER_OFFSET 0x04


// For w/r
#define GPIO_IDR_OFFSET 0x10
#define GPIO_ODR_OFFSET 0x14

const uint8_t BIO_ADDRESS = 0x55;

//enum IO { IN, OUT };

// direction 1 (input), direction 0, output
// Configure the gpio to be either In or out (as well as enabling the clock if not already enabled).
void config_gpio( const char port, const int pin_num, const enum IO direction )
    {
    // Invariants
    assert( port >= 'A' && port <= 'F' ); 
    assert( pin_num >= 0 && pin_num <= 15 );
    assert( direction == IN || direction == OUT );

    // Port index: A == 0, B == 1, etc.
    const uint32_t port_idx = port - 'A';

    // Ensure that rcc_ahb2 for the port is enabled
    uint32_t * const rcc_ahb2enr = ( uint32_t * )( RCC_ADDR + RCC_AHB2ENR_OFFSET );
    *rcc_ahb2enr |= ( uint32_t ) ( 1 << port_idx ); // Shift one into the index of the port (turning on the clock if not already on).

    // Port address differ by 0x400 bytes
    const uint32_t port_offset = ( uint32_t )port_idx * 0x400;

    uint32_t io_mode = direction == IN ? 0b00 : 0b01;
    uint32_t mask = 0b11;

    // Shift to the pin_num's relevant bits9 (in moder register).
    io_mode <<= ( pin_num << 1 ); // Shift by 2*pin_num
    mask    <<= ( pin_num << 1 );

    uint32_t * const gpio_moder = (uint32_t * )( GPIO_ADDR_BASE + port_offset + GPIO_MODER_OFFSET );

    *gpio_moder &= ~mask; // Clear the mode bits
    *gpio_moder |=  io_mode; // Write to it
    } // end config_gpio( )

enum GPIO_MODE read_gpio_state( const char port, const int pin_num )
    {
    // Invariants
    assert( port >= 'A' && port <= 'F' ); 
    assert( pin_num >= 0 && pin_num <= 15 );

    // Port index: A == 0, B == 1, etc.
    const uint32_t port_idx = port - 'A';

    // Port address differ by 0x400 bytes
    const uint32_t port_offset = ( uint32_t )port_idx * 0x400;
    uint32_t const * const gpio_moder = (uint32_t * )( GPIO_ADDR_BASE + port_offset + GPIO_MODER_OFFSET );
    const uint32_t mask = 0b11 << ( pin_num << 1 );

    uint32_t io_mode;
    // Obtain the io_mode
    io_mode = *gpio_moder & mask;
    io_mode >>= ( pin_num << 1 ); // Shift down to the LSBs.

    return (enum GPIO_MODE)io_mode;
    } // end read_gpio_state( )

enum GPIO_MODE read_gpio_t_state( struct GPIO const * const gpio  )
    {
    return read_gpio_state( gpio->port, gpio->pin_num );
    } // end read_gpio_t_state( )

void set_pin_mode( struct GPIO * const gpio, const enum IO direction )
    {
    gpio->pin_mode = direction == IN ? GPIO_INPUT : GPIO_OUTPUT;
    config_gpio( gpio->port, gpio->pin_num, direction );
    } // end set_pin_mode( )

void write_gpio( const char port, const int pin_num, const enum LVL bit_of )
    {
    assert( read_gpio_state( port, pin_num ) == GPIO_OUTPUT ); // Ensures that this is an output pin.
    const uint32_t port_off = (uint32_t ) ( port - 'A' ) * 0x400;
    uint32_t const * const gpio_start_addr =(uint32_t * )( GPIO_ADDR_BASE + port_off );
    const uint32_t mask = 1 << pin_num;

    uint32_t * const gpio_odr = ( uint32_t * ) ( gpio_start_addr + GPIO_ODR_OFFSET );

    *gpio_odr &= ~mask; // Clear pin_numTH bit.
    *gpio_odr |= bit_of << pin_num;
    } // end write_gpio( )
enum LVL read_gpio( const char port, const int pin_num )
    {
    assert( read_gpio_state( port, pin_num ) == GPIO_INPUT ); // Ensures that this is an input pin.
    const uint32_t port_off = (uint32_t ) ( port - 'A' ) * 0x400;
    uint32_t const * const gpio_start_addr =(uint32_t * )( GPIO_ADDR_BASE + port_off );
    const uint32_t mask = 1 << pin_num;
    uint32_t * const gpio_idr = ( uint32_t * ) ( gpio_start_addr + GPIO_IDR_OFFSET );
    uint32_t r_bit;

    r_bit = *gpio_idr & mask;
    r_bit >>= pin_num;

    return ( enum LVL ) r_bit;
    } // end read_gpio( )

void write_gpio_t( struct GPIO * const gpio, const enum LVL bit_of )
    {
    write_gpio( gpio->port, gpio->pin_num, bit_of );
    } // end write_gpio_t( )

enum LVL read_gpio_t( struct GPIO const * const gpio )
    {
    return read_gpio( gpio->port, gpio->pin_num );
    } // end read_gpio_t( )

//------------------------------------------------------------------------------------------------
//
//                                 SparkFun_Bio_Sensor Member Function Definitions
//
//------------------------------------------------------------------------------------------------

void bio_sensor_init( struct SparkFun_Bio_Sensor * const bio_ssor, const GPIO_t rst_pin, const GPIO_t mfio_pin, const uint8_t sample_rate )
    {
    bio_ssor->_reset_pin = rst_pin;
    bio_ssor->_mfio_pin = mfio_pin;
    bio_ssor->_sampleRate = sample_rate;

    // Begin with mfio_pin and rst_pin in output mode.
    set_pin_mode( &( bio_ssor->_reset_pin ), OUT );
    set_pin_mode( &( bio_ssor->_mfio_pin ),  OUT );
    return;
    } // end bio_sensor_init( )
void write_byte( struct SparkFun_Bio_Sensor * const bio_ssor, const uint8_t family_byte, const uint8_t index_byte, uint8_t write_byte )
    {
    return;
    } // end write_byte( )