

#include "main.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h> // For exit( )
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

void write_gpio( const char port, const int pin_num, const GPIO_PinState bit_of )
    {
    assert( read_gpio_state( port, pin_num ) == GPIO_OUTPUT ); // Ensures that this is an output pin.
    const uint32_t port_off = (uint32_t ) ( port - 'A' ) * 0x400;
    uint32_t const * const gpio_start_addr =(uint32_t * )( GPIO_ADDR_BASE + port_off );
    const uint32_t mask = 1 << pin_num;

    uint32_t * const gpio_odr = ( uint32_t * ) ( gpio_start_addr + GPIO_ODR_OFFSET );

    *gpio_odr &= ~mask; // Clear pin_numTH bit.
    *gpio_odr |= bit_of << pin_num;
    } // end write_gpio( )
GPIO_PinState read_gpio( const char port, const int pin_num )
    {
    assert( read_gpio_state( port, pin_num ) == GPIO_INPUT ); // Ensures that this is an input pin.
    const uint32_t port_off = (uint32_t ) ( port - 'A' ) * 0x400;
    uint32_t const * const gpio_start_addr =(uint32_t * )( GPIO_ADDR_BASE + port_off );
    const uint32_t mask = 1 << pin_num;
    uint32_t * const gpio_idr = ( uint32_t * ) ( gpio_start_addr + GPIO_IDR_OFFSET );
    uint32_t r_bit;

    r_bit = *gpio_idr & mask;
    r_bit >>= pin_num;

    return ( GPIO_PinState ) r_bit;
    } // end read_gpio( )

void write_gpio_t( struct GPIO * const gpio, const GPIO_PinState bit_of )
    {
    write_gpio( gpio->port, gpio->pin_num, bit_of );
    } // end write_gpio_t( )

GPIO_PinState read_gpio_t( struct GPIO const * const gpio )
    {
    return read_gpio( gpio->port, gpio->pin_num );
    } // end read_gpio_t( )

//------------------------------------------------------------------------------------------------
//
//                                 SparkFun_Bio_Sensor Member Function Definitions
//
//------------------------------------------------------------------------------------------------

void bio_sensor_init( struct SparkFun_Bio_Sensor * const bio_ssor, I2C_HandleTypeDef * const i2c_h, const uint8_t addr, const GPIO_t rst_pin, const GPIO_t mfio_pin, const uint8_t sample_rate, const uint8_t user_sel_mode )
    {
    bio_ssor->_reset_pin = rst_pin;
    bio_ssor->_mfio_pin = mfio_pin;
    bio_ssor->_addr = addr;
    bio_ssor->_sampleRate = sample_rate;
    bio_ssor->_userSelectedMode = user_sel_mode;
    bio_ssor->_i2c_h = i2c_h;
    switch( user_sel_mode )
    {
    case( BOOTLOADER_MODE ):
        // Call function...
        break;
    case( APP_MODE ):
        // Call function...
        break;
    default:
        bio_ssor->_userSelectedMode = DISABLE; // Disable the sensor (erronous to use any functions)
        break;
    } // end switch

    // Begin with mfio_pin and rst_pin in output mode.
    set_pin_mode( &( bio_ssor->_reset_pin ), OUT );
    set_pin_mode( &( bio_ssor->_mfio_pin ),  OUT );
    return;
    } // end bio_sensor_init( )

 // Family Byte: READ_DEVICE_MODE (0x02) Index Byte: 0x00, Write Byte: 0x00
    // The following function initializes the sensor. To place the MAX32664 into
    // application mode, the MFIO pin must be pulled HIGH while the board is held
    // in reset for 10ms. After 50 addtional ms have elapsed the board should be
    // in application mode and will return two bytes, the first 0x00 is a
    // successful communcation byte, followed by 0x00 which is the byte indicating
    // which mode the IC is in.
uint8_t enter_app_mode( struct SparkFun_Bio_Sensor * const bio_ssor )
    {
    // Set pins to output mode (in order to be able to write to.)
    set_pin_mode( &( bio_ssor->_reset_pin ), OUT );
    set_pin_mode( &( bio_ssor->_mfio_pin ),  OUT );

    write_gpio_t( &bio_ssor->_mfio_pin , GPIO_PIN_SET );
    write_gpio_t( &bio_ssor->_reset_pin, GPIO_PIN_RESET );
    HAL_Delay( 10 ); // Hold reset for 10ms.

    write_gpio_t( &bio_ssor->_reset_pin, GPIO_PIN_SET );

    // Delay until board is in application mode.
    HAL_Delay( 1000 );

    /*
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0,  0); // Reset pin
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1,  1); // MFIO pin
    HAL_Delay(10);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0,  1); // Reset is pulled 
    HAL_Delay(50);
    HAL_Delay(1000);
    */
    uint8_t resp_byte = read_byte( bio_ssor, READ_DEVICE_MODE, 0x00 );
    return resp_byte; // Should be in app_mode ( 0x00 )
    } // end enter_app_mode( )

// Family Byte: READ_DEVICE_MODE (0x02) Index Byte: 0x00, Write Byte: 0x00
// The following function puts the MAX32664 into bootloader mode. To place the MAX32664 into
// bootloader mode, the MFIO pin must be pulled LOW while the board is held
// in reset for 10ms. After 50 addtional ms have elapsed the board should be
// in bootloader mode and will return two bytes, the first 0x00 is a
// successful communcation byte, followed by 0x08 which is the byte indicating
// that the board is in bootloader mode.
uint8_t enter_bootloader( struct SparkFun_Bio_Sensor * const bio_ssor )
    {
    
    // Set pins to output mode (in order to be able to write to.)
    set_pin_mode( &( bio_ssor->_reset_pin ), OUT );
    set_pin_mode( &( bio_ssor->_mfio_pin ),  OUT );
    

    //write_gpio_t( &bio_ssor->_mfio_pin , GPIO_PIN_SET );
    write_gpio_t( &bio_ssor->_reset_pin, GPIO_PIN_RESET ); // Reset...
    HAL_Delay( 6 );
    write_gpio_t( &bio_ssor->_mfio_pin , GPIO_PIN_RESET );
    HAL_Delay( 4 );
    write_gpio_t( &bio_ssor->_reset_pin, GPIO_PIN_SET ); // Pull high
    HAL_Delay(50);

    // To permanantly enter, bootloader mode command is needed.
    // Send these three bytes  ( FAMILY==0x01 INDEX==0x00 WRITE_BYTE==0x08 ).
    uint8_t resp_byte;

    resp_byte = write_byte( bio_ssor, SET_DEVICE_MODE, 0x00, BOOTLOADER_MODE );
    if ( resp_byte != O2_SUCCESS )
        {
        printf( "Issue with write transaction (device mode command into bootloader mode)\n\r" );
        return ERR_TRY_AGAIN;
        } // end if
    
    resp_byte = read_byte( bio_ssor, READ_DEVICE_MODE, 0x00 );
    return resp_byte; // Should return BOOTLOADER_MODE

    /*
        PD0 = RTSN
        PD1 = MFIO

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0,  0);
    HAL_Delay(6);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1,  0);
    HAL_Delay(4);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0,  1);
    HAL_Delay(50);


    //HAL_Delay(1000);
    buf[0] = 0x01; // 
    //buf[1] = 0x08;
    buf[1] = 0x00;
    buf[2] = 0x08;
    ret = HAL_I2C_Master_Transmit(&hi2c1, Write_HM, &buf[0], 3, 5000);
    HAL_Delay(2);

    printf("%d HAL bool\n\r", ret == HAL_OK);
    ret = HAL_I2C_Master_Receive(&hi2c1, Read_HM, &buf[0], 1, 5000);
    if(buf[0] != 0x00 || ret != HAL_OK ){
        printf("Error setting bootloader: code %x\n\r", buf[0]);
    }
    */
    } // end enter_bootloader( )


// This function uses the given family, index, and write byte to communicate
// with the MAX32664 which in turn communicates with downward sensors. There
// are two steps demonstrated in this function. First a write to the MCU
// indicating what you want to do, a delay, and then a read to confirm positive
// transmission.
uint8_t write_byte( struct SparkFun_Bio_Sensor * const bio_ssor, const uint8_t family_byte, const uint8_t index_byte, uint8_t write_byte )
    {
    const uint8_t buff[ 3 ] = { family_byte, index_byte, write_byte };
    uint8_t status_byte;
    HAL_StatusTypeDef ret;

    const uint8_t write_hm_c = bio_ssor->_addr << 1; // LSB is low to indicate write...
    const uint8_t read_hm_c  = ( bio_ssor->_addr << 1 ) | 1; // LSB is high to indicate read...

    ret = HAL_I2C_Master_Transmit( bio_ssor->_i2c_h, write_hm_c, buff, sizeof( buff ), 0xFFFF );
    if ( ret != HAL_OK )
       {
       printf( "Issue with specifying index (write transmission)\n\r" );
       exit( 1 );
       } // end if
    HAL_Delay( CMD_DELAY );

    // Obtain status byte
    ret = HAL_I2C_Master_Receive( bio_ssor->_i2c_h, read_hm_c, &status_byte, sizeof( status_byte ), 0xFFFF );
    return ret;
    } // end write_byte( )

// This function handles all read commands or stated another way, all information
// requests. It starts a request by writing the family byte an index byte, and
// then delays 60 microseconds, during which the MAX32664 retrieves the requested
// information. An I-squared-C request is then issued, and the information is read.
uint8_t read_byte( struct SparkFun_Bio_Sensor const * const bio_ssor, const uint8_t family_byte, const uint8_t index_byte )
    {
    uint8_t *buff;
    const uint16_t buff_len = sizeof( family_byte ) + sizeof( index_byte );
    buff = ( uint8_t * ) malloc( buff_len );
    buff[ 0 ] = family_byte;
    buff[ 1 ] = index_byte;


    uint8_t ret_byte, status_byte;
    HAL_StatusTypeDef ret;
    const uint8_t write_hm_c = bio_ssor->_addr << 1; // LSB is low to indicate write...
    const uint8_t read_hm_c  = ( bio_ssor->_addr << 1 ) | 1; // LSB is high to indicate read...

    ret = HAL_I2C_Master_Transmit( bio_ssor->_i2c_h, write_hm_c, buff, buff_len, 0xFFFF );
    if ( ret != HAL_OK )
       {
       printf( "Issue with specifying index (write transmission)\n\r" );
       exit( 1 );
       } // end if

    // Wait for 2ms for device fordevice to have data.
    HAL_Delay( CMD_DELAY );

    // Now request read two bytes (we are asking for )
    ret = HAL_I2C_Master_Receive( bio_ssor->_i2c_h, read_hm_c, buff, buff_len, 0xFFFF );

    status_byte = buff[ 0 ];

    if ( status_byte != O2_SUCCESS ) // Success
        return status_byte; // Something went wrong!
    
    ret_byte = buff[ 1 ];
    free ( buff );
    return ret_byte;
    } // end read_byte

void process_status_byte( const enum READ_STATUS_BYTE_VALUE status_byte )
    {
    switch( status_byte )
    {
    case( ERR_UNAVAIL_CMD ):
        printf( "ERR_UNAVAIL_CMD. Illegal Family Byte and/or Command Byte was used.\n\r" );
        break;
    case( ERR_UNAVAIL_FUNC ):
        printf( "ERR_UNAVAIL_FUNC. This function is not implemented.\n\r" );
        break;
    case( ERR_DATA_FORMAT ):
        printf( "ERR_DATA_FORMAT. Incorrect number of bytes sent for the requested Family Byte.\n\r" );
        break;
    case( ERR_INPUT_VALUE ):
        printf( "ERR_INPUT_VALUE. Illegal configuration value was attempted to be set.\n\r" );
        break;
    case( ERR_TRY_AGAIN ):
        printf( "ERR_TRY_AGAIN. Device is busy. Try again.\n\r" );
        break;
    case( ERR_BTLDR_GENERAL ):
        printf( "ERR_BTLDR_GENERAL. General error while receiving/flashing a page during the bootloader sequence.\n\r" );
        break;
    case( ERR_BTLDR_CHECKSUM ):
        printf( "ERR_BTLDR_CHECKSUM. Checksum error while decrypting/checking page data.\n\r" );
        break;
    case( ERR_BTLDR_AUTH ):
        printf( "ERR_BTLDR_AUTH. Authorization error.\n\r" );
        break;
    case( ERR_BTLDR_INVALID_APP ):
        printf( "ERR_BTLDR_INVALID_APP. Application not valid.\n\r" );
        break;
    case( ERR_UNKNOWN ): // Probably just as bad as an unknown status_bytes
        printf( "ERR_UNKNOWN. Unknown Error.\n\r" );
        break;
    default:
        printf( "Unknown status byte%d\n\r", status_byte );
        exit( 1 );
    } // end switch
    } // end process_status_byte( )
