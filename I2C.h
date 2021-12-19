/*
 * I2C.h
 *
 * Created: 12/19/2021 9:59:46 AM
 *  Author: arjun
 */ 


#ifndef I2C_H_
#define I2C_H_



#define I2C_READ 1
#define I2C_WRITE 0


#include <avr/io.h>
#include <stdint.h>



//function definitions

void init_i2c();
unsigned char i2c_start_w_address(unsigned char address);
void i2c_start_wait(unsigned char address);
unsigned char i2c_write( unsigned char data );
void i2c_read_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data);
void i2c_write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);
unsigned char i2c_readAck(void);
unsigned char i2c_readNak(void);
unsigned char i2c_rep_start(unsigned char address);
void i2c_stop(void);



#endif /* I2C_H_ */
