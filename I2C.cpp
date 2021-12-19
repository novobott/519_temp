/*
 * I2C.c
 *
 * Created: 12/19/2021 9:35:47 AM
 *  Author: arjun
 */ 

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define SCL_CLOCK  400000L

#include "I2C.h"
#include <stdint.h>
#include<inttypes.h>
#include<compat/twi.h>

void init_i2c()
{
		PORTD &= ~(1 << 0); //Port D0 SCL
		PORTD &= ~(1 << 1); //Port D1 SDA
		TWBR = ((( F_CPU / SCL_CLOCK ) - 16) / 2);
		TWSR = 0;
		TWCR = ( 1 << TWEN ); // enable i2c bus
}


//i2c start with address.
//returns 1 if device not accessible
//returns 0 if device accessible
unsigned char i2c_start_w_address(unsigned char address)
{
	uint8_t   twstatus;

	
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN); // send start condn

	
	while(!(TWCR & (1<<TWINT))); //wait for its exec

	
	twstatus = TW_STATUS & 0xF8; //checking TWI status reg
	if ((twstatus != TW_START) && (twstatus != TW_REP_START)) return 1;

	// send device address
	TWDR = address;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// wail until transmission completed and ACK/NACK has been received
	while(!(TWCR & (1<<TWINT)));

	// check value of TWI Status Register. Mask prescaler bits.
	twstatus = TW_STATUS & 0xF8;
	if ( (twstatus != TW_MT_SLA_ACK) && (twstatus != TW_MR_SLA_ACK) ) return 1;

	return 0;
}

void i2c_start_wait(unsigned char address)
{
	 uint8_t   twst;


	 while ( 1 )
	 {
		 
		 TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN); //start condition
		 
		 
		 while(!(TWCR & (1<<TWINT))); // wait
		  
		 
		 twst = TW_STATUS & 0xF8;
		 if ( (twst != TW_START) && (twst != TW_REP_START)) continue;
		 
		 // send device address
		 TWDR = address;
		 TWCR = (1<<TWINT) | (1<<TWEN);
		 
		 
		 while(!(TWCR & (1<<TWINT))); // wait for completion
		 
		 
		 twst = TW_STATUS & 0xF8;
		 if ( (twst == TW_MT_SLA_NACK )||(twst ==TW_MR_DATA_NACK) )
		 {
			 /* device busy, send stop condition to terminate write operation */
			 TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
			 
			 // wait until stop condition is executed and bus released
			 while(TWCR & (1<<TWSTO));
			 
			 continue;
		 }
		 //if( twst != TW_MT_SLA_ACK) return 1;
		 break;
	 }


}

//use in tandem with i2c start or i2c start wait
unsigned char i2c_write( unsigned char data )
{
	uint8_t   twstatus;
	
	TWDR = data;
	TWCR = (1<<TWINT) | (1<<TWEN);

	
	while(!(TWCR & (1<<TWINT)));

	twstatus = TW_STATUS & 0xF8;
	if( twstatus != TW_MT_DATA_ACK) return 1;
	return 0;

}


void i2c_read_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data)
{

	i2c_start_wait(dev_addr+I2C_WRITE); 	//start i2c to write register address
	i2c_write(reg_addr);			//write address of register to read
	i2c_rep_start(dev_addr+I2C_READ);	//restart i2c to start reading
	*data = i2c_readNak();
	i2c_stop();

}


void i2c_write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
	
	i2c_start_wait(dev_addr+I2C_WRITE);
	i2c_write(reg_addr);
	i2c_write(data);
	i2c_stop();

}

unsigned char i2c_readAck(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	while(!(TWCR & (1<<TWINT)));    

    return TWDR;

}
unsigned char i2c_readNak(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
	
    return TWDR;

}

unsigned char i2c_rep_start(unsigned char address)
{
    return i2c_start_w_address( address );

}/* i2c_rep_start */




void i2c_stop(void)
{
    
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO); //stop condition
	
	
	while(TWCR & (1<<TWSTO));

}
