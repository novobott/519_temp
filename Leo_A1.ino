#include "I2C.h"
#include "MPU_6050.h"
#include "MPU_6050_reg.h"

#include <Mouse.h>

#define F_CPU 16000000UL
#include <avr/io.h>
#include <inttypes.h>
#include <stdint.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>
#include <avr/interrupt.h>
#include <math.h>


uint8_t ret;

int16_t accel_buff[3], gyro_buff[3];
int vx, vy;
void setup() {

  Mouse.begin();
  Serial.begin(9600);
  DDRB |= (1<<0);
  sei();
  init_i2c();
  ret = i2c_start_w_address(MPU6050_ADDRESS + I2C_WRITE);
  if(ret == 0){
    PORTB |= (1<<0);
    _delay_ms(1000);
    PORTB &= ~(1<<0);
  }
  mpu6050_init();

}

void loop() {
  mpu6050_read_gyro_ALL(gyro_buff);

  vx = (gyro_buff[0]+300)/150;  // change 300 from 0 to 355
  vy = -(gyro_buff[2]-0)/150; // same here about "-100"  from -355 to 0

  Serial.print("gx = ");
  Serial.print(gyro_buff[0]);
  
  Serial.print(" | gy = ");
  Serial.print(gyro_buff[1]);
  
  Serial.print(" | gz = ");
  Serial.print(gyro_buff[2]);
  Serial.print("\n");

  Mouse.move(vx, vy);
  delay(20);

}
