
#include <stdio.h>
#include <stm32f1xx.h>
#include "pca9685_servo.h"
#include "math.h"


extern I2C_HandleTypeDef hi2c2;



void PCA9685_Reset()
{
    PCA9685_write(PCA9685_MODE1, 0x00);
}

void PCA9685_Go()
{
    PCA9685_Reset();
}

void SetPWMFreq(float freq)
{
    uint32_t prescale,oldmode,newmode;
    float prescaleval;
    freq *= 0.95;  // Correct for overshoot in the frequency setting
    prescaleval = 25000000;
    prescaleval /= 4096;
    prescaleval /= freq;
    prescaleval -= 1;
    prescale = floor(prescaleval + 0.5);

    oldmode = PCA9685_read(PCA9685_MODE1);
    newmode = (oldmode&0x7F) | 0x10; // sleep
    PCA9685_write(PCA9685_MODE1, newmode); // go to sleep(
    PCA9685_write(PCA9685_PRESCALE, prescale); // set the prescaler
    PCA9685_write(PCA9685_MODE1, oldmode);
    PCA9685_write(PCA9685_MODE1, oldmode | 0xa1);

}

uint16_t calculate_PWM(uint16_t angle){
	float pos;
	pos = (float)angle/90;
  return (uint16_t)(204.8*(0.5+pos));
}

//void setServo(uint32_t num,uint32_t on,uint32_t off)
void setServo(uint32_t num,uint32_t off)
{
    PCA9685_write(LED0_ON_L+4*num,0);
    PCA9685_write(LED0_ON_H+4*num,0>>8);
    PCA9685_write(LED0_OFF_L+4*num,off);
    PCA9685_write(LED0_OFF_H+4*num,off>>8);
}

uint8_t PCA9685_read(uint8_t startAddress) {
    //Send address to start reading from.
    uint8_t tx[1];
    uint8_t buffer[1];
    tx[0]=startAddress;

    HAL_I2C_Master_Transmit(&hi2c2,PCA9685_adrr, tx,1,10000);
    HAL_I2C_Master_Receive(&hi2c2,PCA9685_adrr,buffer,1,10000);
    return buffer[0];
}

void PCA9685_write(uint8_t startAddress, uint8_t buffer) {
    //Send address to start reading from.
    uint8_t tx[2];
    tx[0]=startAddress;
    tx[1]=buffer;
    HAL_I2C_Master_Transmit(&hi2c2,PCA9685_adrr, tx,2,10000);
}


