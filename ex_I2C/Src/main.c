
/*--------------------------I2C------------------------------*/

#include <stdint.h>
#include <string.h>

#define GPIOA_BASEADDR			0x40020000UL
#define GPIOB_BASEADDR			0x40020400UL
#define GPIO_MODER_OFFSET		0x00
#define GPIO_TYPER_OFFSET		0x04
#define GPIO_OSPEEDR_OFFSET		0x08
#define GPIO_PUPDR_OFFSET		0x0C
#define GPIO_IDR_OFFSET			0x10
#define GPIO_ODR_OFFSET			0x14
#define GPIO_AFLR_OFFSET		0x20
#define GPIO_AFHR_OFFSET		0x24

#define RCC_BASE_ADDR			0x40023800UL
#define RCC_AHB1ENR_OFFSET		0x30
#define RCC_APB1ENR_OFFSET		0x40

#define APB1_BASEADDR			0x40000000UL
#define I2C1_BASEADDR			0x40005400UL
#define I2C1_CR1_OFFSET			0x00
#define I2C1_CR2_OFFSET			0x04
#define I2C1_OAR1_OFFSET		0x08
#define I2C1_DR_OFFSET			0x10
#define I2C1_SR1_OFFSET			0x14
#define I2C1_SR2_OFFSET			0x18
#define I2C1_CCR_OFFSET			0x1C
#define I2C1_TRISE_OFFSET		0x20


#define MY_ADDR   	0x61
#define SM_SPEED	100000

#define SLAVE_ADDR  0x68
uint8_t data[] = "We are testing I2C master Tx\n";

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

/*
 * PB6 --> I2C1_SCL
 * PB9 --> I2C1_SDA
 * ALT function mode : 4
 * I2C1-->APB1 (142 MHz)
 */


uint32_t *pGPIOAModeReg=(uint32_t*)(GPIOA_BASEADDR+GPIO_MODER_OFFSET);
uint32_t *pGPIOATypeReg=(uint32_t*)(GPIOA_BASEADDR+GPIO_TYPER_OFFSET);
uint32_t *pGPIOAOSpeedReg=(uint32_t*)(GPIOA_BASEADDR+GPIO_OSPEEDR_OFFSET);
uint32_t *pGPIOAPupdReg=(uint32_t*)(GPIOA_BASEADDR+GPIO_PUPDR_OFFSET);
uint32_t *pGPIOAAFLReg=(uint32_t*)(GPIOA_BASEADDR+GPIO_AFLR_OFFSET);

uint32_t *pGPIOBModeReg=(uint32_t*)(GPIOB_BASEADDR+GPIO_MODER_OFFSET);
uint32_t *pGPIOBTypeReg=(uint32_t*)(GPIOB_BASEADDR+GPIO_TYPER_OFFSET);
uint32_t *pGPIOBOSpeedReg=(uint32_t*)(GPIOB_BASEADDR+GPIO_OSPEEDR_OFFSET);
uint32_t *pGPIOBPupdReg=(uint32_t*)(GPIOB_BASEADDR+GPIO_PUPDR_OFFSET);
uint32_t *pGPIOBAFLReg=(uint32_t*)(GPIOB_BASEADDR+GPIO_AFLR_OFFSET);
uint32_t *pGPIOBAFHReg=(uint32_t*)(GPIOB_BASEADDR+GPIO_AFHR_OFFSET);

uint32_t *pI2C1CR1Reg=(uint32_t*)(I2C1_BASEADDR+I2C1_CR1_OFFSET);
uint32_t *pI2C1SR1Reg=(uint32_t*)(I2C1_BASEADDR+I2C1_SR1_OFFSET);
uint32_t *pI2C1SR2Reg=(uint32_t*)(I2C1_BASEADDR+I2C1_SR2_OFFSET);
uint32_t *pI2C1DRReg=(uint32_t*)(I2C1_BASEADDR+I2C1_DR_OFFSET);

void I2C1GPIOInit(uint8_t PinNo)
{
	*pGPIOBModeReg &= ~(0x3<< (PinNo*2));
	*pGPIOBModeReg |= (0x2<<(PinNo*2)); //assignment to AF.

	*pGPIOBPupdReg &= ~(0x3<<(PinNo*2));
	*pGPIOBPupdReg |= (0x1<<(PinNo*2));

	*pGPIOBTypeReg &= ~(1<< PinNo);
	*pGPIOBTypeReg |= (1<< PinNo);

	*pGPIOBOSpeedReg &= ~(0x3<<(PinNo*2));
	*pGPIOBOSpeedReg |= (0x2<<(PinNo*2));

	if(PinNo<8)
	{
		*pGPIOBAFLReg &=~(0xF<<(PinNo*4));
		*pGPIOBAFLReg |=(0x4<<(PinNo*4)); // assign PB6 to AF4
	}
	else
	{
		*pGPIOBAFHReg &=~(0xF<<((PinNo-8)*4));
		*pGPIOBAFHReg |=(0x4<<((PinNo-8)*4)); // assign PB9 to AF4
	}
}

void I2C_SendData(uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr)
{
	*pI2C1CR1Reg |= ( 1 << 8);  //START condition (S)

	while((*pI2C1SR1Reg & (1<<10)))
	{
		*pI2C1CR1Reg |= ( 1 << 8);  //START condition
	}

	while(!((*pI2C1SR1Reg>>0)&((1<<0)))); //(EV5:SB=1.S condition check edilir yani,SB burada okunur.)

	SlaveAddr = (SlaveAddr << 1);
	SlaveAddr &= ~(1); //SlaveAddr is Slave address + r/nw bit=0. Then, Slave Addr+r/w is send to DR.
	*pI2C1DRReg = SlaveAddr;  // (SR1_SB biti okunarak,ardından Slave adres DR'e yazılarak SB=0 olur.)

	while(!((*pI2C1SR1Reg)&(1<<1))); //(EV6:ADDR=1. ADDR biti check edilir,yani ADDR okunur )

	//clear the ADDR flag ( read SR1 , read SR2)
	uint32_t dummy_read = *pI2C1SR1Reg;
	dummy_read = *pI2C1SR2Reg;
	(void)dummy_read; //(SR1_ADDR biti okunarak, ardından SR1 ve SR2 okunarak ADDR=0 olur.)

	while(Len > 0)
	{
		while(! (*pI2C1SR1Reg & (1<<7)) ); //Wait till TXE is set (EV8_1:TXE=1,then write Data1 to DR.)
		*pI2C1DRReg =*pTxBuffer;
		Len--;
		pTxBuffer++;
	}
	//EV8_1'de TXE=1-->DR empty.Shift Reg. empty.Bu aşamada data'yı DR'e yaz.
	//EV8:TXE=1-->DR empty. Shift Reg NOT empty.DR'e data yazıldığı için Shift dolu.Shift dolu olduğu için DR boş.

	//when Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition. (EV8_2)
	while( ! (*pI2C1SR1Reg & (1<<7)) );//1. wait until TXE is set(empty)
	while( ! (*pI2C1SR1Reg & (1<<2)) );//1. wait until BTF is set

	*pI2C1CR1Reg |=(1<<9);  //STOP Condition
}

int main(void)
{
	uint32_t *pRccAhb1EnrReg=(uint32_t*)(RCC_BASE_ADDR+RCC_AHB1ENR_OFFSET);
	*pRccAhb1EnrReg &=~(1<<0);
	*pRccAhb1EnrReg |=(1<<0);  //GPIOA clock Enable
	*pRccAhb1EnrReg |=(1<<1);  //GPIOB clock Enable

	uint8_t SCLpin=6;
	uint8_t SDApin=9;

	/******** Button PA0*********/
	*pGPIOAModeReg &= ~(0x3<<0);  //PA0 -Input

	*pGPIOAOSpeedReg &= ~(0x3<<0);
	*pGPIOAOSpeedReg |= (0x2<<0); // PA0-Fast

	*pGPIOAPupdReg &= ~(0x3<<0); // PA0- no pull-up pull-down
	//*pGPIOAPupdReg |= (0x1<<0);
	/***************************/

	I2C1GPIOInit(SCLpin);	//SCL--PB6
	I2C1GPIOInit(SDApin);	//SDA--PB9

	uint32_t *pRccApb1EnrReg=(uint32_t*)(RCC_BASE_ADDR+RCC_APB1ENR_OFFSET);
	*pRccApb1EnrReg |=(1<<21);  //I2C1 Clock Enable

	uint32_t TempReg=0;
	TempReg |=(1<<10);
	*pI2C1CR1Reg =TempReg; //ACK enable

	TempReg=0;
	uint32_t SystemClk= 16000000; //Default Source:HSI-->16 MHz
	TempReg |= (SystemClk/1000000);
	uint32_t *pI2C1CR2Reg=(uint32_t*)(I2C1_BASEADDR+I2C1_CR2_OFFSET);
	*pI2C1CR2Reg =(TempReg & 0x3F);

	TempReg=0;
	uint32_t *pI2C1Oar1Reg=(uint32_t*)(I2C1_BASEADDR+I2C1_OAR1_OFFSET);
	TempReg |=(MY_ADDR<<1);
	TempReg |=(1<<14); //RM'de OAR1 Bit 14=1 olmalı.
	*pI2C1Oar1Reg =TempReg;

	TempReg=0;
	uint32_t *pI2C1CcrReg=(uint32_t*)(I2C1_BASEADDR+I2C1_CCR_OFFSET);
	uint16_t ccr_value = ( SystemClk / ( 2 * SM_SPEED ) ); //CCR calculation
	TempReg |= (ccr_value & 0xFFF);
	*pI2C1CcrReg=TempReg;

	TempReg=0;
	TempReg = (SystemClk/1000000U) + 1 ;  //TRISE calculation
	uint32_t *pI2CTriseReg=(uint32_t*)(I2C1_BASEADDR+I2C1_TRISE_OFFSET);
	*pI2CTriseReg = (TempReg & 0x3F);

	*pI2C1CR1Reg |=(1<<0);  //PE:I2C peripheral enable.

	uint32_t *GPIOAIdrReg=(uint32_t*)(GPIOA_BASEADDR+GPIO_IDR_OFFSET);

	while(1)
	{
		while (!((*GPIOAIdrReg ) & 0x00000001)); //wait until press the Btn.
		delay();
		I2C_SendData(data, strlen((char*)data),SLAVE_ADDR);
	}
}
