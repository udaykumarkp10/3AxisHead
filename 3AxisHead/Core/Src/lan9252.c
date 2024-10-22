/*
 * lan9252.c
 *
 *  Created on: Sep 29, 2023
 *      Author: Rajdeep
 */
#include "lan9252.h"

// etc routines
PROCBUFFER Etc_Buffer_Out = {.LANByte = 0};
PROCBUFFER Etc_Buffer_In = {.LANByte = 0};

//reads a directly addressable register
//address = register to read, length = number of bytes to read (1,2,3,4), long is returned but only the requested bytes are meaningful, starting from LsByte
uint32_t Etc_Read_Reg(uint16_t address, uint8_t length){
	ULONG Result;
	UWORD Addr;
	uint8_t i;
	uint8_t xfrbuf[7];				//buffer for spi xfr

	Addr.LANWord = address;
	xfrbuf[0] = COMM_SPI_READ;		//SPI read command
	xfrbuf[1] = Addr.LANByte[1];	//address of the register
	xfrbuf[2] = Addr.LANByte[0];	//to read MSByte first
	for(i=0; i< length; i++){		//fill dummy bytes
		xfrbuf[i+3] = DUMMY_BYTE;
	}

	LAN9252_CS_SET();						//send L+3 bytes and get back into same buffer
	HAL_SPI_TransmitReceive(&LAN9252_PORT, xfrbuf, xfrbuf, length+3, HAL_MAX_DELAY);
	LAN9252_CS_RESET();

	Result.LANLong = 0;
	for(i=0; i<length; i++){		//read the requested number of bytes LSByte first
		Result.LANByte[i] = xfrbuf[i+3];
	}
	return Result.LANLong;
}

// write a directly addressable register, 4 bytes always
// Address = register to write, DataOut = data to write
void Etc_Write_Reg(uint16_t address, uint32_t DataOut)
{
	ULONG Data;
	UWORD Addr;
	uint8_t i;
	uint8_t xfrbuf[7]; 				// buffer for spi xfr

	Addr.LANWord = address;
	Data.LANLong = DataOut;
	xfrbuf[0] = COMM_SPI_WRITE;     // SPI write command
	xfrbuf[1] = Addr.LANByte[1];    // address of the register
	xfrbuf[2] = Addr.LANByte[0];    // to read, MsByte first
	for (i=0; i<4; i++)
	{
		xfrbuf[i+3] = Data.LANByte[i];	// fill bytes to send, lsb first
	}

	LAN9252_CS_SET();						//send 7 bytes and get back into same bfr
	HAL_SPI_TransmitReceive(&LAN9252_PORT, xfrbuf, xfrbuf, 7, HAL_MAX_DELAY);
	LAN9252_CS_RESET();
}

// read an indirectly addressable register
uint32_t Etc_Read_Reg_Wait(uint16_t address, uint8_t length)
{
	ULONG TempLong;
	UWORD Addr;

	Addr.LANWord = address;
	TempLong.LANByte[0] = Addr.LANByte[0];    				//address of the register
	TempLong.LANByte[1] = Addr.LANByte[1];    				//to read, LsByte first
	TempLong.LANByte[2] = length;                 			//number of bytes to read
	TempLong.LANByte[3] = ESC_READ;               			// ESC read

	Etc_Write_Reg(ECAT_CSR_CMD, TempLong.LANLong);			// write the command
	TempLong.LANByte[3] = ECAT_CSR_BUSY;

	do{														// wait for command execution
		TempLong.LANLong = Etc_Read_Reg(ECAT_CSR_CMD, 4);
	}while(TempLong.LANByte[3] & ECAT_CSR_BUSY);

	TempLong.LANLong = Etc_Read_Reg(ECAT_CSR_DATA, length);   // read the requested register

	return TempLong.LANLong;
}

// write an indirectly addressable register, 4 bytes always
void Etc_Write_Reg_Wait(uint16_t address, uint32_t DataOut)
{
	ULONG TempLong;
	UWORD Addr;

	Addr.LANWord = address;
	Etc_Write_Reg(ECAT_CSR_DATA, DataOut);                 	// write the data

	// compose the command
	TempLong.LANByte[0] = Addr.LANByte[0];                 	// address of the register
	TempLong.LANByte[1] = Addr.LANByte[1];                 	// to write, LsByte first
	TempLong.LANByte[2] = 4;                               	// we write always 4 bytes
	TempLong.LANByte[3] = ESC_WRITE;                       	// ESC write

	Etc_Write_Reg(ECAT_CSR_CMD, TempLong.LANLong);         	// write the command
	TempLong.LANByte[3] = ECAT_CSR_BUSY;

	do{														// wait for command execution
		TempLong.LANLong = Etc_Read_Reg(ECAT_CSR_CMD, 4);
	}while(TempLong.LANByte[3] & ECAT_CSR_BUSY);
}

// read from process ram fifo
void Etc_Read_Fifo()
{
	ULONG TempLong;
	uint8_t xfrbuf[35]; 										// buffer for spi xfr
	uint8_t i;

	Etc_Write_Reg(ECAT_PRAM_RD_ADDR_LEN, 0x00201000);   		// we always read 32 bytes (0x0020), output process ram offset 0x1000
	Etc_Write_Reg(ECAT_PRAM_RD_CMD, 0x80000000);        		// start command
	TempLong.LANLong = 0;
	do{                                                   		// wait for data to be transferred                                                   // from the output process ram
		TempLong.LANLong = Etc_Read_Reg(ECAT_PRAM_RD_CMD, 4); 	// to the read fifo
	}while (!(TempLong.LANByte[0] & PRAM_READ_AVAIL) || (TempLong.LANByte[1] != 8));

	xfrbuf[0] = COMM_SPI_READ;                                	// SPI read command
	xfrbuf[1] = 0x00;                                         	// address of the read
	xfrbuf[2] = 0x00;                                         	// fifo MsByte first
	for (i=0; i<32; i++)                                      	// 32 bytes dummy data
	{
		xfrbuf[i+3] = DUMMY_BYTE;
	}

	LAN9252_CS_SET();													//send 35 bytes and get back into same buffer
	HAL_SPI_TransmitReceive(&LAN9252_PORT, xfrbuf, xfrbuf, 35, HAL_MAX_DELAY);
	LAN9252_CS_RESET();

	for (i=0; i<32; i++)                                   		// 32 bytes read data to usable buffer
	{
		Etc_Buffer_Out.LANByte[i] = xfrbuf[i+3];
	}
}

// write to the process ram fifo
void Etc_Write_Fifo()    										// write 32 bytes to the input process ram, through the fifo
{
	ULONG TempLong;
	uint8_t xfrbuf[35]; // buffer for spi xfr
	uint8_t i;

	Etc_Write_Reg(ECAT_PRAM_WR_ADDR_LEN, 0x00201200);   		// we always write 32 bytes (0x0020), input process ram offset 0x1200
	Etc_Write_Reg(ECAT_PRAM_WR_CMD, 0x80000000);        		// start command
	TempLong.LANLong = 0;
	do{                                                   		// check fifo has available space
		TempLong.LANLong = Etc_Read_Reg(ECAT_PRAM_WR_CMD, 4);	// for data to be written
	}
	while(!(TempLong.LANByte[0] & PRAM_WRITE_AVAIL) || (TempLong.LANByte[1] < 8));

	xfrbuf[0] = COMM_SPI_WRITE;                               	// SPI write command
	xfrbuf[1] = 0x00;                                         	// address of the write fifo
	xfrbuf[2] = 0x20;                                         	// MsByte first
	for (i=0; i<32; i++)                                      	// 32 bytes write loop
	{
		xfrbuf[i+3] = Etc_Buffer_In.LANByte[i];
	}

	LAN9252_CS_SET();													//send 35 bytes and get back into same buffer
	HAL_SPI_TransmitReceive(&LAN9252_PORT, xfrbuf, xfrbuf, 35, HAL_MAX_DELAY);
	LAN9252_CS_RESET();
}

// initialize / check the etc interface on SPI, return true if initialization is ok
bool etc_init()
{
	ULONG TempLong;

	Etc_Write_Reg(RESET_CTL, (DIGITAL_RST & ETHERCAT_RST)); 	// LAN9252 reset
	HAL_Delay(100);
	TempLong.LANLong = Etc_Read_Reg(BYTE_TEST, 4);             	// read test register

	if(TempLong.LANLong != 0x87654321)                     		// if the test register is not ok
	{
		/*printf("Bad response received from Etc Test command, data received = ");
		printf("%ld\n", TempLong.LANLong);*/
		return false;
	}

	TempLong.LANLong = Etc_Read_Reg(HW_CFG, 4);              	// check also the READY flag
	if((TempLong.LANLong & READY) == 0){
		/*printf("Ready not received from Etc HW Cfg, data received = ");
		printf("%ld\n", TempLong.LANLong);*/
		return false;
	}

	/*printf("Etc Test Command succeeded\n");*/
  	return true;
}

// one scan of etc
uint8_t etc_scan()
{
	bool WatchDog = 0;
	bool Operational = 0;
	uint8_t i;
	ULONG TempLong;
	uint8_t Status;

	TempLong.LANLong = Etc_Read_Reg_Wait(WDOG_STATUS, 1); 		// read watchdog status
	if ((TempLong.LANByte[0] & 0x01) == 0x01)
		WatchDog = 0;                                           // set/reset the corresponding flag
	else
	{
		WatchDog = 1;
		/*printf("Etc Watchdog active\n");*/
	}

	TempLong.LANLong = Etc_Read_Reg_Wait(AL_STATUS_REG_0, 1);   // read the EtherCAT State Machine status
	Status = TempLong.LANByte[0] & 0x0F;
	if (Status == ESM_OP){                                     	// to see if we are in operational state
		Operational = 1;
	}else{
		Operational = 0;                                        // set/reset the corresponding flag
		/*printf("Etc not operational\n");*/
	}

	//--- process data transfers ----------
	if (WatchDog | !Operational)                              	// if watchdog is active or we are
	{                                                         	// not in operational state, reset
		for (i=0; i<8; i++){                                    // the output buffer
			Etc_Buffer_Out.LANLong[i] = 0;                      //
		}
	}else{
		/*printf("Read fifo\n");*/
		Etc_Read_Fifo();                                        // otherwise transfer process data from
	}                                                         	// the EtherCAT core to the output buffer
	/*printf("Write fifo\n");*/
	Etc_Write_Fifo();                                         	// we always transfer process data from
                                                            	// the input buffer to the EtherCAT core

	if (WatchDog)                                             	// return the status of the State Machine
	{                                                         	// and of the watchdog
		Status |= 0x80;
	}
	return Status;
}
