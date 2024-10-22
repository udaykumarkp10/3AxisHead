/*
 * lan9252.h
 *
 *  Created on: May 22, 2024
 *      Author: LAB
 */

#ifndef INC_LAN9252_H_
#define INC_LAN9252_H_

#include <project.h>

// LAN9252 Data Sheet TABLE 5-1: SYSTEM CONTROL AND STATUS REGISTERS

//---- access to EtherCAT registers -------------------
#define RESET_CTL               0x01F8
#define ECAT_CSR_DATA           0x0300
#define ECAT_CSR_CMD            0x0304
#define ECAT_PRAM_RD_ADDR_LEN   0x0308
#define ECAT_PRAM_RD_CMD        0x030C
#define ECAT_PRAM_WR_ADDR_LEN   0x0310
#define ECAT_PRAM_WR_CMD        0x0314
#define ECAT_PRAM_RD_DATA       0x0000
#define ECAT_PRAM_WR_DATA       0x0020
#define ID_REV                  0x0050
#define IRQ_CFG                 0x0054
#define INT_STS                 0x0058
#define INT_EN                  0x005C
#define BYTE_TEST               0x0064
#define HW_CFG                  0x0074
#define PMT_CTRL                0x0084
#define GPT_CFG                 0x008C
#define GPT_CNT                 0x0090
#define FREE_RUN                0x009C

// LAN9252 Datasheet TABLE 12-15: ETHERCAT CORE CSR REGISTERS
#define TYPE_REG        		0x0000
#define REV_REG         		0x0001
#define BUILD_REG_1       		0x0002
#define BUILD_REG_2       		0x0003
#define FMMU_REG        		0x0004
#define SYNCMANAGER_REG     	0x0005
#define RAM_SIZE_REG      		0x0006
#define PORT_DESCR_REG      	0x0007
#define ESC_FEATUR_REG_1    	0x0008
#define ESC_FEATUR_REG_2    	0x0009
#define CONF_STATION_REG_1    	0x0010
#define CONF_STATION_REG_2    	0x0011
#define CONF_STATION_ALI_REG_1  0x0012
#define CONF_STATION_ALI_REG_2  0x0013

// Write Protection Register
#define WD_REG_EN       		0x0020
#define WD_REG_PR       		0x0021
#define ESC_WD_REG_EN     		0x0030
#define ESC_WD_REG_PR     		0x0031

// Data Link Layer
#define ESC_RST_REG       		0x0040
#define ESC_RST_PDI_REG     	0x0041
#define ECL_DL_CTRL_REG_0   	0x0100
#define ECL_DL_CTRL_REG_1   	0x0101
#define ECL_DL_CTRL_REG_2   	0x0102
#define ECL_DL_CTRL_REG_3   	0x0103
#define PHY_R_W_OFF_1     		0x0108
#define PHY_R_W_OFF_2     		0x0109
#define ECL_DL_STATUS_REG_0   	0x0110
#define ECL_DL_STATUS_REG_1   	0x0111

// Application Layer
#define AL_CTRL_REG_0     		0x0120
#define AL_CTRL_REG_1     		0x0121
#define AL_STATUS_REG_0     	0x0130
#define AL_STATUS_REG_1     	0x0131
#define AL_STATUS_COD_REG_0   	0x0134
#define AL_STATUS_COD_REG_1   	0x0135
#define RUN_LED_OVERRIDE_REG  	0x0138

// PDI (Process Data Interface)
#define PDI_CTRL_REG      		0x0140
#define ESC_CONF_REG      		0x0141
#define ASIC_CONF_REG_0     	0x0142
#define ASIC_CONF_REG_1     	0x0143
#define PDI_CONF_REG      		0x0150
#define SYNC_PDI_CONF_REG     	0x0151
#define EXT_PDI_CONF_REG_0    	0x0152
#define EXT_PDI_CONF_REG_1    	0x0153

// Interrupts
#define ECAT_EVENT_MASK_REG_0 	0x0200
#define ECAT_EVENT_MASK_REG_1 	0x0201
#define AL_EVENT_MASK_REG_0   	0x0204
#define AL_EVENT_MASK_REG_1   	0x0205
#define AL_EVENT_MASK_REG_2   	0x0206
#define AL_EVENT_MASK_REG_3   	0x0207
#define ECAT_EVENT_REQ_REG_0  	0x0210
#define ECAT_EVENT_REQ_REG_1  	0x0211
#define AL_EVENT_REQ_REG_0    	0x0220
#define AL_EVENT_REQ_REG_1    	0x0221
#define AL_EVENT_REQ_REG_2    	0x0222
#define AL_EVENT_REQ_REG_3    	0x0223

// Error Counters
#define RX_ERROR_CNT_REG_0    	0x0300
// ....
#define RX_ERROR_CNT_REG_7    	0x0307
#define FWD_RX_ERROR_CNT_REG_0  0x0308
// ....
#define FWD_RX_ERROR_CNT_REG_B  0x030B
#define ECAT_PRO_UNIT_CNT_ERROR 0x030C
#define PDI_CNT_ERROR     		0x030D
#define PDI_CODE_ERROR      	0x030E
#define LOST_LINK_CNT_REG_0   	0x0310
// ....
#define LOST_LINK_CNT_REG_3   	0x0313

// EEPROM Interface
#define EEPROM_CONF_REG     	0x0500
#define EEPROM_PDI_STATE_REG  	0x0501
#define EEPROM_CTRL_REG_0     	0x0502
#define EEPROM_CTRL_REG_1     	0x0503
#define EEPROM_ADDR_REG_0     	0x0504
// ....
#define EEPROM_ADDR_REG_4     	0x0507
#define EEPROM_DATA_REG_0     	0x0508
// ....
#define EEPROM_DATA_REG_4     	0x050B

// MII Management Interface
#define MII_MANAGE_CTRL_REG_0 	0x0510
#define MII_MANAGE_CTRL_REG_1 	0x0511
#define PHY_ADDR_REG      		0x0512
#define PHY_REGISTER_ADDR_REG 	0x0513
#define PHY_DATA_REG_0      	0x0514
#define PHY_DATA_REG_1      	0x0515
#define MII_MANAGE_ECAT_REG   	0x0516
#define MII_MANAGE_PDI_REG    	0x0517
#define AL_STATUS               0x0130
#define WDOG_STATUS             0x0440

// LAN9252 flags
#define ECAT_CSR_BUSY     		0x80
#define PRAM_READ_BUSY    		0x80
#define PRAM_READ_AVAIL   		0x01
#define PRAM_WRITE_AVAIL  		0x01
#define READY             		0x08000000
#define DIGITAL_RST       		0x00000001
#define ETHERCAT_RST      		0x00000040

// EtherCAT flags
#define ESM_INIT                0x01                  // init
#define ESM_PREOP               0x02                  // pre-operational
#define ESM_BOOT                0x03                  // bootstrap
#define ESM_SAFEOP              0x04                  // safe-operational
#define ESM_OP                  0x08                  // operational

// ESC commands
#define ESC_WRITE        		0x80
#define ESC_READ         		0xC0

// SPI
#define COMM_SPI_READ    		0x03
#define COMM_SPI_WRITE   		0x02
#define DUMMY_BYTE       		0xFF

//defs to define word, long as a series of bytes
typedef union{
	uint16_t LANWord;
	uint8_t LANByte[2];
} UWORD;

typedef union{
	uint32_t LANLong;
	uint16_t LANWord[2];
	uint8_t LANByte[4];
} ULONG;

//process buffer 1 dword
typedef union{
	uint8_t LANByte[32];
	uint32_t LANLong[8];
	float LANFloat[8];
	int32_t LANInt[8];
} PROCBUFFER;

extern PROCBUFFER Etc_Buffer_Out;
extern PROCBUFFER Etc_Buffer_In;

uint32_t Etc_Read_Reg(uint16_t address, uint8_t length);
void Etc_Write_Reg(uint16_t address, uint32_t DataOut);
uint32_t Etc_Read_Reg_Wait(uint16_t address, uint8_t length);
void Etc_Write_Reg_Wait(uint16_t address, uint32_t DataOut);
void Etc_Read_Fifo();
void Etc_Write_Fifo();
bool etc_init();
uint8_t etc_scan();


#endif /* INC_LAN9252_H_ */
