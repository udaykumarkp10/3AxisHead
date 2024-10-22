/*
 * pcap.h
 *
 *  Created on: May 22, 2024
 *      Author: LAB
 */

#ifndef INC_PCAP_H_
#define INC_PCAP_H_

#include <project.h>
#include "lan9252.h"
#include "moving_average.h"

// #define PCAP_ADDR 0x28
#define PCAP_ADDR 0x50
#define PCAP_TEST_ADDR 0x7e
#define PCAP_TEST_DATA 0x11
#define PCAP_CAP_ADDR 0x40 //result reg 0
#define PCAP_TEMP_ADDR 0x58 // result reg 24
#define PCAP_STS_ADDR 0x61 // result reg 33
#define PCAP_MAX_DELAY 500


#define PCAP_TIP_TOUCH 4294967295

#define DEBOUNCE_DELTA_CAPACITANCE	4198000000
#define DEBOUNCE_TIME_US 150
#define DEBOUNCE_CAP_TOLERANCE 100000000

#define N_PCAP_VALUE 10

// Enum for Pcap
typedef enum {
    PCAP_NO_ERROR = 0,
    PCAP_COMM_ERROR = 1,
    PCAP_TIP_TOUCH_ERROR = 2,
} PcapErrorStatus;


bool pcap_init();
PcapErrorStatus pcap_scan();
uint16_t getEventStatusWord_PCAP(PcapErrorStatus *pcap_status);

void delay_us(uint16_t us);


#endif /* INC_PCAP_H_ */
