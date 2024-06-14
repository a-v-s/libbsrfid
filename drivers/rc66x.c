/*
 * rc66x.c
 *
 *  Created on: 28 dec. 2020
 *      Author: andre
 */

#include "rc66x.h"

#include "bshal_spim.h"
#include "bshal_gpio.h"

#include "rc66x_transport.h"

#include <string.h>

// expecting 0x18 or 0x1A
int rc66x_get_chip_version(rc66x_t *rc66x, uint8_t *chip_id) {
	return rc66x_recv(rc66x, RC66X_REG_Version, chip_id, 1);
}

void rc66x_antenna_on(rc66x_t *rc66x) {
	rc66x_or_reg8(rc66x, RC66X_REG_DrvMode, 0x03);

}
void rc66x_antenna_off(rc66x_t *rc66x) {
	rc66x_and_reg8(rc66x, RC66X_REG_DrvMode, ~0x03);
}

void rc66x_reset(rc66x_t *rc66x) {

//	// hard reset if available
//
//		// Note this one reset is active high!
//		bshal_gpio_write_pin(rc66x->transport_instance.spim->rs_pin,
//				rc66x->transport_instance.spim->rs_pol);
//		rc66x->delay_ms(1);
//		bshal_gpio_write_pin(rc66x->transport_instance.spim->rs_pin,
//				!rc66x->transport_instance.spim->rs_pol);
//
		// lese a soft reset
		rc66x_set_reg8(rc66x, RC66X_REG_Command, RC66X_CMD_SoftReset);


}

void rc66x_init(rc66x_t *rc66x) {
	if (!rc66x)
		return;
	if (!rc66x->get_time_ms)
		return;
	if (!rc66x->delay_ms)
		return;
	rc66x->TransceiveData = (void*)rc66x_transceive; //!!
	rc66x_reset(rc66x);

	// Translated from AN12657  4.1.1
	// 1. Cancels previous executions and the state machine returns into IDLE mode
	rc66x_set_reg8(rc66x, RC66X_REG_Command, RC66X_CMD_Idle);

	// 2. Flushes the FIFO and defines FIFO characteristics
	rc66x_set_reg8(rc66x, RC66X_REG_FIFOControl, 0xB0);

	// 3. Fills the FIFO with 0x00 and 0x00.
	uint8_t load_protocol_parameters[] = { 0x00, 0x00 };
	rc66x_send(rc66x, RC66X_REG_FIFOData, load_protocol_parameters,
			sizeof(load_protocol_parameters));

	// 4. Executes LoadProtocol command with parameters 0x00 and 0x00 (FIFO).
	// This	translates to load protocol ISO14443A - 106
	rc66x_set_reg8(rc66x, RC66X_REG_Command, RC66X_CMD_LoadProtocol);

	// 5. Flushes the FIFO and defines FIFO characteristics
	rc66x_set_reg8(rc66x, RC66X_REG_FIFOControl, 0xB0);

	// 6. Switches the RF filed ON.
	rc66x_set_reg8(rc66x, RC66X_REG_DrvMode, 0x8E);

	// 7. Clears all bits in IRQ0
	rc66x_set_reg8(rc66x, RC66X_REG_IRQ0, 0x7F);

	// 8. Switches the CRC extention OFF in tx direction
	rc66x_set_reg8(rc66x, RC66X_REG_TxCrcPreset, 0x18);

	// 9. Switches the CRC extention OFF in rx direction
	rc66x_set_reg8(rc66x, RC66X_REG_RxCrcPreset, 0x18);

	// The rest will go to the communicate with picc stuff

}

rc52x_result_t rc66x_transceive(rc66x_t *rc66x, uint8_t *sendData,
		uint8_t sendLen, uint8_t *recv_data, uint8_t *recv_size,
		uint8_t *validBits, uint8_t rxAlign, uint8_t *collpos, bool sendCRC,
		bool recvCRC) {
	uint8_t waitIRq = 0b00010110;		// RxIRq and IdleIRq + ErrIRQ

	// Prepare values for BitFramingReg
	uint8_t txLastBits = validBits ? *validBits : 0;

	rc66x_set_reg8(rc66x, RC66X_REG_Command, RC66X_CMD_Idle);// Stop any active command.

	rc66x_set_reg8(rc66x, RC66X_REG_IRQ0, 0x7F);// Clear all seven interrupt request bits
	rc66x_set_reg8(rc66x, RC66X_REG_IRQ1, 0x7F);// Clear all seven interrupt request bits
	rc66x_set_reg8(rc66x, RC66X_REG_FIFOControl, 0xB0);	// FlushBuffer = 1, FIFO initialization
	rc66x_send(rc66x, RC66X_REG_FIFOData, sendData, sendLen);// Write sendData to the FIFO
	rc66x_set_reg8(rc66x, RC66X_REG_TxDataNum, 0x08 | txLastBits);
	rc66x_set_reg8(rc66x, RC66X_REG_RxBitCtrl, 0x80 | ((0x7 & rxAlign) << 4));

	rc66x_set_reg8(rc66x, RC66X_REG_TxCrcPreset, sendCRC ? 0x19 : 0x00);
	rc66x_set_reg8(rc66x, RC66X_REG_RxCrcPreset, recvCRC ? 0x19 : 0x00);

	rc66x_set_reg8(rc66x, RC66X_REG_Command, RC66X_CMD_Transceive);	// Execute the command

	uint32_t begin = rc66x->get_time_ms();

	uint8_t irq0, irq1;
	while ((rc66x->get_time_ms() - begin) < RC66X_TIMEOUT_ms) {
		rc66x_get_reg8(rc66x, RC66X_REG_IRQ0, &irq0);
		rc66x_get_reg8(rc66x, RC66X_REG_IRQ1, &irq1);
		if (irq0 & waitIRq) {// One of the interrupts that signal success has been set.
			break;
		}
		if (irq1 & 0x01) {		// Timer interrupt - nothing received in 25ms
			return STATUS_TIMEOUT;
		}
	}

	// 35.7ms and nothing happend. Communication with the MFRC522 might be down.
	if ((rc66x->get_time_ms() - begin) >= RC66X_TIMEOUT_ms) {
		return STATUS_TIMEOUT;
	}

	// Should we delay here to prevent short frame errors??

	// Stop now if any errors except collisions were detected.
	uint8_t errorRegValue;
	rc66x_get_reg8(rc66x, RC66X_REG_Error, &errorRegValue);
	//if (errorRegValue & 0b01100011) {
	if (errorRegValue & 0b01110010) {
		return STATUS_ERROR;
	}

	// if a pointer is supplied for the collpos
	if (collpos) {
		// get the collision position
		rc66x_get_reg8(rc66x, RC66X_REG_RxColl, collpos);
		// check for the valid bit
		if (! (*collpos&0x80)) {
			// if not valid, set to 0
			*collpos = 0;
		} else {
			// if valid, strip valid bit
			*collpos &= 0x7F;
		}
	}


	if (validBits) {
		rc66x_get_reg8(rc66x, RC66X_REG_RxBitCtrl, validBits);
		*validBits &= 0x07;
	}


	// If the caller wants data back, get it from the MFRC522.
	if (recv_data && recv_size) {
		uint8_t fifo_data_len;
		rc66x_get_reg8(rc66x, RC66X_REG_FIFOLength, &fifo_data_len);// Number of bytes in the FIFO
		*recv_size = fifo_data_len;
		if (fifo_data_len > *recv_size) {
			return STATUS_NO_ROOM;
		}
		rc66x_recv(rc66x, RC66X_REG_FIFOData, recv_data, *recv_size);
	}

	// Tell about collisions
	if (errorRegValue & 0x04) {		// CollErr
		return STATUS_COLLISION;
	}

//	// Does this still make sense?
//	if (backData && recv_size && recvCRC) {
//		// In this case a MIFARE Classic NAK is not OK.
//		if (*backLen == 1 && _validBits == 4) {
//			return STATUS_MIFARE_NACK;
//		}
//		// We need at least the CRC_A value and all 8 bits of the last uint8_t must be received.
//		if (*backLen < 2 || _validBits != 0) {
//			return STATUS_CRC_WRONG;
//		}
//	}

	return STATUS_OK;
} // End RC52X_CommunicateWithPICC()

rc66x_result_t rc66x_crypto1_end(bs_pdc_t *pdc) {
	return rc66x_set_reg8(pdc, RC66X_REG_Status, ~(1 << 5));
}

rc66x_result_t rc66x_crypto1_begin(bs_pdc_t *rc66x, picc_t *picc) {
	if (!picc)
		return -1;
	rc66x_result_t result;
	switch (picc->mfc_crypto1.key_a_or_b) {
	case 0x60:
		// Use Key A
		break;
	case 0x61:
		// Use Key B
		break;
	default:
		return -1;
	}

	rc66x_set_reg8(rc66x, RC66X_REG_Command, RC66X_CMD_Idle); // Stop any active command.

	rc66x_set_reg8(rc66x, RC66X_REG_IRQ0, 0x7F); // Clear all seven interrupt request bits
	rc66x_set_reg8(rc66x, RC66X_REG_IRQ1, 0x7F); // Clear all seven interrupt request bits
	rc66x_set_reg8(rc66x, RC66X_REG_FIFOControl, 0xB0);	// FlushBuffer = 1, FIFO initialization

	uint8_t buffer[6];

	rc66x_set_reg8(rc66x, RC66X_REG_FIFOControl, 0xB0);	// FlushBuffer = 1, FIFO initialization
	memcpy(buffer, ((uint8_t*) &picc->mfc_crypto1) + 2, 6);
	rc66x_send(rc66x, RC66X_REG_FIFOData, buffer, 6);
	rc66x_set_reg8(rc66x, RC66X_REG_Command, RC66X_CMD_LoadKey);

	rc66x_set_reg8(rc66x, RC66X_REG_FIFOControl, 0xB0);	// FlushBuffer = 1, FIFO initialization
	memcpy(buffer, &picc->mfc_crypto1, 2);
	memcpy(buffer + 2, picc->uid+ picc->uid_size - 4, 4);
	rc66x_send(rc66x, RC66X_REG_FIFOData, buffer, 6);
	rc66x_set_reg8(rc66x, RC66X_REG_Command, RC66X_CMD_MFAuthent);// Execute the command

	uint8_t status, error;
	uint32_t timeout = rc66x->get_time_ms() + RC66X_TIMEOUT_ms;

	while ((rc66x->get_time_ms()) < timeout) {
		rc66x_get_reg8(rc66x, RC66X_REG_Error, &error);
		rc66x_get_reg8(rc66x, RC66X_REG_Status, &status);
		if (status && (1 << 5)) {
			return STATUS_OK;
		}
	}

	rc66x_set_reg8(rc66x, RC66X_REG_Command, RC66X_CMD_Idle);// Stop any active command.

	return STATUS_ERROR;

}
#include <stdio.h>

void rc66x_test(rc66x_t *rc66x) {
	rc66x_set_reg8(rc66x, RC66X_REG_Command, RC66X_CMD_Idle);
	rc66x_set_reg8(rc66x, RC66X_REG_FIFOControl, 0xB0);
	{ uint8_t data[] = { 0x0A, 0x0A }; rc66x_send(rc66x, 0x05, data, sizeof(data)); }
	rc66x_set_reg8(rc66x, RC66X_REG_Command, RC66X_CMD_LoadProtocol);
	rc66x_set_reg8(rc66x, RC66X_REG_FIFOControl, 0xB0);
	rc66x_set_reg8(rc66x, 0x28, 0x8E);
	{ uint8_t data[] = { 0x01, 0x94, 0x28, 0x12 }; rc66x_send(rc66x, 0x05, data, sizeof(data)); }
	rc66x_set_reg8(rc66x, RC66X_REG_Command, RC66X_CMD_LoadReg);
	rc66x_set_reg8(rc66x, RC66X_REG_Command, RC66X_CMD_Idle);
	rc66x_set_reg8(rc66x, RC66X_REG_FIFOControl, 0xB0);
	rc66x_set_reg8(rc66x, 0x06, 0x7F);
	{ uint8_t data[] = { 0x36, 0x01, 0x00, 0x00 }; rc66x_send(rc66x, 0x05, data, sizeof(data)); }
	rc66x_set_reg8(rc66x, RC66X_REG_Command, RC66X_CMD_Transceive);

	uint8_t waitIRq = 0b00010110;		// RxIRq and IdleIRq + ErrIRQ
	uint8_t irq0, irq1;
	uint32_t begin = rc66x->get_time_ms();
	while ((rc66x->get_time_ms() - begin) < RC66X_TIMEOUT_ms) {
		rc66x_get_reg8(rc66x, RC66X_REG_IRQ0, &irq0);
		rc66x_get_reg8(rc66x, RC66X_REG_IRQ1, &irq1);
		if (irq0 & waitIRq) {// One of the interrupts that signal success has been set.
			puts ("Found");
			break;
		}
		if (irq1 & 0x01) {		// Timer interrupt - nothing received in 25ms
			puts ("Timeout (irq)");
		}
	}

	if ((rc66x->get_time_ms() - begin) >= RC66X_TIMEOUT_ms)
		puts("Timeout (clock)");

	uint8_t uid[8] = {};
	rc66x_recv(rc66x, 0x05, uid, sizeof(uid));
	printf("%02X %02X %02X %02X %02X %02X %02X %02X \n",
				uid[0], uid[1], uid[2], uid[3],
				uid[4], uid[5], uid[6], uid[7]);

}


void mfrc630_ISO15693_init(rc66x_t *rc66x){
	uint8_t protocol = 0x0A;

	// Configure Timers
	rc66x_set_reg8(rc66x, RC66X_REG_T0Control,0x98);  		//configure T0
	rc66x_set_reg8(rc66x, RC66X_REG_T1Control,0x92);			//configure T1 and cascade it with T0
	rc66x_set_reg8(rc66x, RC66X_REG_T2Control,0x20);			//Configure T2 for LFO Autotrimm
	rc66x_set_reg8(rc66x, RC66X_REG_T0ReloadHi,0x03);			//T2 reload value for LFO AutoTrimm
	rc66x_set_reg8(rc66x, RC66X_REG_T0ReloadLo,0xFF);			//T2 reload value high
	rc66x_set_reg8(rc66x, RC66X_REG_T3Control,0x00);			//Configure T3 (for LPCD /Autotrimm)

	//Configure FiFo
	rc66x_set_reg8(rc66x, RC66X_REG_FIFOControl,0x90);		//Set Fifo-Size and Waterlevel
	rc66x_set_reg8(rc66x, RC66X_REG_WaterLevel,0xFE);			//Set Waterlevel

	//Configure RXBITCTRL
	rc66x_set_reg8(rc66x, RC66X_REG_RxBitCtrl,0x80);			//Set RXBITCTLR register

	rc66x_set_reg8(rc66x, RC66X_REG_Command, RC66X_CMD_Idle);	//Cancel any commands
	rc66x_set_reg8(rc66x, RC66X_REG_FIFOControl, 0xB0);			//Flush Fifo
	rc66x_set_reg8(rc66x, RC66X_REG_IRQ0, 0x7F);				//Clear IRQ0
	rc66x_set_reg8(rc66x, RC66X_REG_IRQ1, 0x7F);				//Clear IRQ1

	//Set Timers
	rc66x_set_reg8(rc66x, RC66X_REG_T0ReloadHi,0x18);			//T0 Reload Hi
	rc66x_set_reg8(rc66x, RC66X_REG_T0ReloadLo,0x86);			//T0 Reload Lo
	rc66x_set_reg8(rc66x, RC66X_REG_T1ReloadHi,0x00);			//T1 Reload Hi
	rc66x_set_reg8(rc66x, RC66X_REG_T1ReloadLo,0x00);			//T1 Reload Lo

	// Write in FIFO "Load protocol" params(TxProtocol=Iso15693(0a), RxProtocol=Iso15693(0a),
	rc66x_set_reg8(rc66x, RC66X_REG_FIFOData,protocol);
	rc66x_set_reg8(rc66x, RC66X_REG_FIFOData,protocol);

	rc66x_set_reg8(rc66x, RC66X_REG_IRQ0En, (1<<4));	// Enable IRQ0 interrupt source
	rc66x_set_reg8(rc66x, RC66X_REG_IRQ1En, (1<<6)); 	// Enable IRQ1 interrupt source
	rc66x_set_reg8(rc66x, RC66X_REG_Command,RC66X_CMD_LoadProtocol);	// Execute Rc663 command: "Load protocol"

	rc66x_set_reg8(rc66x, RC66X_REG_IRQ0En, 0x00);		//Disable IRQ
	rc66x_set_reg8(rc66x, RC66X_REG_IRQ1En, 0x00);		//Disable IRQ

	rc66x_set_reg8(rc66x, RC66X_REG_FIFOControl, 0xB0);			//Flush Fifo

	//> Apply RegisterSet
	rc66x_set_reg8(rc66x, RC66X_REG_TxCrcPreset,0x7B);
	rc66x_set_reg8(rc66x, RC66X_REG_RxCrcPreset,0x7B);
	rc66x_set_reg8(rc66x, RC66X_REG_TxDataNum,0x08);
	rc66x_set_reg8(rc66x, RC66X_REG_TxModWidth,0x00);
	rc66x_set_reg8(rc66x, RC66X_REG_TxSym10BurstLen,0x00);
	rc66x_set_reg8(rc66x, RC66X_REG_TXWaitCtrl,0x00);
	rc66x_set_reg8(rc66x, RC66X_REG_FrameCon,0x0F);
	rc66x_set_reg8(rc66x, RC66X_REG_RxCtrl,0x02);
	rc66x_set_reg8(rc66x, RC66X_REG_RxThreshold,0x4E);
	rc66x_set_reg8(rc66x, RC66X_REG_RxAna,0x04);
	rc66x_set_reg8(rc66x, RC66X_REG_RxWait,0x8C);				// Set the RxWait register
	rc66x_set_reg8(rc66x, RC66X_REG_TXWaitCtrl,0xC0);
	rc66x_set_reg8(rc66x, RC66X_REG_TxWaitLo,0x00);

	// Write Timer-0, Timer-1 reload values(high,low)
	rc66x_set_reg8(rc66x, RC66X_REG_T0ReloadHi,0x18);
	rc66x_set_reg8(rc66x, RC66X_REG_T0ReloadLo,0x86);
	rc66x_set_reg8(rc66x, RC66X_REG_T1ReloadHi,0x00);
	rc66x_set_reg8(rc66x, RC66X_REG_T1ReloadLo,0x00);
	rc66x_set_reg8(rc66x, RC66X_REG_TxAmp,0x0A);
	rc66x_set_reg8(rc66x, RC66X_REG_DrvMode,0x81);
	rc66x_set_reg8(rc66x, RC66X_REG_Status,0x00); // Disable MIFARE Crypto1
	//Set Driver
}


int test_read_block(rc66x_t *rc66x, uint8_t* send_data, int send_len
		, uint8_t* recv_data, int *recv_len) {


		//Set timeout for Timer0/Timer1, set reload values
		rc66x_set_reg8(rc66x, RC66X_REG_T0ReloadHi,0x24);
		rc66x_set_reg8(rc66x, RC66X_REG_T0ReloadLo,0xEB);
		rc66x_set_reg8(rc66x, RC66X_REG_T1ReloadHi,0x00);
		rc66x_set_reg8(rc66x, RC66X_REG_T1ReloadLo,0x00);

		rc66x_set_reg8(rc66x, RC66X_REG_Command, RC66X_CMD_Idle);	//Cancel any commands
		rc66x_set_reg8(rc66x, RC66X_REG_FIFOControl, 0xB0);			//Flush Fifo
		rc66x_set_reg8(rc66x, RC66X_REG_IRQ0, 0x7F);				//Clear IRQ0
		rc66x_set_reg8(rc66x, RC66X_REG_IRQ1, 0x7F);				//Clear IRQ1

//		printf("Sending instruction ");
//		for (int i = 0 ; i < send_len; i++) {
//			printf("%02X ", send_data[i]);
//		}
//		puts("");

		//Send instruction to reader
		rc66x_set_reg8(rc66x, RC66X_REG_DrvMode,0x89); 	//Field on

		rc66x_set_reg8(rc66x, RC66X_REG_Command, RC66X_CMD_Idle);	//Cancel any commands
		rc66x_set_reg8(rc66x, RC66X_REG_FIFOControl, 0xB0);			//Flush Fifo
		rc66x_send(rc66x, RC66X_REG_FIFOData, send_data, send_len);

		// clear interrupts
		rc66x_set_reg8(rc66x, RC66X_REG_IRQ0, 0x7F);				//Clear IRQ0
		rc66x_set_reg8(rc66x, RC66X_REG_IRQ1, 0x7F);				//Clear IRQ1

		// Enable IRQ0,IRQ1 interrupt sources
		 rc66x_set_reg8(rc66x, RC66X_REG_IRQ0En, (1<<4)  | (1<<3));
		 rc66x_set_reg8(rc66x, RC66X_REG_IRQ1En,  (1<<6)  | (1<<1) );

		 rc66x_set_reg8(rc66x, RC66X_REG_Command, RC66X_CMD_Transceive);

		  // block until transmission ending
		  uint8_t irq0_value = 0;
		  uint8_t irq1_value = 0;
		  uint32_t timeout= rc66x->get_time_ms() ;
		  while (!((irq0_value & 0x08)== 0x08)) {
			  rc66x_get_reg8(rc66x, RC66X_REG_IRQ0, &irq0_value);
		    if(rc66x->get_time_ms()>(timeout+50)){
		    	puts("Time timeout 1");
		    	break;
		    }
		  }

		  //Wait for timer1 underflow (irq1(0x02) or RxIrQ irq0(0x04;
		  irq0_value =0;
		  timeout= rc66x->get_time_ms();
		  while ( ((irq1_value & 0x02) !=0x02)  && ((irq0_value & 0x04) !=0x04)){
			rc66x_get_reg8(rc66x, RC66X_REG_IRQ0, &irq0_value);
			rc66x_get_reg8(rc66x, RC66X_REG_IRQ1, &irq1_value);
		    if(rc66x->get_time_ms()>(timeout+50)){
		    	puts("Time timeout 2");
			    	break;
			 }
		  }

		  //Check for error
		if((irq1_value & 0x02)){
			puts("IRQ1 marks error");
			return 0x00;								//return error!
		};

		//disable IRQ0,IRQ1
		rc66x_set_reg8(rc66x, RC66X_REG_IRQ0En,0x00);
		rc66x_set_reg8(rc66x, RC66X_REG_IRQ1En,0x00);

		//see if a uid was found:
		//uint16_t fifo_len = mfrc630_fifo_length();
		uint8_t fifo_len;
		rc66x_get_reg8(rc66x, RC66X_REG_FIFOLength, &fifo_len);
//		printf("Fifo Len %d\n", fifo_len);
		if (recv_data && recv_len) {
			if (fifo_len >recv_len) return -fifo_len;
			*recv_len=fifo_len;
			rc66x_recv(rc66x, RC66X_REG_FIFOData, recv_data,fifo_len);
		}

		uint8_t error;
		rc66x_get_reg8(rc66x,0x0A,&error);
//		printf("Reg 0x0A val %02X\n", error);
		if(error) return 0;

		return fifo_len;								//return state - valid
	}

uint16_t mfrc630_ISO15693_readTag(rc66x_t *rc66x, uint8_t* uid, int colpos){

	//Set timeout for Timer0/Timer1, set reload values
	rc66x_set_reg8(rc66x, RC66X_REG_T0ReloadHi,0x24);
	rc66x_set_reg8(rc66x, RC66X_REG_T0ReloadLo,0xEB);
	rc66x_set_reg8(rc66x, RC66X_REG_T1ReloadHi,0x00);
	rc66x_set_reg8(rc66x, RC66X_REG_T1ReloadLo,0x00);

	rc66x_set_reg8(rc66x, RC66X_REG_Command, RC66X_CMD_Idle);	//Cancel any commands
	rc66x_set_reg8(rc66x, RC66X_REG_FIFOControl, 0xB0);			//Flush Fifo
	rc66x_set_reg8(rc66x, RC66X_REG_IRQ0, 0x7F);				//Clear IRQ0
	rc66x_set_reg8(rc66x, RC66X_REG_IRQ1, 0x7F);				//Clear IRQ1

	//Prepare instruction to send to fifo
	uint8_t instruction[12] ={
			0x36,					//set the flags,
			0x01,				//set "Inventory Command"
			0x00,					//set blank
			0x00					//set blank
	};



	int valid_bytes = colpos / 8;
	int valid_bits = colpos % 8;

	printf("Colpos %2d, byte %d bit %d\n",colpos, valid_bytes, valid_bits);

	memcpy(instruction + 4, uid + 2, 8);

//	printf("Pre-instruction ");
//	for (int i = 0 ; i < 12; i++) {
//		printf("%02X ", instruction[i]);
//	}

	switch(valid_bits) {
	case 1:
		instruction[valid_bytes+ 2] &= 0b1;
		break;
	case 2:
		instruction[valid_bytes+ 2] &= 0b11;
		break;
	case 3:
		instruction[valid_bytes+ 2] &= 0b111;
		break;
	case 4:
		instruction[valid_bytes+ 2] &= 0b1111;
		break;
	case 5:
		instruction[valid_bytes+ 2] &= 0b11111;
		break;
	case 6:
		instruction[valid_bytes+ 2] &= 0b111111;
		break;
	case 7:
		instruction[valid_bytes+ 2] &= 0b1111111;
		break;
	}


	int instr_len = 4;
	if (valid_bytes) instr_len += valid_bytes - 2;
	if (valid_bits) {
		instr_len ++;
		instruction[3] = valid_bits + 1;
	}


	printf("Sending instruction ");
	for (int i = 0 ; i < instr_len; i++) {
		printf("%02X ", instruction[i]);
	}
	puts("");



	//Send instruction to reader
	rc66x_set_reg8(rc66x, RC66X_REG_DrvMode,0x89); 	//Field on

	rc66x_set_reg8(rc66x, RC66X_REG_Command, RC66X_CMD_Idle);	//Cancel any commands
	rc66x_set_reg8(rc66x, RC66X_REG_FIFOControl, 0xB0);			//Flush Fifo
	rc66x_send(rc66x, RC66X_REG_FIFOData, instruction, instr_len);

	// clear interrupts
	rc66x_set_reg8(rc66x, RC66X_REG_IRQ0, 0x7F);				//Clear IRQ0
	rc66x_set_reg8(rc66x, RC66X_REG_IRQ1, 0x7F);				//Clear IRQ1

	// Enable IRQ0,IRQ1 interrupt sources
	 rc66x_set_reg8(rc66x, RC66X_REG_IRQ0En, (1<<4)  | (1<<3));
	 rc66x_set_reg8(rc66x, RC66X_REG_IRQ1En,  (1<<6)  | (1<<1) );

	 rc66x_set_reg8(rc66x, RC66X_REG_Command, RC66X_CMD_Transceive);

	  // block until transmission ending
	  uint8_t irq0_value = 0;
	  uint8_t irq1_value = 0;
	  uint32_t timeout= rc66x->get_time_ms() ;
	  while (!((irq0_value & 0x08)== 0x08)) {
		  rc66x_get_reg8(rc66x, RC66X_REG_IRQ0, &irq0_value);
	    if(rc66x->get_time_ms()>(timeout+50)){
	    	puts("Time timeout 1");
	    	break;
	    }
	  }

	  //Wait for timer1 underflow (irq1(0x02) or RxIrQ irq0(0x04;
	  irq0_value =0;
	  timeout= rc66x->get_time_ms();
	  while ( ((irq1_value & 0x02) !=0x02)  && ((irq0_value & 0x04) !=0x04)){
		rc66x_get_reg8(rc66x, RC66X_REG_IRQ0, &irq0_value);
		rc66x_get_reg8(rc66x, RC66X_REG_IRQ1, &irq1_value);
	    if(rc66x->get_time_ms()>(timeout+50)){
	    	puts("Time timeout 2");
		    	break;
		 }
	  }

	  //Check for error
	if((irq1_value & 0x02)){
		puts("IRQ1 marks error");
		return 0x00;								//return error!
	};

	//disable IRQ0,IRQ1
	rc66x_set_reg8(rc66x, RC66X_REG_IRQ0En,0x00);
	rc66x_set_reg8(rc66x, RC66X_REG_IRQ1En,0x00);

	//see if a uid was found:
	//uint16_t fifo_len = mfrc630_fifo_length();
	uint8_t fifo_len;
	rc66x_get_reg8(rc66x, RC66X_REG_FIFOLength, &fifo_len);
	printf("Fifo Len %d\n", fifo_len);
	if(fifo_len != 0x0A){
		return 0x00;								//return error - invalid uid size!
	}

	//transfer UID to variable

	rc66x_recv(rc66x, RC66X_REG_FIFOData, uid,fifo_len);
//	for (int i = 0 ; i < fifo_len; i++) {
//		printf("%02X ",uid[i]);
//	}
//	puts("");

	uint8_t test;
	rc66x_get_reg8(rc66x,0x0A,&test);
	printf("Reg 0x0A val %02X\n", test);

	if (test & 0x04) {
		puts("Collision detected");
		rc66x_get_reg8(rc66x,0x0D,&test);
		printf("Valid : %d at pos %d\n", 0x80==(test&0x80), test&0x7F);
		uint8_t col1[10];
		uint8_t col2[10];
		memcpy(col1,uid,10);
		memcpy(col2,uid,10);
		int col_byte = (test & 0x7F) / 8;
		int col_bit = (test & 0x7F) % 8;
		col1[col_byte] |= (1 << (col_bit));
		col2[col_byte] &= ~(1 << (col_bit));

		printf("col1: ");
		for (int i = 0 ; i < col_byte + (col_bit > 0); i++) {
			printf("%02X ",col1[i]);
		}
		printf("\ncol2: ");
		for (int i = 0 ; i < col_byte + (col_bit > 0); i++) {
			printf("%02X ",col2[i]);
		}

		puts("Anticol pass 1");
		mfrc630_ISO15693_readTag(rc66x,col1, 1+ (test&0x7F));
		puts("Anticol pass 2");
		mfrc630_ISO15693_readTag(rc66x,col2, 1 + (test&0x7F));

	} else {
			printf("\t\t UID:");
			for (int i = 2 ; i < fifo_len; i++) {
				printf("%02X ",uid[i]);
			}
			puts("");

			{
				uint8_t read_instr[10];
				uint8_t response_buffer[16];
				int response_size = sizeof(response_buffer);
				read_instr[0] = 0x22; // optioon: addressed, high data rate
				read_instr[1] = 0x2B; // get info
				memcpy(read_instr + 2, uid + 2, 8);
				test_read_block(rc66x, read_instr, sizeof(read_instr) , response_buffer, &response_size);

				for (int i = 0 ; i < response_size; i++) {
					printf("%02X ",response_buffer[i]);
				}
				puts("");

				if (response_buffer[0]) {
					// error
					puts("Error");
				} else {
					int pos = 10;
					if (response_buffer[1] & 0x01) {
						printf("DSFID   %02X\n", response_buffer[pos++]);
					}
					if (response_buffer[1] & 0x02) {
						printf("AFI     %02X\n", response_buffer[pos++]);
					}
					if (response_buffer[1] & 0x04) {
						uint8_t number_of_blocks = response_buffer[pos++];
						uint8_t size_of_block = response_buffer[pos++];
						number_of_blocks++;
						size_of_block++;
						printf("Size: %d * %d\n", number_of_blocks, size_of_block);
					}
					if (response_buffer[1] & 0x08) {
						printf("IC Ref. %02X\n", response_buffer[pos++]);
					}
				}
			}


			{
			uint8_t read_instr[11];
			uint8_t response_buffer[16];
			int response_size = sizeof(response_buffer);
			read_instr[0] = 0x22; // optioon: addressed, high data rate
			read_instr[1] = 0x20; // read single block
			memcpy(read_instr + 2, uid + 2, 8);
			read_instr[10] = 0x00; // block number;
			while (!response_buffer[0]) {
				response_size = sizeof(response_buffer);
				test_read_block(rc66x, read_instr, sizeof(read_instr) , response_buffer, &response_size);
				printf("Block %2d:\t",read_instr[10]);
				if (response_buffer[0]) {
					printf("Error %02X\n",response_buffer[1]);
					break;
				}

				for (int i = 1 ; i < response_size; i++) printf("%02X ", response_buffer[i]);
				puts("");
				read_instr[10]++;
			}
			}


	}

	return fifo_len;								//return state - valid
}
