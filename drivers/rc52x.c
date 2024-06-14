/*****************************************************************************
 * Based upon https://github.com/miguelbalboa/rfid
 *
 * C port by André van Schoubroeck <andre@blaatschaap.be>
 *
 *
 *****************************************************************************/

// For testing purposes, thsi will not be the final api
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "rc52x_transport.h"
#include "rc52x.h"
#include "rc52x_ref.h"

#include "iso14443a.h"

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

//#include "MFRC522.h"
#include "rc52x.h"
#include "picc.h"

int rc52x_get_chip_version(rc52x_t *rc52x, uint8_t *chip_id) {
	return mfrc522_recv(rc52x, RC52X_REG_VersionReg, chip_id, 1);
}

const char* rc52x_get_chip_name(rc52x_t *rc52x) {
	uint8_t chip_id = 0xFF;
	int result = rc52x_get_chip_version(rc52x, &chip_id);
	if (result)
		return "Error";
	switch (chip_id) {

	case 0x80:
		return "PN512 V1";
	case 0x82:
		return "PN512 V2";

	case 0x88:
		return "FM17522";
	case 0x89:
		return "FM17550";

	case 0x90:
		return "MFRC522 V0";
	case 0x91:
		return "MFRC522 V1";
	case 0x92:
		return "MFRC522 V2";

	case 0xB1:
		return "MRFC523 V1";
	case 0xB2:
		return "MRFC523 V2";

	default:
		return "Unknown";
	}
}

rc52x_ref_status_t rc52x_self_test(rc52x_t *rc52x) {
	uint8_t chip_id = 0xFF;
	int result = rc52x_get_chip_version(rc52x, &chip_id);
	if (result)
		return rc52x_ref_status_error;

	const uint8_t* ref = NULL;
	switch (chip_id) {

	case 0x80:
		// "PN512 V1";
		ref = rc52x_ref_pn512_v1;
		break;

	case 0x82:
		// "PN512 V2";
		ref = rc52x_ref_pn512_v2;
		break;

	case 0x88:
		// Note: this is not in the datasheet
		// "FM17522";
		ref = rc52x_ref_fm17522;
		break;
		// TODO Chip ID for FM17550

	case 0x90:
		// "MFRC522 V0";
		ref = rc52x_ref_mfrc522_V0;
		break;
	case 0x91:
		// "MFRC522 V1";
		ref = rc52x_ref_mfrc522_V1;
				break;
	case 0x92:
		// "MFRC522 V2";
		ref = rc52x_ref_mfrc522_V2;
		break;

	case 0xB1:
		// "MRFC523 V1";
		ref = rc52x_ref_mfrc523_V1;
				break;
	case 0xB2:
		// "MRFC523 V2";
		ref = rc52x_ref_mfrc523_V2;
		break;

	default:
		break;
	}
	if (!ref)
		return rc52x_ref_status_unknown;

	// TODO Perform the self test
	// https://github.com/miguelbalboa/rfid/blob/b2ff919438c2c2924092de8348b19049359dddd0/src/MFRC522.cpp#L326

	return rc52x_ref_status_unknown;

}

/**
 * Initializes the MFRC522 chip.
 */
void rc52x_init(rc52x_t *rc52x) {
	if (!rc52x)
		return;
	if (!rc52x->get_time_ms)
		return;
	if (!rc52x->delay_ms)
		return;

	rc52x->TransceiveData = (void*)rc52x_transceive;
	//rc52x->SetBitFraming = rc52x_set_bit_framing;
	rc52x_reset(rc52x);

	// Reset baud rates
	rc52x_set_reg8(rc52x, RC52X_REG_TxModeReg, 0x00);
	rc52x_set_reg8(rc52x, RC52X_REG_RxModeReg, 0x00);
	// Reset ModWidthReg
	rc52x_set_reg8(rc52x, RC52X_REG_ModWidthReg, 0x26);

	// When communicating with a PICC we need a timeout if something goes wrong.
	// f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
	// TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
	rc52x_set_reg8(rc52x, RC52X_REG_TModeReg, 0x80);// TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
	rc52x_set_reg8(rc52x, RC52X_REG_TPrescalerReg, 0xA9);// TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25μs.
	rc52x_set_reg8(rc52x, RC52X_REG_TReloadReg_Hi, 0x03);// Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
	rc52x_set_reg8(rc52x, RC52X_REG_TReloadReg_Lo, 0xE8);

	rc52x_set_reg8(rc52x, RC52X_REG_TxASKReg, 0x40);// Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
	rc52x_set_reg8(rc52x, RC52X_REG_ModeReg, 0x3D);	// Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)

	// Fix for PN512 compatibility:
	// Set Bit 4 in ControlReg: Set to logic 1, the PN512 acts as initiator, otherwise it acts as target
	rc52x_set_reg8(rc52x, RC52X_REG_ControlReg, 0x10);

	// testing
	// RC52X_SetAntennaGain(rc52x,RxGain_48dB);

	rc52x_antenna_on(rc52x);// Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
} // End RC52X_Init()

/**
 * Performs a soft reset on the MFRC522 chip and waits for it to be ready again.
 */
void rc52x_reset(rc52x_t *rc52x) {
	rc52x_set_reg8(rc52x, RC52X_REG_CommandReg, RC52X_CMD_SoftReset); // Issue the SoftReset command.
	// The datasheet does not mention how long the SoftRest command takes to complete.
	// But the MFRC522 might have been in soft power-down mode (triggered by bit 4 of CommandReg)
	// Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74μs. Let us be generous: 50ms.
	uint8_t count = 0;
	uint8_t commandregval;
	do {
		// Wait for the PowerDown bit in CommandReg to be cleared (max 3x50ms)
		rc52x->delay_ms(50);
		int result = rc52x_get_reg8(rc52x, RC52X_REG_CommandReg,
				&commandregval);
		if (result)
			return;
	} while ((commandregval & (1 << 4)) && (++count) < 3);
} // End RC52X_Reset()

/**
 * Turns the antenna on by enabling pins TX1 and TX2.
 * After a reset these pins are disabled.
 */
void rc52x_antenna_on(rc52x_t *rc52x) {
	rc52x_set_reg8(rc52x, RC52X_REG_TxControlReg, 0x83);
} // End RC52X_AntennaOn()

/**
 * Turns the antenna off by disabling pins TX1 and TX2.
 */
void rc52x_antenna_off(rc52x_t *rc52x) {
	rc52x_set_reg8(rc52x, RC52X_REG_TxControlReg, 0x80);
} // End RC52X_AntennaOff()

rc52x_result_t rc52x_transceive(rc52x_t *rc52x, uint8_t *sendData, ///< Pointer to the data to transfer to the FIFO.
		size_t sendLen,		///< Number of uint8_ts to transfer to the FIFO.
		uint8_t *backData,///< nullptr or pointer to buffer if data should be read back after executing the command.
		size_t *backLen,///< In: Max number of uint8_ts to write to *backData. Out: The number of uint8_ts returned.
		uint8_t *validBits,	///< In/Out: The number of valid bits in the last uint8_t. 0 for 8 valid bits. Default nullptr.
		uint8_t rxAlign,///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
		uint8_t *collisionPos, bool sendCRC, bool recvCRC) {
	uint8_t waitIRq = 0x30;		// RxIRq and IdleIRq
	// Prepare values for BitFramingReg
	uint8_t txLastBits = validBits ? *validBits : 0;
	uint8_t bitFraming = (rxAlign << 4) + txLastBits;// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

	int result;
	if (sendCRC) {
		rc52x_or_reg8(rc52x, RC52X_REG_TxModeReg, 0x80);
	} else {
		rc52x_and_reg8(rc52x, RC52X_REG_TxModeReg, ~0x80);
	}

	if (recvCRC) {
		rc52x_or_reg8(rc52x, RC52X_REG_RxModeReg, 0x80);
	} else {
		rc52x_and_reg8(rc52x, RC52X_REG_RxModeReg, ~0x80);
	}

	rc52x_and_reg8(rc52x, RC52X_REG_CollReg, ~0x80);

	rc52x_set_reg8(rc52x, RC52X_REG_CommandReg, RC52X_CMD_Idle);// Stop any active command.
	rc52x_set_reg8(rc52x, RC52X_REG_ComIrqReg, 0x7F);// Clear all seven interrupt request bits
	rc52x_set_reg8(rc52x, RC52X_REG_FIFOLevelReg, 0x80);// FlushBuffer = 1, FIFO initialization
	mfrc522_send(rc52x, RC52X_REG_FIFODataReg, sendData, sendLen);// Write sendData to the FIFO
	rc52x_set_reg8(rc52x, RC52X_REG_BitFramingReg, bitFraming);	// Bit adjustments
	rc52x_set_reg8(rc52x, RC52X_REG_CommandReg, RC52X_CMD_Transceive);// Execute the command

	rc52x_or_reg8(rc52x, RC52X_REG_BitFramingReg, 0x80);// StartSend=1, transmission of data starts

	// Wait for the command to complete.
	// In RC52X_Init() we set the TAuto flag in TModeReg. This means the timer automatically starts when the PCD stops transmitting.

	uint8_t regval;

	uint32_t timeout = rc52x->get_time_ms() + RC52X_TIMEOUT_ms;

	while ((rc52x->get_time_ms()) < timeout) {
		result = rc52x_get_reg8(rc52x, RC52X_REG_ComIrqReg, &regval);
		if (result)
			return STATUS_ERROR;
		if (regval & waitIRq) {	// One of the interrupts that signal success has been set.
			break;
		}
		if (regval & 0x01) {	// Timer interrupt - nothing received in 25ms
			return STATUS_TIMEOUT;
		}
	}

	if (rc52x->get_time_ms() >= timeout) {
		return STATUS_TIMEOUT;
	}

	// Stop now if any errors except collisions were detected.
	uint8_t errorRegValue;
	result = rc52x_get_reg8(rc52x, RC52X_REG_ErrorReg, &errorRegValue);
	if (result)
		return STATUS_ERROR;
//	if (errorRegValue & 0x13) {	 // BufferOvfl ParityErr ProtocolErr
//		return STATUS_ERROR;
//	}

	uint8_t _validBits = 0;

	// If the caller wants data back, get it from the MFRC522.
	if (backData && backLen) {
		uint8_t n;
		result = rc52x_get_reg8(rc52x, RC52X_REG_FIFOLevelReg, &n);
		if (result)
			return STATUS_ERROR;

		if (n > *backLen) {
			return STATUS_NO_ROOM;
		}
		*backLen = n;							// Number of uint8_ts returned
		result = mfrc522_recv(rc52x, RC52X_REG_FIFODataReg, backData, n);
		if (result)
			return STATUS_ERROR;

		result = rc52x_get_reg8(rc52x, RC52X_REG_ControlReg, &_validBits);
		_validBits &= 0x07;
		if (result)
			return STATUS_ERROR;

		if (validBits) {
			*validBits = _validBits;
		}
	}

	// Tell about collisions
	if (errorRegValue & 0x08) {		// CollErr
		if (!collisionPos)
			return STATUS_COLLISION;

		uint8_t valueOfCollReg;
		result = rc52x_get_reg8(rc52x, RC52X_REG_CollReg, &valueOfCollReg);
		if (result)
			return STATUS_ERROR;

		if (valueOfCollReg & 0x20) { // CollPosNotValid
			*collisionPos = -1;
			return STATUS_COLLISION; // Without a valid collision position we cannot continue
		}

		*collisionPos = valueOfCollReg & 0x1F; // Values 0-31, 0 means bit 32.

		if (*collisionPos == 0) {
			*collisionPos = 32;
		}

		return STATUS_COLLISION;
	}

//	// Perform CRC_A validation if requested.
//	if (backData && backLen && recvCRC) {
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

rc52x_result_t rc52x_set_bit_framing(bs_pdc_t *pdc, int rxAlign, int txLastBits) {
	return rc52x_set_reg8(pdc, RC52X_REG_BitFramingReg,
			(rxAlign << 4) | txLastBits); // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
}

rc52x_result_t rc52x_crypto1_end(bs_pdc_t *pdc) {
	return rc52x_and_reg8(pdc, RC52X_REG_Status2Reg, ~0x08);
}

rc52x_result_t rc52x_crypto1_begin(bs_pdc_t *rc52x, picc_t *picc) {
	if (!picc)
		return -1;
	rc52x_result_t result;
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

	rc52x_set_reg8(rc52x, RC52X_REG_ComIrqReg, 0x7F); // Clear all seven interrupt request bits
	rc52x_set_reg8(rc52x, RC52X_REG_CommandReg, RC52X_CMD_Idle); // Stop any active command.
	rc52x_set_reg8(rc52x, RC52X_REG_FIFOLevelReg, 0x80); // FlushBuffer = 1,

	uint8_t buffer[12];
	memcpy(buffer, &picc->mfc_crypto1, 8);
	memcpy(buffer + 8, picc->uid + picc->uid_size - 4, 4);

	mfrc522_send(rc52x, RC52X_REG_FIFODataReg, buffer, 12);

	rc52x_set_reg8(rc52x, RC52X_REG_CommandReg, RC52X_CMD_MFAuthent);

	uint32_t timeout = rc52x->get_time_ms() + RC52X_TIMEOUT_ms;
	uint8_t regval;

	while ((rc52x->get_time_ms()) < timeout) {
		result = rc52x_get_reg8(rc52x, RC52X_REG_ComIrqReg, &regval);
		if (result)
			return STATUS_ERROR;
		if (regval & 0x10) {

			result = rc52x_get_reg8(rc52x, RC52X_REG_Status2Reg, &regval);

			if (regval & 0x08) {
				return STATUS_OK;
			}
			break;

		}
		if (regval & 0x01) {	// Timer interrupt - nothing received in 25ms
			rc52x_set_reg8(rc52x, RC52X_REG_CommandReg, RC52X_CMD_Idle); // Stop any active command.
			return STATUS_TIMEOUT;
		}
	}

	rc52x_set_reg8(rc52x, RC52X_REG_CommandReg, RC52X_CMD_Idle); // Stop any active command.

	return STATUS_ERROR;

}
