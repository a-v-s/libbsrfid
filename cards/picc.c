/*
 * picc.c
 *
 *  Created on: 29 dec. 2020
 *      Author: andre
 */

#include "picc.h"

#include <string.h>

#include "pdc.h"

pdc_result_t picc_reqa(bs_pdc_t * pdc, picc_t * picc) {
	//return PICC_RequestA(pdc, picc);

	uint8_t validBits = 7; // Short Frame
	uint8_t command = PICC_CMD_REQA;
	size_t atqa_size = sizeof(picc->atqa);
	pdc_result_t status = pdc->TransceiveData(pdc, &command, 1, &picc->atqa, &atqa_size,
			&validBits, 0, NULL, false, false);
	//status = RC52X_TransceiveData(rc52x, &command, 1, bufferATQA, bufferSize, &validBits, 0, false);
	if (status != STATUS_OK) {
		return status;
	}
	if (atqa_size != 2 || validBits != 0) {	// ATQA must be exactly 16 bits.
		return STATUS_ERROR;
	}
	return status;
}

// TODO: Implement a new ISO 14443 A AntiCollision
// 	Replace what PICC_Select does, with proper AntiCollision
//	Thus detects all the PICCs in the field, up to the number
//	of picc_t elements in a buffer provided.
pdc_result_t picc_anticol_iso14443a(bs_pdc_t * pdc, picc_t * picc_array, int *picc_count){
	uint8_t send_buffer[7];
	// Zero the picc_array for initial state;
	memset(picc_array, 0, *picc_count * sizeof(picc_t));
	int index = 0;
	pdc_result_t result;
	// Note that if one or more cards are present, they'll
	// answer the REQA command. Therefore this may already collide.
	// Thus, can we even trust any data in the picc->atqa?
	// As this contains the UID size, but cards of different UID
	// sizes might be present. Furthermore, the bit frame says
	// one bit out of 5 must be set. While known cards that
	// do not implement anti collision, such as Topaz,
	// set none, according to the specs, setting any
	// number other then 1 should mean no bitframe anticol.
	// I don't think we should rely on any
	// of this and just proceed with anti collision,
	// and handle something like Topaz separately,
	// if we even decide to handle those as I haven't
	// encountered any in the wild, those are rare.
	// The company appears to be gone, and the NFC Forum
	// appears to have abandoned NFC Type 1 anyways.

	result = picc_reqa(pdc,picc_array);
	if (result!=STATUS_OK && result != STATUS_COLLISION)
		return result;

	// At this point at least one card is present

	// If we cannot assume the length of the UID from the ATQA
	// when multiple cards are present. We will only look at
	// SAK responses. Furthermore, I would place the collisionpos/validbits
	// in the picc_t:
	// When we have a collision, we copy the picc_t into an empty
	// position in the array, flipping the colliding bit,
	// then we repeat with the current known bits in the current
	// picc_t, and attempt to complete the anticollision,
	// then we repeat it for the next.

	// Note, up to the complete UID (in the current cascadelevel)
	// CRC is disabled.
	// After the UD  a "BCC" is added, XOR'ing the CRC bytes

	char send_buff[7];
	size_t send_size = 2;
	char recv_buff[7];
	size_t recv_size = 7;
	bool send_crc = false;
	uint8_t coll_pos = -1;
	//uint8_t valid_bits = 0;

	send_buff[0]= PICC_CMD_SEL_CL1;
	send_buff[1]= 0x20;

	memset(recv_buff,0,sizeof(recv_buff));

	result = pdc->TransceiveData(pdc, send_buff, send_size,
			recv_buff, &recv_size, NULL, 0,
			&coll_pos, send_crc, false);


	if (result == STATUS_COLLISION) {
		uint8_t valid_bytes = coll_pos / 8;
		uint8_t valid_bits = coll_pos % 8;



		send_buff[0]= PICC_CMD_SEL_CL1;

		send_buff[1]= ((2 + valid_bytes) << 4) ;
		send_buff[1]|= valid_bits;
		memcpy ( send_buff + 2, recv_buff, valid_bytes + 1);
		send_size = 3 + valid_bytes;
		memset(recv_buff,0,sizeof(recv_buff));



		result = pdc->TransceiveData(pdc, send_buff, send_size,
					recv_buff, &recv_size, &valid_bits, 0,
					&coll_pos, send_crc, false);

		// with 2 PICCS in the field, we should now have success
		// (and a SAK telling us to enter the next cascade level.

		if (result) {
			return result;
		}
		return result;


//		// Now we split
//		// So we get one entry with the collision bit set to 1, and the next set to 0
//		// TODO, handle the indices so we can handle multiple collisions next
//		memcpy ( picc_array[0]->uid, recv_buff, valid_bytes + 1);
//		memcpy ( picc_array[1]->uid, recv_buff, valid_bytes + 1);
//		picc_array[1]->uid[valid_bytes+1] ^= 1 << valid_bits;
//		// We should move the valid bytes/bits into the picc
//		// and make this function recursive

	}



	return -1;
}


/**
 * Transmits a REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
rc52x_result_t PICC_RequestA(bs_pdc_t *pdc, picc_t *picc) {
	picc->protocol = picc_protocol_iso14443a;
	size_t size = sizeof(iso14443a_atqa_t);
	return PICC_REQA_or_WUPA(pdc, PICC_CMD_REQA, (void*)&picc->atqa, &size);
} // End PICC_RequestA()

/**
 * Transmits a Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
rc52x_result_t PICC_WakeupA(bs_pdc_t *pdc, picc_t *picc) {
	if (picc->protocol != picc_protocol_iso14443a)
		return STATUS_INVALID;
	size_t size = sizeof(iso14443a_atqa_t);
	return PICC_REQA_or_WUPA(pdc, PICC_CMD_WUPA, (void*)&picc->atqa, &size);
} //  // End PICC_WakeupA()

/**
 * Transmits REQA or WUPA commands.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
rc52x_result_t PICC_REQA_or_WUPA(bs_pdc_t *pdc, uint8_t command, ///< The command to send - PICC_CMD_REQA or PICC_CMD_WUPA
		uint8_t *bufferATQA, ///< The buffer to store the ATQA (Answer to request) in
		size_t *bufferSize ///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
		) {
	uint8_t validBits = 0;
	rc52x_result_t status;

	if (bufferATQA == NULL || *bufferSize < 2) { // The ATQA response is 2 bytes long.
		return STATUS_NO_ROOM;
	}

	// Do we need to keep this into the port?
	//RC52X_ClearRegisterBitMask(rc52x, RC52X_REG_CollReg, 0x80);// ValuesAfterColl=1 => Bits received after collision are cleared.

	validBits = 7;// For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) uint8_t. TxLastBits = BitFramingReg[2..0]

	status = pdc->TransceiveData(pdc, &command, 1, bufferATQA, bufferSize,
			&validBits, 0, NULL, false, false);
	//status = RC52X_TransceiveData(rc52x, &command, 1, bufferATQA, bufferSize, &validBits, 0, false);
	if (status != STATUS_OK) {
		return status;
	}
	if (*bufferSize != 2 || validBits != 0) {	// ATQA must be exactly 16 bits.
		return STATUS_ERROR;
	}
	return STATUS_OK;
} // End PICC_REQA_or_WUPA()

/**
 * Transmits SELECT/ANTICOLLISION commands to select a single PICC.
 * Before calling this function the PICCs must be placed in the READY(*) state by calling PICC_RequestA() or PICC_WakeupA().
 * On success:
 * 		- The chosen PICC is in state ACTIVE(*) and all other PICCs have returned to state IDLE/HALT. (Figure 7 of the ISO/IEC 14443-3 draft.)
 * 		- The UID size and value of the chosen PICC is returned in *uid along with the SAK.
 *
 * A PICC UID consists of 4, 7 or 10 bytes.
 * Only 4 uint8_ts can be specified in a SELECT command, so for the longer UIDs two or three iterations are used:
 * 		UID size	Number of UID uint8_ts		Cascade levels		Example of PICC
 * 		========	===================		==============		===============
 * 		single				 4						1				MIFARE Classic
 * 		double				 7						2				MIFARE Ultralight
 * 		triple				10						3				Not currently in use?
 *
 *
 * uid:  Pointer to Uid struct. Normally output, but can also be used to supply a known UID.
 * validBits: The number of known UID bits supplied in *uid. Normally 0. If set you must also supply uid->size.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
rc52x_result_t PICC_Select(bs_pdc_t *pdc, picc_t *picc, uint8_t validBits) {
	bool uidComplete;
	bool selectDone;
	bool useCascadeTag;
	uint8_t cascadeLevel = 1;
	rc52x_result_t result;
	uint8_t count;
	uint8_t checkBit;
	uint8_t index;
	uint8_t uidIndex; // The first index in uid->uidByte[] that is used in the current Cascade Level.
	int8_t currentLevelKnownBits; // The number of known UID bits in the current Cascade Level.
	uint8_t buffer[9]; // The SELECT/ANTICOLLISION commands uses a 7 uint8_t standard frame + 2 uint8_ts CRC_A
	uint8_t bufferUsed;	// The number of uint8_ts used in the buffer, ie the number of uint8_ts to transfer to the FIFO.
	uint8_t rxAlign;// Used in BitFramingReg. Defines the bit position for the first bit received.
	uint8_t txLastBits;	// Used in BitFramingReg. The number of valid bits in the last transmitted uint8_t.
	uint8_t *responseBuffer;
	size_t responseLength;
	bool sendCRC;

	// Description of buffer structure:
	//		uint8_t 0: SEL 				Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
	//		uint8_t 1: NVB					Number of Valid Bits (in complete command, not just the UID): High nibble: complete uint8_ts, Low nibble: Extra bits.
	//		uint8_t 2: UID-data or CT		See explanation below. CT means Cascade Tag.
	//		uint8_t 3: UID-data
	//		uint8_t 4: UID-data
	//		uint8_t 5: UID-data
	//		uint8_t 6: BCC					Block Check Character - XOR of uint8_ts 2-5
	//		uint8_t 7: CRC_A
	//		uint8_t 8: CRC_A
	// The BCC and CRC_A are only transmitted if we know all the UID bits of the current Cascade Level.
	//
	// Description of uint8_ts 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
	//		UID size	Cascade level	uint8_t2	uint8_t3	uint8_t4	uint8_t5
	//		========	=============	=====	=====	=====	=====
	//		 4 uint8_ts		1			uid0	uid1	uid2	uid3
	//		 7 uint8_ts		1			CT		uid0	uid1	uid2
	//						2			uid3	uid4	uid5	uid6
	//		10 uint8_ts		1			CT		uid0	uid1	uid2
	//						2			CT		uid3	uid4	uid5
	//						3			uid6	uid7	uid8	uid9

	// Sanity checks
	if (validBits > 80) {
		return STATUS_INVALID;
	}

	// Prepare MFRC522
	// TODO
	//RC52X_ClearRegisterBitMask(rc52x, RC52X_REG_CollReg, 0x80);// ValuesAfterColl=1 => Bits received after collision are cleared.

	// Repeat Cascade Level loop until we have a complete UID.
	uidComplete = false;
	while (!uidComplete) {
		// Set the Cascade Level in the SEL uint8_t, find out if we need to use the Cascade Tag in uint8_t 2.
		switch (cascadeLevel) {
		case 1:
			buffer[0] = PICC_CMD_SEL_CL1;
			uidIndex = 0;
			useCascadeTag = validBits && picc->uid_size > 4;// When we know that the UID has more than 4 uint8_ts
			break;

		case 2:
			buffer[0] = PICC_CMD_SEL_CL2;
			uidIndex = 3;
			useCascadeTag = validBits && picc->uid_size > 7;// When we know that the UID has more than 7 uint8_ts
			break;

		case 3:
			buffer[0] = PICC_CMD_SEL_CL3;
			uidIndex = 6;
			useCascadeTag = false;						// Never used in CL3.
			break;

		default:
			return STATUS_INTERNAL_ERROR;
			break;
		}

		// How many UID bits are known in this Cascade Level?
		currentLevelKnownBits = validBits - (8 * uidIndex);
		if (currentLevelKnownBits < 0) {
			currentLevelKnownBits = 0;
		}
		// Copy the known bits from uid->uid[] to buffer[]
		index = 2; // destination index in buffer[]
		if (useCascadeTag) {
			buffer[index++] = PICC_CMD_CT;
		}
		uint8_t BytesToCopy = currentLevelKnownBits / 8
				+ (currentLevelKnownBits % 8 ? 1 : 0); // The number of uint8_ts needed to represent the known bits for this level.
		if (BytesToCopy) {
			uint8_t maxBytes = useCascadeTag ? 3 : 4; // Max 4 uint8_ts in each Cascade Level. Only 3 left if we use the Cascade Tag
			if (BytesToCopy > maxBytes) {
				BytesToCopy = maxBytes;
			}
			for (count = 0; count < BytesToCopy; count++) {
				buffer[index++] = picc->uid[uidIndex + count];
			}
		}
		// Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
		if (useCascadeTag) {
			currentLevelKnownBits += 8;
		}

		// Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
		selectDone = false;
		while (!selectDone) {
			// Find out how many bits and uint8_ts to send and receive.
			if (currentLevelKnownBits >= 32) { // All UID bits in this Cascade Level are known. This is a SELECT.
				//Serial.print(F("SELECT: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
				buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole uint8_ts
				// Calculate BCC - Block Check Character
				buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];

				sendCRC = true;

				txLastBits = 0; // 0 => All 8 bits are valid.
				bufferUsed = 7;
				// Store response in the last 3 uint8_ts of buffer (BCC and CRC_A - not needed after tx)
				responseBuffer = &buffer[6];
				responseLength = 3;
			} else { // This is an ANTICOLLISION.
					 //Serial.print(F("ANTICOLLISION: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
				txLastBits = currentLevelKnownBits % 8;
				count = currentLevelKnownBits / 8;// Number of whole uint8_ts in the UID part.
				index = 2 + count;// Number of whole uint8_ts: SEL + NVB + UIDs
				buffer[1] = (index << 4) + txLastBits;// NVB - Number of Valid Bits
				bufferUsed = index + (txLastBits ? 1 : 0);
				// Store response in the unused part of buffer
				responseBuffer = &buffer[index];
				responseLength = sizeof(buffer) - index;
				sendCRC = false;
			}

			//??rxAlign = txLastBits;// Having a separate variable is overkill. But it makes the next line easier to read.
			rxAlign = 0;
			uint8_t collisionPos;
			// Transmit the buffer and receive the response.
			result = pdc->TransceiveData(pdc, buffer, bufferUsed,
					responseBuffer, &responseLength, &txLastBits, rxAlign,
					&collisionPos, sendCRC, false);
			if (result == STATUS_COLLISION) {

				if (collisionPos <= currentLevelKnownBits) { // No progress - should not happen
					return STATUS_INTERNAL_ERROR;
				}
				// Choose the PICC with the bit set.
				currentLevelKnownBits = collisionPos;
				count = currentLevelKnownBits % 8; // The bit to modify
				checkBit = (currentLevelKnownBits - 1) % 8;
				index = 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First uint8_t is index 0.
				buffer[index] |= (1 << checkBit);
			} else if (result != STATUS_OK) {
				return result;
			} else { // STATUS_OK
				if (currentLevelKnownBits >= 32) { // This was a SELECT.
					selectDone = true; // No more anticollision
					// We continue below outside the while.
				} else { // This was an ANTICOLLISION.
						 // We now have all 32 bits of the UID in this Cascade Level
					currentLevelKnownBits = 32;
					// Run loop again to do the SELECT.
				}
			}
		} // End of while (!selectDone)

		// We do not check the CBB - it was constructed by us above.

		// Copy the found UID uint8_ts from buffer[] to uid->uid[]
		index = (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
		BytesToCopy = (buffer[2] == PICC_CMD_CT) ? 3 : 4;
		for (count = 0; count < BytesToCopy; count++) {
			picc->uid[uidIndex + count] = buffer[index++];
		}

		// Check response SAK (Select Acknowledge)
		if (responseLength != 3 || txLastBits != 0) { // SAK must be exactly 24 bits (1 uint8_t + CRC_A).
			return STATUS_ERROR;
		}

		/*
		 // Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those uint8_ts are not needed anymore.
		 result = RC52X_CalculateCRC(rc52x, responseBuffer, 1, &buffer[2]);
		 if (result != STATUS_OK) {
		 return result;
		 }

		 if ((buffer[2] != responseBuffer[1])
		 || (buffer[3] != responseBuffer[2])) {
		 return STATUS_CRC_WRONG;
		 }
		 */

		if (responseBuffer[0] & 0x04) { // Cascade bit set - UID not complete yes
			cascadeLevel++;
		} else {
			uidComplete = true;
			picc->sak.as_uint8 = responseBuffer[0];
		}
	} // End of while (!uidComplete)

	// Set correct uid->uid_size
	picc->uid_size = 3 * cascadeLevel + 1;

	return STATUS_OK;
} // End PICC_Select()

/**
 * Instructs a PICC in state ACTIVE(*) to go to state HALT.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
rc52x_result_t PICC_HaltA(bs_pdc_t *pdc) {
	rc52x_result_t result;
	uint8_t buffer[4];

	// Build command buffer
	buffer[0] = PICC_CMD_HLTA;
	buffer[1] = 0;

	// Send the command.
	// The standard says:
	//		If the PICC responds with any modulation during a period of 1 ms after the end of the frame containing the
	//		HLTA command, this response shall be interpreted as 'not acknowledge'.
	// We interpret that this way: Only STATUS_TIMEOUT is a success.
	result = pdc->TransceiveData(pdc, buffer, 2, NULL, NULL, NULL, 0, NULL,
			true, true);

	if (result == STATUS_TIMEOUT) {
		return STATUS_OK;
	}
	if (result == STATUS_OK) { // That is ironically NOT ok in this case ;-)
		return STATUS_ERROR;
	}
	return result;
} // End PICC_HaltA()

int PICC_RATS(bs_pdc_t *pdc, picc_t *picc) {
	int status;
	uint8_t buffer[4];

	buffer[0] = PICC_CMD_RATS;
	buffer[1] = 0x00; // 0x00 = 16 bytes

	size_t backsize = 16;
	status = pdc->TransceiveData(pdc, buffer, 2, &picc->rats, &backsize, NULL,
			0, NULL, true, true);
	if (status)
		return status;

	picc->iso14443_4_pcb = 2;
	return status;

}

int PICC_APDU (bs_pdc_t *pdc, picc_t *picc,
		uint8_t CLA,uint8_t INS,uint8_t P1,uint8_t P2,uint8_t Lc,uint8_t *Data,uint8_t Le,
		void* recv_buffer, size_t *recv_size) {
		uint8_t send_buffer[7 + Lc];
		int offset = 0;
		send_buffer[offset++] = picc->iso14443_4_pcb;
		picc->iso14443_4_pcb ^= 1;
		send_buffer[offset++] = CLA;
		send_buffer[offset++] = INS;
		send_buffer[offset++] = P1;
		send_buffer[offset++] = P2;
		if (Lc && Data) {
			send_buffer[offset++] = Lc;

			memcpy(send_buffer + offset++, Data, Lc);
		}
		send_buffer[Lc+offset++] = Le;
		return pdc->TransceiveData(pdc, send_buffer, offset, recv_buffer, recv_size, NULL,
					0, NULL, true, true);


}

int MIFARE_GET_VERSION(bs_pdc_t *pdc, picc_t *picc) {
	uint8_t buffer[3];
	int result;
	// Build command buffer
	buffer[0] = 0x60;
	// Calculate CRC_A
//	result = RC52X_CalculateCRC(rc52x, buffer, 1, &buffer[2]);
//	if (result != STATUS_OK) {
//		return result;
//	}
	size_t backsize = 10;

	return pdc->TransceiveData(pdc, buffer, 1, &picc->version_response,
			&backsize, NULL, 0, NULL, true, true);

}

int DESFIRE_GET_VERSION(bs_pdc_t *pdc, picc_t *picc) {
	rc52x_result_t result;
//	size_t backsize = 10;
////	{
////		uint8_t buffer[7] = { 0x00, 0xA4, 0x00, 0x00, 0x00, 0x00 };
////
////
////
////
////		result = pdc->TransceiveData(pdc, buffer, 6,
////				&picc->version_response, &backsize, NULL, 0, NULL, true, true);
////
////	}
//
//	{
//		uint8_t send_buffer[7] = { 0x02, 0x90, 0x60, 0x00, 0x00, 0x00 };
//
//		// ADPU must be prefixed with a 0x02
//		// This is a "Protocol Control Byte"
//
//		// Answer will also be prefixed with 0x02
//		// and suffixed with a status code 0x91 0xAF
//
//		uint8_t response_buffer[20];
//		backsize = sizeof(response_buffer);
//
//		result = pdc->TransceiveData(pdc, send_buffer, 6, response_buffer,
//				&backsize, NULL, 0, NULL, true, true);
//		if (0 == result && backsize == 10 && response_buffer[8] == 0x91
//				&& response_buffer[9] == 0xAF) {
//			memcpy((&picc->version_response) + 1, response_buffer + 1, 9);
//
//			send_buffer[2] = 0xAF;
//
//			int countdown = 5;
//			while (response_buffer[9] == 0xAF && backsize == 10) {
//
//				send_buffer[0] ^= 1;
//				backsize = sizeof(response_buffer);
//				memset (response_buffer, 0, sizeof(response_buffer));
//				result = pdc->TransceiveData(pdc, send_buffer, 6,
//						response_buffer, &backsize, NULL, 0, NULL, true, true);
//
//				countdown--;
//				if (!countdown)
//					break;
//			}
//
//			if (0 == result && backsize == 10 && response_buffer[8] == 0x91
//					&& response_buffer[9] == 0x00) {
//				return 0;
//			}
//
//		}
//
//	}
//
//	return -1;
	uint8_t response_buffer[20];
	size_t response_size  = sizeof(response_buffer);
	result=PICC_APDU(pdc, picc, 0x90, 0x60, 0x00, 0x00, 0x00,NULL, 0x00,  response_buffer, &response_size);
	 response_size  = sizeof(response_buffer);
	 result=PICC_APDU(pdc, picc,0x90, 0xAF, 0x00, 0x00, 0x00,NULL, 0x00,  response_buffer, &response_size);
	 response_size  = sizeof(response_buffer);
	 result=PICC_APDU(pdc, picc, 0x90, 0xAF, 0x00, 0x00, 0x00,NULL, 0x00,  response_buffer, &response_size);
	return result;

}


int MIFARE_READ(bs_pdc_t *pdc, picc_t *picc, int page, uint8_t *data) {
	uint8_t buffer[4];
	int result;
// Build command buffer
	buffer[0] = 0x30;
	buffer[1] = page;

	size_t backsize = 16;
	uint8_t validBits = 0;

	result = pdc->TransceiveData(pdc, buffer, 2, data, &backsize, &validBits, 0,
			NULL, true, true);

	if (STATUS_OK == result && 1 == backsize && 4 == validBits) {
		// We've received a status in stead of data
		switch (*data) {
		case 0x0:
			// invalid argument
			result = STATUS_INVALID;
			break;
		case 0x1:
			// crc error
			result = STATUS_CRC_WRONG;
			break;
		case 0x4:
			// auth error
			result = STATUS_AUTH_ERROR;
			break;
		case 0x5:
			// eeprom error
			result = STATUS_EEPROM_ERROR;
			break;
		case 0xa:
			// no error
			// should not get a status when
			// the read operation is valid
		default:
			result = STATUS_ERROR;
			break;
		}
		return result;
	}
	if (backsize != 16) {
		return STATUS_ERROR;
	}
	return result;

}

int MFU_Write(bs_pdc_t *pdc, picc_t *picc, int page, uint8_t *data) {
	uint8_t buffer[8];
	int result;
// Build command buffer
	buffer[0] = 0xA2;
	buffer[1] = page;
	memcpy(buffer + 2, data, 4);

	uint8_t backBuffer[1];
	size_t backsize = 1;
	uint8_t validBits = 0;

	result = pdc->TransceiveData(pdc, buffer, 6, backBuffer, &backsize,
			&validBits, 0, NULL, true, false);

	if (STATUS_OK == result && 1 == backsize && 4 == validBits) {
		// We've received a status in stead of data
		switch (*backBuffer) {
		case 0x0:
			// invalid argument
			result = STATUS_INVALID;
			break;
		case 0x1:
			// crc error
			result = STATUS_CRC_WRONG;
			break;
		case 0x4:
			// auth error
			result = STATUS_AUTH_ERROR;
			break;
		case 0x5:
			// eeprom error
			result = STATUS_EEPROM_ERROR;
			break;
		case 0xa:
			// Expecting this result when writing
			result = STATUS_OK;
			break;
		default:
			result = STATUS_ERROR;
			break;
		}
	}

	return result;
}

