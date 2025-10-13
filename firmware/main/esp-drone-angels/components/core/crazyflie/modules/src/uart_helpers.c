/* ------------------------------ Module Start ------------------------------ */

/**
 * @file uart_helpers.c
 * @brief Implements the functionality for the UART helper functions.
 *
 *                  $$$$$$$\            $$\ $$\ $$\
 *                  $$  __$$\           $$ |$$ |$  |
 *                  $$ |  $$ | $$$$$$\  $$ |$$ |\_/$$$$$$$\
 *                  $$ |  $$ |$$  __$$\ $$ |$$ |  $$  _____|
 *                  $$ |  $$ |$$$$$$$$ |$$ |$$ |  \$$$$$$\
 *                  $$ |  $$ |$$   ____|$$ |$$ |   \____$$\
 *                  $$$$$$$  |\$$$$$$$\ $$ |$$ |  $$$$$$$  |
 *                  \_______/  \_______|\__|\__|  \_______/
 *
 *            $$$$$$\                                $$\
 *           $$  __$$\                               $$ |
 *           $$ /  $$ |$$$$$$$\   $$$$$$\   $$$$$$\  $$ | $$$$$$$\
 *           $$$$$$$$ |$$  __$$\ $$  __$$\ $$  __$$\ $$ |$$  _____|
 *           $$  __$$ |$$ |  $$ |$$ /  $$ |$$$$$$$$ |$$ |\$$$$$$\
 *           $$ |  $$ |$$ |  $$ |$$ |  $$ |$$   ____|$$ | \____$$\
 *           $$ |  $$ |$$ |  $$ |\$$$$$$$ |\$$$$$$$\ $$ |$$$$$$$  |
 *           \__|  \__|\__|  \__| \____$$ | \_______|\__|\_______/
 *                               $$\   $$ |
 *                               \$$$$$$  |
 *                                \______/
 * 
 * This file contains the definitions for functions and variables declared in
 * uart_helpers.h, providing the core logic for the UART helper functions.
 *
 * @author Erwin Bauernschmitt <22964301@student.uwa.edu.au>
 * @date 6 October 2025
 * 
 * @copyright
 *   © 2025 Erwin Bauernschmitt
 *   Licensed under GPLv3.0; see the LICENSE file
 */

/* -------------------------------- Includes -------------------------------- */

// Standard includes
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

// App-specific includes
#include "uart_helpers.h"

/* ------------------------------- Public APIs ------------------------------ */

/**
 * @brief Compute CRC-16/CCITT-FALSE over a byte buffer.
 *
 * This computes the CRC-16 variant commonly called **CCITT-FALSE**:
 * - Polynomial: 0x1021 (x^16 + x^12 + x^5 + 1)
 * - Initial value: 0xFFFF
 * - Refin: false (no input reflection)
 * - Refout: false (no output reflection)
 * - XorOut: 0x0000
 *
 * The algorithm processes each input byte MSB-first by xoring it into the
 * high 8 bits of the 16-bit CRC register, then iterating 8 times:
 * shift left by 1, and if the previous MSB was set, xor with 0x1021.
 *
 * @param[in] data  Pointer to the input buffer to checksum.
 * @param[in] len   Number of bytes in @p data to process.
 * @return 16-bit CRC value. 
 *
 * @note Complexity: O(len). No lookup table (small footprint).
 * @note The receiver must use the same CRC parameters to verify.
 */
uint16_t crc16_ccitt(const uint8_t* data, size_t len) 
{
	uint16_t crc = 0xFFFF;
	for (size_t i = 0; i < len; ++i) 
	{
		crc ^= (uint16_t)data[i] << 8;
		for (int b = 0; b < 8; ++b) 
		{
			if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
			else              crc <<= 1;
		}
	}
	return crc;
}

/**
 * @brief Encode a byte buffer using COBS (Consistent Overhead Byte Stuffing).
 *
 * COBS removes all zero bytes from the encoded output, allowing 0x00 to be used
 * as a frame delimiter on the wire. The encoding divides the input into blocks
 * of up to 254 non-zero bytes, each preceded by a "code byte" that tells the
 * decoder how many non-zero bytes follow before the next zero (or block end).
 *
 * Rules:
 * - Each block starts with one code byte (written later), followed by up to 254
 *   non-zero data bytes.
 * - Encountering a zero in the input closes the current block: write the code
 *   value, start a new block (code initially 1).
 * - If a block reaches 254 non-zero bytes (code would become 0xFF), close it
 *   with code=0xFF and start a new block.
 * - After the last input byte, close the final block by writing its code.
 *
 * The output does NOT include a trailing 0x00 delimiter; append that yourself
 * after this function returns to mark the end of frame on the wire.
 *
 * @param[in]  input   Pointer to input buffer (may contain zeros).
 * @param[in]  length  Number of input bytes to encode.
 * @param[out] output  Destination buffer for encoded bytes. 
 *
 * @return Number of bytes written to @p output (encoded length, no delimiter).
 *
 * @note Complexity: O(length).
 * @note This function never writes 0x00 into @p output. The caller must append
 *       a single 0x00 delimiter on the wire between frames.
 */
size_t cobs_encode(const uint8_t* input, size_t length, uint8_t* output) 
{
	const uint8_t* in = input;
	const uint8_t* end = input + length;

	uint8_t* out = output;
	uint8_t* code_ptr = out++;   // Reserve space for first code byte
	uint8_t  code = 1;           // Counts bytes since last zero (1..255)

	while (in < end) 
	{
		if (*in == 0) 
		{
			// Close current block at the code_ptr
			*code_ptr = code;
			// Start a new block
			code_ptr = out++;  // reserve space for the next code byte
			code = 1;
			++in;
		} 
		else 
		{
			*out++ = *in++;
			if (++code == 0xFF)		// 254 data bytes in this block (0xFF code)
			{
				*code_ptr = 0xFF;   // close full block
				code_ptr = out++;   // start a new block
				code = 1;
			}
		}
	}

	// Close the final block
	*code_ptr = code;

	return (size_t)(out - output);
}