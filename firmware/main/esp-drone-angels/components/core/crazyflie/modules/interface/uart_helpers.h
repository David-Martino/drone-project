/* ------------------------------ Module Start ------------------------------ */

/**
 * @file uart_helpers.h
 * @brief Public APIs for packaging data for UART transmission.
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
 * This file contains public APIs for functions that are useful for packaging 
 * data for UART transmission.
 *
 * @author Erwin Bauernschmitt <22964301@student.uwa.edu.au>
 * @date 6 October 2025
 * 
 * @copyright
 *   © 2025 Erwin Bauernschmitt
 *   Licensed under GPLv3.0; see the LICENSE file
 */

/* ----------------------------- Include Guard ------------------------------ */

#ifndef __UART_HELPERS_H__
#define __UART_HELPERS_H__

/* -------------------------------- Includes -------------------------------- */

// Standard includes
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

/* ----------------------------- Configurations ----------------------------- */

#if defined(__GNUC__)
#define FORCE_INLINE __attribute__((always_inline)) inline
#else
#define FORCE_INLINE inline
#endif

/* ------------------------------ Public APIs ------------------------------- */

/**
 * @brief Write an 8-bit unsigned integer.
 *
 * Writes @p value into @p buffer at @p offset. 
 *
 * @param[out] buffer  Destination byte buffer (must not be NULL).
 * @param[in]  offset  Index in @p buffer to write to.
 * @param[in]  value   The 8-bit value to write.
 * @return The next free offset (i.e., @p offset + 1).
 */
static FORCE_INLINE size_t put_u8(uint8_t* buffer, size_t offset, uint8_t value) 
{
	buffer[offset++] = value; return offset;
}

/**
 * @brief Write a 16-bit unsigned integer in little-endian order.
 *
 * Writes @p value into @p buffer starting at @p offset using little-endian
 * byte order (least significant byte first). The caller must ensure that
 * the buffer contains at least 2 bytes from @p offset.
 *
 * @param[out] buffer  Destination byte buffer (must not be NULL).
 * @param[in]  offset  Start index in @p buffer to write to.
 * @param[in]  value   The 16-bit value to write.
 * @return The next free offset (i.e., @p offset + 2).
 *
 * @note Little-endian encoding: b[offset] = LSB, b[offset+1] = MSB.
 */
static FORCE_INLINE size_t put_u16_le(uint8_t* buffer, size_t offset, uint16_t value) 
{
	buffer[offset++] = (uint8_t)(value      );
	buffer[offset++] = (uint8_t)(value >>  8);
	return offset;
}

/**
 * @brief Write a 16-bit signed integer in little-endian order.
 *
 * Writes @p value into @p buffer starting at @p offset using little-endian
 * byte order (least significant byte first). The caller must ensure that
 * the buffer contains at least 2 bytes from @p offset. The signed @p value 
 * just gets cast to uint16_t when calling put_u16_le.
 *
 * @param[out] buffer  Destination byte buffer (must not be NULL).
 * @param[in]  offset  Start index in @p buffer to write to.
 * @param[in]  value   The 16-bit value to write.
 * @return The next free offset (i.e., @p offset + 2).
 *
 * @note Little-endian encoding: b[offset] = LSB, b[offset+1] = MSB.
 */
static FORCE_INLINE size_t put_i16_le(uint8_t* buffer, size_t offset, int16_t value) 
{
	return put_u16_le(buffer, offset, (uint16_t)value);
}

/**
 * @brief Write a 32-bit unsigned integer in little-endian order.
 *
 * Writes @p value into @p buffer starting at @p offset using little-endian
 * byte order (least significant byte first). The caller must ensure that
 * the buffer contains at least 4 bytes from @p offset.
 *
 * @param[out] buffer  Destination byte buffer (must not be NULL).
 * @param[in]  offset  Start index in @p buffer to write to.
 * @param[in]  value   The 32-bit value to write.
 * @return The next free offset (i.e., @p offset + 4).
 *
 * @note Little-endian encoding: b[offset] = LSB, b[offset+3] = MSB.
 */
static FORCE_INLINE size_t put_u32_le(uint8_t* buffer, size_t offset, uint32_t value) 
{
	buffer[offset++] = (uint8_t)(value      );
	buffer[offset++] = (uint8_t)(value >>  8);
	buffer[offset++] = (uint8_t)(value >> 16);
	buffer[offset++] = (uint8_t)(value >> 24);
	return offset;
}

/**
 * @brief Write a 32-bit float in little-endian order.
 *
 * Writes @p value into @p buffer starting at @p offset using little-endian
 * byte order (least significant byte first). The caller must ensure that
 * the buffer contains at least 4 bytes from @p offset. The float @p value 
 * just gets bit-cast to uint32_t before calling put_u32_le.
 *
 * @param[out] buffer  Destination byte buffer (must not be NULL).
 * @param[in]  offset  Start index in @p buffer to write to.
 * @param[in]  value   The 32-bit value to write.
 * @return The next free offset (i.e., @p offset + 4).
 *
 * @note Little-endian encoding: b[offset] = LSB, b[offset+3] = MSB.
 */
static FORCE_INLINE size_t put_f32_le(uint8_t* buffer, size_t offset, float value) 
{
	uint32_t bits;
	memcpy(&bits, &value, sizeof bits);  // bit-preserving copy
	return put_u32_le(buffer, offset, bits);
}

/**
 * @brief Write a 64-bit unsigned integer in little-endian order.
 *
 * Writes @p value into @p buffer starting at @p offset using little-endian
 * byte order (least significant byte first). The caller must ensure that
 * the buffer contains at least 8 bytes from @p offset.
 *
 * @param[out] buffer  Destination byte buffer (must not be NULL).
 * @param[in]  offset  Start index in @p buffer to write to.
 * @param[in]  value   The 64-bit value to write.
 * @return The next free offset (i.e., @p offset + 8).
 *
 * @note Little-endian encoding: b[offset] = LSB, b[offset+7] = MSB.
 */
static FORCE_INLINE size_t put_u64_le(uint8_t* buffer, size_t offset, uint64_t value) 
{
	buffer[offset++] = (uint8_t)(value      );
	buffer[offset++] = (uint8_t)(value >>  8);
	buffer[offset++] = (uint8_t)(value >> 16);
	buffer[offset++] = (uint8_t)(value >> 24);
	buffer[offset++] = (uint8_t)(value >> 32);
	buffer[offset++] = (uint8_t)(value >> 40);
	buffer[offset++] = (uint8_t)(value >> 48);
	buffer[offset++] = (uint8_t)(value >> 56);
	return offset;
}

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
 * @param[in] data  Pointer to the input buffer to checksum (may be NULL if len==0).
 * @param[in] len   Number of bytes in @p data to process.
 * @return 16-bit CRC value (host-endian). For wire format, write it out little-endian
 *         with your existing put_u16_le().
 *
 * @note Complexity: O(len). No lookup table (small footprint).
 * @note The receiver must use the same CRC parameters to verify.
 */
uint16_t crc16_ccitt(const uint8_t* data, size_t len);

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
 * @return Number of bytes written to @p output (encoded length, without delimiter).
 *
 * @note Complexity: O(length).
 * @note This function never writes 0x00 into @p output. The caller should append
 *       a single 0x00 delimiter on the wire between frames.
 */
size_t cobs_encode(const uint8_t* input, size_t length, uint8_t* output);

#endif 

/* ------------------------------- Module End ------------------------------- */
