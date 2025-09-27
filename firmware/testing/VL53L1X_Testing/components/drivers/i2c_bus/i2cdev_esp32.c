/**
 *
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * i2cdev.c - Functions to write to I2C devices
 */
#define DEBUG_MODULE "I2CDEV"

#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "stm32_legacy.h"
#include "i2cdev.h"
#include "i2c_drv.h"
#include "nvicconf.h"
#include "debug_cf.h"

int i2cdevInit(I2C_Dev *dev)
{
    i2cdrvInit(dev);
    return true;
}

bool i2cdevRead(I2C_Dev *dev, uint8_t devAddress, uint16_t len, uint8_t *data)
{
    return i2cdevReadReg8(dev, devAddress, I2CDEV_NO_MEM_ADDR, len, data);
}

bool i2cdevReadByte(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                    uint8_t *data)
{
    return i2cdevReadReg8(dev, devAddress, memAddress, 1, data);
}

bool i2cdevReadBit(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                   uint8_t bitNum, uint8_t *data)
{
    uint8_t byte;
    bool status;

    status = i2cdevReadReg8(dev, devAddress, memAddress, 1, &byte);
    *data = byte & (1 << bitNum);

    return status;
}

bool i2cdevReadBits(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                    uint8_t bitStart, uint8_t length, uint8_t *data)
{
    bool status;
    uint8_t byte;

    if ((status = i2cdevReadByte(dev, devAddress, memAddress, &byte)) == true) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        byte &= mask;
        byte >>= (bitStart - length + 1);
        *data = byte;
    }

    return status;
}

bool i2cdevReadReg8(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                    uint16_t len, uint8_t *data)
{
    if (xSemaphoreTake(dev->isBusFreeMutex, (TickType_t)5) == pdFALSE) {
        return false;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (memAddress != I2CDEV_NO_MEM_ADDR) {
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (devAddress << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK_EN);
        i2c_master_write_byte(cmd, memAddress, I2C_MASTER_ACK_EN);
    }
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddress << 1) | I2C_MASTER_READ, I2C_MASTER_ACK_EN);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(dev->def->i2cPort, cmd, (TickType_t)5);
    i2c_cmd_link_delete(cmd);

    xSemaphoreGive(dev->isBusFreeMutex);

#if defined CONFIG_I2CBUS_LOG_READWRITES

    if (!err) {
        char str[length * 5 + 1];

        for (size_t i = 0; i < length; i++) {
            sprintf(str + i * 5, "0x%s%X ", (data[i] < 0x10 ? "0" : ""), data[i]);
        }

        I2CBUS_LOG_RW("[port:%d, slave:0x%X] Read_ %d bytes from register 0x%X, data: %s", port, devAddr, length, regAddr, str);
    }

#endif
#if defined CONFIG_I2CBUS_LOG_ERRORS
#if defined CONFIG_I2CBUS_LOG_READWRITES
    else {
#else

    if (err) {
#endif
        I2CBUS_LOGE("[port:%d, slave:0x%X] Failed to read %d bytes from register 0x%X, error: 0x%X",
                    port, devAddr, length, regAddr, err);
    }

#endif

    if (err == ESP_OK) {
        return TRUE;
    } else {
        return false;
    }

}

bool i2cdevReadReg16(I2C_Dev *dev, uint8_t devAddress, uint16_t memAddress,
                     uint16_t len, uint8_t *data)
{
    if (xSemaphoreTake(dev->isBusFreeMutex, (TickType_t)5) == pdFALSE) {
        return false;
    }

    uint8_t memAddress8[2];
    memAddress8[0] = (uint8_t)((memAddress >> 8) & 0x00FF);
    memAddress8[1] = (uint8_t)(memAddress & 0x00FF);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (memAddress != I2C_NO_INTERNAL_ADDRESS) {
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (devAddress << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK_EN);
        i2c_master_write(cmd, memAddress8, 2, I2C_MASTER_ACK_EN);
    }
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddress << 1) | I2C_MASTER_READ, I2C_MASTER_ACK_EN);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(dev->def->i2cPort, cmd, (TickType_t)200); // @@ was 5 originally
    i2c_cmd_link_delete(cmd);

    xSemaphoreGive(dev->isBusFreeMutex);

#if defined CONFIG_I2CBUS_LOG_READWRITES

    if (!err) {
        char str[length * 5 + 1];

        for (size_t i = 0; i < length; i++) {
            sprintf(str + i * 5, "0x%s%X ", (data[i] < 0x10 ? "0" : ""), data[i]);
        }

        I2CBUS_LOG_RW("[port:%d, slave:0x%X] Read_ %d bytes from register 0x%X, data: %s", port, devAddr, length, regAddr, str);
    }

#endif
#if defined CONFIG_I2CBUS_LOG_ERRORS
#if defined CONFIG_I2CBUS_LOG_READWRITES
    else {
#else

    if (err) {
#endif
        I2CBUS_LOGE("[port:%d, slave:0x%X] Failed to read %d bytes from register 0x%X, error: 0x%X",
                    port, devAddr, length, regAddr, err);
    }

#endif

    if (err == ESP_OK) {
       //  ESP_LOGE("i2cdev", "Read Success!"); // @@
        return TRUE;
    } else {
        // ESP_LOGE("i2cdev", "Read Failure!"); // @@
        return false;
    }
}

bool i2cdevWriteByte(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                     uint8_t data)
{
    return i2cdevWriteReg8(dev, devAddress, memAddress, 1, &data);
}

bool i2cdevWriteBit(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                    uint8_t bitNum, uint8_t data)
{
    uint8_t byte;
    i2cdevReadByte(dev, devAddress, memAddress, &byte);
    byte = (data != 0) ? (byte | (1 << bitNum)) : (byte & ~(1 << bitNum));
    return i2cdevWriteByte(dev, devAddress, memAddress, byte);
}

bool i2cdevWriteBits(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                     uint8_t bitStart, uint8_t length, uint8_t data)
{
    bool status;
    uint8_t byte;

    if ((status = i2cdevReadByte(dev, devAddress, memAddress, &byte)) == true) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask;                     // zero all non-important bits in data
        byte &= ~(mask);                  // zero all important bits in existing byte
        byte |= data;                     // combine data with existing byte
        status = i2cdevWriteByte(dev, devAddress, memAddress, byte);
    }

    return status;
}


bool i2cdevWriteReg8(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                     uint16_t len, uint8_t *data)
{
    if (xSemaphoreTake(dev->isBusFreeMutex, (TickType_t)5) == pdFALSE) {
        return false;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddress << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK_EN);
    if (memAddress != I2CDEV_NO_MEM_ADDR) {
        i2c_master_write_byte(cmd, memAddress, I2C_MASTER_ACK_EN);
    }
    i2c_master_write(cmd, (uint8_t *)data, len, I2C_MASTER_ACK_EN);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(dev->def->i2cPort, cmd, (TickType_t)5);
    i2c_cmd_link_delete(cmd);

    xSemaphoreGive(dev->isBusFreeMutex);

#if defined CONFIG_I2CBUS_LOG_READWRITES

    if (!err) {
        char str[length * 5 + 1];

        for (size_t i = 0; i < length; i++) {
            sprintf(str + i * 5, "0x%s%X ", (data[i] < 0x10 ? "0" : ""), data[i]);
        }

        I2CBUS_LOG_RW("[port:%d, slave:0x%X] Write %d bytes to register 0x%X, data: %s",
                      port, devAddr, length, regAddr, str);
    }

#endif
#if defined CONFIG_I2CBUS_LOG_ERRORS
#if defined CONFIG_I2CBUS_LOG_READWRITES
    else {
#else

    if (err) {
#endif
        I2CBUS_LOGE("[port:%d, slave:0x%X] Failed to write %d bytes to__ register 0x%X, error: 0x%X",
                    port, devAddr, length, regAddr, err);
    }

#endif

    if (err == ESP_OK) {
        return TRUE;
    } else {
        return false;
    }
}

bool i2cdevWriteReg16(I2C_Dev *dev, uint8_t devAddress, uint16_t memAddress,
                      uint16_t len, uint8_t *data)
{
    if (xSemaphoreTake(dev->isBusFreeMutex, (TickType_t)5) == pdFALSE) {
        printf("\ni2cdevWriteReg16: Failed to take Bus semaphore!\n"); // @@
        return false;
    }

    uint8_t memAddress8[2];
    memAddress8[0] = (uint8_t)((memAddress >> 8) & 0x00FF);
    memAddress8[1] = (uint8_t)(memAddress & 0x00FF);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddress << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK_EN);
    if (memAddress != I2C_NO_INTERNAL_ADDRESS) {
        i2c_master_write(cmd, memAddress8, 2, I2C_MASTER_ACK_EN);
    }
    i2c_master_write(cmd, (uint8_t *)data, len, I2C_MASTER_ACK_EN);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(dev->def->i2cPort, cmd, (TickType_t)100); // @@
    i2c_cmd_link_delete(cmd);

    xSemaphoreGive(dev->isBusFreeMutex);
#if defined CONFIG_I2CBUS_LOG_READWRITES

    if (!err) {
        char str[length * 5 + 1];

        for (size_t i = 0; i < length; i++) {
            sprintf(str + i * 5, "0x%s%X ", (data[i] < 0x10 ? "0" : ""), data[i]);
        }

        I2CBUS_LOG_RW("[port:%d, slave:0x%X] Write %d bytes to register 0x%X, data: %s",
                      port, devAddr, length, regAddr, str);
    }

#endif

#if defined CONFIG_I2CBUS_LOG_ERRORS
#if defined CONFIG_I2CBUS_LOG_READWRITES
    else {
#else

    if (err) {
#endif
        I2CBUS_LOGE("[port:%d, slave:0x%X] Failed to write %d bytes to__ register 0x%X, error: 0x%X",
                    port, devAddr, length, regAddr, err);
    }

#endif

    if (err == ESP_OK) {
        return TRUE;
    } else {
        printf("\ni2cdevWriteReg16: Semaphore acquired, but new issue!\n"); // @@ 
        printf("[port:%d, slave:0x%X] Failed to write %d bytes to__ register 0x%X %x (or %u?), error: 0x%X\n",
                    dev->def->i2cPort, devAddress, len, memAddress8[0],memAddress8[1],memAddress, err); // @@
        return false;
    }
}


// @@ courtesy of ChatGPT:

// #define I2C_MAX_WRITE  32   // safe chunk size (ST usually uses 32 or 64)

// bool i2cdevWriteReg16(I2C_Dev *dev, uint8_t devAddress, uint16_t memAddress,
//                       uint16_t len, uint8_t *data)
// {
//     if (xSemaphoreTake(dev->isBusFreeMutex, pdMS_TO_TICKS(5)) == pdFALSE) {
//         ESP_LOGE("i2cdev", "Failed to take Bus semaphore");
//         return false;
//     }

//     esp_err_t err = ESP_OK;
//     uint16_t offset = 0;

//     while (offset < len) {
//         uint16_t chunk_len = (len - offset > I2C_MAX_WRITE) ? I2C_MAX_WRITE : (len - offset);

//         uint8_t memAddress8[2] = {
//             (uint8_t)((memAddress >> 8) & 0xFF),
//             (uint8_t)(memAddress & 0xFF)
//         }; // @@ this seems a bit sus...

//         i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//         i2c_master_start(cmd);
//         i2c_master_write_byte(cmd, (devAddress << 1) | I2C_MASTER_WRITE, true);
//         i2c_master_write(cmd, memAddress8, 2, true);
//         i2c_master_write(cmd, data + offset, chunk_len, true);
//         i2c_master_stop(cmd);
//         err = i2c_master_cmd_begin(dev->def->i2cPort, cmd, pdMS_TO_TICKS(200)); // @@ originally 5
//         i2c_cmd_link_delete(cmd);

//         if (err != ESP_OK) break;

//         offset += chunk_len;
//         memAddress += chunk_len;  // increment register address

//         vTaskDelay(pdMS_TO_TICKS(2)); // @@ small delay between chunks for stability
//         // ESP_LOGI("i2cdev", "%d: 0x%x into 0x%x%x",offset, *(data + offset), memAddress8[0], memAddress8[1]); // @@
//     }

//     xSemaphoreGive(dev->isBusFreeMutex);

//     if (err != ESP_OK) {
//         //ESP_LOGE("i2cdev", "Writing into register 0x%x%x failed at offset %d (chunk %d): err=0x%x", memAddress8[0], memAddress8[1], offset, len, err);
//         // ESP_LOGE("i2cdev", "Writing Fail"); // @@
//         return false;
//     }
//     else {
//         // ESP_LOGE("i2cdev", "Success!"); @@
//          //ESP_LOGE("i2cdev", "Write successful! Into register 0x%x%x", memAddress8[0], memAddress8[1]);
//     }
//     // DEBUGGING NOTES:
//     // It succeeds a few times before failing. Interestingly, it seems to be independent of the data, since 
//     // I can see the same message being successfully sent as is in the failed message - the fail message always
//     // seems to be 0x3fcd0280
//     // ChatGPT thinks it may be signal integrity / timing issue but I think it might just be to do with the device state = maybe its expected behaviour?
//     // Only reading -0.000mm
//     // may need to reverse eng the packets? ** check which command is publishing them, hopefully that will have information on what they are
//     // then can figure out what 0x3fcd0280 means

//     // could also try putting proper 10k pullups on i2c or wiring it differently but the distance is very short so eh..  


//     return true;
// }
