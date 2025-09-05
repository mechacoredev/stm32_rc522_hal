/*
 * rc522.c
 *
 *  Created on: Sep 2, 2025
 *      Author: Enes
 */

#include "rc522.h"
#include "stdlib.h"

typedef enum {
    RC522_CMD_IDLE              = 0x00, // no action, cancels current command
    RC522_CMD_MEM               = 0x01, // stores 25 bytes into the internal buffer
    RC522_CMD_GENERATE_RANDOMID = 0x02, // generates a 10-byte random ID number
    RC522_CMD_CALC_CRC          = 0x03, // activates CRC coprocessor / self test
    RC522_CMD_TRANSMIT          = 0x04, // transmits data from FIFO buffer
    RC522_CMD_NOCMDCHANGE       = 0x07, // no command change, only update CommandReg bits
    RC522_CMD_RECEIVE           = 0x08, // activates receiver circuits
    RC522_CMD_TRANSCEIVE        = 0x0C, // transmit FIFO data and auto-activate receiver
    RC522_CMD_RESERVED          = 0x0D, // reserved for future use
    RC522_CMD_MFAUTHENT         = 0x0E, // MIFARE authentication as reader
    RC522_CMD_SOFTRESET         = 0x0F  // reset MFRC522
} rc522_command_t;

typedef enum {
    // Page 0: Command and Status
    RC522_REG_RESERVED_00      = 0x00,
    RC522_REG_COMMAND          = 0x01,
    RC522_REG_COM_IEN          = 0x02,
    RC522_REG_DIV_IEN          = 0x03,
    RC522_REG_COM_IRQ          = 0x04,
    RC522_REG_DIV_IRQ          = 0x05,
    RC522_REG_ERROR            = 0x06,
    RC522_REG_STATUS1          = 0x07,
    RC522_REG_STATUS2          = 0x08,
    RC522_REG_FIFO_DATA        = 0x09,
    RC522_REG_FIFO_LEVEL       = 0x0A,
    RC522_REG_WATER_LEVEL      = 0x0B,
    RC522_REG_CONTROL          = 0x0C,
    RC522_REG_BIT_FRAMING      = 0x0D,
    RC522_REG_COLL             = 0x0E,
    RC522_REG_RESERVED_0F      = 0x0F,

    // Page 1: Command
    RC522_REG_RESERVED_10      = 0x10,
    RC522_REG_MODE             = 0x11,
    RC522_REG_TX_MODE          = 0x12,
    RC522_REG_RX_MODE          = 0x13,
    RC522_REG_TX_CONTROL       = 0x14,
    RC522_REG_TX_ASK           = 0x15,
    RC522_REG_TX_SEL           = 0x16,
    RC522_REG_RX_SEL           = 0x17,
    RC522_REG_RX_THRESHOLD     = 0x18,
    RC522_REG_DEMOD            = 0x19,
    RC522_REG_RESERVED_1A      = 0x1A,
    RC522_REG_RESERVED_1B      = 0x1B,
    RC522_REG_MF_TX            = 0x1C,
    RC522_REG_MF_RX            = 0x1D,
    RC522_REG_RESERVED_1E      = 0x1E,
    RC522_REG_SERIAL_SPEED     = 0x1F,

    // Page 2: Configuration
    RC522_REG_RESERVED_20      = 0x20,
    RC522_REG_CRC_RESULT       = 0x21, // MSB + LSB
    RC522_REG_CRC_RESULT_L     = 0x22,
    RC522_REG_RESERVED_23      = 0x23,
    RC522_REG_MOD_WIDTH        = 0x24,
    RC522_REG_RESERVED_25      = 0x25,
    RC522_REG_RF_CFG           = 0x26,
    RC522_REG_GS_N             = 0x27,
    RC522_REG_CW_GSP           = 0x28,
    RC522_REG_MOD_GSP          = 0x29,
    RC522_REG_TMODE            = 0x2A,
    RC522_REG_TPRESCALER       = 0x2B,
    RC522_REG_TRELOAD_HI       = 0x2C, // 16-bit
    RC522_REG_TRELOAD_LO       = 0x2D,
    RC522_REG_TCOUNTER_VAL     = 0x2E, // 16-bit
    RC522_REG_TCOUNTER_VAL_L   = 0x2F,

    // Page 3: Test Register
    RC522_REG_RESERVED_30      = 0x30,
    RC522_REG_TEST_SEL1        = 0x31,
    RC522_REG_TEST_SEL2        = 0x32,
    RC522_REG_TEST_PIN_EN      = 0x33,
    RC522_REG_TEST_PIN_VALUE   = 0x34,
    RC522_REG_TEST_BUS         = 0x35,
    RC522_REG_AUTO_TEST        = 0x36,
    RC522_REG_VERSION          = 0x37,
    RC522_REG_ANALOG_TEST      = 0x38,
    RC522_REG_TEST_DAC1        = 0x39,
    RC522_REG_TEST_DAC2        = 0x3A,
    RC522_REG_TEST_ADC         = 0x3B,
    RC522_REG_RESERVED_3C      = 0x3C,
    RC522_REG_RESERVED_3D      = 0x3D,
    RC522_REG_RESERVED_3E      = 0x3E,
    RC522_REG_RESERVED_3F      = 0x3F
} rc522_register_map_t;


struct rc522_t{
	SPI_HandleTypeDef* spi_handle;
	GPIO_TypeDef* rst_port;
	GPIO_TypeDef* cs_port;
	GPIO_TypeDef* irq_port;
	uint16_t rst_pin;
	uint16_t cs_pin;
	uint16_t irq_pin;
	uint32_t max_delay;
	uint8_t wait_irq;
	uint8_t irq_en;
	uint8_t command;
};

uint8_t static read_register(rc522_handle dev, uint8_t register_address){
	uint8_t address = ((register_address<<1)&0x7E)|0x80;
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(dev->spi_handle, &address, 1, dev->max_delay);
	uint8_t rx_buffer;
	HAL_SPI_Receive(dev->spi_handle, &rx_buffer, 1, dev->max_delay);
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
	return rx_buffer;
}

rc522_return_status_t static write_register(rc522_handle dev, uint8_t register_address, uint8_t txdata){
	uint8_t buffer[2];
	buffer[0]=(register_address<<1)&0x7E;
	buffer[1]=txdata;
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
	if(HAL_SPI_Transmit(dev->spi_handle, buffer, 2, dev->max_delay)!=HAL_OK){
		return rc522_fail;
	}
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
	return rc522_ok;
}

void static set_bit_mask(rc522_handle dev, uint8_t register_address, uint8_t mask){
	uint8_t buffer = read_register(dev, register_address);
	write_register(dev, register_address, buffer|mask);
}

void static clear_bit_mask(rc522_handle dev, uint8_t register_address, uint8_t mask){
	uint8_t buffer = read_register(dev, register_address);
	write_register(dev, register_address, buffer&(~mask));
}

void static antenna_on(rc522_handle dev){
	set_bit_mask(dev, RC522_REG_TX_CONTROL, 0x03);
}

void static antenna_off(rc522_handle dev){
	clear_bit_mask(dev, RC522_REG_TX_CONTROL, 0x03);
}

void static reset(rc522_handle dev){
	write_register(dev, RC522_REG_COMMAND, RC522_CMD_SOFTRESET);
}

rc522_handle rc522_init(rc522_config_t* config){
	rc522_handle dev = (rc522_handle)malloc(sizeof(struct rc522_t));
	if(dev==NULL) return NULL;
	dev->cs_pin=config->cs_pin;
	dev->cs_port=config->cs_port;
	dev->irq_pin=config->irq_pin;
	dev->irq_port=config->irq_port;
	dev->max_delay=config->max_delay;
	dev->rst_pin=config->rst_pin;
	dev->rst_port=config->rst_port;
	dev->spi_handle=config->spi_handle;
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(dev->rst_port, dev->rst_pin, GPIO_PIN_SET);
	reset(dev);
	return dev;
}

rc522_return_status_t rc522_configure(rc522_handle dev, rc522_config_t* config){
	if(write_register(dev, RC522_REG_TMODE, config->tmode.raw)!=rc522_ok){
		return write_fail;
	}
	if(write_register(dev, RC522_REG_TPRESCALER, config->tprescaler.raw)!=rc522_ok){
		return write_fail;
	}
	if(write_register(dev, RC522_REG_TRELOAD_LO, config->treloadlo.raw)!=rc522_ok){
		return write_fail;
	}
	if(write_register(dev, RC522_REG_TRELOAD_HI, config->treloadhi.raw)!=rc522_ok){
		return write_fail;
	}
	if(write_register(dev, RC522_REG_TX_ASK, config->txask.raw)!=rc522_ok){
		return write_fail;
	}
	if(write_register(dev, RC522_REG_MODE, config->mode.raw)!=rc522_ok){
		return write_fail;
	}
	if(write_register(dev, RC522_REG_TX_MODE, config->txmode.raw)!=rc522_ok){
		return write_fail;
	}
	if(write_register(dev, RC522_REG_RX_MODE, config->rxmode.raw)!=rc522_ok){
		return write_fail;
	}
	if(write_register(dev, RC522_REG_RF_CFG, config->rfcfg.raw)!=rc522_ok){
		return write_fail;
	}
	antenna_on(dev);
	return rc522_ok;
}

void rc522_clear_interrupt_bits(rc522_handle dev){
	clear_bit_mask(dev, RC522_REG_COM_IRQ, 0x80);
}

rc522_return_status_t static card_command(rc522_handle dev, uint8_t* txdata, uint8_t txlen){
	switch(dev->command){
		case RC522_CMD_MFAUTHENT:
		{
			dev->irq_en=0x12; // irq_en = interrupt request enable
			dev->wait_irq=0x10; // wait_irq = wait interrupt request
			break;
		}
		case RC522_CMD_TRANSCEIVE:
		{
			dev->irq_en=0x63; // eskiden 77 sonra 63
			dev->wait_irq=0x30;
			break;
		}
		default:
		{
			dev->irq_en=0;
			dev->wait_irq=0;
			break;
		}
	}
	if(write_register(dev, RC522_REG_COM_IEN, dev->irq_en|0x80)!=rc522_ok){
		return write_fail;
	}
	write_register(dev, RC522_REG_DIV_IEN, 0x00);
	write_register(dev, RC522_REG_COM_IRQ, 0x7F);  // 1 yazarak temizle
	write_register(dev, RC522_REG_DIV_IRQ, 0x7F);  // garanti olsun diye DivIrq da temizle
	set_bit_mask(dev, RC522_REG_FIFO_LEVEL, 0x80);
	if(write_register(dev, RC522_REG_COMMAND, RC522_CMD_IDLE)!=rc522_ok){
		return write_fail;
	}
	for(uint8_t i=0; i<txlen; i++){
		if(write_register(dev, RC522_REG_FIFO_DATA, txdata[i])!=rc522_ok){
			return write_fail;
		}
	}
	if(write_register(dev, RC522_REG_COMMAND, dev->command)!=rc522_ok){
		return write_fail;
	}
	if(dev->command==RC522_CMD_TRANSCEIVE){
		set_bit_mask(dev, RC522_REG_BIT_FRAMING, 0x80);
	}
	return rc522_ok;
}

rc522_return_status_t static card_answer(rc522_handle dev, uint8_t* rxdata, uint32_t* rxlen){
	if(dev->command==RC522_CMD_TRANSCEIVE){
		clear_bit_mask(dev, RC522_REG_BIT_FRAMING, 0x80);
	}
	uint8_t irq_status = read_register(dev, RC522_REG_COM_IRQ);
	if(irq_status&0x01){
		return rc522_timeout;
	}
	uint8_t error_status = read_register(dev, RC522_REG_ERROR);
	if(error_status&0x1B){
		return rc522_fail;
	}
	if(irq_status&0x01&dev->irq_en){
		return rc522_fail;
	}
	// fifoyu silmiyorum çünkü fifo'yu zaten her card_command başında sıfırlıyoruz
	if(dev->command==RC522_CMD_TRANSCEIVE){
		uint8_t lastbits = read_register(dev, RC522_REG_CONTROL) & 0x07;
		uint8_t byte_len = read_register(dev, RC522_REG_FIFO_LEVEL);
		if(lastbits){
			*rxlen = (byte_len-1)*8 + lastbits;
		}else {
			*rxlen = byte_len*8;
		}
		for(uint8_t i=0; i<byte_len; i++){
			rxdata[i]=read_register(dev, RC522_REG_FIFO_DATA);
		}
	}
	return rc522_ok;
}

rc522_return_status_t rc522_request(rc522_handle dev, uint8_t reqmode){
	write_register(dev, RC522_REG_BIT_FRAMING, 0x07);
	dev->command=RC522_CMD_TRANSCEIVE;
	card_command(dev, &reqmode, 1);
	return rc522_ok;
}

rc522_return_status_t rc522_request_answer(rc522_handle dev, uint8_t* rxdata, uint32_t* rxlen){
	uint8_t status = card_answer(dev, rxdata, rxlen);
    if(*rxlen != 0x10){
        return rc522_fail;
    }
    return status;
}

rc522_return_status_t rc522_anticoll_start(rc522_handle dev){
    uint8_t buffer[2];
    buffer[0] = RC522_PICC_CMD_ANTICOLL; // 0x93
    buffer[1] = 0x20;                    // NVB = 32 (UID için 4 byte + BCC)

    // BitFraming sıfırla
    if(write_register(dev, RC522_REG_BIT_FRAMING, 0x00) != rc522_ok){
        return write_fail;
    }

    // Komutu başlat
    dev->command = RC522_CMD_TRANSCEIVE;
    return card_command(dev, buffer, 2);
}

rc522_return_status_t rc522_anticoll_answer(rc522_handle dev, uint8_t* uid, uint16_t* uid_len){
    rc522_return_status_t status = card_answer(dev, uid, uid_len);
    if(status != rc522_ok){
        return status;
    }

    // UID uzunluğu 40 bit (5 byte) olmalı
    if(*uid_len != 40){
        return rc522_fail;
    }

    // XOR checksum kontrolü
    uint8_t bcc = uid[0] ^ uid[1] ^ uid[2] ^ uid[3];
    if(bcc != uid[4]){
        return rc522_fail;
    }

    return rc522_ok;
}

