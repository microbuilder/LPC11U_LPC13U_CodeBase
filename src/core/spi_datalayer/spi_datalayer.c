/*
 * spi_datalayer.c
 *
 *  Created on: Nov 11, 2013
 *      Author: Huynh Duc Hau
 */
#include "projectconfig.h"
#include "core/gpio/gpio.h"
#include "core/ssp0_slave/ssp0_slave.h"
#include "core/ssp0/ssp0.h"
#include "core/spi_datalayer/spi_datalayer.h"
#include <string.h>

/**************************************************************************
*  ---------------SPI DATA LAYER MASTER FUNCTION--------------------------
*
**************************************************************************/

SPI_DATALAYER_PACKET_T spi_DL_packet = {
		SPI_DATALAYER_SYNC1,
		SPI_DATALAYER_SYNC2,
		0,
};

SPI_STATUS_RESPONSE_T spi_DL_master_status;
uint8_t temp_spi_rx_buf[SPI_DATALAYER_PACKET_SIZE];

/* This variable is used to measure number of frame is delayed on spi bus. */
static uint8_t packet_delay = 0;

/**************************************************************************/
/*!
    @brief Master operation: reset bus.
*/
/**************************************************************************/
void spi_DL_master_resetbus(void)
{
	memset(temp_spi_rx_buf, SPI_DATALAYER_SYNC1, SPI_DATALAYER_PACKET_SIZE);
	ssp0Send(temp_spi_rx_buf, SPI_DATALAYER_PACKET_SIZE);
}

/**************************************************************************/
/*!
    @brief Bus error handler.
*/
/**************************************************************************/
/* spi datalink bus error handler. */
void spi_DL_master_bus_error_handler(void)
{
	/* if slave device delay time is too high
	 * or other reason. Stop it. */
	while(1);
}
/**************************************************************************/
/*!
    @brief Master operation: Find number of delayed frame issued by slave.

    @param[in]  None
    @return     Number of frame is delayed from slave.
*/
/**************************************************************************/
uint8_t spi_DL_master_findrecvdelay(void)
{
	uint32_t i;
	for(i = 0; i < SPI_DATALAYER_PACKET_SIZE - 1; i++)
	{
		if((temp_spi_rx_buf[i] == spi_DL_packet.address) &&
				(temp_spi_rx_buf[i + 1] == spi_DL_packet.cmd))
		{
			break;
		}
	}
	return i - 2;
}

void spi_DL_master_send_packet(uint8_t address, uint8_t cmd, uint32_t len)
{
	uint32_t transfer_error_count;
	//uint8_t spi_datalink_packet_delay = 0;
	spi_DL_packet.address = address;
	spi_DL_packet.cmd = cmd;
	spi_DL_packet.checksum = -(address + cmd);
	spi_DL_packet.datalen = len;
	transfer_error_count = SPI_DATALAYER_MAX_TRANSFER_ERROR;
	do
	{
		transfer_error_count--;
		if(transfer_error_count == 0)
		{
			transfer_error_count = SPI_DATALAYER_MAX_TRANSFER_ERROR;
			spi_DL_master_resetbus();
		}
		ssp0Transfer(temp_spi_rx_buf, (uint8_t*)&spi_DL_packet, sizeof(SPI_DATALAYER_PACKET_T));
		if(packet_delay == 0)
		{
			packet_delay = spi_DL_master_findrecvdelay();
			/* if slave has longer delay time, never play with it. */
			if(packet_delay >= (SPI_DATALAYER_PACKET_SIZE - sizeof(SPI_STATUS_RESPONSE_T)))
				spi_DL_master_bus_error_handler();
		}
	}while(temp_spi_rx_buf[packet_delay + 4] != SPI_PACKET_STATUS_MASK);
}
/**************************************************************************/
/*!
    @brief Get slave status.

    @param[in]  address
    			device address.
    @return     status.
*/
/**************************************************************************/
uint8_t spi_DL_master_getstatus(uint8_t address)
{
	spi_DL_master_send_packet(address, SPI_CMD_GETSTATUS, sizeof(SPI_STATUS_RESPONSE_T));
	return temp_spi_rx_buf[packet_delay + 6];
}

/**************************************************************************/
/*!
    @brief Get slave TX buffer length.

    @param[in]  address
    			device address.
    @return     slave TX buffer length.
*/
/**************************************************************************/
uint8_t spi_DL_master_get_slave_bufferlen(uint8_t address)
{
	spi_DL_master_send_packet(address, SPI_CMD_GETSTATUS, sizeof(SPI_STATUS_RESPONSE_T));
	return *(uint32_t*)&temp_spi_rx_buf[packet_delay + 7];
}

/**************************************************************************/
/*!
    @brief Send a packet data.

    @param[in]  address
    			device address
    @param[in]  buf
    			address of data buffer needs to be send.
	@param[in]  len
    			length of data buffer.
    @return     None.
*/
/**************************************************************************/
void spi_DL_master_send_data(uint8_t address, uint8_t* buf, uint8_t len)
{

	memcpy(spi_DL_packet.data, buf, len);
	spi_DL_master_send_packet(address, SPI_CMD_SENDDATA, len);
}

/**************************************************************************/
/*!
    @brief Receive a packet data.

    @param[in]  address
    			device address
    @param[in]  buf
    			address of data buffer needs to be receive.
	@param[in]  len
    			length of data buffer.
    @return     None.
*/
/**************************************************************************/
void spi_DL_master_recv_data(uint8_t address, uint8_t len)
{
	memset(spi_DL_packet.data, 0x00, SPI_DATALAYER_PACKET_DATA_SIZE);
	spi_DL_master_send_packet(address, SPI_CMD_RECVDATA, len);
}

/**************************************************************************/
/*!
    @brief Send data. This function will divide data into blocks. Then transmit
    		it using packet send function.

    @param[in]  address
    			device address
    @param[in]  buf
    			address of data buffer needs to be send.
	@param[in]  len
    			length of data buffer.
    @return     None.
*/
/**************************************************************************/
void spi_DL_master_send(uint8_t address, uint8_t const * buf, uint8_t len)
{
	uint8_t* temp_buf = (uint8_t*)buf;
	uint32_t block_num;
	uint32_t block_remainlen;
	/* divide input data into block of 32 bytes. */
	block_num = len/SPI_DATALAYER_PACKET_DATA_SIZE;
	block_remainlen = len % SPI_DATALAYER_PACKET_DATA_SIZE;
	packet_delay = 0;

	for(int i = 0; i < block_num; i++)
	{
		/* wait until slave rx buffer empty. */
		while(spi_DL_master_getstatus(address) & SPI_SLAVE_STATUS_RX_BUFFER_FULL);
		/* send a block data to slave. */
		spi_DL_master_send_data(address, temp_buf, SPI_DATALAYER_PACKET_DATA_SIZE);
		temp_buf += SPI_DATALAYER_PACKET_DATA_SIZE;
	}

	/* send last block. */
	if(block_remainlen)
		spi_DL_master_send_data(address, temp_buf, block_remainlen);
}

/**************************************************************************/
/*!
    @brief Receive data. This function will divide data into blocks. Then receive
    		it using packet receive function.

    @param[in]  address
    			device address
    @param[in]  buf
    			address of data buffer needs to be send.
	@param[in]  len
    			length of data buffer.
    @return     None.
*/
/**************************************************************************/
void spi_DL_master_recv(uint8_t address, uint8_t const * buf, uint8_t len)
{
	uint8_t * temp_buf = (uint8_t *)buf;
	uint32_t block_num;
	uint32_t block_remainlen;
	uint8_t remain_block_data_len;
	/* divide input data into block of 32 bytes. */
	block_num = len/SPI_DATALAYER_PACKET_DATA_SIZE;
	block_remainlen = len % SPI_DATALAYER_PACKET_DATA_SIZE;
	packet_delay = 0;

	/* Read first block. This is special block. */
	if(block_num)
	{
		while(spi_DL_master_getstatus(address) & SPI_SLAVE_STATUS_TX_BUFFER_EMPTY);

		spi_DL_master_recv_data(address, SPI_DATALAYER_PACKET_DATA_SIZE);
		memcpy(temp_buf, &temp_spi_rx_buf[packet_delay + 6], SPI_DATALAYER_PACKET_DATA_SIZE - packet_delay);
		temp_buf += SPI_DATALAYER_PACKET_DATA_SIZE;
	}
	/* Read remain blocks. */
	for(int i = 1; i < block_num; i++)
	{
		while(spi_DL_master_getstatus(address) & SPI_SLAVE_STATUS_TX_BUFFER_EMPTY);

		spi_DL_master_recv_data(address, SPI_DATALAYER_PACKET_DATA_SIZE);
		/* copy data from buffer to output. */
		memcpy(temp_buf - packet_delay, temp_spi_rx_buf, packet_delay);
		memcpy(temp_buf, &temp_spi_rx_buf[packet_delay + 6], SPI_DATALAYER_PACKET_DATA_SIZE - packet_delay);
		temp_buf += SPI_DATALAYER_PACKET_DATA_SIZE;
	}
	remain_block_data_len = packet_delay;

	/* read last block. */
	if(block_remainlen)
	{
		spi_DL_master_recv_data(address, block_remainlen);
		memcpy(temp_buf - packet_delay, temp_spi_rx_buf, packet_delay);
		memcpy(temp_buf, &temp_spi_rx_buf[packet_delay + 6], SPI_DATALAYER_PACKET_DATA_SIZE - packet_delay);
		if(packet_delay + block_remainlen >= SPI_DATALAYER_PACKET_DATA_SIZE)
		{
			remain_block_data_len = packet_delay + block_remainlen + 1 - SPI_DATALAYER_PACKET_DATA_SIZE;
			temp_buf += block_remainlen;
		}
	}
	if(remain_block_data_len)
	{
		spi_DL_packet.address = address;
		spi_DL_packet.cmd = SPI_CMD_GETSTATUS;
		spi_DL_packet.checksum = -(address + SPI_CMD_GETSTATUS);
		spi_DL_packet.datalen = sizeof(SPI_STATUS_RESPONSE_T);
		ssp0Transfer(temp_spi_rx_buf, (uint8_t*)&spi_DL_packet, sizeof(SPI_DATALAYER_PACKET_T));
		memcpy(temp_buf - remain_block_data_len, temp_spi_rx_buf, remain_block_data_len);
	}

}

/**************************************************************************/
/*!
    @brief Initialize master spi DL.

    @param[in]  None
    @return     None.
*/
/**************************************************************************/
void spi_DL_master_init(void)
{
	ssp0Init();

  /* Use pin P0_2 for SSEL. */
  LPC_IOCON->PIO0_2 &= ~0x07;
  LPC_IOCON->PIO0_2 |= 0x01;

  spi_DL_master_status.status = 0;
  spi_DL_master_status.datalen = 0;
  packet_delay = 0;
}
/**************************************************************************
*  ---------------SPI DATA LAYER SLAVE FUNCTION--------------------------
*
**************************************************************************/
static SPI_STATUS_RESPONSE_T spi_DL_slave_status;

static volatile uint32_t spi_DL_slave_rx_buffer_len = 0;
static volatile uint32_t spi_DL_slave_tx_buffer_len = 0;

static uint8_t spi_DL_slave_rx_buffer[SPI_DATALAYER_PACKET_DATA_SIZE];
static uint8_t spi_DL_slave_tx_buffer[SPI_DATALAYER_PACKET_DATA_SIZE];

static SPI_TRANSFER_STATE_T spi_DL_slave_transferstate = SPI_TRANSFER_STATE_SYNC1;

/**************************************************************************/
/*!
    @brief This handler is call when slave has transfered data.

    @param[in]  None
    @return     None.
*/
/**************************************************************************/
void spi_DL_slave_data_available(SPI_CMD_T cmd)
{
}

/**************************************************************************/
/*!
    @brief Disable SPI slave output.

    @param[in]  None
    @return     None.
*/
/**************************************************************************/
void spi_DL_slave_output_disable(void)
{
	//LPC_SSP0->CR1 |= (1 << 3);
}

/**************************************************************************/
/*!
    @brief Enable SPI slave output.

    @param[in]  None
    @return     None.
*/
/**************************************************************************/
void spi_DL_slave_output_enable(void)
{
	//LPC_SSP0->CR1 &= ~(1 << 3);
}

/**************************************************************************/
/*!
    @brief Initialize slave spi DL.

    @param[in]  None
    @return     None.
*/
/**************************************************************************/
void spi_DL_slave_init(void)
{
	ssp0_slaveInit();
	ssp0_slave_enable_RXIRQ();
	LPC_IOCON->PIO0_2 &= ~0x07;
	LPC_IOCON->PIO0_2 |= 0x01;
	spi_DL_slave_status.status = SPI_SLAVE_STATUS_TX_BUFFER_EMPTY;
	spi_DL_slave_status.datalen = 0;
	spi_DL_slave_rx_buffer_len = 0;
	spi_DL_slave_tx_buffer_len = 0;
	spi_DL_slave_transferstate = SPI_TRANSFER_STATE_SYNC1;
	spi_DL_slave_output_disable();
}

/**************************************************************************/
/*!
    @brief Read slave rx buffer.

    @param[in]  buffer
    			address of data buffer
    @param[in]  len
    			address of data length variable. Len of rx buffer will be stored.
    @return     None.
*/
/**************************************************************************/
void spi_DL_slave_read_buffer(uint8_t* buffer, uint32_t* len)
{
	memcpy(buffer, spi_DL_slave_rx_buffer, spi_DL_slave_rx_buffer_len);
	*len = spi_DL_slave_rx_buffer_len;
	spi_DL_slave_status.status &= ~SPI_SLAVE_STATUS_RX_BUFFER_FULL;
}

/**************************************************************************/
/*!
    @brief Write slave tx buffer.

    @param[in]  buffer
    			address of data buffer
    @param[in]  len
    			buffer len. Must be < SPI_DATALAYER_PACKET_DATA_SIZE.
    @return     None.
*/
/**************************************************************************/
uint32_t writebuffercount = 0;
void spi_DL_slave_write_buffer(uint8_t* buffer, uint32_t len)
{
	writebuffercount++;
	memcpy(spi_DL_slave_tx_buffer, buffer, len);
	spi_DL_slave_status.datalen = len;
	//spi_DL_slave_tx_buffer_len = len;
	spi_DL_slave_status.status &= ~SPI_SLAVE_STATUS_TX_BUFFER_EMPTY;
}

uint8_t spi_DL_slave_get_status(void)
{
	return spi_DL_slave_status.status;
}

uint32_t spi_DL_slave_get_rx_buffer_len(void)
{
	return spi_DL_slave_rx_buffer_len;
}

void spi_DL_slave_set_status(uint8_t status, uint8_t datalen)
{
	spi_DL_slave_status.status = status;
	spi_DL_slave_status.datalen = datalen;
}

void spi_DL_restart_transfer(void)
{
	spi_DL_slave_transferstate = SPI_TRANSFER_STATE_SYNC1;
}

void spi_DL_reset_bus(void)
{
	spi_DL_slave_transferstate = SPI_TRANSFER_STATE_SYNC2;
	spi_DL_slave_set_status(0,0);
}

static volatile uint8_t ignore_packet;
static volatile uint8_t packet_error;
static uint8_t packet_datalen;
static volatile uint8_t packet_checksum;
static uint8_t databuffer_index;
static uint8_t* databuffer;
static SPI_CMD_T spi_DL_cmd;

/**************************************************************************/
/*!
    @brief spi DL slave state machine.

    @param[in]  SSP_data
    			input data.
    @return     Output data to Master.
*/
/**************************************************************************/
uint8_t spi_DL_slave_state_machine(uint8_t SSP_data)
{

	uint32_t SSP_retdata = SSP_data; /* echo back data. */

	switch(spi_DL_slave_transferstate)
	{
		case SPI_TRANSFER_STATE_SYNC1:
			if(SSP_data == SPI_DATALAYER_SYNC1)
				spi_DL_slave_transferstate++;
			break;
		case SPI_TRANSFER_STATE_SYNC2:
			if(SSP_data == SPI_DATALAYER_SYNC2)
			{
				/* sync recognized, reset packet variables. */
				packet_checksum = 0;
				packet_error = 0;
				ignore_packet = 0;
				databuffer = NULL;
				databuffer_index = 0;
				spi_DL_cmd = 0;
				spi_DL_slave_transferstate++;
			}else if(SSP_data == SPI_DATALAYER_SYNC1)
			{
				/* not change the state. But recognize this is bus reset. */
				//spi_DL_slave_output_disable();
				spi_DL_slave_set_status(0,0);
			}else
			{
				/* wrong data, restart parser. */
				spi_DL_slave_transferstate = SPI_TRANSFER_STATE_SYNC1;
			}

			break;
		case SPI_TRANSFER_STATE_ADDRESS:
			if(SSP_data == SPI_DATALAYER_ADDRESS)
			{
				/* start checksum from this state. */
				packet_checksum += SSP_data;
				/* device is addressed. Enable slave output. */
				//spi_DL_slave_output_enable();
				spi_DL_slave_transferstate++;
			}else
			{
				ignore_packet = 1;
			}
		break;

		case SPI_TRANSFER_STATE_COMMAND:
			spi_DL_slave_transferstate++;

			/* if wrong address, ignore remain data from here. */
			if(ignore_packet)
				break;

			packet_checksum += SSP_data;
			if((SSP_data >= SPI_CMD_GETSTATUS) && (SSP_data <= SPI_CMD_RECVDATA))
			{
				spi_DL_cmd = SSP_data;
				if(spi_DL_cmd == SPI_CMD_GETSTATUS) databuffer = (uint8_t*)&spi_DL_slave_status;
			}else
			{
				/* wrong command. */
				packet_error = SPI_PACKET_STATUS_UNDEFCOMMAND;
			}
			break;
		case SPI_TRANSFER_STATE_CHECKSUM:
			spi_DL_slave_transferstate++;

			if(ignore_packet)
				break;

			packet_checksum += SSP_data;
			if(packet_checksum)
			{
				/* command checksum error. */
				packet_error |= SPI_PACKET_STATUS_CHECKSUM_ERROR;
			}
			SSP_retdata = SPI_PACKET_STATUS_MASK | packet_error;
			/* ignore packet when undefined command or checksum error.*/
			if(packet_error)
				ignore_packet = 1;

			break;
		case SPI_TRANSFER_STATE_DATALEN:
			spi_DL_slave_transferstate++;
			if(ignore_packet)
				break;

			packet_datalen = SSP_data;
			databuffer_index = 0;
			break;
		case SPI_TRANSFER_STATE_DATA:
			if(ignore_packet == 0)
			{
				switch(spi_DL_cmd)
				{
					case SPI_CMD_GETSTATUS:
						if(databuffer_index < sizeof(SPI_STATUS_RESPONSE_T))
						{
							SSP_retdata = databuffer[databuffer_index];
						}
						break;
					case SPI_CMD_SENDDATA:
						spi_DL_slave_rx_buffer[databuffer_index] = SSP_data;
						break;
					case SPI_CMD_RECVDATA:
						SSP_retdata = spi_DL_slave_tx_buffer[databuffer_index];
					default:
						break;
				}
			}
			databuffer_index++;
			/* reach end of transfer. restart it. */
			if(databuffer_index >= SPI_DATALAYER_PACKET_DATA_SIZE)
			{
				if(ignore_packet == 0)
				{
					/* a good packet has been received. It is time to process.
					 * but remind to set status to busy to inform master
					 * that slave is processing data. */
					if(spi_DL_cmd == SPI_CMD_SENDDATA)
					{
						spi_DL_slave_rx_buffer_len = packet_datalen;
						spi_DL_slave_status.status |= SPI_SLAVE_STATUS_RX_BUFFER_FULL;
						spi_DL_slave_data_available(spi_DL_cmd);
					}
					else if(spi_DL_cmd == SPI_CMD_RECVDATA)
					{
						spi_DL_slave_status.status |= SPI_SLAVE_STATUS_TX_BUFFER_EMPTY;
						spi_DL_slave_status.datalen = 0;
						spi_DL_slave_data_available(spi_DL_cmd);
					}
				}
				/* disable slave output until next packet addressed. */
				//spi_DL_slave_output_disable();
				spi_DL_restart_transfer();
			}
			break;
		default:
			break;
	  }

	return SSP_retdata;
}
