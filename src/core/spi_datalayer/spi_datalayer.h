/*
 * spi_datalayer.h
 *
 *  Created on: Nov 10, 2013
 *      Author: Huynh Duc Hau
 */

#ifndef SPI_DATALAYER_H_
#define SPI_DATALAYER_H_

#define SPI_DATALAYER_SYNC1	(0xA5)
#define SPI_DATALAYER_SYNC2	(0x5A)
#define SPI_DATALAYER_ADDRESS (0x01)

#define SPI_DATALAYER_MAX_TRANSFER_ERROR	(0xFFFF)

#define SPI_DATALAYER_PACKET_DATA_SIZE	(64)

#define SPI_DATALAYER_PACKET_SIZE	(sizeof(SPI_DATALAYER_PACKET_T))

typedef enum {
	/* command getstatus is a special command. No delay, result is return immediately.*/
	SPI_CMD_GETSTATUS = 0xA0,
	/* Tell slave that master need to transfer this data packet. */
	SPI_CMD_SENDDATA,
	/* Tell slave to send back data. */
	SPI_CMD_RECVDATA
} SPI_CMD_T;

typedef enum {
	SPI_TRANSFER_STATE_SYNC1,
	SPI_TRANSFER_STATE_SYNC2,
	SPI_TRANSFER_STATE_ADDRESS,
	SPI_TRANSFER_STATE_COMMAND,
	SPI_TRANSFER_STATE_CHECKSUM,
	SPI_TRANSFER_STATE_DATALEN,
	SPI_TRANSFER_STATE_DATA,

} SPI_TRANSFER_STATE_T;

typedef enum {
	SPI_SLAVE_STATUS_LISTENING = 0,
	SPI_SLAVE_STATUS_RX_BUFFER_FULL = 1,
	SPI_SLAVE_STATUS_TX_BUFFER_EMPTY = 2
} SPI_SLAVE_STATUS_T;

#define SPI_PACKET_STATUS_MASK						(0x50)
#define SPI_PACKET_STATUS_UNDEFCOMMAND				(2)
#define SPI_PACKET_STATUS_CHECKSUM_ERROR			(4)

typedef struct {
	volatile uint8_t status;
	volatile uint32_t datalen;
} __attribute__((packed())) SPI_STATUS_RESPONSE_T;

typedef struct {
	uint8_t	sync1;
	uint8_t sync2;
	uint8_t address;
	uint8_t cmd;
	uint8_t checksum;
	uint8_t	datalen;
	uint8_t data[SPI_DATALAYER_PACKET_DATA_SIZE];
} __attribute__((packed())) SPI_DATALAYER_PACKET_T;

/**************** Master API *******************/
void    spi_DL_master_init(void);
uint8_t spi_DL_master_getstatus(uint8_t address);
uint8_t spi_DL_master_get_slave_bufferlen(uint8_t address);
void    spi_DL_master_send(uint8_t address, uint8_t const * buf, uint8_t len);
void    spi_DL_master_recv(uint8_t address, uint8_t const * buf, uint8_t len);

/**************** Slave API *******************/
void     spi_DL_slave_init(void);
void     spi_DL_slave_read_buffer(uint8_t* buffer, uint32_t* len);
void     spi_DL_slave_write_buffer(uint8_t* buffer, uint32_t len);
uint8_t  spi_DL_slave_get_status(void);
uint32_t spi_DL_slave_get_rx_buffer_len(void);
void     spi_DL_slave_set_status(uint8_t status, uint8_t datalen);
uint8_t  spi_DL_slave_state_machine(uint8_t SSP_data);
#endif /* SPI_DATALAYER_H_ */
