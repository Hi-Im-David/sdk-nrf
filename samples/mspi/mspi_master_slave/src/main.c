/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/mspi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>


LOG_MODULE_REGISTER(mspi_slave, LOG_LEVEL_DBG);


#define MSPI_PERIPHERAL_NODE 	DT_NODELABEL(peripheral)
#define MSPI_CONTROLLER_NODE 	DT_NODELABEL(controller)


#define DATA_LINES_MAX 4

#define SCK_FREQUENCY MHZ(1)

#define CMD_LEN_MAX 2
#define ADDR_LEN_MAX 4
/* Make sure it is divisible by 4 */
#define DATA_LEN_MAX 52

#define PRINT_RAW_DATA 0

static uint8_t packet_buf[DATA_LEN_MAX];
static uint8_t rx_buff[DATA_LEN_MAX];

static const struct device *mspi_peripheral_dev 	= 	DEVICE_DT_GET(MSPI_PERIPHERAL_NODE);
static const struct device *mspi_controller_dev 	= 	DEVICE_DT_GET(MSPI_CONTROLLER_NODE);

static const struct mspi_dev_id mspi_id_tx = {
	.dev_idx = 0,
};

static const struct mspi_dev_id mspi_id_rx = {
	.dev_idx = 0,
};

void setup_buffer(void)
{

	for (int i = 0; i < DATA_LEN_MAX; ++i) {
		packet_buf[i] = (uint8_t)i;
	}
	return;
}

void print_rx_buff(uint8_t * input_buff) {
    for (size_t i = 0; i < DATA_LEN_MAX; i++) {

        printk("returned buffer [%u] = %u\n", i, input_buff[i]);
    }
}

void async_cb(struct mspi_callback_context *mspi_cb_ctx)
{
	// volatile struct user_context *usr_ctx = mspi_cb_ctx->ctx;
	printk("reeeeeeeeeeeeee\r\n");

	print_rx_buff(mspi_cb_ctx->mspi_evt.evt_data.packet->data_buf);
}

struct user_context {
	uint32_t status;
	uint32_t total_packets;
};

int main()
{
	setup_buffer();
	if(!device_is_ready(mspi_peripheral_dev))
	{
		LOG_ERR("MSPI device %s is not ready", mspi_peripheral_dev->name);
	}

	if(!device_is_ready(mspi_controller_dev))
	{
		LOG_ERR("MSPI device %s is not ready", mspi_controller_dev->name);
	}

	struct mspi_dev_cfg rx_dev_cfg = {
		.ce_num = 1,
		.freq = SCK_FREQUENCY,
		.io_mode = MSPI_IO_MODE_QUAD_1_1_4,
		.data_rate = MSPI_DATA_RATE_SINGLE,
		.cpp = MSPI_CPP_MODE_0,
		.endian = MSPI_XFER_BIG_ENDIAN,
		.ce_polarity = MSPI_CE_ACTIVE_LOW,
	};
	struct mspi_xfer_packet rx_packet = {
		.dir = MSPI_RX,
		.cmd = 0,
		.address = 0,
		.data_buf = rx_buff,
		.num_bytes = DATA_LEN_MAX,
	};
	struct mspi_xfer rx_xfer = {
		.xfer_mode   = MSPI_PIO,
		.packets     = &rx_packet,
		.num_packet  = 1,
		.timeout     = 1,
		.async		 = true,
	};

	struct mspi_dev_cfg tx_dev_cfg = {
		.ce_num = 1,
		.freq = SCK_FREQUENCY,
		.io_mode = MSPI_IO_MODE_QUAD_1_1_4,
		.data_rate = MSPI_DATA_RATE_SINGLE,
		.cpp = MSPI_CPP_MODE_0,
		.endian = MSPI_XFER_BIG_ENDIAN,
		.ce_polarity = MSPI_CE_ACTIVE_LOW,
	};

	struct mspi_xfer_packet tx_packet = {
		.dir = MSPI_TX,
		.cmd = 0xF,
		.address = 0xF,
		.data_buf = packet_buf,
		.num_bytes = DATA_LEN_MAX,
	};
	struct mspi_xfer tx_xfer = {
		.xfer_mode   = MSPI_PIO,
		.packets     = &tx_packet,
		.num_packet  = 1,
		.timeout     = 10,
		.cmd_length = 1,
		.addr_length = 1,
	};
	uint8_t cmd_lines, addr_lines, data_lines;
	cmd_lines = 1;
	addr_lines = 1;
	data_lines = 4;
	int rc;
	volatile struct user_context read_ctx;

	uint8_t cmd_addr_cycles = (tx_xfer.cmd_length * 8 / cmd_lines)
			+ (tx_xfer.addr_length * 8 / addr_lines);
	tx_xfer.tx_dummy = 8 - (cmd_addr_cycles % 8);

	read_ctx.total_packets  = rx_xfer.num_packet;
	read_ctx.status         = ~0;
	struct mspi_callback_context cb_ctx;
	cb_ctx.ctx = (void *)&read_ctx;


	rc = mspi_dev_config(mspi_peripheral_dev, &mspi_id_rx,
			     MSPI_DEVICE_CONFIG_ALL, &rx_dev_cfg);
	if (rc) {
		printk("Failed to config peripheral\n");
	}
	rc = mspi_dev_config(mspi_controller_dev, &mspi_id_tx,
			     MSPI_DEVICE_CONFIG_ALL, &tx_dev_cfg);
	if (rc) {
		printk("Failed to config controller\n");
	}

	rc = mspi_register_callback(mspi_peripheral_dev, &mspi_id_rx, MSPI_BUS_XFER_COMPLETE,
					(mspi_callback_handler_t)async_cb, &cb_ctx);
	if (rc) {
		printk("Failed to register callback\n");
	}

	while(1)
	{
		printk("Setting Async rx...\r\n");
		rc = mspi_transceive(mspi_peripheral_dev, &mspi_id_rx, &rx_xfer);
		if(rc != 0)
		{
			printk("returned: %d\r\n", rc);
		}


		k_msleep(5);
		printk("- 8-bit command, 24-bit address\n");
		
		rc = mspi_transceive(mspi_controller_dev, &mspi_id_tx, &tx_xfer);
		if(rc != 0)
		{
			printk("returned: %d\r\n", rc);
		}
		k_msleep(5000);
	}

	// while(1)
	// {
	// 	printk("Waiting\r\n");
	// 	k_msleep(1000);
	// }

}
