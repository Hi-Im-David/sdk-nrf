/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/mspi.h>
#include <zephyr/ztest.h>
#include <string.h>

#define MSPI_MASTER_NODE DT_NODELABEL(controller)
#define MSPI_SLAVE_NODE DT_NODELABEL(peripheral)

#define SCK_FREQUENCY MHZ(8)

#define CMD_LEN_MAX 2
#define ADDR_LEN_MAX 4
#define DATA_LEN_MAX 32

#define PRINT_RAW_DATA 0

static uint8_t packet_buf1[DATA_LEN_MAX] __aligned(4);;
static uint8_t packet_buf2[DATA_LEN_MAX] __aligned(4);;

static const struct device *mspi_master = DEVICE_DT_GET(MSPI_MASTER_NODE);
static const struct device *mspi_slave = DEVICE_DT_GET(MSPI_SLAVE_NODE);

static const struct mspi_dev_id tx_id = {
	.dev_idx = 0,
};
static const struct mspi_dev_id rx_id = {
	.dev_idx = 0,
};

struct k_sem async_sem;
static uint8_t rx_buff1[DATA_LEN_MAX + CMD_LEN_MAX + ADDR_LEN_MAX] __aligned(4);;
static uint8_t rx_buff2[DATA_LEN_MAX + CMD_LEN_MAX + ADDR_LEN_MAX] __aligned(4);;
static void *setup(void);

void print_rx_buff(uint8_t * input_buff) {
    for (size_t i = 0; i < DATA_LEN_MAX + CMD_LEN_MAX + ADDR_LEN_MAX; i++) {

        TC_PRINT("returned buffer [%u] = 0x%2x\n", i, input_buff[i]);
    }
}

static void mspi_slave_callback(struct mspi_callback_context *mspi_cb_ctx)
{
	ARG_UNUSED(mspi_cb_ctx);
	k_sem_give(&async_sem);
}


static void configure_devices(enum mspi_io_mode io_mode)
{
	struct mspi_dev_cfg master_cfg = {
		.ce_num = 1,
		.freq = SCK_FREQUENCY,
		.io_mode = io_mode,
		.data_rate = MSPI_DATA_RATE_SINGLE,
		.cpp = MSPI_CPP_MODE_0,
		.endian = MSPI_XFER_BIG_ENDIAN,
		.ce_polarity = MSPI_CE_ACTIVE_LOW,
	};

	struct mspi_dev_cfg slave_cfg = {
		.ce_num = 1,
		.freq = SCK_FREQUENCY,
		.io_mode = io_mode,
		.data_rate = MSPI_DATA_RATE_SINGLE,
		.cpp = MSPI_CPP_MODE_0,
		.endian = MSPI_XFER_BIG_ENDIAN,
		.ce_polarity = MSPI_CE_ACTIVE_LOW,
	};

	int rc;

	rc = mspi_dev_config(mspi_master, &tx_id,
			     MSPI_DEVICE_CONFIG_ALL, &master_cfg);
	zassert_false(rc < 0, "mspi_dev_config() master failed: %d", rc);

	rc = mspi_dev_config(mspi_slave, &rx_id,
			     MSPI_DEVICE_CONFIG_ALL, &slave_cfg);
	zassert_false(rc < 0, "mspi_dev_config() slave failed: %d", rc);

	static struct mspi_callback_context cb_ctx;

	rc = mspi_register_callback(mspi_slave, &rx_id, MSPI_BUS_XFER_COMPLETE,
				    (mspi_callback_handler_t)mspi_slave_callback, &cb_ctx);
	zassert_false(rc < 0, "mspi_register_callback() failed: %d", rc);
}

static void test_tx_transfer_multi_packet(struct mspi_xfer *tx_xfer, struct mspi_xfer *rx_xfer)
{
	uint32_t rx_idx;
	int rc;

	/* Clean up RX buffers */
	for (int p = 0; p < rx_xfer->num_packet; ++p) {
		memset(rx_xfer->packets[p].data_buf, 0xAA,
		       rx_xfer->packets[p].num_bytes);
	}

	k_sem_reset(&async_sem);

	/* Start slave transfer in async mode */
	rc = mspi_transceive(mspi_slave, &rx_id, rx_xfer);
	zassert_false(rc < 0, "mspi_transceive() slave failed: %d", rc);

	/* Small delay to ensure slave is ready */
	k_msleep(10);

	/* Start master transfer */
	rc = mspi_transceive(mspi_master, &tx_id, tx_xfer);
	zassert_false(rc < 0, "mspi_transceive() master failed: %d", rc);

	/* Wait for slave transfer to complete */
	rc = k_sem_take(&async_sem, K_MSEC(500));
	zassert_false(rc < 0, "slave transfer timeout");

	/* Verify each packet */
	for (int p = 0; p < tx_xfer->num_packet && p < rx_xfer->num_packet; ++p) {
		const struct mspi_xfer_packet *tx_packet = &tx_xfer->packets[p];
		const struct mspi_xfer_packet *rx_packet = &rx_xfer->packets[p];
		rx_idx = 0;

		/* Verify command */
		for (int i = 0; i < tx_xfer->cmd_length; ++i) {
			uint8_t shift = (tx_xfer->cmd_length - 1 - i) * 8;
			uint8_t expected = (tx_packet->cmd >> shift) & 0xff;
			uint8_t actual = rx_packet->data_buf[rx_idx++];

			if (PRINT_RAW_DATA) {
				TC_PRINT("packet %d command at index %d: 0x%02X : 0x%02X\n",
				       p, i, actual, expected);
			}

			zassert_equal(actual, expected,
				      "packet %d command mismatch at index %d: 0x%02X != 0x%02X",
				      p, i, actual, expected);
		}

		/* Verify address */
		for (int i = 0; i < tx_xfer->addr_length; ++i) {
			uint8_t shift = (tx_xfer->addr_length - 1 - i) * 8;
			uint8_t expected = (tx_packet->address >> shift) & 0xff;
			uint8_t actual = rx_packet->data_buf[rx_idx++];

			if (PRINT_RAW_DATA) {
				TC_PRINT("packet %d address at index %d: 0x%02X : 0x%02X\n",
				       p, i, actual, expected);
			}

			zassert_equal(actual, expected,
				      "packet %d address mismatch at index %d: 0x%02X != 0x%02X",
				      p, i, actual, expected);
		}

		/* Verify data */
		for (int i = 0; i < tx_packet->num_bytes; ++i) {
			uint8_t expected = tx_packet->data_buf[i];
			uint8_t actual = rx_packet->data_buf[rx_idx++];

			if (PRINT_RAW_DATA) {
				TC_PRINT("packet %d data at index %d: 0x%02X : 0x%02X\n",
				       p, i, actual, expected);
			}

			zassert_equal(actual, expected,
				      "packet %d data mismatch at index %d: 0x%02X != 0x%02X",
				      p, i, actual, expected);
		}
	}
}

static void test_tx_transfers(uint32_t transfer_length)
{
	configure_devices(MSPI_IO_MODE_QUAD);

	struct mspi_xfer_packet tx_packet1 = {
		.dir = MSPI_TX,
		.cmd = 0x1234,
		.address = 0x98765432,
		.data_buf = packet_buf1,
	};

	struct mspi_xfer_packet tx_packet2 = {
		.dir = MSPI_TX,
		.cmd = 0xABCD,
		.address = 0x12345678,
		.data_buf = packet_buf2,
	};

	struct mspi_xfer_packet tx_packets[] = {tx_packet1, tx_packet2};

	struct mspi_xfer_packet rx_packet1 = {
		.dir = MSPI_RX,
		.cmd = 0,
		.address = 0,
		.data_buf = rx_buff1,
		.cb_mask = MSPI_BUS_XFER_COMPLETE_CB,
	};

	struct mspi_xfer_packet rx_packet2 = {
		.dir = MSPI_RX,
		.cmd = 0,
		.address = 0,
		.data_buf = rx_buff2,
		.cb_mask = MSPI_BUS_XFER_COMPLETE_CB,
	};

	struct mspi_xfer_packet rx_packets[] = {rx_packet1, rx_packet2};

		struct mspi_xfer tx_xfer = {
#if defined(CONFIG_MSPI_DMA)
		.xfer_mode = MSPI_DMA,
#else
		.xfer_mode = MSPI_PIO,
#endif
		.packets = tx_packets,
		.num_packet = 2,
		.timeout = 1000,
	};

	struct mspi_xfer rx_xfer = {
#if defined(CONFIG_MSPI_DMA)
		.xfer_mode = MSPI_DMA,
#else
		.xfer_mode = MSPI_PIO,
#endif
		.packets = rx_packets,
		.num_packet = 2,
		.timeout = 1000,
		.async = true,
	};

	if(transfer_length == 0) {
		TC_PRINT("- 8-bit command only\n");
		tx_xfer.cmd_length = 1;
		tx_xfer.addr_length = 0;
		tx_packets[0].num_bytes = 0;
		tx_packets[1].num_bytes = 0;
		rx_packets[0].num_bytes = tx_xfer.cmd_length + tx_xfer.addr_length + tx_packets[0].num_bytes;
		rx_packets[1].num_bytes = tx_xfer.cmd_length + tx_xfer.addr_length + tx_packets[1].num_bytes;
		test_tx_transfer_multi_packet(&tx_xfer, &rx_xfer);

		TC_PRINT("- 16-bit command only\n");
		tx_xfer.cmd_length = 2;
		tx_xfer.addr_length = 0;
		tx_packets[0].num_bytes = 0;
		tx_packets[1].num_bytes = 0;
		rx_packets[0].num_bytes = tx_xfer.cmd_length + tx_xfer.addr_length + tx_packets[0].num_bytes;
		rx_packets[1].num_bytes = tx_xfer.cmd_length + tx_xfer.addr_length + tx_packets[1].num_bytes;
		test_tx_transfer_multi_packet(&tx_xfer, &rx_xfer);

		TC_PRINT("- 8-bit command and 24-bit address only\n");
		tx_xfer.cmd_length = 1;
		tx_xfer.addr_length = 3;
		tx_packets[0].num_bytes = 0;
		tx_packets[1].num_bytes = 0;
		rx_packets[0].num_bytes = tx_xfer.cmd_length + tx_xfer.addr_length + tx_packets[0].num_bytes;
		rx_packets[1].num_bytes = tx_xfer.cmd_length + tx_xfer.addr_length + tx_packets[1].num_bytes;
		test_tx_transfer_multi_packet(&tx_xfer, &rx_xfer);

	} else {
		TC_PRINT("- 8-bit command, 24-bit address\n");
		tx_xfer.cmd_length = 1;
		tx_xfer.addr_length = 3;
		tx_packets[0].num_bytes = transfer_length;
		tx_packets[1].num_bytes = transfer_length;
		rx_packets[0].num_bytes = tx_xfer.cmd_length + tx_xfer.addr_length + tx_packets[0].num_bytes;
		rx_packets[1].num_bytes = tx_xfer.cmd_length + tx_xfer.addr_length + tx_packets[1].num_bytes;
		test_tx_transfer_multi_packet(&tx_xfer, &rx_xfer);

		TC_PRINT("- 8-bit command, 32-bit address\n");
		tx_xfer.cmd_length = 1;
		tx_xfer.addr_length = 4;
		tx_packets[0].num_bytes = transfer_length;
		tx_packets[1].num_bytes = transfer_length;
		rx_packets[0].num_bytes = tx_xfer.cmd_length + tx_xfer.addr_length + tx_packets[0].num_bytes;
		rx_packets[1].num_bytes = tx_xfer.cmd_length + tx_xfer.addr_length + tx_packets[1].num_bytes;
		test_tx_transfer_multi_packet(&tx_xfer, &rx_xfer);

		TC_PRINT("- 16-bit command, 24-bit address\n");
		tx_xfer.cmd_length = 2;
		tx_xfer.addr_length = 3;
		tx_packets[0].num_bytes = transfer_length;
		tx_packets[1].num_bytes = transfer_length;
		rx_packets[0].num_bytes = tx_xfer.cmd_length + tx_xfer.addr_length + tx_packets[0].num_bytes;
		rx_packets[1].num_bytes = tx_xfer.cmd_length + tx_xfer.addr_length + tx_packets[1].num_bytes;
		test_tx_transfer_multi_packet(&tx_xfer, &rx_xfer);

		TC_PRINT("- 16-bit command, 32-bit address\n");
		tx_xfer.cmd_length = 2;
		tx_xfer.addr_length = 4;
		tx_packets[0].num_bytes = transfer_length;
		tx_packets[1].num_bytes = transfer_length;
		rx_packets[0].num_bytes = tx_xfer.cmd_length + tx_xfer.addr_length + tx_packets[0].num_bytes;
		rx_packets[1].num_bytes = tx_xfer.cmd_length + tx_xfer.addr_length + tx_packets[1].num_bytes;
		test_tx_transfer_multi_packet(&tx_xfer, &rx_xfer);

		TC_PRINT("- Just data \n");
		tx_xfer.cmd_length = 0;
		tx_xfer.addr_length = 0;
		tx_packets[0].num_bytes = transfer_length;
		tx_packets[1].num_bytes = transfer_length;
		rx_packets[0].num_bytes = tx_xfer.cmd_length + tx_xfer.addr_length + tx_packets[0].num_bytes;
		rx_packets[1].num_bytes = tx_xfer.cmd_length + tx_xfer.addr_length + tx_packets[1].num_bytes;
		test_tx_transfer_multi_packet(&tx_xfer, &rx_xfer);
	}
}

/* 32-bit data frame size alignment */
ZTEST(mspi_tx_rx_loopback, data_frame_size_32)
{
	test_tx_transfers(DATA_LEN_MAX);
}

/* 16-bit data frame size alignment */
ZTEST(mspi_tx_rx_loopback, data_frame_size_16)
{
	test_tx_transfers(DATA_LEN_MAX-2);
}

/* 8-bit data frame size alignment */
ZTEST(mspi_tx_rx_loopback, data_frame_size_8)
{
	test_tx_transfers(DATA_LEN_MAX-1);
}

/* Transmitting a small buffer */
ZTEST(mspi_tx_rx_loopback, buffer_small)
{
	test_tx_transfers(4);
}

/* Just command/address */
ZTEST(mspi_tx_rx_loopback, just_command_and_address)
{
	test_tx_transfers(0);
}

static void *setup(void)
{
	k_sem_init(&async_sem, 0, 1);

	#if defined(CONFIG_MSPI_DMA)
		TC_PRINT("Using MSPI peripheral in DMA mode\n");
	#else
		TC_PRINT("Using MSPI peripheral in PIO (FIFO) mode\n");
	#endif

	for (int i = 0; i < DATA_LEN_MAX; ++i) {
		packet_buf1[i] = (uint8_t)i;
		packet_buf2[i] = (uint8_t)(0xFF - i);
	}

	return NULL;
}

static void before(void *fixture)
{
	ARG_UNUSED(fixture);

	zassert_true(device_is_ready(mspi_master),
		"MSPI master device %s is not ready", mspi_master->name);

	zassert_true(device_is_ready(mspi_slave),
		"MSPI slave device %s is not ready", mspi_slave->name);
}

ZTEST_SUITE(mspi_tx_rx_loopback, NULL, setup, before, NULL, NULL);
