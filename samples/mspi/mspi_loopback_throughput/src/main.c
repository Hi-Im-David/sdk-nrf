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
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <math.h>

LOG_MODULE_REGISTER(mspi_throughput, LOG_LEVEL_INF);

/* Test Configuration */
#define MSPI_MASTER_NODE    DT_NODELABEL(controller)
#define MSPI_SLAVE_NODE     DT_NODELABEL(peripheral)

#define QSPI_CLOCK_FREQ     MHZ(64)  /* should be 8MHz as specified for FPGA */
#define MBYTE        (1024U * 1024U)   /* bytes per MiB */
#define TOTAL_TRANSFER_SIZE MBYTE
#define FPGA_CLOCK_OFFSET 4

/* bits per second in quad mode: clk * 4
 *  bytes per second: (clk * 4) / 8  == clk/2
 */
#define IDEAL_BYTES_PER_SEC  ((uint64_t)QSPI_CLOCK_FREQ / 2ULL)

/* ideal MiB/s */
#define IDEAL_THROUGHPUT_MBYTE    ((double)IDEAL_BYTES_PER_SEC / (double)MBYTE)

/* Test Parameters */
#define BUFFER_SIZES_COUNT  7
#define DESCRIPTOR_SIZES_COUNT 3
#define TEST_DIRECTIONS     1

#define MAX_DESCRIPTORS 16

/* In bytes */
#define ADDR_LEN 3
#define CMD_LEN 1

/* Buffer sizes to test (in bytes) */
static const uint32_t buffer_sizes[BUFFER_SIZES_COUNT] = {
    4, 16, 64, 256, 1024, 1500, 2304
};

/* Descriptor counts to test */
static const uint32_t descriptor_counts[DESCRIPTOR_SIZES_COUNT] = {
    1, 4, MAX_DESCRIPTORS
};

/* Test directions */
static const enum mspi_xfer_direction test_directions[TEST_DIRECTIONS] = {
    MSPI_TX, MSPI_RX
};

/* Device references */
static const struct device *mspi_master = DEVICE_DT_GET(MSPI_MASTER_NODE);
static const struct device *mspi_slave = DEVICE_DT_GET(MSPI_SLAVE_NODE);

/* Device IDs */
static const struct mspi_dev_id master_id = { .dev_idx = 0 };
static const struct mspi_dev_id slave_id = { .dev_idx = 0 };

static uint8_t tx_buffer[2304]; /* Largest buffer size */
static uint8_t rx_buffer[2304]; /* Largest buffer size */


/* Performance measurement variables */
static volatile bool transfer_complete = false;
static volatile int transfer_status = 0;
static volatile uint32_t transfer_start_cycles = 0;
static volatile uint32_t transfer_end_cycles = 0;
static volatile uint32_t cpu_cycles = 0;

/* Note: Using k_cycle_get_32() for high-precision timing instead of millisecond timers */
/* CPU cycles provide much higher resolution for accurate throughput measurements */

/* Test results structure - optimized for memory */
struct test_result {
    enum mspi_xfer_direction direction;
    uint16_t buffer_size;
    uint8_t descriptor_count;
    uint32_t transfer_cycles;    /* Store time in CPU cycles for high precision */
    double throughput_mbps;    /* Throughput in MB/s*/
    double cpu_load_mips;     /* CPU load in MIPS */
    double cpu_load_per;      /* CPU load (%) */
    uint16_t cpu_freq_mhz;
    uint16_t efficiency;	/* Ideal transfer vs read (%) * 100 */
};

static struct test_result test_results[TEST_DIRECTIONS * BUFFER_SIZES_COUNT * DESCRIPTOR_SIZES_COUNT];
static uint32_t test_result_index = 0;

/* Callback context */
struct test_callback_context {
    struct k_sem completion_sem;
    int status;
};

static struct test_callback_context callback_ctx;

/* Forward declarations */
static void setup_test_buffers(void);
static int configure_mspi_devices(void);
static int run_single_test(enum mspi_xfer_direction direction, 
                          uint32_t buffer_size, 
                          uint32_t descriptor_count);
// static void transfer_completion_callback(struct mspi_callback_context *ctx, ...);
static void calculate_and_store_results(enum mspi_xfer_direction direction,
                                      uint32_t buffer_size,
                                      uint32_t descriptor_count,
				      uint32_t num_transfers,
                                      uint32_t total_buffers);
static void print_test_results(void);
static void print_summary_report(void);

/* Transfer completion callback */
// static void transfer_completion_callback(struct mspi_callback_context *ctx, ...)
// {
//     // transfer_complete = true;
//     // transfer_status = ctx->mspi_evt.evt_data.status;
//     // k_sem_give(&callback_ctx.completion_sem);
//     // LOG_INF("Received slave");
	
// }

/* Setup test buffers with pattern */
static void setup_test_buffers(void)
{
    for (uint32_t i = 0; i < sizeof(tx_buffer); i++) {
        tx_buffer[i] = (uint8_t)(i);
    }
}

/* Configure MSPI devices for loopback test */
static int configure_mspi_devices(void)
{
    struct mspi_dev_cfg master_cfg = {
        .ce_num = 1,
        .freq = QSPI_CLOCK_FREQ,
        .io_mode = MSPI_IO_MODE_QUAD,
        .data_rate = MSPI_DATA_RATE_SINGLE,
        .cpp = MSPI_CPP_MODE_0,
        .endian = MSPI_XFER_BIG_ENDIAN,
        .ce_polarity = MSPI_CE_ACTIVE_LOW,
        .rx_dummy = 8,
        .tx_dummy = 0,
        .cmd_length = CMD_LEN,
        .addr_length = ADDR_LEN,
    };

    struct mspi_dev_cfg slave_cfg = {
        .ce_num = 1,
        .freq = QSPI_CLOCK_FREQ,
        .io_mode = MSPI_IO_MODE_QUAD,
        .data_rate = MSPI_DATA_RATE_SINGLE,
        .cpp = MSPI_CPP_MODE_0,
        .endian = MSPI_XFER_BIG_ENDIAN,
        .ce_polarity = MSPI_CE_ACTIVE_LOW,
        .rx_dummy = 8,
        .tx_dummy = 0,
        .cmd_length = CMD_LEN,
        .addr_length = ADDR_LEN,
    };

    int rc;

    /* Configure master device */
    rc = mspi_dev_config(mspi_master, &master_id, MSPI_DEVICE_CONFIG_ALL, &master_cfg);
    if (rc != 0) {
        LOG_ERR("Failed to configure MSPI master: %d", rc);
        return rc;
    }

    /* Configure slave device */
    rc = mspi_dev_config(mspi_slave, &slave_id, MSPI_DEVICE_CONFIG_ALL, &slave_cfg);
    if (rc != 0) {
        LOG_ERR("Failed to configure MSPI slave: %d", rc);
        return rc;
    }

    /* Register callback for completion notification */
    k_sem_init(&callback_ctx.completion_sem, 0, 1);
    callback_ctx.status = 0;

    /* Create MSPI callback context */
    // struct mspi_callback_context mspi_cb_ctx = {
    //     .ctx = &callback_ctx
    // };

    // rc = mspi_register_callback(mspi_slave, &slave_id, MSPI_BUS_XFER_COMPLETE,
    //                            transfer_completion_callback, &mspi_cb_ctx);
    // if (rc != 0) {
    //     LOG_ERR("Failed to register callback: %d", rc);
    //     return rc;
    // }

    return 0;
}
#include <zephyr/drivers/gpio.h>
#define LED0_NODE DT_ALIAS(led0)

/* Setup all packets upfront - BEFORE timing measurement */
struct mspi_xfer_packet tx_packets[MAX_DESCRIPTORS];
struct mspi_xfer_packet rx_packets[MAX_DESCRIPTORS];

/* Run a single test case */
static int run_single_test(enum mspi_xfer_direction direction, 
                          uint32_t buffer_size, 
                          uint32_t descriptor_count)
{
    int rc;
    printk("buffer_size = %u\r\n", buffer_size);
    uint32_t total_buffers = TOTAL_TRANSFER_SIZE / buffer_size;
    uint32_t remaining_bytes = TOTAL_TRANSFER_SIZE % buffer_size;
    static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
    gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);

    if (remaining_bytes > 0) {
        total_buffers++; /* Need one more buffer for remaining bytes */
    }

    /* Calculate how many transfers we need to make */
    uint32_t num_transfers = (total_buffers + descriptor_count - 1) / descriptor_count;

    // /* Setup all packets upfront - BEFORE timing measurement */
    // struct mspi_xfer_packet tx_packets[MAX_DESCRIPTORS];
    // struct mspi_xfer_packet rx_packets[MAX_DESCRIPTORS];

    if (descriptor_count > MAX_DESCRIPTORS) {
        LOG_ERR("Descriptor count too high: %u (max %u)", descriptor_count, MAX_DESCRIPTORS);
        return -EINVAL;
    }

    /* Setup packets */
    for (uint32_t i = 0; i < MAX_DESCRIPTORS; i++) {
        tx_packets[i].dir = MSPI_TX;
        tx_packets[i].cmd = 0x03;
        tx_packets[i].address = 0x000000;
        tx_packets[i].data_buf = tx_buffer;
        /* tx_packets[i].num_bytes will be set inside the timing loop */

        rx_packets[i].dir = MSPI_RX;
        rx_packets[i].data_buf = rx_buffer;
        /* rx_packets[i].num_bytes will be set inside the timing loop */
    }


    /* Setup tx and rx transfer descriptions */
    struct mspi_xfer tx_xfer = {
        .xfer_mode = MSPI_DMA,
        /* tx_xfer.packets will be set inside the timing loop */
        /* tx_xfer.num_packet will be set inside the timing loop */
        .timeout = 10000,
        .cmd_length = CMD_LEN,
        .addr_length = ADDR_LEN,
        .async = false,
        .tx_dummy = 0,
    };

    struct mspi_xfer rx_xfer = {
        .xfer_mode = MSPI_DMA,
        /* rx_xfer.packets will be set inside the timing loop */
        /* rx_xfer.num_packet will be set inside the timing loop */
        .timeout = 10000,
        .cmd_length = CMD_LEN,
        .addr_length = ADDR_LEN,
        .async = true,
        .tx_dummy = 0,
    };

    LOG_INF("Transfer calculation: total_size=%u, buffer_size=%u, total_buffers=%u, remaining_bytes=%u, num_transfers=%u",
            TOTAL_TRANSFER_SIZE, buffer_size, total_buffers, remaining_bytes, num_transfers);

    transfer_complete = false;
    transfer_status = 0;

    gpio_pin_toggle_dt(&led);

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* Start performance measurement AFTER all packet setup is complete */
    transfer_start_cycles = k_cycle_get_32();
    DWT->CYCCNT = 0;

    /* Process transfers in chunks based on descriptor count - MINIMAL OVERHEAD */
    /* Only update num_bytes field inside the timing loop */
    /* All other packet fields (dir, cmd, address, data_buf) are pre-configured */
    for (uint32_t transfer_idx = 0; transfer_idx < num_transfers; transfer_idx++) {
        uint32_t start_buffer = transfer_idx * descriptor_count;
        uint32_t end_buffer = MIN(start_buffer + descriptor_count, total_buffers);
        uint32_t current_buffers = end_buffer - start_buffer;

        /* Only update the num_bytes field - everything else is pre-configured */
        for (uint32_t i = 0; i < current_buffers; i++) {
            uint32_t buffer_idx = start_buffer + i;
            uint32_t current_buffer_size = (buffer_idx == total_buffers - 1 && remaining_bytes > 0) ? 
                                         remaining_bytes : buffer_size;
            tx_packets[i].num_bytes = current_buffer_size;
            rx_packets[i].num_bytes = current_buffer_size + CMD_LEN + ADDR_LEN;
        }
        tx_xfer.packets = tx_packets;
        tx_xfer.num_packet = current_buffers;

        if(direction == MSPI_RX) {

            rx_xfer.packets = rx_packets;
            rx_xfer.num_packet = current_buffers;

            rc = mspi_transceive(mspi_slave, &slave_id, &rx_xfer);
            if (rc != 0) {
                LOG_ERR("Failed to start transfer chunk %u: %d", transfer_idx, rc);
                return rc;
            }

        }

        rc = mspi_transceive(mspi_master, &master_id, &tx_xfer);
        if (rc != 0) {
            LOG_ERR("Failed to start transfer chunk %u: %d", transfer_idx, rc);
            return rc;
        }

    } /* End of transfer chunks loop */
    cpu_cycles = DWT->CYCCNT;
    transfer_end_cycles = k_cycle_get_32();
    printk("cpu cycles = %u\n", cpu_cycles);

    gpio_pin_toggle_dt(&led);

    printk("total transfer time = %d us\r\n", transfer_end_cycles - transfer_start_cycles);

    /* Store results */
    calculate_and_store_results(direction, buffer_size, descriptor_count, num_transfers, total_buffers);

    LOG_INF("Test completed: dir=%d, buf=%d, desc=%d, cycles=%u",
            direction, buffer_size, descriptor_count, (uint32_t)(transfer_end_cycles - transfer_start_cycles));

    return 0;
}

/* Calculate and store test results */
static void calculate_and_store_results(enum mspi_xfer_direction direction,
                                      uint32_t buffer_size,
                                      uint32_t descriptor_count,
				                        uint32_t num_transfers,
                                      uint32_t total_buffers)
{
    if (test_result_index >= ARRAY_SIZE(test_results)) {
        LOG_ERR("Test results array full");
        return;
    }

    struct test_result *result = &test_results[test_result_index];

    result->direction = direction;
    result->buffer_size = (uint16_t)buffer_size;
    result->descriptor_count = (uint8_t)descriptor_count;
    result->transfer_cycles = (uint32_t)(transfer_end_cycles - transfer_start_cycles); /* Store CPU cycles directly */

    /* Calculate throughput in MB/s (scaled down to fit in uint16_t) */
    uint64_t timer_freq = (uint64_t)sys_clock_hw_cycles_per_sec();
    double transfer_time_sec = (double)((transfer_end_cycles - transfer_start_cycles)) / (double)timer_freq;
    LOG_INF("transfer_time_sec = %lf", transfer_time_sec);
    printk("(double)(TOTAL_TRANSFER_SIZE + (num_transfers*(ADDR_LEN + CMD_LEN))) = %lf\r\n", (double)(TOTAL_TRANSFER_SIZE + (num_transfers*(ADDR_LEN + CMD_LEN))));
    result->throughput_mbps  = (double)(TOTAL_TRANSFER_SIZE + (num_transfers*(ADDR_LEN + CMD_LEN))) / MBYTE / transfer_time_sec;
    double efficiency = (double)(result->throughput_mbps)/IDEAL_THROUGHPUT_MBYTE;
    double estimated_latency_us = 1000000.0*(transfer_time_sec*(1.0-efficiency)/(double)total_buffers);
      LOG_INF("throughput = %lf\r\n", result->throughput_mbps);
    LOG_INF("efficiency = %lf, estimated_latency_us = %lf\r\n", efficiency, estimated_latency_us);

    /* Calculate CPU load */
    result->cpu_freq_mhz = (uint16_t)256;

    /* Calculate MIPS (scaled down to fit in uint16_t) */
    result->cpu_load_mips = (double)((double)cpu_cycles / transfer_time_sec / 1000000.0);
    result->cpu_load_per = (double)((double)(cpu_cycles/result->cpu_freq_mhz) / (transfer_time_sec*1000000.0))*100.0;



    test_result_index++;
}

static void print_test_results(void)
{
    LOG_INF("=== TEST RESULTS ===");
    LOG_INF("%-8s %-8s %-8s %-12s %-12s %-8s %-8s %-8s", 
            "Dir", "BufSize", "DescCnt", "Time", "Throughput", "CPU", "CPU Load", "Efficiency");
    LOG_INF("%-8s %-8s %-8s %-12s %-12s %-8s %-8s %-8s", 
            "", "(bytes)", "", "(us)", "(MB/s)", "(MIPS)", "(%)", "(%)");
    LOG_INF("--------------------------------------------------------------------------------");

    for (uint32_t i = 0; i < test_result_index; i++) {
        struct test_result *result = &test_results[i];
        const char *direction_str = (result->direction == MSPI_TX) ? "TX" : "RX";

        LOG_INF("%-8s %-8u %-8u %-12u %-12.3f %-8.2f %-8.2f %-8.3f",
                direction_str, result->buffer_size, result->descriptor_count,
                result->transfer_cycles, (double)result->throughput_mbps,
                (double)result->cpu_load_mips, result->cpu_load_per,
		(double)(result->throughput_mbps)/IDEAL_THROUGHPUT_MBYTE);
    }
}

/* Print summary report with scaling analysis */
static void print_summary_report(void)
{
    LOG_INF("\n=== SUMMARY REPORT ===");

    /* Find best performance configurations */
    double best_tx_throughput = 0.0;
    double best_rx_throughput = 0.0;
    uint32_t best_tx_buf_size = 0;
    uint32_t best_rx_buf_size = 0;
    uint32_t best_tx_desc_count = 0;
    uint32_t best_rx_desc_count = 0;

    for (uint32_t i = 0; i < test_result_index; i++) {
        struct test_result *result = &test_results[i];

        if (result->direction == MSPI_TX && (double)result->throughput_mbps> best_tx_throughput) {
            best_tx_throughput = (double)result->throughput_mbps;
            best_tx_buf_size = result->buffer_size;
            best_tx_desc_count = result->descriptor_count;
        }

        if (result->direction == MSPI_RX && (double)result->throughput_mbps > best_rx_throughput) {
            best_rx_throughput = (double)result->throughput_mbps;
            best_rx_buf_size = result->buffer_size;
            best_rx_desc_count = result->descriptor_count;
        }
    }

    LOG_INF("Best TX Performance: %.2f MB/s (buf=%u, desc=%u)", 
            best_tx_throughput, best_tx_buf_size, best_tx_desc_count);
    LOG_INF("Best RX Performance: %.2f MB/s (buf=%u, desc=%u)", 
            best_rx_throughput, best_rx_buf_size, best_rx_desc_count);

    /* Scaling analysis for SoC vs FPGA */
    LOG_INF("\n=== SCALING ANALYSIS ===");
    LOG_INF("FPGA QSPI Clock: 8 MHz");
    LOG_INF("SoC QSPI Clock: 64 MHz (8x faster)");

    double scaled_tx_throughput = best_tx_throughput * 8.0 * FPGA_CLOCK_OFFSET;
    double scaled_rx_throughput = best_rx_throughput * 8.0 * FPGA_CLOCK_OFFSET;

    LOG_INF("FPGA Clocks are scaled differently so must also times by %d", FPGA_CLOCK_OFFSET);
    LOG_INF("Projected SoC TX Throughput: %.2f MB/s", scaled_tx_throughput);
    LOG_INF("Projected SoC RX Throughput: %.2f MB/s", scaled_rx_throughput);

}

/* Main test function */
int main(void)
{
    int rc;

    LOG_INF("=== QSPI Throughput Test ===");
    LOG_INF("Target: 1 MByte transfer with scatter-gather DMA");
    LOG_INF("QSPI Clock: %u MHz", QSPI_CLOCK_FREQ / 1000000);

    /* Check device readiness */
    if (!device_is_ready(mspi_master)) {
        LOG_ERR("MSPI master device not ready");
        return -ENODEV;
    }

    if (!device_is_ready(mspi_slave)) {
        LOG_ERR("MSPI slave device not ready");
        return -ENODEV;
    }

    /* Setup test environment */
    setup_test_buffers();

    rc = configure_mspi_devices();
    if (rc != 0) {
        LOG_ERR("Failed to configure MSPI devices: %d", rc);
        return rc;
    }

    LOG_INF("Starting throughput tests...");
    LOG_INF("Ideal throughput with a frequency of %d is %.3f MB/s", QSPI_CLOCK_FREQ, (double) IDEAL_THROUGHPUT_MBYTE);

    LOG_INF("Total test cases: %u", TEST_DIRECTIONS * BUFFER_SIZES_COUNT * DESCRIPTOR_SIZES_COUNT);

    // enum mspi_xfer_direction direction = test_directions[0];
    // uint32_t buffer_size = buffer_sizes[BUFFER_SIZES_COUNT-2];//BUFFER_SIZES_COUNT-6];
    // uint32_t descriptor_count = descriptor_counts[0];
    // int buf_idx = BUFFER_SIZES_COUNT-1;

    /* Delay needed for cycle measurement as other background processes must finish */
    k_msleep(500);

    // CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	// DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	// DWT->CYCCNT = 0;

    // while (1) {
	// 	transfer_start_cycles = k_cycle_get_32();
	// 	DWT->CYCCNT = 0;
    //     printk(".\r\n");

	// 	// printk("LED state: %s\n", led_state ? "ON" : "OFF");
	// 	// printk("second DWT->CYCCNT = %d\r\n", DWT->CYCCNT);
	// 	k_msleep(50);
	// 	cpu_cycles = DWT->CYCCNT;
	// 	transfer_end_cycles = k_cycle_get_32();
	// 	printk("cpu_cycle = %d\r\n", cpu_cycles);
	// 	printk("time taken us = %d\r\n", transfer_end_cycles-transfer_start_cycles);

	// }

    // rc = run_single_test(MSPI_TX, buffer_sizes[BUFFER_SIZES_COUNT-1], descriptor_counts[DESCRIPTOR_SIZES_COUNT-1]);
    // rc = run_single_test(MSPI_TX, buffer_sizes[1], descriptor_counts[1]);


    /* Run all test permutations */
    for (uint32_t dir_idx = 0; dir_idx < TEST_DIRECTIONS; dir_idx++) {
        for (uint32_t buf_idx = 0; buf_idx < BUFFER_SIZES_COUNT; buf_idx++) {
            for (uint32_t desc_idx = 0; desc_idx < DESCRIPTOR_SIZES_COUNT; desc_idx++) {
                enum mspi_xfer_direction direction = test_directions[dir_idx];
                uint32_t buffer_size = buffer_sizes[buf_idx];
                uint32_t descriptor_count = descriptor_counts[desc_idx];

                LOG_INF("Running test %u/%u: dir=%d, buf=%u, desc=%u",
                        test_result_index + 1, 
                        TEST_DIRECTIONS * BUFFER_SIZES_COUNT * DESCRIPTOR_SIZES_COUNT,
                        direction, buffer_size, descriptor_count);

                rc = run_single_test(direction, buffer_size, descriptor_count);
                if (rc != 0) {
                    LOG_ERR("Test failed: %d", rc);
                    /* Continue with other tests */
                }

                /* Small delay between tests */
                k_msleep(100);
            }
        }
    }

    /* Print results */
    print_test_results();
    print_summary_report();

    LOG_INF("=== QSPI Throughput Test Complete ===");

    return 0;
}
