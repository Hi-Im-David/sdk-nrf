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

#define QSPI_CLOCK_FREQ     MHZ(8)  /* should be 8MHz as specified for FPGA */
#define MBYTE        (1024ULL * 1024ULL)   /* bytes per MiB */
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

/* In bytes */
#define ADDR_LEN 3
#define CMD_LEN 1

/* Buffer sizes to test (in bytes) */
static const uint32_t buffer_sizes[BUFFER_SIZES_COUNT] = {
    4, 16, 64, 256, 1024, 1500, 2304
};

/* Descriptor counts to test */
static const uint32_t descriptor_counts[DESCRIPTOR_SIZES_COUNT] = {
    1, 4, 16
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

/* Test buffers - reuse single buffer to save memory */
static uint8_t shared_buffer[2304]; /* Largest buffer size - reused for all transfers */

/* Performance measurement variables */
static volatile bool transfer_complete = false;
static volatile int transfer_status = 0;
static volatile uint32_t transfer_start_cycles = 0;  /* Use CPU cycles for high precision */
static volatile uint32_t transfer_end_cycles = 0;    /* Use CPU cycles for high precision */

/* Note: Using k_cycle_get_32() for high-precision timing instead of millisecond timers */
/* CPU cycles provide much higher resolution for accurate throughput measurements */

/* Test results structure - optimized for memory */
struct test_result {
    enum mspi_xfer_direction direction;
    uint16_t buffer_size;
    uint8_t descriptor_count;
    uint32_t transfer_cycles;    /* Store time in CPU cycles for high precision */
    uint16_t throughput_mbps;    /* Throughput in MB/s * 1000 */
    uint16_t cpu_load_mips;      /* MIPS * 100 */
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
static void transfer_completion_callback(struct mspi_callback_context *ctx, ...);
static void calculate_and_store_results(enum mspi_xfer_direction direction,
                                      uint32_t buffer_size,
                                      uint32_t descriptor_count,
                                      uint32_t num_transfers);
static void print_test_results(void);
static void print_summary_report(void);

/* Transfer completion callback */
static void transfer_completion_callback(struct mspi_callback_context *ctx, ...)
{
    transfer_complete = true;
    transfer_status = ctx->mspi_evt.evt_data.status;
    k_sem_give(&callback_ctx.completion_sem);
    // LOG_INF("Finished transfer");
	
}

/* Setup test buffers with pattern */
static void setup_test_buffers(void)
{
    for (uint32_t i = 0; i < sizeof(shared_buffer); i++) {
        shared_buffer[i] = (uint8_t)(i);
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
    struct mspi_callback_context mspi_cb_ctx = {
        .ctx = &callback_ctx
    };

    // rc = mspi_register_callback(mspi_master, &master_id, MSPI_BUS_XFER_COMPLETE,
    //                            transfer_completion_callback, &mspi_cb_ctx);
    // if (rc != 0) {
    //     LOG_ERR("Failed to register callback: %d", rc);
    //     return rc;
    // }

    return 0;
}
#include <zephyr/drivers/gpio.h>
#define LED0_NODE DT_ALIAS(led0)

/* Run a single test case */
static int run_single_test(enum mspi_xfer_direction direction, 
                          uint32_t buffer_size, 
                          uint32_t descriptor_count)
{
    int rc;
    uint32_t total_buffers = TOTAL_TRANSFER_SIZE / buffer_size;
    uint32_t remaining_bytes = TOTAL_TRANSFER_SIZE % buffer_size;
    static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
    gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    
    if (remaining_bytes > 0) {
        total_buffers++; /* Need one more buffer for remaining bytes */
    }

    /* Calculate how many transfers we need to make */
    uint32_t num_transfers = (total_buffers + descriptor_count - 1) / descriptor_count;
    
    /* Setup all packets upfront - BEFORE timing measurement */
    /* Use fixed-size array to avoid stack overflow, limit to reasonable size */
    #define MAX_PACKETS 16  /* Only need max descriptor_count packets, not total_buffers */
    struct mspi_xfer_packet packets[MAX_PACKETS];
    
    if (descriptor_count > MAX_PACKETS) {
        LOG_ERR("Descriptor count too high: %u (max %u)", descriptor_count, MAX_PACKETS);
        return -EINVAL;
    }
    
    
    for (uint32_t i = 0; i < MAX_PACKETS; i++) {
        packets[i].dir = direction;
        packets[i].cb_mask = MSPI_BUS_NO_CB;
        packets[i].cmd = 0x03;
        packets[i].address = 0x000000;
        packets[i].data_buf = shared_buffer;
        /* packets[i].num_bytes will be set inside the timing loop */
    }
    
    LOG_INF("Transfer calculation: total_size=%u, buffer_size=%u, total_buffers=%u, remaining_bytes=%u, num_transfers=%u",
            TOTAL_TRANSFER_SIZE, buffer_size, total_buffers, remaining_bytes, num_transfers);
    
    transfer_complete = false;
    transfer_status = 0;

    gpio_pin_toggle_dt(&led);

    /* Start performance measurement AFTER all packet setup is complete */
    transfer_start_cycles = k_cycle_get_32();

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
            packets[i].num_bytes = current_buffer_size;
        }
        
        /* Create transfer structure - use the reused packet array */
        struct mspi_xfer xfer = {
            .xfer_mode = MSPI_DMA,
            .packets = packets,  /* Reuse the same packet array */
            .num_packet = current_buffers,
            .timeout = 10000,
            .cmd_length = 1,
            .addr_length = 3,
            .async = false,
            .tx_dummy = 0,
        };

        /* Start transfer for this chunk - MINIMAL OVERHEAD */
        rc = mspi_transceive(mspi_master, &master_id, &xfer);
        if (rc != 0) {
            LOG_ERR("Failed to start transfer chunk %u: %d", transfer_idx, rc);
            return rc;
        }
    } /* End of transfer chunks loop */

    transfer_end_cycles = k_cycle_get_32();

    gpio_pin_toggle_dt(&led);

    printk("total transfer time = %d us\r\n", transfer_end_cycles - transfer_start_cycles);

    /* Store results */
    calculate_and_store_results(direction, buffer_size, descriptor_count, num_transfers);

    LOG_INF("Test completed: dir=%d, buf=%d, desc=%d, cycles=%u",
            direction, buffer_size, descriptor_count, (uint32_t)(transfer_end_cycles - transfer_start_cycles));

    return 0;
}

/* Calculate and store test results */
static void calculate_and_store_results(enum mspi_xfer_direction direction,
                                      uint32_t buffer_size,
                                      uint32_t descriptor_count,
                                      uint32_t num_transfers)
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
    uint64_t cpu_freq = (uint64_t)sys_clock_hw_cycles_per_sec();
    LOG_INF("cpu freq = %ld", cpu_freq);
    double transfer_time_sec = (double)((transfer_end_cycles - transfer_start_cycles)) / (double)cpu_freq;
    LOG_INF("transfer_time_sec = %lf");
    double throughput = (double)(TOTAL_TRANSFER_SIZE + (num_transfers*(ADDR_LEN + CMD_LEN))) / MBYTE / transfer_time_sec;
    result->throughput_mbps = (uint16_t)(throughput * 1000); /* Scale by 1000 for 3 decimal places */
    
    /* Calculate CPU load */
    result->cpu_freq_mhz = (uint16_t)(cpu_freq / 1000000);
    
    /* Calculate MIPS (scaled down to fit in uint16_t) */
    uint32_t cpu_cycles = transfer_end_cycles - transfer_start_cycles;
    result->cpu_load_mips = (uint16_t)((double)cpu_cycles / transfer_time_sec / 1000000.0 * 100);
    
    test_result_index++;
}

static void print_test_results(void)
{
    LOG_INF("=== TEST RESULTS ===");
    LOG_INF("%-8s %-8s %-8s %-12s %-12s %-8s %-8s %-8s", 
            "Dir", "BufSize", "DescCnt", "Cycles", "Throughput", "CPU", "CPU", "Efficiency");
    LOG_INF("%-8s %-8s %-8s %-12s %-12s %-8s %-8s %-8s", 
            "", "(bytes)", "", "", "(MB/s)", "(MIPS)", "(MHz)", "(%%)");
    LOG_INF("--------------------------------------------------------------------------------");

    for (uint32_t i = 0; i < test_result_index; i++) {
        struct test_result *result = &test_results[i];
        const char *direction_str = (result->direction == MSPI_TX) ? "TX" : "RX";
        
        LOG_INF("%-8s %-8u %-8u %-12u %-12.3f %-8.2f %-8u %-8.3f",
                direction_str, result->buffer_size, result->descriptor_count,
                result->transfer_cycles, (double)result->throughput_mbps / 1000.0,
                (double)result->cpu_load_mips / 100.0, result->cpu_freq_mhz,
		(double)(result->throughput_mbps/ 1000.0)/IDEAL_THROUGHPUT_MBYTE);
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
        
        if (result->direction == MSPI_TX && (double)result->throughput_mbps / 100.0 > best_tx_throughput) {
            best_tx_throughput = (double)result->throughput_mbps / 100.0;
            best_tx_buf_size = result->buffer_size;
            best_tx_desc_count = result->descriptor_count;
        }
        
        if (result->direction == MSPI_RX && (double)result->throughput_mbps / 100.0 > best_rx_throughput) {
            best_rx_throughput = (double)result->throughput_mbps / 100.0;
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
    LOG_INF("Eff %.3f MB/s", (double) 3.58/IDEAL_THROUGHPUT_MBYTE);
    
    LOG_INF("Total test cases: %u", TEST_DIRECTIONS * BUFFER_SIZES_COUNT * DESCRIPTOR_SIZES_COUNT);

    enum mspi_xfer_direction direction = test_directions[0];
    // uint32_t buffer_size = buffer_sizes[BUFFER_SIZES_COUNT-2];//BUFFER_SIZES_COUNT-6];
    // uint32_t descriptor_count = descriptor_counts[0];
    // int buf_idx = BUFFER_SIZES_COUNT-1;

    rc = run_single_test(direction, buffer_sizes[BUFFER_SIZES_COUNT-1], descriptor_counts[DESCRIPTOR_SIZES_COUNT-1]);


    /* Run all test permutations */
    // for (uint32_t dir_idx = 0; dir_idx < TEST_DIRECTIONS; dir_idx++) {
    //     for (uint32_t buf_idx = 0; buf_idx < BUFFER_SIZES_COUNT; buf_idx++) {
    //         for (uint32_t desc_idx = 0; desc_idx < DESCRIPTOR_SIZES_COUNT; desc_idx++) {
    //             enum mspi_xfer_direction direction = test_directions[0];
    //             uint32_t buffer_size = buffer_sizes[buf_idx];
    //             uint32_t descriptor_count = descriptor_counts[desc_idx];
                
    //             LOG_INF("Running test %u/%u: dir=%d, buf=%u, desc=%u",
    //                     test_result_index + 1, 
    //                     TEST_DIRECTIONS * BUFFER_SIZES_COUNT * DESCRIPTOR_SIZES_COUNT,
    //                     direction, buffer_size, descriptor_count);
                
    //             rc = run_single_test(direction, buffer_size, descriptor_count);
    //             if (rc != 0) {
    //                 LOG_ERR("Test failed: %d", rc);
    //                 /* Continue with other tests */
    //             }
    //             printk("3\r\n");
                
    //             /* Small delay between tests */
    //             k_msleep(100);
    //         }
    //     }
    // }
    
    /* Print results */
    print_test_results();
    print_summary_report();
    
    LOG_INF("=== QSPI Throughput Test Complete ===");
    
    return 0;
}
