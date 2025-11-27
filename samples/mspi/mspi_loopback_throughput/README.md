# QSPI Loopback Throughput Test

## Overview

This test evaluates the throughput performance of QSPI (Quad Serial Peripheral Interface) using Zephyr driver when running on FPGA, with the goal of ensuring sufficient throughput to enable Wi-Fi at the fastest supported rate.

## Test Specification

### Hardware Configuration
- **QSPI00**: Configured as master device
- **QSPI01**: Configured as slave device  
- **Clock Frequency**: 8 MHz (as specified for FPGA)
- **Transfer Mode**: Scatter-gather DMA with configurable buffer sizes

### Test Parameters
The test runs **42 test cases** covering all permutations of:

1. **Direction**: Read (RX) / Write (TX)
2. **Buffer Size**: 4, 16, 64, 256, 1024, 1500, 2304 bytes
3. **Descriptor Count**: 1, 4, 16 descriptors per API call (chunks the 1MB transfer)

### Test Configuration
- **Total Transfer Size**: 1 MByte per test case
- **DMA Mode**: Scatter-gather with configurable packet sizes
- **Descriptor Count Testing**: 
  - 1 descriptor: 1MB transfer split into many small API calls
  - 4 descriptors: 1MB transfer split into medium API calls  
  - 16 descriptors: 1MB transfer split into fewer, larger API calls
- **Timeout**: 10 seconds per transfer chunk
- **Async Mode**: Enabled for non-blocking operation

## Building and Running

### Prerequisites
- Zephyr development environment
- Nordic nRF71xx SDK
- FPGA board with QSPI support
- Minimum 4KB RAM available for test execution

### Build Commands
```bash
# Build for your target board
west build -b <your_board> -p

# Flash to device
west flash

# Monitor output
west espressif monitor
```

### Device Tree Configuration
The test uses a device tree overlay (`qspi_loopback.overlay`) that configures:
- QSPI00 as master with CS0 on GPIO17
- QSPI01 as slave with CS1 on GPIO18  
- Loopback connections via GPIO19-22
- DMA channels for scatter-gather transfers

## Test Execution

### Automatic Test Sequence
The test automatically runs all 42 test cases in sequence:
1. For each direction (TX/RX)
2. For each buffer size (4 to 2304 bytes)
3. For each descriptor count (1, 4, 16)

### Performance Measurements
Each test case measures:
- **Transfer Time**: Total time to complete 1 MByte transfer
- **Throughput**: Data rate in MB/s
- **CPU Load**: Instructions per second (MIPS)
- **CPU Frequency**: Operating frequency in MHz

### Results Output
The test provides:
1. **Detailed Results**: Individual test case performance
2. **Summary Report**: Best performing configurations
3. **Scaling Analysis**: FPGA vs SoC performance projection
4. **Wi-Fi Assessment**: Throughput adequacy for Wi-Fi standards

## Results Analysis

### Throughput Scaling
- **FPGA**: 8 MHz QSPI clock
- **SoC**: 64 MHz QSPI clock (8x faster)
- **Projected Performance**: FPGA results × 8

### Wi-Fi Requirements
- **Wi-Fi 6 (802.11ax)**: ~1200 MB/s
- **Wi-Fi 5 (802.11ac)**: ~862 MB/s
- **Assessment**: Determines if QSPI throughput is sufficient

### Performance Optimization
The test helps identify:
- Optimal buffer sizes for maximum throughput
- Best descriptor counts for efficient DMA usage
- CPU load characteristics for different configurations

## Expected Output

```
=== QSPI Throughput Test ===
Target: 1 MByte transfer with scatter-gather DMA
QSPI Clock: 8 MHz
Starting throughput tests...
Total test cases: 42

Running test 1/42: dir=0, buf=4, desc=1
Test completed: dir=0, buf=4, desc=1, time=123456789 ns, cycles=12345
...

=== DETAILED TEST RESULTS ===
Dir     BufSize  DescCnt  Time(ns)     Throughput  CPU     CPU
        (bytes)          (MB/s)        (MIPS)      (MHz)
------------------------------------------------------------
TX      4        1        123456789    8.12        12.34   64
...

=== SUMMARY REPORT ===
Best TX Performance: 8.12 MB/s (buf=1024, desc=16)
Best RX Performance: 7.89 MB/s (buf=1024, desc=16)

=== SCALING ANALYSIS ===
FPGA QSPI Clock: 8 MHz
SoC QSPI Clock: 64 MHz (8x faster)
Projected SoC TX Throughput: 64.96 MB/s
Projected SoC RX Throughput: 63.12 MB/s

=== WI-FI THROUGHPUT ASSESSMENT ===
Wi-Fi 6 (802.11ax) Max Rate: ~9.6 Gbps = ~1200 MB/s
Wi-Fi 5 (802.11ac) Max Rate: ~6.9 Gbps = ~862 MB/s
✗ QSPI throughput INSUFFICIENT for high-speed Wi-Fi
  Consider: Higher QSPI clock, wider bus, or multiple QSPI channels
```

## Memory Optimization

### Design Philosophy
The test is designed to work with very limited RAM:
- **Total Buffer Usage**: Only 2304 bytes (largest test buffer size)
- **Stack Allocation**: Packet arrays allocated on stack (max 16 descriptors)
- **No Heap Fragmentation**: Reuses same memory for all transfers
- **Minimal Stack Usage**: 4KB main stack, 4KB heap pool

### Memory Requirements
- **Minimum RAM**: 8KB for test execution
- **Total Memory Usage**: ~8.1KB (3.2KB static + 1.9KB stack + 3.0KB heap)
- **Buffer Reuse**: Single 2.3KB buffer handles all transfer sizes
- **Efficient DMA**: Scatter-gather with minimal descriptor overhead
- **Memory Efficiency**: 27.9% buffer reuse efficiency

## Troubleshooting

### Common Issues
1. **Device Not Ready**: Check device tree configuration
2. **Transfer Timeout**: Verify loopback connections
3. **DMA Errors**: Check DMA channel configuration
4. **Memory Issues**: Ensure sufficient RAM (8KB minimum total)

### Debug Options
- Enable debug logging: `CONFIG_LOG_DEFAULT_LEVEL=4`
- Increase log buffer: `CONFIG_LOG_BUFFER_SIZE=32768`
- Enable DMA debug: `CONFIG_DMA_LOG_LEVEL_DBG=y`

## Technical Details

### Memory Management
- Single buffer reuse to minimize memory footprint (2304 bytes total)
- Stack-based packet allocation to avoid heap fragmentation
- Memory-optimized for limited RAM environments
- Configurable heap size optimized for small systems

### Performance Counters
- High-resolution timing using system uptime
- CPU cycle counting for load measurement
- Asynchronous completion callbacks

### DMA Configuration
- Scatter-gather mode for efficient large transfers
- Configurable buffer sizes and descriptor counts
- Automatic packet size adjustment for remaining bytes

## Future Enhancements

### Additional Metrics
- Processor stall cycle measurement
- Memory bandwidth analysis
- Power consumption profiling

### Extended Testing
- Multiple QSPI channel testing
- Different clock frequency testing
- Temperature variation testing

### Automation
- Automated result collection
- Performance regression testing
- Continuous integration support 