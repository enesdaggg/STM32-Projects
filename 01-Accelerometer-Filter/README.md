# Accelerometer Filter - Real-Time Digital Signal Processing

## Overview

This project implements **multi-order IIR filtering** (2nd, 4th, 8th order) for processing accelerometer data from the LIS3DSH MEMS sensor on the **STM32F407VG Discovery board**. The system calculates both **pitch and roll angles** from 3-axis accelerometer readings and applies **Butterworth low-pass filters** to reduce noise. Data is streamed over UART and visualized in real-time using an optimized **MATLAB dashboard** with performance benchmarking capabilities.

## Hardware Components

- **MCU:** STM32F407VGT6 (ARM Cortex-M4F @ 168MHz)
- **Sensor:** LIS3DSH 3-axis MEMS accelerometer (onboard, SPI interface)
- **Communication:** UART (115200 baud) via ST-LINK USB
- **Performance:** DWT cycle counter for precise execution time measurement

## System Specifications

### Digital Filter Design
- **Type:** IIR Butterworth Low-Pass Filter
- **Orders Available:** 2nd, 4th, and 8th order (switchable via onboard blue button)
- **Sampling Frequency (Fs):** 100 Hz (precise timing via STATUS register polling)
- **Cutoff Frequency (Fc):** 5 Hz
- **Implementation:** Direct Form with separate state variables for pitch and roll
- **Runtime Control:** Press blue button (PA0) to cycle through orders; LED blinks indicate current order

### Filter Coefficients

All filters designed with Fs=100Hz, Fc=5Hz using MATLAB's `butter()` function.

#### 2nd Order
```c
B: [0.020083, 0.040167, 0.020083]
A: [1.000000, -1.561018, 0.641352]
```

#### 4th Order
```c
B: [0.000416, 0.001664, 0.002496, 0.001664, 0.000416]
A: [1.000000, -3.180638, 3.861194, -2.112155, 0.438265]
```

#### 8th Order
```c
B: [0.000000, 0.000001, 0.000005, 0.000010, 0.000012, 0.000010, 0.000005, 0.000001, 0.000000]
A: [1.000000, -6.390365, 18.000338, -29.171099, 29.731375, -19.505632, 8.040996, -1.903669, 0.198100]
```

**Verification:** Run `Verify_Filter_Coefficients.m` in MATLAB to confirm coefficients match `butter()` output.

### Sensor Configuration
- **Full Scale Range:** ±2g
- **Output Data Rate:** 100 Hz
- **Resolution:** 16-bit
- **Interface:** SPI Mode 0 (CPOL=0, CPHA=0)
- **STATUS Register:** ZYXDA bit polling for fresh data
- **Sensitivity:** 0.06 mg/LSB

## Communication Protocol

### Data Stream Format
```
S,<order>,<pitch_raw>,<pitch_filt>,<roll_raw>,<roll_filt>,E\r\n
```

**Example:**
```
S,2,12.45,11.23,-5.67,-4.89,E
S,4,14.67,12.01,-6.12,-5.34,E
```

### Performance Message Format
```
PERF,<order>,<cycles>,<time_us>\r\n
```

**Example:**
```
PERF,2,156,0.93
PERF,4,287,1.71
PERF,8,523,3.11
```

### Filter Change Notification Format
```
FILTER_CHANGE,<new_order>\r\n
```

**Purpose:** Sent when filter order changes via button press. MATLAB dashboard automatically clears all data buffers to prevent DC offset artifacts from mixing different filter outputs.

**Example:**
```
FILTER_CHANGE,4
```

## Building and Flashing

### Prerequisites
- STM32CubeIDE (version 1.10.0 or later)
- STM32F407VG Discovery board
- USB cable (Mini-B)
- MATLAB R2020b or later (for dashboard)

### Steps

1. **Build Project:** `Project → Build Project (Ctrl+B)`

2. **Flash to Board:** `Run → Debug (F11)`

3. **Change Filter Order at Runtime:**
   - Press the **blue button** on the board (User Button, PA0)
   - Watch the **blue LED** (LD6, PD15) blink to indicate new order:
     - **2 blinks** = 2nd order filter
     - **4 blinks** = 4th order filter
     - **8 blinks** = 8th order filter
   - Filter cycles: 2 → 4 → 8 → 2 → ...
   - Performance test runs automatically after each change

## Using the MATLAB Dashboard

### Launch and Connect
```matlab
DSP_Dashboard
```
- Select COM port from dropdown
- Click "Connect" button
- Performance test results appear in console

### Features
- **Time Domain Plots:** 25Hz update (smooth real-time)
- **FFT Analysis:** 5Hz update (optimized computation)
- **3D Visualization:** 100Hz update (full rate)
- **Metrics Display:** Filter order, SNR, NRR, RMS, sample count, performance comparison
- **Performance Panel:** Real-time cycle count and execution time for all filter orders

### Controls
- **Pause:** Freeze display
- **Clear:** Reset buffers (keeps 3D model at neutral)
- **Record:** Save to timestamped .mat file

## Filter Performance Comparison

| Order | CPU Cycles | Time (µs) | NRR (dB) | Trade-off |
|-------|-----------|-----------|----------|-----------|
| 2     | ~150-200  | ~0.9-1.2  | ~10-12   | Fast, moderate filtering |
| 4     | ~250-350  | ~1.5-2.1  | ~15-18   | **Best balance** |
| 8     | ~450-600  | ~2.7-3.6  | ~22-25   | Excellent filtering, slower |

### Testing Procedure

1. Flash firmware to board
2. Press blue button to select filter order (watch LED blinks)
3. Observe `PERF` message in MATLAB console
4. Run dashboard for 60 seconds
5. Record NRR values from metrics panel
6. Press button again to test next order

## Verifying Filter Coefficients

Run in MATLAB:
```matlab
Verify_Filter_Coefficients
```

This script:
- Automatically reads coefficients from `Core/Src/main.c`
- Compares coefficients against `butter()` output
- Displays frequency and phase responses
- Shows step responses
- Reports -3dB cutoff frequencies
- All coefficients should match within 1e-5 tolerance

### Understanding Verification Plots

The verification script generates two figures to help you understand filter characteristics:

**Figure 1: Filter Frequency Response Verification**
- **Top Row (Magnitude Plots):** Shows how much each frequency is attenuated
  - Red dashed line: Cutoff frequency (5 Hz)
  - Black dashed line: -3dB point (half-power frequency)
  - Higher order filters have sharper cutoff (steeper slope)
  
- **Bottom Row (Phase Plots):** Shows time delay introduced by filter
  - Steeper phase = more delay
  - 8th order has most delay but sharpest filtering

**Figure 2: Step Response Comparison**
- Shows filter response to sudden input change (0 → 1)
- **Overshoot:** 4th and 8th order have slight oscillation before settling
- **Settling time:** Higher order = slower settling
- **Trade-off:** Sharp filtering vs. transient response

**Practical Interpretation:**
- **2nd order:** Fast response, moderate filtering
- **4th order:** Best balance (recommended)
- **8th order:** Excellent filtering but slower transient response

## Key Functions

### STM32 Firmware

**`Apply_IIR_Filter(float input, FilterState_t *state)`**
- Variable-order IIR filter implementation
- Runtime coefficient selection based on `current_filter_order`
- Circular buffer state management

**`Check_Button_Press(void)`**
- Detects blue button press with 300ms debouncing
- Cycles through filter orders: 2 → 4 → 8 → 2
- Resets filter states on order change
- Runs 20-iteration warm-up to prevent transient offset
- Triggers LED feedback and performance test

**`Blink_LED_Times(uint8_t times)`**
- Visual feedback via blue LED (LD6)
- Blinks 2, 4, or 8 times based on filter order
- 150ms on/off periods (minimal performance impact)

**`Run_Performance_Test(void)`**
- 1000-iteration averaging for accuracy
- DWT cycle counter measurement
- Automatic execution after filter order change
- Transmits results via UART

**`LIS3DSH_Init(void)`**
- Verifies device ID (0x3F)
- Configures 100Hz ODR, ±2g range
- Returns 1 on success

### MATLAB Dashboard

**`updatePlots()`**
- Persistent handle caching (fast rendering)
- Pre-allocated circular buffers
- Smart xlim updates
- FFT throttling (every 5th update)
- Automatic buffer clearing on filter order change (prevents DC offset)

**`updateMetrics()`**
- RMS calculation for raw and filtered signals
- SNR: `10*log10(signal_power/noise_power)` - Signal quality indicator
- NRR: `10*log10(var(raw)/var(filtered))` - Noise reduction effectiveness
- Performance comparison with color-coded display
- Filter order highlighting (bold for active filter)


## Technical Details

### User Interface
- **Blue Button (PA0):** Filter order selection
- **Blue LED (LD6/PD15):** Visual feedback (2/4/8 blinks)
- **Debounce:** 300ms software debouncing
- **Overhead:** <1ms per button check (negligible at 100Hz sampling)

### SPI Communication
- **CS:** PE3, **CLK:** PA5, **MISO:** PA6, **MOSI:** PA7
- **Mode:** 0 (CPOL=0, CPHA=0)
- **Speed:** ~5.25 MHz (APB2/16)

### DWT Cycle Counter
```c
CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // Enable trace
DWT->CYCCNT = 0;                                  // Reset
DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;             // Enable
```
Resolution: 1 cycle = 5.95 ns @ 168MHz

## Performance Optimizations

### STM32
- Variable-order filter with runtime selection (switch statement)
- Button check integrated in main loop (<1ms overhead)
- LED feedback uses HAL_Delay only during transitions
- DWT verification before benchmarking
- Filter states reset on order change to prevent transients

### MATLAB
- Persistent handle caching (95% faster)
- Vectorized operations
- Smart update throttling (25Hz time, 5Hz FFT)
- Pre-allocated buffers

**Results:** Smooth 25 FPS visualization, 10-20% CPU usage, <5ms latency, button response <1ms

## Project Structure

```
Accelerometer_Filter/
├── Core/Src/main.c                 # Main application
├── DSP_Dashboard.m                 # MATLAB dashboard
├── Generate_Filter_Coefficients.m  # Coefficient generator
├── Verify_Filter_Coefficients.m    # Verification tool
└── README.md                       # This file
```

## References

- [STM32F407VG Datasheet](https://www.st.com/resource/en/datasheet/stm32f407vg.pdf)
- [LIS3DSH Datasheet](https://www.st.com/resource/en/datasheet/lis3dsh.pdf)
- [STM32F4 Discovery Manual](https://www.st.com/resource/en/user_manual/um1472-discovery-kit-with-stm32f407vg-mcu-stmicroelectronics.pdf)
