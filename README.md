# STM32 Embedded Systems Projects

A comprehensive portfolio of embedded systems projects using STM32 microcontrollers, featuring real-time signal processing, sensor integration, and advanced control algorithms.

## 🚀 Projects Overview

### 01. [Accelerometer Filter - Multi-Order IIR Digital Signal Processing](./01-Accelerometer-Filter)
**Status:** ✅ Complete  
**Hardware:** STM32F407VG Discovery Board, LIS3DSH MEMS Accelerometer  
**Key Features:**
- Multi-order IIR Butterworth filtering (2nd, 4th, 8th order)
- Real-time pitch/roll angle calculation
- Runtime filter order switching via button press
- MATLAB dashboard with live visualization and FFT analysis
- DWT cycle counter performance benchmarking
- 100Hz sampling with precise STATUS register polling

**Technical Highlights:**
- Digital filter implementation with Direct Form structure
- SPI communication for sensor data acquisition
- UART streaming protocol for real-time data transmission
- Performance comparison: 150-600 CPU cycles depending on filter order
- Comprehensive coefficient verification against MATLAB `butter()`

**Skills Demonstrated:** DSP, IIR Filters, Real-time Systems, SPI/UART Communication, Performance Optimization, MATLAB Integration

[📂 View Project](./01-Accelerometer-Filter) | [📊 Dashboard Demo](./01-Accelerometer-Filter#using-the-matlab-dashboard)

---

## 🛠️ Shared Resources

### Libraries
Common libraries and utilities used across multiple projects:
- **DSP:** Filter implementations, signal processing utilities
- **Sensors:** Driver libraries for common sensors (IMU, magnetometer, etc.)
- **Communication:** UART, SPI, I2C abstraction layers

### HAL Configurations
Reusable STM32 HAL configurations for different microcontrollers and peripherals.

### Utils
Helper functions, debugging tools, and common utilities.

---

## 🎯 Technical Skills Portfolio

### Microcontrollers & Platforms
- **STM32F4 Series:** ARM Cortex-M4F @ 168MHz
- **Development Tools:** STM32CubeIDE, STM32CubeMX
- **Programming:** C (Embedded), MATLAB

### Signal Processing
- IIR/FIR Digital Filter Design & Implementation
- Real-time FFT Analysis
- Noise Reduction Algorithms
- Sensor Fusion Techniques

### Communication Protocols
- SPI (Sensor Interfacing)
- UART (Data Streaming)
- I2C
- USB (Future)

### Development Practices
- Performance Profiling (DWT Cycle Counter)
- Real-time Constraint Management
- Modular Code Architecture
- Hardware-Software Co-verification

---

## 📊 Project Statistics

| Metric | Value |
|--------|-------|
| Total Projects | 1 (Growing) |
| Lines of Code | 138,000+ |
| Microcontrollers Used | STM32F407VG |
| Languages | C, MATLAB |
| Sensors Integrated | LIS3DSH (3-axis accelerometer) |

---

## 🔧 Development Environment

### Hardware
- STM32F407VG Discovery Board
- Various sensors and peripherals
- ST-LINK programmer/debugger

### Software
- **IDE:** STM32CubeIDE (Eclipse-based)
- **HAL:** STM32 Hardware Abstraction Layer
- **Visualization:** MATLAB R2020b+
- **Version Control:** Git/GitHub

---

## 📖 How to Use This Repository

Each project folder contains:
- Complete source code and configuration files
- Detailed README with technical specifications
- Build and flash instructions
- Performance analysis and results
- Verification scripts (where applicable)

### Building Projects
1. Open project folder in STM32CubeIDE
2. Build: `Project → Build Project (Ctrl+B)`
3. Flash: `Run → Debug (F11)`
4. Run visualization tools (MATLAB scripts where applicable)

---

## 🎓 Learning Resources

Projects in this repository demonstrate practical implementations of concepts from:
- Digital Signal Processing (DSP)
- Real-time Embedded Systems
- Sensor Data Acquisition
- Hardware-Software Integration
- Performance Optimization

---

## 📫 Contact & Collaboration

This repository showcases hands-on experience with embedded systems development, real-time signal processing, and hardware-software integration. Each project includes comprehensive documentation, verification tools, and performance analysis.

---

## 📝 License

Individual projects may have different licenses. Check each project folder for specific licensing information. HAL drivers and CMSIS files retain their original STMicroelectronics licenses.

---

**Note:** This is an active development repository. Projects are continuously improved and new projects are added regularly.
