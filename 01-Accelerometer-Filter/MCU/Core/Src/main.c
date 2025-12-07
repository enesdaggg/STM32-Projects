/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Accelerometer Filter - Real-time Accelerometer Data Processing
  ******************************************************************************
  * @project        : Accelerometer Filter - Dual-Axis Digital Signal Processing
  * @description    : This project implements a 2nd order IIR Butterworth
  *                   low-pass digital filter for processing accelerometer data
  *                   from the LIS3DSH MEMS sensor on STM32F407VG Discovery board.
  * 
  * @hardware       : STM32F407VGT6 MCU
  *                   LIS3DSH 3-axis accelerometer (SPI interface)
  * 
  * @features       : - Real-time pitch and roll angle calculation
  *                   - Digital IIR filter (Fc=5Hz, Fs=100Hz) for noise reduction
  *                   - UART data streaming for visualization (MATLAB Dashboard)
  *                   - 100Hz sampling rate with DWT cycle counter for benchmarking
  * 
  * @author         : Enes
  * @date           : December 2025
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <math.h>   // Required for atan2, sqrt, etc.
#include <stdio.h>  // Required for sprintf
#include <string.h> // Required for strlen

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Structure to hold state variables for each filter instance (Pitch, Roll)
typedef struct
{
  float x_history[9]; // Input history: x[n], x[n-1], ..., x[n-8] (max 8th order)
  float y_history[9]; // Output history: y[n], y[n-1], ..., y[n-8] (max 8th order)
} FilterState_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* LIS3DSH Register Addresses */
#define LIS3DSH_WHO_AM_I        0x0F  // Device identification register
#define LIS3DSH_CTRL_REG3       0x23  // Interrupt control register
#define LIS3DSH_CTRL_REG4       0x20  // Control register 4 (ODR, Power mode)
#define LIS3DSH_CTRL_REG5       0x24  // Control register 5 (Full scale, BDU)
#define LIS3DSH_CTRL_REG6       0x25  // Control register 6 (FIFO)
#define LIS3DSH_STATUS          0x27  // Status register
#define LIS3DSH_OUT_X_L         0x28  // X-axis LSB
#define LIS3DSH_OUT_X_H         0x29  // X-axis MSB
#define LIS3DSH_OUT_Y_L         0x2A  // Y-axis LSB
#define LIS3DSH_OUT_Y_H         0x2B  // Y-axis MSB
#define LIS3DSH_OUT_Z_L         0x2C  // Z-axis LSB
#define LIS3DSH_OUT_Z_H         0x2D  // Z-axis MSB

/* LIS3DSH Device ID */
#define LIS3DSH_ID              0x3F

/* LIS3DSH Configuration Values */
#define LIS3DSH_ODR_100HZ       0x67  // 100Hz ODR, XYZ axes enabled

/* LIS3DSH Sensitivity (±2g range, 16-bit resolution) */
#define LIS3DSH_SENSITIVITY     0.06f // mg/LSB

/* Mathematical Constants */
#define RAD_TO_DEG              57.2958f // 180/PI

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* =============================================================================
 * IIR DIGITAL FILTER COEFFICIENTS
 * Design: Butterworth Low-Pass Filter (2nd, 4th, 8th Order)
 * Sampling Frequency (Fs): 100 Hz
 * Cutoff Frequency (Fc): 5 Hz
 * Generated using MATLAB's Filter Design Tool
 * ============================================================================= */

// 2nd Order Butterworth Filter Coefficients
const float B_COEFFS_2[3] = {0.020083f, 0.040167f, 0.020083f};
const float A_COEFFS_2[3] = {1.000000f, -1.561018f, 0.641352f};

// 4th Order Butterworth Filter Coefficients
const float B_COEFFS_4[5] = {0.000416f, 0.001664f, 0.002496f, 0.001664f, 0.000416f};
const float A_COEFFS_4[5] = {1.000000f, -3.180638f, 3.861194f, -2.112155f, 0.438265f};

// 8th Order Butterworth Filter Coefficients
const float B_COEFFS_8[9] = {0.000000f, 0.000001f, 0.000005f, 0.000010f, 0.000012f, 0.000010f, 0.000005f, 0.000001f, 0.000000f};
const float A_COEFFS_8[9] = {1.000000f, -6.390365f, 18.000338f, -29.171099f, 29.731375f, -19.505632f, 8.040996f, -1.903669f, 0.198100f};

/* Runtime Filter Order Control */
volatile uint8_t current_filter_order = 2;  // Default: 2nd order (changeable via button)

/* Button Debouncing */
static uint32_t last_button_press = 0;
#define BUTTON_DEBOUNCE_MS 300

/* Filter State Instances */
FilterState_t pitch_filter = {0}; // State for Pitch
FilterState_t roll_filter = {0};  // State for Roll

/* Sensor Data Variables */
static int16_t raw_acc_x, raw_acc_y, raw_acc_z; // Raw 16-bit accelerometer data
static float acc_x_g, acc_y_g, acc_z_g;         // Acceleration in g units

static float pitch_raw, pitch_filtered;         // Raw and filtered pitch angle (degrees)
static float roll_raw, roll_filtered;           // Raw and filtered roll angle (degrees)

/* Benchmarking Variables */
static uint32_t start_cycles, end_cycles, total_cycles;

/* UART Communication Buffer */
static char uart_tx_buffer[128];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

void Check_Button_Press(void);
void Blink_LED_Times(uint8_t times);
float Apply_IIR_Filter(float input, FilterState_t *pState);
void Run_Performance_Test(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief  Initializes the DWT (Data Watchpoint and Trace) unit for cycle counting.
 * Used to measure code execution time precisely.
 */
void DWT_Init(void)
{
  // Enable TRC (Trace)
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  
  // Reset cycle counter
  DWT->CYCCNT = 0;
  
  // Enable cycle counter
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief  Check for button press and cycle through filter orders
 * @param  None
 * @retval None
 */
void Check_Button_Press(void)
{
  // Read button state (active high when pressed)
  if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_SET)
  {
    // Debounce check
    uint32_t current_time = HAL_GetTick();
    if ((current_time - last_button_press) > BUTTON_DEBOUNCE_MS)
    {
      last_button_press = current_time;

      // Cycle through filter orders: 2 -> 4 -> 8 -> 2
      if (current_filter_order == 2)
        current_filter_order = 4;
      else if (current_filter_order == 4)
        current_filter_order = 8;
      else
        current_filter_order = 2;

      // Reset filter states when changing order
      memset(&pitch_filter, 0, sizeof(FilterState_t));
      memset(&roll_filter, 0, sizeof(FilterState_t));
      
      // Warm-up filter with current sensor values to prevent transient offset
      // Run filter 20 times with current raw values to initialize states
      float current_pitch = pitch_raw;
      float current_roll = roll_raw;
      for (uint8_t i = 0; i < 20; i++)
      {
        Apply_IIR_Filter(current_pitch, &pitch_filter);
        Apply_IIR_Filter(current_roll, &roll_filter);
      }

      // Notify MATLAB to clear buffers
      sprintf(uart_tx_buffer, "FILTER_CHANGE,%d\r\n", current_filter_order);
      HAL_UART_Transmit(&huart2, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), HAL_MAX_DELAY);

      // Blink LED to indicate new order (2 = 2 blinks, 4 = 4 blinks, 8 = 8 blinks)
      uint8_t blink_count = (current_filter_order == 2) ? 2 : (current_filter_order == 4) ? 4 : 8;
      Blink_LED_Times(blink_count);

      // Run performance test for new filter order
      Run_Performance_Test();
    }
  }
}

/**
 * @brief  Blink blue LED (LD6) specified number of times
 * @param  times: Number of blinks
 * @retval None
 */
void Blink_LED_Times(uint8_t times)
{
  for (uint8_t i = 0; i < times; i++)
  {
    HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_SET);  // Turn on blue LED
    HAL_Delay(150);                                            // LED on for 150ms
    HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_RESET); // Turn off
    if (i < times - 1)  // Don't delay after last blink
      HAL_Delay(150);   // LED off for 150ms between blinks
  }
}

/**
 * @brief  Applies IIR Filter (2nd, 4th, or 8th Order) to a specific axis.
 * @param  input: Raw signal value
 * @param  pState: Pointer to the filter state structure (history buffers)
 * @retval Filtered output value
 */
float Apply_IIR_Filter(float input, FilterState_t *pState)
{
  const float *B;
  const float *A;
  uint8_t order;

  // Select coefficients based on runtime filter order
  switch (current_filter_order)
  {
    case 2:
      B = B_COEFFS_2;
      A = A_COEFFS_2;
      order = 2;
      break;
    case 4:
      B = B_COEFFS_4;
      A = A_COEFFS_4;
      order = 4;
      break;
    case 8:
      B = B_COEFFS_8;
      A = A_COEFFS_8;
      order = 8;
      break;
    default:
      B = B_COEFFS_2;
      A = A_COEFFS_2;
      order = 2;
      break;
  }

  // Shift input history (Delay line)
  for (int8_t i = order; i > 0; i--)
  {
    pState->x_history[i] = pState->x_history[i - 1];
  }
  pState->x_history[0] = input;

  // Shift output history
  for (int8_t i = order; i > 0; i--)
  {
    pState->y_history[i] = pState->y_history[i - 1];
  }

  // Compute Difference Equation: y[n] = sum(b[i]*x[n-i]) - sum(a[i]*y[n-i])
  float feedforward = 0.0f;
  float feedback = 0.0f;

  for (uint8_t i = 0; i <= order; i++)
  {
    feedforward += B[i] * pState->x_history[i];
  }

  for (uint8_t i = 1; i <= order; i++)
  {
    feedback += A[i] * pState->y_history[i];
  }

  pState->y_history[0] = feedforward - feedback;

  return pState->y_history[0];
}

/**
 * @brief  Write a byte to LIS3DSH register via SPI
 * @param  reg_addr: Register address to write to
 * @param  value: Data byte to write
 * @retval None
 */
static void LIS3DSH_Write(uint8_t reg_addr, uint8_t value)
{
  uint8_t tx_data[2];
  tx_data[0] = reg_addr & 0x7F; // Ensure bit 7 = 0 for write operation
  tx_data[1] = value;

  HAL_GPIO_WritePin(MEMS_CS_GPIO_Port, MEMS_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, tx_data, 2, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(MEMS_CS_GPIO_Port, MEMS_CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief  Read a byte from LIS3DSH register via SPI
 * @param  reg_addr: Register address to read from
 * @retval Register value (uint8_t)
 */
static uint8_t LIS3DSH_Read_Reg(uint8_t reg_addr)
{
  uint8_t tx_buf[2] = {0};
  uint8_t rx_buf[2] = {0};
  
  tx_buf[0] = reg_addr | 0x80; // Set bit 7 for read operation
  tx_buf[1] = 0x00;
  
  HAL_GPIO_WritePin(MEMS_CS_GPIO_Port, MEMS_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 2, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(MEMS_CS_GPIO_Port, MEMS_CS_Pin, GPIO_PIN_SET);
  
  return rx_buf[1];
}

/**
 * @brief  Initialize LIS3DSH Accelerometer Sensor
 * @details Verifies device ID and configures sensor for continuous operation:
 *          - Data rate: 100 Hz
 *          - Full scale: ±2g
 *          - All axes (X, Y, Z) enabled
 * @retval 1 if initialization successful, 0 if failed
 */
static uint8_t LIS3DSH_Init(void)
{
  uint8_t tx_buf[2] = { 0 };
  uint8_t rx_buf[2] = { 0 };
  uint8_t chip_id;

  // Step 1: Read and verify WHO_AM_I register
  tx_buf[0] = LIS3DSH_WHO_AM_I | 0x80; // Set bit 7 for read operation
  tx_buf[1] = 0x00;

  HAL_GPIO_WritePin(MEMS_CS_GPIO_Port, MEMS_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 2, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(MEMS_CS_GPIO_Port, MEMS_CS_Pin, GPIO_PIN_SET);

  chip_id = rx_buf[1];

  // Verify device ID (LIS3DSH = 0x3F, LIS302DL = 0x3B, LIS3DH = 0x32/0x33)
  if (chip_id != 0x3F && chip_id != 0x3B && chip_id != 0x32 && chip_id != 0x33)
  {
    return 0; // Initialization failed
  }

  // Step 2: Configure sensor registers (order is important!)
  HAL_Delay(10);

  // Configure full-scale range (±2g)
  LIS3DSH_Write(LIS3DSH_CTRL_REG5, 0x00);
  HAL_Delay(10);

  // Disable interrupts
  LIS3DSH_Write(LIS3DSH_CTRL_REG3, 0x00);
  HAL_Delay(10);

  // Disable FIFO
  LIS3DSH_Write(LIS3DSH_CTRL_REG6, 0x00);
  HAL_Delay(10);

  // Activate sensor: 100Hz ODR, XYZ axes enabled
  LIS3DSH_Write(LIS3DSH_CTRL_REG4, LIS3DSH_ODR_100HZ);
  HAL_Delay(100); // Wait for sensor to stabilize and start producing data

  return 1; // Initialization successful
}

/**
 * @brief  Read raw acceleration data from LIS3DSH sensor
 * @details Reads 6 bytes (X_L, X_H, Y_L, Y_H, Z_L, Z_H) via individual SPI transactions.
 *          Note: Multi-byte read mode (auto-increment) is not used due to hardware limitations.
 * @param  pX: Pointer to store X-axis raw value
 * @param  pY: Pointer to store Y-axis raw value
 * @param  pZ: Pointer to store Z-axis raw value
 * @retval None
 */
static void LIS3DSH_Read_Acc_Raw(int16_t *pX, int16_t *pY, int16_t *pZ)
{
  uint8_t tx_buf[2];
  uint8_t rx_buf[2];
  uint8_t data[6];

  // Read 6 consecutive registers individually (OUT_X_L through OUT_Z_H)
  for (uint8_t i = 0; i < 6; i++)
  {
    tx_buf[0] = (LIS3DSH_OUT_X_L + i) | 0x80; // Set bit 7 for read
    tx_buf[1] = 0x00;

    HAL_GPIO_WritePin(MEMS_CS_GPIO_Port, MEMS_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(MEMS_CS_GPIO_Port, MEMS_CS_Pin, GPIO_PIN_SET);

    data[i] = rx_buf[1];
  }

  // Assemble 16-bit signed values from LSB and MSB
  *pX = (int16_t) ((data[1] << 8) | data[0]);
  *pY = (int16_t) ((data[3] << 8) | data[2]);
  *pZ = (int16_t) ((data[5] << 8) | data[4]);
}

/**
 * @brief  Benchmarks the filter performance.
 * Measures average execution time over 1000 iterations.
 * Sends the CPU cycles used for filtering via UART.
 */
void Run_Performance_Test(void)
{
  float test_input = 10.0f;
  float test_output;
  char msg[128];
  uint32_t sum_cycles = 0;
  const uint16_t iterations = 1000;
  
  // Verify DWT counter is running
  uint32_t test_start = DWT->CYCCNT;
  for (volatile int i = 0; i < 100; i++);  // Small delay
  uint32_t test_end = DWT->CYCCNT;
  
  if (test_end == test_start)
  {
    // DWT not working, send error message
    sprintf(msg, "PERF,%d,ERROR,DWT_NOT_ENABLED\r\n", current_filter_order);
    HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 100);
    return;
  }

  // Run multiple iterations for accurate measurement
  for (uint16_t i = 0; i < iterations; i++)
  {
    // Start cycle counter (read current value)
    start_cycles = DWT->CYCCNT;

    // Apply filter
    test_output = Apply_IIR_Filter(test_input, &pitch_filter);

    // End cycle counter
    end_cycles = DWT->CYCCNT;
    sum_cycles += (end_cycles - start_cycles);
  }

  uint32_t avg_cycles = sum_cycles / iterations;
  float execution_time_us = (float)avg_cycles / 168.0f; // At 168MHz

  // Send result: "PERF,<order>,<cycles>,<time_us>"
  sprintf(msg, "PERF,%d,%lu,%.2f\r\n", current_filter_order, avg_cycles, execution_time_us);
  HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 100);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  DWT_Init();  // Enable Cycle Counter for benchmarking

  // Initialize CS pin to high (idle state)
  HAL_GPIO_WritePin(MEMS_CS_GPIO_Port, MEMS_CS_Pin, GPIO_PIN_SET);
  HAL_Delay(100); // Power-up delay for sensor stabilization

  // Initialize LIS3DSH accelerometer
  if (LIS3DSH_Init() == 0)
  {
    while (1) // Initialization failed - blink red LED indefinitely
    {
      HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
      HAL_Delay(250);
    }
  }

  // Initialization successful - flash green LED
  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);

  // Run performance benchmark and send results via UART
  Run_Performance_Test();
  HAL_Delay(100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  uint32_t last_tick = HAL_GetTick();  // Initialize timing for precise 100Hz sampling
  
  while (1)
  {
    // Step 0: Check for button press to change filter order
    Check_Button_Press();

    // Step 1: Wait for new sensor data (check STATUS register for data ready)
    uint8_t status = 0;
    do {
      status = LIS3DSH_Read_Reg(LIS3DSH_STATUS);
    } while (!(status & 0x08));  // Wait until ZYXDA (data ready) bit is set
    
    // Step 2: Read raw accelerometer data from sensor (guaranteed fresh data)
    LIS3DSH_Read_Acc_Raw(&raw_acc_x, &raw_acc_y, &raw_acc_z);

    // Step 3: Convert raw values to acceleration in g units
    // LIS3DSH sensitivity: 0.06 mg/LSB at ±2g range -> multiply by 0.001 to get g
    acc_x_g = (float) raw_acc_x * (LIS3DSH_SENSITIVITY * 0.001f);
    acc_y_g = (float) raw_acc_y * (LIS3DSH_SENSITIVITY * 0.001f);
    acc_z_g = (float) raw_acc_z * (LIS3DSH_SENSITIVITY * 0.001f);

    // Step 4: Calculate Pitch and Roll angles from accelerometer data
    // Pitch (rotation around Y-axis): arctan2(X, Z)
    // Roll (rotation around X-axis): arctan2(Y, Z)
    // Note: Yaw cannot be calculated with accelerometer only (requires gyroscope/magnetometer)
    
    pitch_raw = atan2f(acc_x_g, acc_z_g) * RAD_TO_DEG;
    roll_raw = atan2f(acc_y_g, acc_z_g) * RAD_TO_DEG;

    // Step 5: Apply IIR low-pass filter to reduce noise (separate state for each axis)
    pitch_filtered = Apply_IIR_Filter(pitch_raw, &pitch_filter);
    roll_filtered = Apply_IIR_Filter(roll_raw, &roll_filter);

    // Step 6: Transmit data via UART for visualization (non-blocking with timeout)
    // Protocol: "S,<order>,<pitch_raw>,<pitch_filt>,<roll_raw>,<roll_filt>,E\r\n"
    int len = sprintf(uart_tx_buffer, "S,%d,%.2f,%.2f,%.2f,%.2f,E\r\n",
                      current_filter_order, pitch_raw, pitch_filtered, roll_raw, roll_filtered);

    HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buffer, len, 5);  // 5ms timeout instead of blocking

    // Step 7: Maintain precise 100Hz sampling rate (10ms period)
    // Wait until exactly 10ms has passed since last iteration
    while ((HAL_GetTick() - last_tick) < 10);
    last_tick = HAL_GetTick();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MEMS_CS_GPIO_Port, MEMS_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : MEMS_CS_Pin */
  GPIO_InitStruct.Pin = MEMS_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(MEMS_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add their own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
