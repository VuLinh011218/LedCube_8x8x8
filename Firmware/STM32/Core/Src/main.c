/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "stdlib.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define LATCH_GPIO_Port   GPIOA
#define Latch_Pin         GPIO_PIN_4

// ====== TUỲ CHỌN DÂY DỢ ======
#define INVERT_BITS          0   // 1 nếu cột là active-low (0 = sáng)
#define REVERSE_BIT_ORDER    1   // 1 nếu cột 0 đang đi vào Q7 (MSB<->LSB)
#define REVERSE_BYTE_ORDER   0   // 1 nếu byte[0] đang rơi vào 74HC595 cuối

#define LAYER_ACTIVE_HIGH    1   // 1 nếu GPIO=SET => bật tầng; 0 nếu ngược lại

static uint8_t cube[8][8];   // [z][y], mỗi byte chứa 8 bit cho x
static uint8_t scan_z = 0;

static uint32_t lastRainTick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static void sendTo74HC595(uint8_t const *data8);
static void disable_all_layers(void);
static void enable_layer(uint8_t z);
static void cube_clear(void);
static void rain_effect_update(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Đảo bit trong 1 byte (MSB<->LSB)
static inline uint8_t reverse_bits(uint8_t b) {
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    return b;
}

// Chuẩn hoá 1 byte hàng trước khi gửi ra 74HC595
static inline uint8_t normalize_row(uint8_t b) {
    if (REVERSE_BIT_ORDER) b = reverse_bits(b); // trái-phải
    if (INVERT_BITS)       b = (uint8_t)~b;     // 0<->1 (active-low)
    return b;
}

// Gửi 8 byte (8 hàng) cho 1 tầng
static void sendTo74HC595(uint8_t const *data8)
{
    uint8_t buf[8];
    for (int i = 0; i < 8; i++) {
        uint8_t b = data8[i];
        buf[i] = normalize_row(b);
    }
    if (REVERSE_BYTE_ORDER) {
        // đảo thứ tự 8 byte (y: 0..7 -> 7..0)
        for (int i = 0; i < 4; i++) {
            uint8_t t = buf[i]; buf[i] = buf[7-i]; buf[7-i] = t;
        }
    }
    HAL_GPIO_WritePin(LATCH_GPIO_Port, Latch_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, buf, 8, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(LATCH_GPIO_Port, Latch_Pin, GPIO_PIN_SET);
}

// ========== Tắt tất cả tầng ==========
static void disable_all_layers(void)
{
    // RESET = 0, SET = 1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, LAYER_ACTIVE_HIGH ? GPIO_PIN_RESET : GPIO_PIN_SET);
    for (int i = 3; i <= 9; i++) {
        HAL_GPIO_WritePin(GPIOB, (uint16_t)(1u << i), LAYER_ACTIVE_HIGH ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
}


// ========== Bật 1 tầng ==========
static void enable_layer(uint8_t z)
{
    GPIO_TypeDef *port = NULL; uint16_t pin = 0;
    switch (z) {
        case 0: port = GPIOA; pin = GPIO_PIN_15; break;
        case 1: port = GPIOB; pin = GPIO_PIN_3;  break;
        case 2: port = GPIOB; pin = GPIO_PIN_4;  break;
        case 3: port = GPIOB; pin = GPIO_PIN_5;  break;
        case 4: port = GPIOB; pin = GPIO_PIN_6;  break;
        case 5: port = GPIOB; pin = GPIO_PIN_7;  break;
        case 6: port = GPIOB; pin = GPIO_PIN_8;  break;
        case 7: port = GPIOB; pin = GPIO_PIN_9;  break;
        default: return;
    }
    HAL_GPIO_WritePin(port, pin, LAYER_ACTIVE_HIGH ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// ========== Xoá khối ==========
static void cube_clear(void)
{
    for (int z = 0; z < 8; z++)
        for (int y = 0; y < 8; y++)
            cube[z][y] = 0x00;
}

// ========== Cập nhật mưa ==========
// Ý tưởng: dịch các tầng xuống (7->6->...->0), xoá tầng 7, sinh vài hạt mới
//static void rain_effect_update(void)
//{
//    // Dịch xuống
//    for (int z = 0; z < 7; z++) {
//        for (int y = 0; y < 8; y++) {
//            cube[z][y] = cube[z+1][y];
//        }
//    }
//    // Xoá tầng 7
//    for (int y = 0; y < 8; y++) cube[7][y] = 0x00;
//
//    // Sinh hạt mới (tuỳ chỉnh số lượng/prob)
//    // Cách 1: số hạt cố định mỗi frame
//    // for (int i = 0; i < 2; i++) { uint8_t x = rand()%8, y = rand()%8; cube[7][y] |= (1u<<x); }
//
//    // Cách 2: xác suất – đều và “tơi” hơn
//    const float spawnProb = 0.20f;  // 20% mỗi vị trí
//    for (int y = 0; y < 8; y++) {
//        for (int x = 0; x < 8; x++) {
//            if ((rand() & 0xFFFF) < (uint16_t)(spawnProb * 65535.0f)) {
//                cube[7][y] |= (1u << x);
//            }
//        }
//    }
//}

// ========== Cập nhật mưa: spawn tại z=0, dịch lên z lớn ==========
static void rain_effect_update(void)
{
    // Dịch các lớp "lên trên": (z=7..1) lấy dữ liệu từ (z-1)
    for (int z = 7; z > 0; z--) {
        for (int y = 0; y < 8; y++) {
            cube[z][y] = cube[z-1][y];
        }
    }

    // Xoá tầng gốc (z=0) trước khi sinh hạt mới
    for (int y = 0; y < 8; y++) cube[0][y] = 0x00;

    // Sinh hạt mưa mới tại z=0
    const float spawnProb = 0.20f;  // 20% xác suất mỗi ô
    for (int y = 0; y < 8; y++) {
        for (int x = 0; x < 8; x++) {
            if ((rand() & 0xFFFF) < (uint16_t)(spawnProb * 65535.0f)) {
                cube[0][y] |= (1u << x);
            }
        }
    }
}

// ========== Timer IRQ: quét tầng ==========
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        disable_all_layers();
        sendTo74HC595(cube[scan_z]);
        enable_layer(scan_z);
        scan_z = (uint8_t)((scan_z + 1) & 0x07);
    }
}

void Cube_AllOff_Immediate(void)
{
  __disable_irq();

  // 1) Dừng quét để ngắt không bật tầng nữa
  HAL_TIM_Base_Stop_IT(&htim2);

  // 2) Tắt toàn bộ tầng (chú ý mức active của phần cứng bạn)
  disable_all_layers();   // với bạn: PA15, PB3..PB9 = RESET là tắt

  // 3) Ghi 0 vào chuỗi 74HC595 và chốt LATCH
  uint8_t zeros[8] = {0};
  HAL_GPIO_WritePin(LATCH_GPIO_Port, Latch_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, zeros, 8, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(LATCH_GPIO_Port, Latch_Pin, GPIO_PIN_SET);

  // 4) Xoá buffer phần mềm để khi bật lại cũng không sáng
  cube_clear();

  // 5) (Tuỳ chọn) Nếu có nối /OE và /SRCLR thì chặn luôn đầu ra
  #ifdef CUBE_OE_Pin
    // /OE mức HIGH = tắt (tri-state)
    HAL_GPIO_WritePin(CUBE_OE_GPIO_Port, CUBE_OE_Pin, GPIO_PIN_SET);
  #endif
  #ifdef CUBE_SRCLR_Pin
    // /SRCLR mức LOW = xoá, sau đó trả lại HIGH để sẵn sàng
    HAL_GPIO_WritePin(CUBE_SRCLR_GPIO_Port, CUBE_SRCLR_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CUBE_SRCLR_GPIO_Port, CUBE_SRCLR_Pin, GPIO_PIN_SET);
  #endif

  __enable_irq();
}

// Xoá 1 mặt phẳng z
static inline void plane_clear(uint8_t z) {
  for (int y=0; y<8; y++) cube[z][y] = 0x00;
}

// State cho hiệu ứng
static uint8_t  colIdx = 0;         // 0..63
static uint32_t lastStepTick = 0;

// Gọi lặp trong while(1): chạy trên 1 tầng z
void effect_columns_layer_tick(uint8_t z, uint32_t step_ms)
{
  uint32_t now = HAL_GetTick();
  if (now - lastStepTick < step_ms) return;
  lastStepTick = now;

  // Tính y, x từ chỉ số cột 0..63
  uint8_t y = colIdx / 8;
  uint8_t x = colIdx % 8;

  // Vẽ
//  plane_clear(z);
  cube[z][y] = (uint8_t)(1u << x);

  // Next
  colIdx = (uint8_t)((colIdx + 1) & 0x3F); // %64
}

//static uint8_t  vColIdx = 0;         // 0..63
//static uint32_t vLastStep = 0;
//
//void effect_columns_vertical_tick(uint32_t step_ms)
//{
//  uint32_t now = HAL_GetTick();
//  if (now - vLastStep < step_ms) return;
//  vLastStep = now;
//
//  uint8_t y = vColIdx / 8;
//  uint8_t x = vColIdx % 8;
//
//  // Clear toàn khối rồi bật trục z ở (x,y)
//  cube_clear();
//  for (int z = 0; z < 8; z++) {
//    cube[z][y] |= (uint8_t)(1u << x);
//  }
//
//  vColIdx = (uint8_t)((vColIdx + 1) & 0x3F);
//}


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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  // Nếu dùng PB3/PB4 làm GPIO, tắt JTAG (giữ SWD):
    __HAL_AFIO_REMAP_SWJ_NOJTAG();

    // Seed random (đơn giản)
    srand( (unsigned)HAL_GetTick() ^ 0xA5A5u );

    cube_clear();

    // Bật timer quét tầng
    HAL_TIM_Base_Start_IT(&htim2);

//    // Nhịp cập nhật hiệu ứng
//    const uint32_t frame_ms = 80;  // 50–120ms tuỳ khẩu vị
//    lastRainTick = HAL_GetTick();

//    Cube_AllOff_Immediate();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  effect_columns_layer_tick(0, 100);   // chạy trên z=0, mỗi 100 ms sang 1 cột

//	  effect_columns_vertical_tick(120);

//	uint32_t now = HAL_GetTick();
//	if (now - lastRainTick >= frame_ms)
//		{
//			lastRainTick = now;
//	        rain_effect_update();
//	    }
//
////	       (tuỳ chọn) báo nhịp
//	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//	HAL_Delay(500);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Latch_GPIO_Port, Latch_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, R2_Pin|R3_Pin|R4_Pin|R5_Pin
                          |R6_Pin|R7_Pin|R8_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Latch_Pin */
  GPIO_InitStruct.Pin = Latch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Latch_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : R1_Pin */
  GPIO_InitStruct.Pin = R1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(R1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : R2_Pin R3_Pin R4_Pin R5_Pin
                           R6_Pin R7_Pin R8_Pin */
  GPIO_InitStruct.Pin = R2_Pin|R3_Pin|R4_Pin|R5_Pin
                          |R6_Pin|R7_Pin|R8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
  /* User can add his own implementation to report the HAL error return state */
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
