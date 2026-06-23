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
#include "math.h"
#include <stdio.h>
#include <stdbool.h>
#include "MY_NRF24.h"
#include "string.h"
#include "stdlib.h"   // cho atoi()

#define LED_ON()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET)
#define LED_OFF() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET)
#define RX_ADDRESS   0x0101010101LL
#define TX_ADDRESS   0x0202020202LL


#define PWM_UPDATE_HZ      20000U
#define SVM_TABLE_MAX    2500



#define INV_SECTOR_WIDTH   (TWO_PI / 6.0f)
#define PWM_PERIOD_COUNTS  (htim1.Init.Period + 1)
#define FREQ_MIN   4.0f
#define FREQ_MAX   50.0f
#define PI          3.14159265
#define PWM_FREQ    5000.0        // 5kHz
#define Ts          (1.0 / PWM_FREQ)
#define VDC         24         // DC bus voltage
#define TWO_PI      (2 * PI)
#define RX_BUFFER_SIZE 20
float theta = 0.0;

float freq_out;
float t = 0.0;
float Vs, m, alpha, T1, T2, T0;



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_TOTAL_SAMPLES        200      // số điểm cuối cùng để gửi
#define ADC_SAMPLES_PER_PACKET   8
#define ADC_TOTAL_PACKETS       25
#define FEEDBACK_PACKET_TYPE     0xA5

#define ADC_SAMPLE_RATE_HZ       5000U   // 200 mẫu / 40ms
#define ADC_CAPTURE_TIME_MS      40U     // 2 chu kì 50Hz

#define DUTY_MIN   0.05f
#define DUTY_MAX   0.95f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
// --- THÔNG SỐ ĐỘNG CƠ CẬP NHẬT MỚI ---

volatile float motor_cos_phi = 0.98f;     // Hệ số công suất
volatile float i_rms_set = 0.0f;          // Biến lưu kết quả dòng điện pha RMS đặt (Ampe)
// -------------------------------------
volatile float Vs_local;

volatile uint8_t tx_frame_active = 0;
volatile uint8_t tx_frame_done = 0;

volatile uint16_t svm_ccr1[SVM_TABLE_MAX];
volatile uint16_t svm_ccr2[SVM_TABLE_MAX];
volatile uint16_t svm_ccr3[SVM_TABLE_MAX];

volatile uint16_t svm_table_size = 100;
volatile uint16_t svm_index = 0;
volatile uint8_t  svm_enable = 0;
volatile uint8_t svm_need_rebuild = 0;
volatile uint32_t ramp_tick_div = 0;
volatile float last_freq_for_rebuild = 0.0f;

volatile bool ok;
volatile uint8_t adc_busy = 0;
volatile uint8_t adc_ready = 0;

#define ADC_CHANNELS 4
uint16_t adc_buffer[ADC_TOTAL_SAMPLES];
uint16_t adc_copy[ADC_TOTAL_SAMPLES];
volatile uint8_t adc_frame_id = 0;

volatile float V_F_K = 1.0f;
volatile float V_f_freq =  FREQ_MIN;
volatile float freq_cmd = FREQ_MIN;   // từ biến trở
volatile float freq_run = FREQ_MIN;   // dùng trong SVM
float target_freq = FREQ_MIN;
volatile bool status = 0.0f;
typedef struct __attribute__((packed))
{
    uint8_t type;          // FEEDBACK_PACKET_TYPE
    uint8_t state;         // on/off
    uint8_t frame_id;      // ID frame ADC
    uint8_t packet_id;     // 0..24
    uint16_t freq_set_x10; // Tần số đặt x10
    uint16_t freq_act_x10; // Tần số thực x10
    uint16_t rms_set_x10;  // Dòng đặt x10
    uint16_t rms_act_x10;  // Dòng thực / Điện áp Avg x10
    uint16_t min_adc;      // Đáy sóng (tính tại gốc)
    uint16_t max_adc;      // Đỉnh sóng (tính tại gốc)
    uint16_t samples[ADC_SAMPLES_PER_PACKET]; // Mảng 8 mẫu (16 bytes)
} FeedbackPacket_t;

// --- CÁC BIẾN CHO MÁY TRẠNG THÁI GỬI DỮ LIỆU ---
FeedbackPacket_t tx_packets_buffer[ADC_TOTAL_PACKETS];
volatile uint8_t packets_to_send = 0;
volatile uint8_t current_packet_idx = 0;
uint32_t last_tx_time = 0;
typedef struct {
     uint8_t cmd;
     uint8_t state;
     uint8_t speed;
     uint8_t wave_select; // Kênh sóng yêu cầu từ màn hình
 } Payload_t;

 Payload_t rx; // Biến lưu dữ liệu nhận được
 typedef struct {
     uint8_t on_off;
     uint8_t speed;
     uint8_t wave_select; // Lưu trạng thái kênh hiện tại
 } SystemState_t;

 SystemState_t system_state = {0,0,0};

 volatile uint8_t flag_new_data = 0;

	//Dieu che SVM
// void SVM_Update(void)
// {
//     // 1) Tính biên độ theo V/f
//     Vs = V_F_K * freq_run;
//     m  = Vs / VDC;
//
//     // Giới hạn modulation index
//     if (m > 0.866f) m = 0.866f;
//     if (m < 0.0f)   m = 0.0f;
//
//     // 2) Cập nhật góc điện
//     theta += TWO_PI * freq_run * Ts;
//
//     if (theta >= TWO_PI)
//         theta -= TWO_PI;
//     else if (theta < 0.0f)
//         theta += TWO_PI;
//
//     // 3) Xác định sector (1..6) và vị trí nội bộ trong sector
//     uint8_t sector = (uint8_t)(theta / INV_SECTOR_WIDTH);   // 0..5
//     float theta_sector = theta - ((float)sector * INV_SECTOR_WIDTH);
//     float u = theta_sector / INV_SECTOR_WIDTH;              // 0..1
//
//     // 4) Xấp xỉ nhẹ cho T1, T2 (bỏ sin/cos/sqrt/fmod)
//     //    Giữ nguyên tinh thần SVM: vector đầu giảm, vector sau tăng
//     float K = Ts * m;
//
//     T1 = K * (1.0f - u);
//     T2 = K * u;
//     T0 = Ts - T1 - T2;
//
//     if (T0 < 0.0f) T0 = 0.0f;
//
//     // 5) Tính duty cho 3 pha theo từng sector
//     float Ta, Tb, Tc;
//
//     switch (sector)
//     {
//         case 0: // sector 1
//             Ta = (T1 + T2 + T0 * 0.5f) / Ts;
//             Tb = (T2 + T0 * 0.5f) / Ts;
//             Tc = (T0 * 0.5f) / Ts;
//             break;
//
//         case 1: // sector 2
//             Ta = (T1 + T0 * 0.5f) / Ts;
//             Tb = (T1 + T2 + T0 * 0.5f) / Ts;
//             Tc = (T0 * 0.5f) / Ts;
//             break;
//
//         case 2: // sector 3
//             Ta = (T0 * 0.5f) / Ts;
//             Tb = (T1 + T2 + T0 * 0.5f) / Ts;
//             Tc = (T2 + T0 * 0.5f) / Ts;
//             break;
//
//         case 3: // sector 4
//             Ta = (T0 * 0.5f) / Ts;
//             Tb = (T1 + T0 * 0.5f) / Ts;
//             Tc = (T1 + T2 + T0 * 0.5f) / Ts;
//             break;
//
//         case 4: // sector 5
//             Ta = (T2 + T0 * 0.5f) / Ts;
//             Tb = (T0 * 0.5f) / Ts;
//             Tc = (T1 + T2 + T0 * 0.5f) / Ts;
//             break;
//
//         default: // sector 6
//             Ta = (T1 + T2 + T0 * 0.5f) / Ts;
//             Tb = (T0 * 0.5f) / Ts;
//             Tc = (T1 + T0 * 0.5f) / Ts;
//             break;
//     }
//
//     // 6) Chặn duty an toàn
//     if (Ta < 0.0f) Ta = 0.0f; if (Ta > 1.0f) Ta = 1.0f;
//     if (Tb < 0.0f) Tb = 0.0f; if (Tb > 1.0f) Tb = 1.0f;
//     if (Tc < 0.0f) Tc = 0.0f; if (Tc > 1.0f) Tc = 1.0f;
//
//     // 7) Ghi CCR
//     uint16_t ccr1 = (uint16_t)(Ta * PWM_PERIOD_COUNTS);
//     uint16_t ccr2 = (uint16_t)(Tb * PWM_PERIOD_COUNTS);
//     uint16_t ccr3 = (uint16_t)(Tc * PWM_PERIOD_COUNTS);
//
//     if (ccr1 > htim1.Init.Period) ccr1 = htim1.Init.Period;
//     if (ccr2 > htim1.Init.Period) ccr2 = htim1.Init.Period;
//     if (ccr3 > htim1.Init.Period) ccr3 = htim1.Init.Period;
//
//     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr1);
//     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr2);
//     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr3);
// }
 void Build_SVM_Table(float fout_hz)
 {
     if (fout_hz < 0.1f)
     {
         svm_enable = 0;
         svm_table_size = 1;
         svm_index = 0;
         svm_ccr1[0] = 0;
         svm_ccr2[0] = 0;
         svm_ccr3[0] = 0;
         return;
     }

     if (fout_hz > 50.0f) fout_hz = 50.0f;

     uint16_t N = 500;

     if (N < 12) N = 12;
     if (N > SVM_TABLE_MAX) N = SVM_TABLE_MAX;

     svm_table_size = N;
       Vs_local = V_F_K * fout_hz;
     float m_local  = Vs_local / VDC;

     if (m_local > 0.866f) m_local = 0.866f;  // Mở rộng không gian vector lên kịch trần
     if (m_local < 0.0f)   m_local = 0.0f;

     for (uint16_t i = 0; i < N; i++)
     {
         float theta_local = TWO_PI * ((float)i / (float)N);

         uint8_t sector = (uint8_t)(theta_local / INV_SECTOR_WIDTH); // 0..5
         if (sector > 5) sector = 5;

         float theta_sector = theta_local - ((float)sector * INV_SECTOR_WIDTH);
         float u = theta_sector / INV_SECTOR_WIDTH; // 0..1

         // SVM nhẹ tuyến tính theo sector


         // SVM chuẩn dùng sin → sóng mượt
         float T1n = 1.73205f * m_local * sinf(INV_SECTOR_WIDTH - theta_sector);
                  float T2n = 1.73205f * m_local * sinf(theta_sector);

                  float sum_T = T1n + T2n;
                  float T0n;

                  // Nếu tổng thời gian vượt quá 1 chu kỳ PWM, thu nhỏ T1 và T2 theo đúng tỷ lệ
                  if (sum_T > 1.0f)
                  {
                      T1n = T1n / sum_T;
                      T2n = T2n / sum_T;
                      T0n = 0.0f;
                  }
                  else
                  {
                      T0n = 1.0f - sum_T;
                  }

         float Ta, Tb, Tc;

         switch (sector)
         {
             case 0:
                 Ta = T1n + T2n + T0n * 0.5f;
                 Tb = T2n + T0n * 0.5f;
                 Tc = T0n * 0.5f;
                 break;

             case 1:
                 Ta = T1n + T0n * 0.5f;
                 Tb = T1n + T2n + T0n * 0.5f;
                 Tc = T0n * 0.5f;
                 break;

             case 2:
                 Ta = T0n * 0.5f;
                 Tb = T1n + T2n + T0n * 0.5f;
                 Tc = T2n + T0n * 0.5f;
                 break;

             case 3:
                 Ta = T0n * 0.5f;
                 Tb = T1n + T0n * 0.5f;
                 Tc = T1n + T2n + T0n * 0.5f;
                 break;

             case 4:
                 Ta = T2n + T0n * 0.5f;
                 Tb = T0n * 0.5f;
                 Tc = T1n + T2n + T0n * 0.5f;
                 break;

             default:
                 Ta = T1n + T2n + T0n * 0.5f;
                 Tb = T0n * 0.5f;
                 Tc = T1n + T0n * 0.5f;
                 break;
         }

         // Giới hạn duty an toàn
         if (Ta < DUTY_MIN) Ta = DUTY_MIN;
         if (Ta > DUTY_MAX) Ta = DUTY_MAX;

         if (Tb < DUTY_MIN) Tb = DUTY_MIN;
         if (Tb > DUTY_MAX) Tb = DUTY_MAX;

         if (Tc < DUTY_MIN) Tc = DUTY_MIN;
         if (Tc > DUTY_MAX) Tc = DUTY_MAX;

         uint16_t ccr1 = (uint16_t)(Ta * htim1.Init.Period);
         uint16_t ccr2 = (uint16_t)(Tb * htim1.Init.Period);
         uint16_t ccr3 = (uint16_t)(Tc * htim1.Init.Period);

         if (ccr1 > htim1.Init.Period) ccr1 = htim1.Init.Period;
         if (ccr2 > htim1.Init.Period) ccr2 = htim1.Init.Period;
         if (ccr3 > htim1.Init.Period) ccr3 = htim1.Init.Period;

         svm_ccr1[i] = ccr1;
         svm_ccr2[i] = ccr2;
         svm_ccr3[i] = ccr3;
     }

     svm_enable = 1;
     // --- THÊM CODE TÍNH DÒNG ĐIỆN RMS ĐẶT ---
          // Vs_local đang là điện áp pha đỉnh (Peak Phase Voltage)
          // 1. Tính điện áp pha RMS = Peak / căn(2)
          float v_phase_rms_set = Vs_local / 1.4142135f;

          // 1. Khai báo thông số tải thực tế đang cắm

              float motor_cos_phi = 0.98f;     // Hệ số công suất 0.98


 }
void VF_Frequency_Ramp(float target_freq);
	//Cap nhat SVM
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        // =====================================================
        // 1) Xuất duty PWM từ bảng SVM (cực nhẹ, cực nhanh)
        // =====================================================
        // CHỈ XUẤT XUNG KHI HỆ THỐNG ĐANG BẬT (system_state.on_off == 1)
        if (svm_enable && svm_table_size > 0 && system_state.on_off == 1)
        {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, svm_ccr1[svm_index]);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, svm_ccr2[svm_index]);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, svm_ccr3[svm_index]);

            static float svm_phase = 0.0f;

            svm_phase += freq_run / PWM_UPDATE_HZ * svm_table_size;

            if (svm_phase >= svm_table_size)
                svm_phase -= svm_table_size;

            svm_index = (uint16_t)svm_phase;
        }
        else
        {
            // NẾU TẮT: Ép Duty Cycle về 0
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        }

        // =====================================================
        // 2) Ramp tần số ngay trong interrupt nhưng CHỈ RẤT NHẸ
        //    TIM1 đang ~5kHz => 5 interrupt ~ 1ms
        // =====================================================
        ramp_tick_div++;
        if (ramp_tick_div >= 5)
        {
            ramp_tick_div = 0;

            VF_Frequency_Ramp(target_freq);
            freq_run = V_f_freq;

            if (fabsf(freq_run - last_freq_for_rebuild) >= 0.1f)
            {
                svm_need_rebuild = 1;
                last_freq_for_rebuild = freq_run;
            }
        }
    }
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
static bool NRF24_isInitialized(void);
void Task_RF_Receive(void);
void Process_ADC_And_RF_Transmit(void);
uint16_t Read_ADC(void);
static void NRF24_Recovery(void);
void Task_RF(void);
void Task_SVM_Background(void);
void Start_ADC_Capture_200Samples(void);
void Calculate_Actual_Params(uint16_t *buffer, float f_set, float *out_freq_act, float *out_val, uint8_t sel);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static bool NRF24_isInitialized(void)
{
  // Kiểm tra thanh ghi SETUP_AW (1..3 ứng với 3..5 bytes address)
  uint8_t setup = NRF24_read_register(REG_SETUP_AW);
  return (setup >= 1 && setup <= 3);
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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_SPI2_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
NRF24_begin(GPIOB, GPIO_PIN_6, GPIO_PIN_5, hspi2);// khởi tạo nrf CS ,CE
    // Nếu init ok -> bật LED 100ms để báo ready
     if (NRF24_isInitialized())
     {
  	   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

     }
     else

     {
    	 while(1)
    	     	   {


    		 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    	     		           HAL_Delay(100); // Nháy nhanh 100ms
    	     	   }

     }
  //Code PFC
  HAL_Delay(500);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
  HAL_Delay(500);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  //duty pwm for PFC IGBT
  float dutyPFC = 0.2;
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, dutyPFC * (htim2.Init.Period + 1) -1);

  //Code Inverter
  //Turn on timer1 for SVM
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0); // Duty 0% → low-side mở
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
  //nap tu bootstrap
  HAL_Delay(100);
  V_f_freq = FREQ_MIN;
  freq_run = FREQ_MIN;
  V_F_K = 1.0f;
  Build_SVM_Table(freq_run);
  svm_enable = 1;
  HAL_TIM_Base_Start_IT(&htim1);	  // Start TIM1 interrupt for SVM update
       // Cấu hình khớp TX
       NRF24_setPALevel(RF24_PA_0dB);
       NRF24_setCRCLength(RF24_CRC_16);    // CRC 16-bit
       NRF24_setAutoAck(true);
       NRF24_setDataRate(RF24_250KBPS);
       NRF24_setChannel(115);               // kênh 76
       NRF24_enableDynamicPayloads();
       NRF24_openReadingPipe(1, RX_ADDRESS);
       NRF24_openWritingPipe(TX_ADDRESS);
       NRF24_flush_rx();
     // Vào chế độ lắng nghe
         NRF24_startListening();
         Start_ADC_Capture_200Samples();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  // 1) Nhận lệnh RF càng nhanh càng tốt
	      Task_RF_Receive();

	      // 2) Rebuild bảng SVM ở nền khi interrupt yêu cầu
	      Task_SVM_Background();

	      // 3) Gửi feedback ADC / RF


	    Process_ADC_And_RF_Transmit();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 1;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 15;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 399;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 63;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 199;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Relay_GPIO_Port, Relay_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CE_Pin|CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Relay_Pin */
  GPIO_InitStruct.Pin = Relay_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Relay_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CE_Pin CS_Pin */
  GPIO_InitStruct.Pin = CE_Pin|CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Task_RF_Receive(void)
{
    if (NRF24_available())
    {

        memset(&rx, 0, sizeof(rx));
        NRF24_read(&rx, sizeof(rx));

        if (rx.cmd == 1)

        {
            system_state.on_off = rx.state;
            system_state.speed  = rx.speed;

            // ĐỔI TỐC ĐỘ LẤY MẪU THEO YÊU CẦU SÓNG
            if (system_state.wave_select != rx.wave_select)
            {
                system_state.wave_select = rx.wave_select;
                float f_sample_base = freq_run;
                                if (f_sample_base < 0.1f) f_sample_base = 0.1f;

                                uint32_t new_arr = (uint32_t)(10000.0f / f_sample_base) - 1;
                                if (new_arr > 65535) new_arr = 65535;

                                __HAL_TIM_SET_AUTORELOAD(&htim3, new_arr);
            }

            flag_new_data = 1;

            if(system_state.on_off == 1)
            {
                 target_freq = FREQ_MIN + (system_state.speed / 100.0f) * (FREQ_MAX - FREQ_MIN);
                 __HAL_TIM_MOE_ENABLE(&htim1);
            }
            else
            {
                 target_freq = FREQ_MIN;
                 __HAL_TIM_MOE_DISABLE(&htim1);
            }
            freq_cmd = target_freq;
        }
    }
}
uint16_t Read_ADC(void)
{
    uint16_t value = 0;

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    value = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    return value;
}

void VF_Frequency_Ramp(float target_freq)
{
    // ===============================================================
    // 1. THÊM BỘ SỐ SÀN: TẠO "VÙNG ĐI CHẬM" ĐỂ CỨU ÁP Ở 10HZ
    // ===============================================================
    float current_freq_step;

    // Nếu tần số đang loanh quanh mốc yếu áp (8Hz đến 12Hz)
    // Phải đi siêu chậm để rotor bám kịp từ trường, chống vọt dòng
    if (V_f_freq >= 19.0f && V_f_freq <= 23.0f) {
        current_freq_step = 0.0005f;
    } else {
        current_freq_step = 0.0032f; // Ngoài vùng đó thì tăng tốc độ bình thường
    }

    /* 2. Giới hạn tần số mục tiêu */
    if(target_freq < FREQ_MIN) target_freq = FREQ_MIN;
    if(target_freq > FREQ_MAX) target_freq = FREQ_MAX;

    /* 3. CHỈ RAMP TẦN SỐ (Không dùng -= VF_STEP ở đây nữa) */
    if(V_f_freq < target_freq)
    {
        V_f_freq += current_freq_step;
        if(V_f_freq > target_freq) V_f_freq = target_freq;
    }
    else if(V_f_freq > target_freq)
    {
        V_f_freq -= current_freq_step;
        if(V_f_freq < target_freq) V_f_freq = target_freq;
    }

    // ===============================================================

    // ===============================================================
    if (V_f_freq <= 4.0f)
    {
        V_F_K = 0.65f;
    }
    else if (V_f_freq <= 20.0f)
    {
        // Công thức đường thẳng thay cho việc trừ dồn:
        // Đảm bảo 100% khi V_f_freq = 10.0 thì V_F_K = 0.456000
        V_F_K = 0.65f - 0.016875f * (V_f_freq - 4.0f);
    }
    else
    {
        // Quá 10Hz là chốt cứng ở 0.456 mãi mãi
        V_F_K = 0.38f;
    }

    /* 5. Chốt chặn an toàn cuối cùng */
    if(V_F_K > 0.65f) V_F_K = 0.65f;
    if(V_F_K < 0.38f) V_F_K = 0.38;
}
void Process_ADC_And_RF_Transmit(void)
{
    static uint8_t retry_count = 0;
    static uint8_t  fail_count       = 0;        // FIX: Đếm lỗi trong 1 frame
    static uint32_t frame_start_time = 0;        // FIX: Đồng hồ bắt đầu frame

    if (adc_ready == 1 && tx_frame_active == 0)
    {
        adc_ready = 0;
        tx_frame_active = 1;
        tx_frame_done = 0;
        fail_count     = 0;                         // FIX: Reset bộ đếm lỗi
        frame_start_time = HAL_GetTick();           // FIX: Ghi nhận thời điểm bắt đầu

        uint8_t sel = system_state.wave_select;
                if(sel > 2) sel = 0;

                // COPY TRỰC TIẾP: Vì mảng gốc giờ đã là 200 điểm của đúng sóng cần thiết
        memcpy(adc_copy, adc_buffer, sizeof(adc_buffer));
        // 1. BỘ LỌC ĐỒ THỊ (WAVEFORM FILTER) CHO I_CUON_CAM VÀ U_DC
        // 1. TÌM MIN/MAX CỦA ĐỒ THỊ NGAY TỪ ĐẦU (BẮT DÒNG ĐỈNH)
                // =======================================================
        // =======================================================
                // 1. LỌC GAI NHIỄU ĐỈNH (CHỐNG KÍCH HOẠT ẢO KHỐI BẢO VỆ DO GAI XUNG IGBT)
                // =======================================================
                uint16_t local_min = 4095;
                uint16_t local_max = 0;

                // Quét lọc trung bình trượt 3 điểm để san phẳng các gai nhọn lọt cuộn cảm
                for (int i = 1; i < ADC_TOTAL_SAMPLES - 1; i++)
                {
                    uint16_t filtered_sample = (adc_copy[i-1] + adc_copy[i] + adc_copy[i+1]) / 3;
                    if (filtered_sample < local_min) local_min = filtered_sample;
                    if (filtered_sample > local_max) local_max = filtered_sample;
                }
                if (local_max == local_min) local_max = local_min + 1; // Chống chia 0
                // =======================================================
                // 2. CHỐT CHẶN BẢO VỆ THEO DÒNG ĐỈNH (PEAK CURRENT) -10A -> 10A
                // =======================================================
                // Tính toán dòng điện đỉnh âm và đỉnh dương ra Ampe thực tế
                float v_min_pin = (float)local_min * 3.3f / 4095.0f;
                float v_max_pin = (float)local_max * 3.3f / 4095.0f;

                float peak_current_min = (v_min_pin - 1.5f) / (8.2f * 0.68f * 0.025f);
                float peak_current_max = (v_max_pin - 1.5f) / (8.2f * 0.68f * 0.025f);

                // NGƯỠNG NGẮT: Cài đặt ở 9.5A và -9.5A (An toàn tuyệt đối trong dải 10A)
                if (peak_current_max > 9.5f || peak_current_min < -9.5f)
                {
                    // 1. Khóa cứng ngõ ra PWM phần cứng ngay lập tức
                    __HAL_TIM_MOE_DISABLE(&htim1);
                    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
                    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
                    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

                    // 2. Chuyển trạng thái hệ thống về OFF
                    system_state.on_off = 0;
                    svm_enable = 0;

                    // 3. Reset tần số về mức an toàn ban đầu
                    target_freq = FREQ_MIN;
                    freq_run = FREQ_MIN;
                    freq_cmd = FREQ_MIN;

                    // 4. Bật đèn LED báo hiệu lỗi Quá dòng
                    LED_ON();
                }

                // =======================================================
                // 3. TÍNH TOÁN CÁC THÔNG SỐ RMS VÀ TẦN SỐ ĐỂ HIỂN THỊ
                // =======================================================
                float act_freq = 0;
                float act_val = 0.0f;
                Calculate_Actual_Params(adc_copy, freq_run, &act_freq, &act_val, sel);
                // 1. Tính điện áp pha RMS hiện tại
                                float v_phase_rms = Vs_local / 1.4142135f;

                                // 2. Tính CÔNG SUẤT THỰC TẾ đang tiêu thụ (Watt)
                                float power_act = 3.0f * v_phase_rms * act_val * motor_cos_phi;
                // =======================================================
                // ĐÓNG GÓI SIÊU GỌN GÀNG VÀO 25 GÓI TIN
                // =======================================================
                for(uint8_t pkt_id = 0; pkt_id < ADC_TOTAL_PACKETS; pkt_id++)
                {
                    memset(&tx_packets_buffer[pkt_id], 0, sizeof(FeedbackPacket_t));

                    tx_packets_buffer[pkt_id].type          = FEEDBACK_PACKET_TYPE;
                    tx_packets_buffer[pkt_id].state         = system_state.on_off;
                    tx_packets_buffer[pkt_id].frame_id      = adc_frame_id;
                    tx_packets_buffer[pkt_id].packet_id     = pkt_id;

                    // Truyền đi chung 1 kiểu thông số
                                tx_packets_buffer[pkt_id].freq_set_x10  = (uint16_t)(target_freq * 10.0f);
                                tx_packets_buffer[pkt_id].freq_act_x10  = (uint16_t)(act_freq * 10.0f);
                                tx_packets_buffer[pkt_id].rms_set_x10   = 0;
                                tx_packets_buffer[pkt_id].rms_act_x10   = (uint16_t)(power_act * 10.0f);
                    // Nhét Min, Max vào để Remote xài
                    tx_packets_buffer[pkt_id].min_adc       = local_min;
                    tx_packets_buffer[pkt_id].max_adc       = local_max;

                    uint16_t start_index = pkt_id * ADC_SAMPLES_PER_PACKET;

                    // Chép 8 mẫu ADC vào gói
                    for(uint8_t i = 0; i < ADC_SAMPLES_PER_PACKET; i++)
                    {
                        tx_packets_buffer[pkt_id].samples[i] = adc_copy[start_index + i];
                    }
                }

                packets_to_send = ADC_TOTAL_PACKETS;
                current_packet_idx = 0;
                retry_count = 0;
                last_tx_time = HAL_GetTick();
            }
    // FIX: KIỂM TRA TIMEOUT TOÀN FRAME (1.5 giây)
       // Nếu gửi 1 frame mà mất hơn 1.5s → NRF24 bị kẹt, phục hồi ngay
    if (tx_frame_active && (HAL_GetTick() - frame_start_time > 1500))
        {
            NRF24_Recovery();           // Xả FIFO + xóa cờ lỗi + restart listening
            tx_frame_active    = 0;
            packets_to_send    = 0;
            retry_count        = 0;
            adc_busy           = 0;     // Giải phóng lock ADC phòng khi bị treo
            Start_ADC_Capture_200Samples();
            return;
        }
    if (tx_frame_active && packets_to_send > 0)
    {
        if (HAL_GetTick() - last_tx_time >=1)
        {
            NRF24_stopListening();
            ok = NRF24_write(&tx_packets_buffer[current_packet_idx], sizeof(FeedbackPacket_t));
            NRF24_startListening();

            if (ok) {
                current_packet_idx++;
                packets_to_send--;
                retry_count = 0;
            } else {
                retry_count++;
                fail_count++;
                if (retry_count >= 3) {
                	NRF24_stopListening();
                	NRF24_flush_tx();                   // Xả packet lỗi khỏi FIFO
                    NRF24_write_register(0x07, 0x70);  // Xóa MAX_RT + TX_DS + RX_DR
                    NRF24_startListening();

                    current_packet_idx++;
                    packets_to_send--;
                    retry_count = 0;
                }


            }
            last_tx_time = HAL_GetTick();
        }
    }

    if (tx_frame_active && packets_to_send == 0)
    {
        tx_frame_active = 0;
        tx_frame_done = 1;
        Start_ADC_Capture_200Samples();
    }
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1)
    {
        HAL_TIM_Base_Stop(&htim3);      // dừng trigger timer
        HAL_ADC_Stop_DMA(&hadc1);       // dừng DMA ADC

        adc_busy = 0;
        adc_ready = 1;
        adc_frame_id++;

        tx_frame_done = 0;
    }
}
void Task_SVM_Background(void)
{
    if (svm_need_rebuild)
    {
        svm_need_rebuild = 0;
        Build_SVM_Table(freq_run);
        // ==========================================================
                // CẬP NHẬT TIMER 3 ĐỂ ĐỒNG BỘ TỐC ĐỘ LẤY MẪU ADC
                // Mục tiêu: 200 điểm ADC = 2 chu kỳ sóng ngõ ra
                // ==========================================================
        if (system_state.wave_select == 0)
        {
               float f_out = freq_run;

                // 1. Chống lỗi chia cho 0 hoặc tần số quá nhỏ
                if (f_out < 0.1f) f_out = 0.1f;

                // 2. Tính toán giá trị thanh ghi ARR mới
                // F_timer_in = 64MHz. Prescaler = 63 -> F_count = 1MHz.
                // F_sample cần = 100 * f_out.
                // ARR = (1,000,000 / (100 * f_out)) - 1 = (10000 / f_out) - 1
                uint32_t new_arr = (uint32_t)(10000.0f / f_out) - 1;

                // 3. Bảo vệ tràn bộ đệm (Timer 3 là 16-bit, max 65535)
                // Với dải 4Hz -> 50Hz, ARR dao động từ 2499 -> 199 nên rất an toàn
                if (new_arr > 65535) new_arr = 65535;

                // 4. Ghi trực tiếp vào thanh ghi ARR của Timer 3 (Macro của HAL)
                __HAL_TIM_SET_AUTORELOAD(&htim3, new_arr);
            }
        }
    }

void Start_ADC_Capture_200Samples(void)
{
    if (adc_busy) return;

    adc_busy = 1;
    adc_ready = 0;

    memset(adc_buffer, 0, sizeof(adc_buffer));

    HAL_TIM_Base_Stop(&htim3);
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    HAL_ADC_Stop_DMA(&hadc1);
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC);

    // =======================================================
    // CHỌN KÊNH ĐỘNG: Lái ADC tới đúng chân cảm biến đang cần
    // =======================================================
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;

    switch (system_state.wave_select)
    {
            case 0: sConfig.Channel = ADC_CHANNEL_1; break; // Pha A
            case 1: sConfig.Channel = ADC_CHANNEL_2; break; // Pha B
            case 2: sConfig.Channel = ADC_CHANNEL_5; break; // Pha C
            default: sConfig.Channel = ADC_CHANNEL_1; break;
    }

    // Nạp cấu hình kênh mới vào phần cứng
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    // =======================================================

    // Kích hoạt DMA, bây giờ nó chỉ lấy đúng 200 điểm của kênh vừa chọn
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_TOTAL_SAMPLES);
    HAL_TIM_Base_Start(&htim3);
}
void Calculate_Actual_Params(uint16_t *buffer, float f_set, float *out_freq_act, float *out_val, uint8_t sel)
{
    uint32_t sum_adc = 0;
    for (int i = 0; i < ADC_TOTAL_SAMPLES; i++) sum_adc += buffer[i];

    float dc_offset_adc = (float)sum_adc / (float)ADC_TOTAL_SAMPLES;

    float sum_sq = 0.0f;
    for (int i = 0; i < ADC_TOTAL_SAMPLES; i++)
    {
        float diff = (float)buffer[i] - dc_offset_adc;
        sum_sq += (diff * diff);
    }
    float rms_adc = sqrtf(sum_sq / (float)ADC_TOTAL_SAMPLES);

    // Tính thô ra Volt tại chân vi điều khiển
    float rms_volts = (rms_adc * 3.3f) / 4095.0f;       // RMS của thành phần xoay chiều (AC)
    float avg_volts = (dc_offset_adc * 3.3f) / 4095.0f; // Giá trị trung bình DC

    // Cả 3 pha dùng chung 1 công thức vi sai phần cứng
        *out_val = rms_volts / (8.2f * 0.68f * 0.025f);

        // =======================================================
        // TÍNH TẦN SỐ THỰC TẾ BẰNG THUẬT TOÁN ZERO-CROSSING
        // =======================================================
      static  float smooth_adc[ADC_TOTAL_SAMPLES];
                smooth_adc[0] = (float)buffer[0];
                smooth_adc[1] = (float)buffer[1];
                smooth_adc[ADC_TOTAL_SAMPLES - 2] = (float)buffer[ADC_TOTAL_SAMPLES - 2];
                smooth_adc[ADC_TOTAL_SAMPLES - 1] = (float)buffer[ADC_TOTAL_SAMPLES - 1];

                // Lọc mượt 5 điểm
                for (int i = 2; i < ADC_TOTAL_SAMPLES - 2; i++) {
                    smooth_adc[i] = ((float)buffer[i-2] + (float)buffer[i-1] + (float)buffer[i] +
                                     (float)buffer[i+1] + (float)buffer[i+2]) / 5.0f;
                }

                // Tạo 2 vạch Hysteresis (tránh bậc thang)
                float peak_amplitude = rms_adc * 1.414f;
                float hysteresis = peak_amplitude * 0.20f;
                float thresh_high = dc_offset_adc + hysteresis;
                float thresh_low  = dc_offset_adc - hysteresis;

                float first_cross = -1.0f;
                float last_cross = -1.0f;
                int cycles = 0;
                uint8_t current_state = (smooth_adc[0] > dc_offset_adc) ? 1 : 0;

                for (int i = 1; i < ADC_TOTAL_SAMPLES; i++)
                {
                    if (current_state == 0 && smooth_adc[i] > thresh_high)
                    {
                        current_state = 1;
                        float v1 = smooth_adc[i-1];
                        float v2 = smooth_adc[i];
                        if (v2 != v1)
                        {
                            float cross_idx = (i - 1) + (thresh_high - v1) / (v2 - v1);
                            if (first_cross < 0.0f) first_cross = cross_idx;
                            last_cross = cross_idx;
                            cycles++;
                        }
                    }
                    else if (current_state == 1 && smooth_adc[i] < thresh_low)
                    {
                        current_state = 0;
                    }
                }

                if (cycles >= 2 && first_cross >= 0.0f) {
                    float points_per_cycle = (last_cross - first_cross) / (cycles - 1);
                    *out_freq_act = (100.0f * f_set) / points_per_cycle;

                    // Loại trừ rác đo đạc
                    if (*out_freq_act > 60.0f || *out_freq_act < 2.0f) {
                         *out_freq_act = f_set;
                    }
                } else {
                    *out_freq_act = f_set;
                }
        }
/* USER CODE BEGIN 4 */

// ============================================================
// HÀM PHỤC HỒI NRF24 KHẨN CẤP
// Gọi khi NRF24 bị stuck: xả FIFO + xóa cờ lỗi + restart
// ============================================================
static void NRF24_Recovery(void)
{
    NRF24_stopListening();

    // 1. Xả sạch cả TX FIFO (chứa packet lỗi) và RX FIFO
    NRF24_flush_tx();
    NRF24_flush_rx();

    // 2. Xóa tất cả cờ ngắt trong STATUS register (0x07)
    //    Bit 6=RX_DR, Bit 5=TX_DS, Bit 4=MAX_RT → ghi 1 để xóa
    //    0x70 = 0b0111_0000
    NRF24_write_register(0x07, 0x70);

    // 3. Chờ NRF24 ổn định
    HAL_Delay(5);

    // 4. Quay về chế độ lắng nghe
    NRF24_startListening();
}
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
