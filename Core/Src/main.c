/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "hrtim.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include "My_OLED.h"
#include "arm_math.h"
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

/* USER CODE BEGIN PV */

const float calc_phy_conv_voltage_0001=0.300f;
const float calc_phy_conv_current_0001=0.01015f;
const float calc_phy_conv_voltage_offset_0001=613.5f;
const float calc_phy_conv_current_offset_0001=20.75675f;
const float calc_phy_conv_voltage_0002=0.3035f;
const float calc_phy_conv_current_0002=0.0101f;
const float calc_phy_conv_voltage_offset_0002=620.6575f;
const float calc_phy_conv_current_offset_0002=20.6545f;
const float calc_phy_conv_voltage_0003=0.290f;
const float calc_phy_conv_current_0003=0.01020f;
const float calc_phy_conv_voltage_offset_0003=593.05f;
const float calc_phy_conv_current_offset_0003=20.859f;
const float calc_phy_conv_voltage_0004=0.297f;
const float calc_phy_conv_current_0004=0.01010f;
const float calc_phy_conv_voltage_offset_0004=607.365f;
const float calc_phy_conv_current_offset_0004=20.6545f;
const float calc_phy_conv_voltage_0005=0.3035f;
const float calc_phy_conv_current_0005=0.0100f;
const float calc_phy_conv_voltage_offset_0005=620.6575f;
const float calc_phy_conv_current_offset_0005=20.45f;

const float phy_calc_conv_offset=2045;

float hzc=5.0f;
float frequency=60.0f;
float num=200;
float fhrtim=160000000;
float div=7;

enum EC_DeBug now_EC_DeBug=vm_;

SV S={0};
QPR QPR1={0},QPR2={0};
SVPWM SVPWM1={0};

uint16_t ADC1_value[3],ADC2_value[3];

float va_setpoint,vb_setpoint;
float va_measurement,vb_measurement,vc_measurement;
float vdc_measurement;
float a,b,c,d,f;//中间值
float va_setpoint_data[200];
float vb_setpoint_data[200];
float den;



uint8_t TX_buffer[50];

// 根据你的需求定义最大可发送浮点数个数
#define VOFA_TX_MAX_FLOATS     (64u)

// JustFloat 协议的帧尾，固定为 0x7F800000 (表示 +Inf)[reference:8]
static const uint32_t vofa_tail = 0x7F800000u;

// 发送缓冲区：数据区 + 帧尾
static uint8_t vofa_tx_buf[VOFA_TX_MAX_FLOATS * 4 + 4];




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_UART4_Init();
  MX_SPI3_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_HRTIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);

  OLED_Init();

  SV_init(&S,26.12789f,frequency,num);
  QPR_Init(&QPR1,2*PI*hzc,frequency,num);
  QPR_Init(&QPR2,2*PI*hzc,frequency,num);
  QPR1.kp=0.3f,QPR1.kr=20.0f;
  QPR2.kp=0.3f,QPR2.kr=20.0f;
  SVPWM_init(&SVPWM1,60.0f,frequency,num,div,fhrtim);

  HAL_ADCEx_Calibration_Start(&hadc1,ADC_DIFFERENTIAL_ENDED);
  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC1_value,sizeof(ADC1_value)/sizeof(uint16_t));
  HAL_ADCEx_Calibration_Start(&hadc2,ADC_DIFFERENTIAL_ENDED);
  HAL_ADC_Start_DMA(&hadc2,(uint32_t*)ADC2_value,sizeof(ADC2_value)/sizeof(uint16_t));

  three_phase_inverter_interface_head();

  HAL_HRTIM_WaveformCountStart(&hhrtim1, HRTIM_TIMERID_TIMER_A);
  HAL_HRTIM_WaveformCountStart(&hhrtim1, HRTIM_TIMERID_TIMER_B);
  HAL_HRTIM_WaveformCountStart(&hhrtim1, HRTIM_TIMERID_TIMER_C);

  HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2);//开启通道输出
  HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TB1|HRTIM_OUTPUT_TB2);//开启通道输出
  HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TC1|HRTIM_OUTPUT_TC2);//开启通道输出

  get_va_setpoint_data(va_setpoint_data,num);
  get_vb_setpoint_data(vb_setpoint_data,num);

  __HAL_HRTIM_TIMER_ENABLE_IT(&hhrtim1,HRTIM_TIMERINDEX_TIMER_A,HRTIM_TIM_IT_UPD);//开启更新中断,开启QPR

  DeBug_interface_head();

  HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_4);
  HAL_TIM_Base_Stop(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    vc_measurement=ADC1_value[1]*calc_phy_conv_voltage_0003-calc_phy_conv_voltage_offset_0003;
    vdc_measurement=ADC2_value[1]*calc_phy_conv_voltage_0004-calc_phy_conv_voltage_offset_0004;

    three_phase_inverter_interface_main();
    DeBug_interface_main();


   // VOFA_SendFloatDMA(&huart4,(float[]){va_setpoint,vb_setpoint},2);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 42;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
inline void QPR_Init(QPR *QPR,float wc,float frequency,float num) {
  QPR->wc=wc;
  QPR->t_control=1.0f/frequency/num;
  QPR->w0=2.0*PI*frequency;
  QPR->k=2.0f/QPR->t_control;

   // QPR->w0 = QPR->k * tanf(QPR->w0 * QPR->t / 2.0f);   // 用畸变后的值参与系数计算
   // QPR->wc = QPR->k * tanf(QPR->wc * QPR->t / 2.0f);

  float A0=QPR->k*QPR->k+2.0f*QPR->wc*QPR->k+QPR->w0*QPR->w0;
  QPR->a1=2.0f*(QPR->w0*QPR->w0-QPR->k*QPR->k)/A0;
  QPR->a2=(QPR->k*QPR->k-2.0f*QPR->wc*QPR->k+QPR->w0*QPR->w0)/A0;
  QPR->b0=2.0f*QPR->k*QPR->wc/A0;
  QPR->b2=-QPR->b0;

  QPR->e1=0;
  QPR->e2=0;
  QPR->y0=0;
  QPR->y1=0;
  QPR->y2=0;
  QPR->u0=0;
  QPR->u1=0;
  QPR->q0=0;
  QPR->q1=0;
}

float QPR_Compute(QPR *QPR,float e0) {
  QPR->y0=QPR->b0*e0+QPR->b2*QPR->e2-QPR->a1*QPR->y1-QPR->a2*QPR->y2;
  QPR->u0=QPR->kp*e0+QPR->kr*QPR->y0;

  QPR->e2=QPR->e1;
  QPR->e1=e0;
  QPR->y2=QPR->y1;
  QPR->y1=QPR->y0;


  return QPR->u0;
}
void QPR_QSG_Compute(QPR *QPR,float e0) {
  QPR->y0=QPR->b0*(e0-QPR->e2)-QPR->a1*QPR->y1-QPR->a2*QPR->y2;
  QPR->u0=QPR->kp*e0+QPR->kr*QPR->y0;
  QPR->q0=QPR->q1+(QPR->u0+QPR->u1)*0.5f*QPR->w0*QPR->t_control;

  QPR->e2=QPR->e1;
  QPR->e1=e0;
  QPR->y2=QPR->y1;
  QPR->y1=QPR->y0;
  QPR->u1=QPR->u0;
  QPR->q1=QPR->q0;
}
float P_Compute(float p,float e0) {
  return (p*e0);
}

void SV_init(SV *S,float vm,float frequency,float num) {
  S->theta=0;
  S->vm=vm;
  S->t_control=1.0f/frequency/num;
  S->wout=2.0*PI*frequency;
}
int32_t rad_to_q31(float rad) {
  float norm_rad=rad/PI;

  int32_t result=(int32_t)(norm_rad*0x80000000);
  return result;
}
float sin_cos_q31_to_float(int32_t sin_cos_q31_value) {
  return (float)sin_cos_q31_value / 2147483648.0f;
}
// void phy_conv_calc(void) {
//   calc_va_setpoint=va_setpoint*phy_calc_conv_voltage_0001+phy_calc_conv_offset;
//   calc_vb_setpoint=vb_setpoint*phy_calc_conv_voltage_0002+phy_calc_conv_offset;
// }
void calc_conv_phy(void) {
  va_measurement=ADC1_value[0]*calc_phy_conv_voltage_0001-calc_phy_conv_voltage_offset_0001;
  vb_measurement=ADC2_value[0]*calc_phy_conv_voltage_0002-calc_phy_conv_voltage_offset_0002;

}
void SVPWM_init(SVPWM *SVPWM,float vdc,float frequency,float num,float div,float fhrtim) {
  SVPWM->vdc=vdc;
  SVPWM->Ts=1.0f/frequency/num/div;
  SVPWM->arr=fhrtim/2.0f/div/frequency/num;
  SVPWM->mid=SVPWM->Ts/SVPWM->vdc;
}
void Midvalue_Compute(float va_control,float vb_control) {
  a=va_control+2.0f*vb_control;
  b=va_control-vb_control;
  c=2.0f*va_control+vb_control;
  d=-va_control+vb_control;
}
void harmonic_insert(SVPWM *svpwm) {
  float min1,min2,max1,max2,sum;
  a = (svpwm->A_Phase + 1.0f) * 0.5f;
  b = (svpwm->B_Phase + 1.0f) * 0.5f;
  c = (svpwm->C_Phase + 1.0f) * 0.5f;
  max1 =  fmaxf(a,b);
  max2 =  fmaxf(max1,c);
  min1 =  fminf(a,b);
  min2 =  fminf(min1,c);
  sum  = (1.0f - max2 - min2) * 0.5f;
  svpwm->Vta = sum + a;
  svpwm->Vtb = sum + b;
  svpwm->Vtc = sum + c;
}
uint8_t Sector_Judgment(void) {
  uint8_t N=0;
  if (a>0){N+=1;}
  if (b>0){N+=2;}
  if (c<0){N+=4;}

  switch (N) {
    case 3:
      return 1;
    case 1:
      return 2;
    case 5:
      return 3;
    case 4:
      return 4;
    case 6:
      return 5;
    case 2:
      return 6;
  }
}
void vector_actiontime(SVPWM *SVPWM,uint8_t sector) {
  switch (sector) {
    case 1:
      SVPWM->T1=SVPWM->mid*b;
      SVPWM->T2=SVPWM->mid*a;
      break;
    case 2:
      SVPWM->T1=SVPWM->mid*d;
      SVPWM->T2=SVPWM->mid*c;
      break;
    case 3:
      SVPWM->T1=SVPWM->mid*a;
      SVPWM->T2=SVPWM->mid*(-c);
      break;
    case 4:
      SVPWM->T1=SVPWM->mid*(-a);
      SVPWM->T2=SVPWM->mid*d;
      break;
    case 5:
      SVPWM->T1=SVPWM->mid*(-c);
      SVPWM->T2=SVPWM->mid*b;
      break;
    case 6:
      SVPWM->T1=SVPWM->mid*c;
      SVPWM->T2=SVPWM->mid*(-a);
      break;
  }
  if ((SVPWM->T1+SVPWM->T2)>SVPWM->Ts) {
    SVPWM->T1=SVPWM->T1/(SVPWM->T1+SVPWM->T2)*SVPWM->Ts;
    SVPWM->T2=SVPWM->Ts-SVPWM->T1;
  }
  SVPWM->T0=SVPWM->Ts-SVPWM->T1-SVPWM->T2;
}
void frequency_conv(float fhrtim,float frequency,float div,float num) {
  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].PERxR = fhrtim/2.0f/div/frequency/num;
  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].PERxR = fhrtim/2.0f/div/frequency/num;
  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].PERxR = fhrtim/2.0f/div/frequency/num;
}
void vector_action(SVPWM *SVPWM,uint8_t sector) {
  switch (sector) {
    case 1:
      HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR=SVPWM->T0*0.25f/SVPWM->Ts*SVPWM->arr;
      HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR=HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR+SVPWM->T1*0.5f/SVPWM->Ts*SVPWM->arr;
      HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CMP1xR=HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR+SVPWM->T2*0.5f/SVPWM->Ts*SVPWM->arr+1;
      break;
    case 2:
      HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR=SVPWM->T0*0.25f/SVPWM->Ts*SVPWM->arr;
      HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR=HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR+SVPWM->T1*0.5f/SVPWM->Ts*SVPWM->arr;
      HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CMP1xR=HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR+SVPWM->T2*0.5f/SVPWM->Ts*SVPWM->arr+1;
      break;
    case 3:
      HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR=SVPWM->T0*0.25f/SVPWM->Ts*SVPWM->arr;
      HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CMP1xR=HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR+SVPWM->T1*0.5f/SVPWM->Ts*SVPWM->arr;
      HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR=HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CMP1xR+SVPWM->T2*0.5f/SVPWM->Ts*SVPWM->arr+1;
      break;
    case 4:
      HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CMP1xR=SVPWM->T0*0.25f/SVPWM->Ts*SVPWM->arr;
      HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR=HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CMP1xR+SVPWM->T1*0.5f/SVPWM->Ts*SVPWM->arr;
      HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR=HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR+SVPWM->T2*0.5f/SVPWM->Ts*SVPWM->arr+1;
      break;
    case 5:
      HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CMP1xR=SVPWM->T0*0.25f/SVPWM->Ts*SVPWM->arr;
      HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR=HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CMP1xR+SVPWM->T1*0.5f/SVPWM->Ts*SVPWM->arr;
      HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR=HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR+SVPWM->T2*0.5f/SVPWM->Ts*SVPWM->arr+1;
      break;
    case 6:
      HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR=SVPWM->T0*0.25f/SVPWM->Ts*SVPWM->arr;
      HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CMP1xR=HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR+SVPWM->T1*0.5f/SVPWM->Ts*SVPWM->arr;
      HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR=HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CMP1xR+SVPWM->T2*0.5f/SVPWM->Ts*SVPWM->arr+1;
      break;
  }
}
void VOFA_SendFloatDMA(UART_HandleTypeDef *huart, float *data, uint16_t num)
{
  // 1. 安全检查：如果上一次 DMA 传输还没完成，直接返回，避免冲突
  if (huart->gState != HAL_UART_STATE_READY) {
    return;
  }

  // 2. 防止数据溢出
  if (num > VOFA_TX_MAX_FLOATS) {
    num = VOFA_TX_MAX_FLOATS;
  }

  // 3. 计算数据部分的字节数
  uint16_t bytes_data = num * sizeof(float); // sizeof(float) 通常为 4

  // 4. 将浮点数据拷贝到发送缓冲区
  memcpy(vofa_tx_buf, data, bytes_data);

  // 5. 在数据后面追加 4 字节的帧尾
  memcpy(vofa_tx_buf + bytes_data, &vofa_tail, sizeof(vofa_tail));

  // 6. 启动 DMA 发送：发送 (数据 + 帧尾)
  HAL_UART_Transmit_DMA(huart, vofa_tx_buf, bytes_data + sizeof(vofa_tail));
}

void get_va_setpoint_data(float data[],int num) {
  for (int i=0;i<num;i++) {
    data[i] = S.vm*cos(2*i*PI/num);
  }
}
void get_vb_setpoint_data(float data[],int num) {
  for (int i=0;i<num;i++) {
    data[i] =S.vm*cos(2*i*PI/num-2.0/3.0*PI);
  }
}
void HAL_HRTIM_RegistersUpdateCallback(HRTIM_HandleTypeDef *hhrtim,uint32_t TimerIdx) {
  static uint8_t count = 0;
  static uint8_t count1 = 0;
  if (TimerIdx==HRTIM_TIMERINDEX_TIMER_A) {
    count++;
    if (count>252) {
      count=0;
    }
    if (count%7==0) {
      va_setpoint=va_setpoint_data[count1];
      vb_setpoint=vb_setpoint_data[count1];
      count1++;
      if (count1>=200) {
        count1=0;
      }
      //VOFA_SendFloatDMA(&huart4,(float[]){va_setpoint,vb_setpoint,count},3);

      calc_conv_phy();
      QPR_QSG_Compute(&QPR1,va_setpoint-va_measurement);
      QPR_QSG_Compute(&QPR2,vb_setpoint-vb_measurement);
      SVPWM1.A_Phase = QPR1.u0 / sqrtf(QPR1.u0*QPR1.u0 + QPR1.q0*QPR1.q0);
      SVPWM1.B_Phase = QPR2.u0 / sqrtf(QPR2.u0*QPR2.u0 + QPR2.q0*QPR2.q0);
      SVPWM1.C_Phase=-(SVPWM1.A_Phase+SVPWM1.B_Phase);

      harmonic_insert(&SVPWM1);

      VOFA_SendFloatDMA(&huart4,(float[]){SVPWM1.A_Phase,SVPWM1.B_Phase,SVPWM1.C_Phase},3);

      HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR=SVPWM1.Vta*SVPWM1.arr;
      HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR=SVPWM1.Vtb*SVPWM1.arr;
      HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CMP1xR=SVPWM1.Vtc*SVPWM1.arr;

      //VOFA_SendFloatDMA(&huart4,(float[]){SVPWM1.Vta,SVPWM1.Vtb,SVPWM1.Vtc},3);
      //VOFA_SendFloatDMA(&huart4,(float[]){count,count1,den},3);


    }
  }
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin==EC11D_Pin)
  {
    now_EC_DeBug=(now_EC_DeBug+1)%7;
  }
  if (GPIO_Pin==EC11A_Pin)
  {
    if (HAL_GPIO_ReadPin(EC11A_GPIO_Port,EC11A_Pin)==0) {
      if (HAL_GPIO_ReadPin(EC11B_GPIO_Port,EC11B_Pin)==0) {
        switch (now_EC_DeBug) {
          case vm_:
            S.vm+=0.2f;
            get_va_setpoint_data(va_setpoint_data,200);
            get_vb_setpoint_data(vb_setpoint_data,200);
            break;
          case hzc_:
            hzc+=0.5f;
            QPR_Init(&QPR1,2*PI*hzc,frequency,num);
            QPR_Init(&QPR2,2*PI*hzc,frequency,num);
            break;
          case kp1_:
            QPR1.kp+=0.2f;
            break;
          case kp2_:
            QPR2.kp+=0.2f;
            break;
          case kr1_:
            QPR1.kr+=0.5f;
            break;
          case kr2_:
            QPR2.kr+=0.5f;
            break;
          case vdc_:
            SVPWM1.vdc+=0.2f;
            SVPWM_init(&SVPWM1,SVPWM1.vdc,frequency,num,div,fhrtim);
            break;
          default:
            break;
        }
      }
    }
  }
  if (GPIO_Pin==EC11B_Pin) {
    if (HAL_GPIO_ReadPin(EC11B_GPIO_Port,EC11B_Pin)==0) {
      if (HAL_GPIO_ReadPin(EC11A_GPIO_Port,EC11A_Pin)==0) {
        switch (now_EC_DeBug) {
          case vm_:
            S.vm-=0.2f;
            get_va_setpoint_data(va_setpoint_data,200);
            get_vb_setpoint_data(vb_setpoint_data,200);
            break;
          case hzc_:
            hzc-=0.5f;
            QPR_Init(&QPR1,2*PI*hzc,frequency,num);
            QPR_Init(&QPR2,2*PI*hzc,frequency,num);
            break;
          case kp1_:
            QPR1.kp-=0.2f;
            break;
          case kp2_:
            QPR2.kp-=0.2f;
            break;
          case kr1_:
            QPR1.kr-=0.5f;
            break;
          case kr2_:
            QPR2.kr-=0.5f;
            break;
          case vdc_:
            SVPWM1.vdc-=0.2f;
            SVPWM_init(&SVPWM1,SVPWM1.vdc,frequency,num,div,fhrtim);
            break;
          default:
            break;
        }
      }
    }
  }
  if (GPIO_Pin==KEY0_Pin) {
      frequency*=0.5f;
      frequency_conv(fhrtim,frequency,div,num);
      SV_init(&S,26.12789f,frequency,num);
      QPR_Init(&QPR1,2*PI*hzc,frequency,num);
      QPR_Init(&QPR2,2*PI*hzc,frequency,num);
      SVPWM_init(&SVPWM1,60.0f,frequency,num,div,fhrtim);
  }
  if (GPIO_Pin==KEY1_Pin) {

  }
  if (GPIO_Pin==KEY2_Pin) {

  }
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
