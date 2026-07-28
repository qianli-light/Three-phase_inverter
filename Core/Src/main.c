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
#include "cordic.h"
#include "dma.h"
#include "hrtim.h"
#include "spi.h"
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
const float phy_calc_conv_voltage=0;
const float phy_calc_conv_current=0;
const float phy_calc_conv_offset=2045;
const float p1=5.0,p2=5.0;

enum EC_DeBug now_EC_DeBug=vm_;

SV S={0};
QPR QPR1={0},QPR2={0};
SVPWM SVPWM1={0};

HRTIM_CompareCfgTypeDef ACMP1_TA={0},ACMP1_TB={0},ACMP1_TC={0};

uint16_t ADC1_value[3],ADC2_value[4];

CORDIC_ConfigTypeDef sConfig;
int32_t sin_q31,cos_a_q31,theta_q31=0,cos_b_q31;

float va_setpoint,vb_setpoint,ia_setpoint,ib_setpoint;
float calc_va_setpoint,calc_vb_setpoint,calc_ia_setpoint,calc_ib_setpoint;
float va_measurement,vb_measurement,vc_measurement,ia_measurement,ib_measurement,ic_measurement;
float va_control,vb_control;
float a,b,c,d,f;//中间值




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
  MX_CORDIC_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  SV_init(&S,2*PI*50,0.0001f,24.0f);
  QPR_Init(&QPR1,2*PI*30,0.0001f,2*PI*50,0.1f,0.1f);
  QPR_Init(&QPR2,2*PI*30,0.0001f,2*PI*50,0.1f,0.1f);
  SVPWM_init(&SVPWM1,30.0f,0.0000125,10000);

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

  __HAL_HRTIM_TIMER_ENABLE_IT(&hhrtim1,HRTIM_TIMERINDEX_TIMER_A,HRTIM_TIM_IT_UPD);//开启更新中断,开启QPR

  DeBug_interface_head();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


    three_phase_inverter_interface_main();
    DeBug_interface_main();
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 25;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void QPR_Init(QPR *QPR,float wc,float t,float w0,float kp,float kr) {
  QPR->wc=wc;
  QPR->t=t;
  QPR->w0=w0;
  QPR->kp=kp;
  QPR->kr=kr;
  QPR->k=2.0f/QPR->t;

  float A0=QPR->k*QPR->k+2.0f*QPR->wc*QPR->k+QPR->w0*QPR->w0;
  QPR->a1=2.0f*(QPR->w0*QPR->w0-QPR->k*QPR->k)/A0;
  QPR->a2=(QPR->k*QPR->k-2.0f*QPR->wc*QPR->k+QPR->w0*QPR->w0)/A0;
  QPR->b0=2.0f*QPR->k*QPR->wc/A0;
  QPR->b2=-QPR->b0;
}

float QPR_Compute(QPR *QPR,float e0) {
  QPR->y0=QPR->b0*e0+QPR->b2*QPR->e2-QPR->a1*QPR->y1-QPR->a2*QPR->y2;
  QPR->u0=QPR->kp*e0+QPR->kr*QPR->y0;

  QPR->e2=QPR->e1;
  QPR->e1=e0;
  QPR->y2=QPR->y1;
  QPR->y1=QPR->y0;

  return QPR->u0;//需要加上输出限幅;
}

float P_Compute(const float p,float e0) {
  return (p*e0);
}

void SV_init(SV *S,float w0,float t0,float vm) {
  S->theta=0;
  S->vm=vm;
  S->t0=t0;
  S->w0=w0;
}
int32_t rad_to_q31(float rad) {
  float norm_rad=rad/PI;

  int32_t result=(int32_t)(norm_rad*0x80000000);
  return result;
}
float sin_cos_q31_to_float(int32_t sin_cos_q31_value) {
  return (float)sin_cos_q31_value / 2147483648.0f;
}
void phy_conv_calc(void) {
  calc_va_setpoint=va_setpoint*phy_calc_conv_voltage+phy_calc_conv_offset;
  calc_vb_setpoint=vb_setpoint*phy_calc_conv_voltage+phy_calc_conv_offset;
}
void SVPWM_init(SVPWM *SVPWM,float vdc,float Ts,float arr) {
  SVPWM->vdc=vdc;
  SVPWM->Ts=Ts;
  SVPWM->arr=arr;
  SVPWM->mid=SVPWM->Ts/SVPWM->vdc;
}
void Midvalue_Compute(float va_control,float vb_control) {
  a=va_control+2.0f*vb_control;
  b=va_control-vb_control;
  c=2.0f*va_control+vb_control;
  d=-va_control+vb_control;
}
uint8_t Sector_Judgment(void) {
  uint8_t N=0,sector=0;
  if (a>0){N+=1;}
  if (b>0){N+=2;}
  if (c<0){N+=4;}

  switch (N) {
    case 3:
      sector=1;
      break;
    case 1:
      sector=2;
      break;
    case 5:
      sector=3;
      break;
    case 4:
      sector=4;
      break;
    case 6:
      sector=5;
      break;
    case 2:
      sector=6;
      break;
  }
  return sector;
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
void vector_action(SVPWM *SVPWM,uint8_t sector) {
  switch (sector) {
    case 1:
      ACMP1_TA.CompareValue=(uint32_t)(SVPWM->T0*0.25f/SVPWM->Ts*SVPWM->arr);
      ACMP1_TB.CompareValue=(uint32_t)(ACMP1_TA.CompareValue+SVPWM->T1*0.5f/SVPWM->Ts*SVPWM->arr);
      ACMP1_TC.CompareValue=(uint32_t)(ACMP1_TB.CompareValue+SVPWM->T2*0.5f/SVPWM->Ts*SVPWM->arr);
      HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, &ACMP1_TA);
      HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, &ACMP1_TB);
      HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_COMPAREUNIT_1, &ACMP1_TC);
      break;
    case 2:
      ACMP1_TB.CompareValue=(uint32_t)(SVPWM->T0*0.25f/SVPWM->Ts*SVPWM->arr);
      ACMP1_TA.CompareValue=(uint32_t)(ACMP1_TB.CompareValue+SVPWM->T1*0.5f/SVPWM->Ts*SVPWM->arr);
      ACMP1_TC.CompareValue=(uint32_t)(ACMP1_TA.CompareValue+SVPWM->T2*0.5f/SVPWM->Ts*SVPWM->arr);
      HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, &ACMP1_TA);
      HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, &ACMP1_TB);
      HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_COMPAREUNIT_1, &ACMP1_TC);
      break;
    case 3:
      ACMP1_TB.CompareValue=(uint32_t)(SVPWM->T0*0.25f/SVPWM->Ts*SVPWM->arr);
      ACMP1_TC.CompareValue=(uint32_t)(ACMP1_TB.CompareValue+SVPWM->T1*0.5f/SVPWM->Ts*SVPWM->arr);
      ACMP1_TA.CompareValue=(uint32_t)(ACMP1_TC.CompareValue+SVPWM->T2*0.5f/SVPWM->Ts*SVPWM->arr);
      HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, &ACMP1_TA);
      HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, &ACMP1_TB);
      HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_COMPAREUNIT_1, &ACMP1_TC);
      break;
    case 4:
      ACMP1_TC.CompareValue=(uint32_t)(SVPWM->T0*0.25f/SVPWM->Ts*SVPWM->arr);
      ACMP1_TB.CompareValue=(uint32_t)(ACMP1_TC.CompareValue+SVPWM->T1*0.5f/SVPWM->Ts*SVPWM->arr);
      ACMP1_TA.CompareValue=(uint32_t)(ACMP1_TB.CompareValue+SVPWM->T2*0.5f/SVPWM->Ts*SVPWM->arr);
      HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, &ACMP1_TA);
      HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, &ACMP1_TB);
      HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_COMPAREUNIT_1, &ACMP1_TC);
      break;
    case 5:
      ACMP1_TC.CompareValue=(uint32_t)(SVPWM->T0*0.25f/SVPWM->Ts*SVPWM->arr);
      ACMP1_TA.CompareValue=(uint32_t)(ACMP1_TC.CompareValue+SVPWM->T1*0.5f/SVPWM->Ts*SVPWM->arr);
      ACMP1_TB.CompareValue=(uint32_t)(ACMP1_TA.CompareValue+SVPWM->T2*0.5f/SVPWM->Ts*SVPWM->arr);
      HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, &ACMP1_TA);
      HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, &ACMP1_TB);
      HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_COMPAREUNIT_1, &ACMP1_TC);
      break;
    case 6:
      ACMP1_TA.CompareValue=(uint32_t)(SVPWM->T0*0.25f/SVPWM->Ts*SVPWM->arr);
      ACMP1_TC.CompareValue=(uint32_t)(ACMP1_TA.CompareValue+SVPWM->T1*0.5f/SVPWM->Ts*SVPWM->arr);
      ACMP1_TB.CompareValue=(uint32_t)(ACMP1_TC.CompareValue+SVPWM->T2*0.5f/SVPWM->Ts*SVPWM->arr);
      HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, &ACMP1_TA);
      HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, &ACMP1_TB);
      HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_COMPAREUNIT_1, &ACMP1_TC);
      break;
  }
}
void HAL_HRTIM_RegistersUpdateCallback(HRTIM_HandleTypeDef *hhrtim,uint32_t TimerIdx) {
  static uint8_t count = 0;
  static uint8_t sector = 0;
  if (TimerIdx==HRTIM_TIMERINDEX_TIMER_A) {
    count++;
    if (count%8==0) {
      sConfig.Function=CORDIC_FUNCTION_COSINE;
      HAL_CORDIC_Configure(&hcordic, &sConfig);
      CORDIC->WDATA=theta_q31;

      S.theta+=S.w0*S.t0;
      if (S.theta>2*PI) {
        S.theta-=2*PI;
      }
      theta_q31=rad_to_q31(S.theta);

      cos_a_q31=CORDIC->RDATA;
      va_setpoint=S.vm*sin_cos_q31_to_float(cos_a_q31);
      cos_b_q31=cos_a_q31-0x55555555;
      vb_setpoint=S.vm*sin_cos_q31_to_float(cos_b_q31);
      phy_conv_calc();
      calc_ia_setpoint=QPR_Compute(&QPR1,calc_va_setpoint-ADC2_value[0]);
      calc_ib_setpoint=QPR_Compute(&QPR2,calc_vb_setpoint-ADC2_value[1]);
      va_control=P_Compute(p1,calc_ia_setpoint-ADC1_value[0]);
      vb_control=P_Compute(p2,calc_ib_setpoint-ADC1_value[1]);
      Midvalue_Compute(va_control,vb_control);
      sector=Sector_Judgment();
      vector_actiontime(&SVPWM1,sector);
      vector_action(&SVPWM1,sector);
    }
  }
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin==EC11D_Pin)
  {
    now_EC_DeBug=(now_EC_DeBug+1)%5;
  }
  if (GPIO_Pin==EC11A_Pin)
  {
    if (HAL_GPIO_ReadPin(EC11A_GPIO_Port,EC11A_Pin)==0) {
      if (HAL_GPIO_ReadPin(EC11B_GPIO_Port,EC11B_Pin)==0) {
        switch (now_EC_DeBug) {
          case vm_:
            S.vm+=0.2f;
            break;
          case kp1_:
            QPR1.kp+=0.1f;
            break;
          case kr1_:
            QPR1.kr+=0.1f;
            break;
          case kp2_:
            QPR2.kp+=0.1f;
            break;
          case kr2_:
            QPR2.kr+=0.1f;
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
            break;
          case kp1_:
            QPR1.kp-=0.1f;
            break;
          case kr1_:
            QPR1.kr-=0.1f;
            break;
          case kp2_:
            QPR2.kp-=0.1f;
            break;
          case kr2_:
            QPR2.kr-=0.1f;
            break;
          default:
            break;
        }
      }
    }
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
