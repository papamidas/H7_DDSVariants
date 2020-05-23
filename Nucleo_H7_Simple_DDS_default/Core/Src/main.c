/* USER CODE BEGIN Header */
/**
  *
  *  A simple 2-channel DDS generator:
  *  Each time TIM6 has counted down to zero an event is generated that triggers
  *  the DAC and generates an interrupt.
  *  The interrupt service routine is used for calculating the new values of the
  *  phase registers of both channels, truncates the phase values, does a sine
  *  table lookup and stores the result in the dual DAC output register
  *
  *  @author (of non-automatically generated code:) DM1CR 2020
  *
  *
 **/

/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <inttypes.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	uint32_t phase;			// Phase Register
	uint32_t phaseInc;      // Phase Increment Register
	uint16_t amplitude;     // Output Amplitude
	uint16_t offset;        // Output Offset
	float*   wavtab;        // Pointer to signed Sine Table or other (signed) Waveform Table
} DDS_typedef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*
 *  Defs for H7 peripheral clock and DAC
 */
#define APB1_CLOCK 200000000
#define DACMAX_DEFINE 4095
const uint16_t DACMAX = DACMAX_DEFINE;
/*
 * TIMER 6 is driving the DAC trigger:
 */
#define TIM6_ARR_RESETVAL_DEFINE 19999  // TIM6_ARR = Timer 6 auto-reload register
uint16_t tim6ARRVal = TIM6_ARR_RESETVAL_DEFINE;
/*
 *  DDS main parameters
 */
#define DDS_PHASEBITS_DEFINE 32
const uint8_t DDS_PHASEBITS = DDS_PHASEBITS_DEFINE;
#define DDS_PHASERES pow(2,32)            // how many phase values can be stored in dds_phase
#define DDS_WANTEDSAMPLEPERIOD_US 100     // desired time between samples at start of program
#define DDS_DACBITS 12                    // resolution of DAC-outputs in bits
#define DDS_AMPLITUDE_ADCSTEPS 1900       // waveform amplitude at start of program
#define DDS_OFFSET_ADCSTEPS 2047          // waveform offset at start of program
#define DDS_SINETABLELEN_BITS 14          // length of waveform table in bits; should have 2 bits more than DAC
/*
 *  DDS command list
 */
#define CMD_INPUT_CH1 'X'                 // command for changing to channel 1 input mode
#define CMD_INPUT_CH2 'Y'                 // command for changing to channel 2 input mode
#define CMD_INPUT_SYNC 'Z'                // command for sync'ing channel 2 to channel 1
#define CMD_INPUT_DELTA 'D'               // command for phase jump of channel 2 (phase increments)
#define CMD_INPUT_JUMP 'J'                // command for phase jump of channel 2 (degrees)
#define CMD_INPUT_DDSFREQ 'F'             // command for changing to output frequency input mode
#define CMD_INPUT_DDS_SAMPLEFREQ 'S'      // command for changing to sample frequency input mode (eq for both ch)
#define CMD_INPUT_DDS_SAMPLEPERIOD 'P'    // command for changing to sample period input mode (eq for both ch)
#define CMD_INPUT_DDS_AMPLITUDE 'A'       // command for changing to amplitude input mode
#define CMD_INPUT_DDS_OFFSET 'O'          // command for changing to offset input mode

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

DAC_HandleTypeDef hdac1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
volatile DDS_typedef dds[2];
float                dds_sinetable[1<<DDS_SINETABLELEN_BITS];
volatile float       dds_samplePeriod_us;   // sample period in us

volatile uint8_t     uart_incoming_char[1];
volatile char        uart_inputString[200];
volatile uint8_t     uart_stringComplete;
volatile char        uart_command;
volatile uint8_t     uart_commandIsKnown;
volatile char        uart_ddsMode = 'X';

// simple CPU load measurement with a counter
#define CPULOADTICKS 500 // number of systicks used for load measurement
double cpuLoadNoLoadCnt; // this variable will hold the reference count value

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void update_TIM6(uint16_t new_tim6_divisor);
double calc_dds_fout(double dacclock, uint32_t phaseinc, uint8_t phasebits);
uint32_t calc_phaseinc(double fout_Hz, double dacperiod_s, uint8_t phasebits);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
 	if(htim == &htim6){
      HAL_GPIO_WritePin(PROF0_GPIO_Port, PROF0_Pin, GPIO_PIN_SET);
	  uint32_t dhr12rd = (uint32_t)&hdac1.Instance->DHR12RD;
	  dds[0].phase += dds[0].phaseInc;
	  dds[1].phase += dds[1].phaseInc;
	  /*
	   *  write to right aligned dual channel DAC register:
	   */
	  uint16_t truncatedphase1 = dds[0].phase >> (32-DDS_SINETABLELEN_BITS);
	  uint16_t truncatedphase2 = dds[1].phase >> (32-DDS_SINETABLELEN_BITS);

	  uint32_t dacval1 = (dds[0].amplitude * dds[0].wavtab[truncatedphase1]) + dds[0].offset;
	  uint32_t dacval2 = (dds[1].amplitude * dds[1].wavtab[truncatedphase2]) + dds[1].offset;
	  dacval2 <<= 16;

	  *(__IO uint32_t *)dhr12rd = dacval1 + dacval2;

	  HAL_GPIO_WritePin(PROF0_GPIO_Port, PROF0_Pin, GPIO_PIN_RESET);

	  return;
  }
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	char inChar;
	if (huart == &huart3) {
		inChar = uart_incoming_char[0];
		if (inChar == '\r' ){
			inChar = '\n';    // if terminal sends return instead of newline
		}
		if (isalpha(inChar)){
		    // incoming character is a letter, so it is probably a command:
		    uart_command = inChar;
		    uart_stringComplete = 1;
		    uart_inputString[0] = '\0'; // clear string if user has entered a letter
		}
		else {
			// non-alpha char is added to the inputString:
		    strncat((char*)uart_inputString, &inChar, 1);
		    // if the incoming character is a newline, set a flag so the main loop can
		    // do something about it:
    		if ((inChar == '\n')) {
		       uart_stringComplete = 1;
    		}
		}
		// not elegant, but simple: telling HAL to receive next char...
		HAL_UART_Receive_IT(huart, (uint8_t *)uart_incoming_char, 1);
		return;
	}

	UNUSED(huart);
}

// calculation of the output frequency of the DDS, given the phase increment value:
double calc_dds_fout(double dacclock, uint32_t phaseinc, uint8_t phasebits)
{
	return (dacclock * phaseinc / ((uint64_t)1 << phasebits));
}

// calculation of the phase increment value of the DDS, given the output frequency:
uint32_t calc_phaseinc(double fout_Hz, double dacperiod_s, uint8_t phasebits)
{
	return (fout_Hz * dacperiod_s * ((uint64_t)1 << phasebits) + 0.5);
}

// helper function for updating tim6 ARR value:
void update_TIM6(uint16_t new_tim6_divisor)
{
	float current_dds_samplefrequency_0 = calc_dds_fout( 1.0/dds_samplePeriod_us * 1.0e6,
            dds[0].phaseInc, DDS_PHASEBITS);
	float current_dds_samplefrequency_1 = calc_dds_fout( 1.0/dds_samplePeriod_us * 1.0e6,
            dds[1].phaseInc, DDS_PHASEBITS);
	tim6ARRVal = new_tim6_divisor-1;
	__HAL_TIM_SET_AUTORELOAD(&htim6, tim6ARRVal);  // update reload register of TIM6
	dds_samplePeriod_us = 1.0/APB1_CLOCK * (tim6ARRVal+1) * 1.0e6;
	dds[0].phaseInc = current_dds_samplefrequency_0 * dds_samplePeriod_us * 1.0e-6 * DDS_PHASERES;
	dds[1].phaseInc = current_dds_samplefrequency_1 * dds_samplePeriod_us * 1.0e-6 * DDS_PHASERES;
}

double getCPULoadCntrValue(uint32_t ticks){
    uint32_t cpuLoadLoopCntrStartValue = uwTick;  // see SysTickInterrupt!
    uint64_t cpuLoadDummyCntr = 0;
    while(uwTick-cpuLoadLoopCntrStartValue < ticks) {
        cpuLoadDummyCntr++;
    }
    return cpuLoadDummyCntr;
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
  MX_USART3_UART_Init();
  MX_DAC1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  setvbuf(stdout, NULL, _IONBF, 0);  // no line buffering of printf wanted
  HAL_UART_Receive_IT(&huart3, (uint8_t*)uart_incoming_char, 1);

  // initialize sine table
  int sintablen = 1 << DDS_SINETABLELEN_BITS;
  for (int i = 0; i < sintablen; i++) {
  	  dds_sinetable[i] = sin(2.0 * M_PI * i/sintablen);
  }
  printf("\r\nWavetable calculated\r\n");

  // reference measurement: number of loops before loading CPU with TIM6 interrupts
  cpuLoadNoLoadCnt = getCPULoadCntrValue(CPULOADTICKS);

  // Initializing DDS-Parameters of both channels:

  dds[0].wavtab = dds_sinetable;
  dds[0].amplitude = DDS_AMPLITUDE_ADCSTEPS;
  dds[0].offset = DDS_OFFSET_ADCSTEPS;

  dds[1].wavtab = dds_sinetable;
  dds[1].amplitude = DDS_AMPLITUDE_ADCSTEPS;
  dds[1].offset = DDS_OFFSET_ADCSTEPS;

  printf("Start:");
  dds_samplePeriod_us = 1.0/APB1_CLOCK * (tim6ARRVal+1) * 1.0e6;
  dds[0].phaseInc = calc_phaseinc(1000.0, dds_samplePeriod_us / 1.0e6, DDS_PHASEBITS);
  dds[1].phaseInc = calc_phaseinc(1000.0, dds_samplePeriod_us / 1.0e6, DDS_PHASEBITS);
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim6);
  __HAL_TIM_SET_AUTORELOAD(&htim6, tim6ARRVal);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (uart_stringComplete) {
	    switch (toupper(uart_command)) {       // change to appropriate input mode if command is known
	      case CMD_INPUT_DDSFREQ:         uart_commandIsKnown = 1;
	                                      if (isdigit(uart_inputString[0])) {
											float f = atof((char*)uart_inputString);
	                                        uint32_t pinc = calc_phaseinc(f, dds_samplePeriod_us * 1.0e-6, DDS_PHASEBITS);
	                                        switch (uart_ddsMode) {
                	                                    	uint32_t prim;
	                                        				case CMD_INPUT_CH1:  dds[0].phaseInc = pinc;
	                                        				                     break;
	                                        				case CMD_INPUT_CH2:  dds[1].phaseInc = pinc;
                        				                                         break;
	                                        				case CMD_INPUT_SYNC: prim = __get_PRIMASK();
	                                                                             __disable_irq();
	                                                                             dds[0].phaseInc = pinc;
	                                        				                     dds[1].phaseInc = pinc;
	                                                                             if (!prim) {
	                                                                               __enable_irq();
	                                                                             }
   				                                                                 break;
	                                        				default:             printf("\nerr: unknown dds mode\r\n");
	                                        									 break;
	                                        }
	                                        printf("\n%f\r\n", f);
	                                      }
	                                      printf("\r\nEnter dds_freq: ");
	                                      break;
	      case CMD_INPUT_DDS_SAMPLEFREQ:  uart_commandIsKnown = 1;
	                                      if (isdigit(uart_inputString[0])) {
	                                        float f = atof((char*)uart_inputString);
	                                        update_TIM6(APB1_CLOCK / f);
	                                        printf("\n%f\r\n", 1.0/dds_samplePeriod_us * 1.0e6);
	                                      }
	                                      printf("\r\nEnter dds_sampleFreq_Hz: ");
	                                      break;
	      case CMD_INPUT_DDS_SAMPLEPERIOD:uart_commandIsKnown = 1;
	                                      if (isdigit(uart_inputString[0])) {
	                                        float p = atof((char*)uart_inputString);
	                                        update_TIM6(p * 1.0e-6 * APB1_CLOCK);
	                                        printf("\n%f\r\n", dds_samplePeriod_us);
	                                      }
	                                      printf("\r\nEnter dds_samplePeriod_us: ");
	                                      break;
	      case CMD_INPUT_DDS_AMPLITUDE:   uart_commandIsKnown = 1;
	                                      if (isdigit(uart_inputString[0])) {
	                                    	uint16_t new_amplitude = atoi((char*)uart_inputString);
	                                        switch (uart_ddsMode) {
                               				  case CMD_INPUT_CH1:  dds[0].amplitude = new_amplitude;
		                                                           break;
		                                      case CMD_INPUT_CH2:  dds[1].amplitude = new_amplitude;
	                        				                       break;
		                                      case CMD_INPUT_SYNC: dds[0].amplitude = new_amplitude;
		                                                           dds[1].amplitude = new_amplitude;
                                                                   break;
	                            			  default:             printf("\nerr: unknown dds mode\r\n");
		                                        			       break;
	                                        }
	                                        printf("\n%d\r\n", new_amplitude);
	                                      }
	                                      printf("\r\nEnter dds_amplitude_adcsteps: ");
	                                      break;
	      case CMD_INPUT_DDS_OFFSET:      uart_commandIsKnown = 1;
	                                      if (isdigit(uart_inputString[0])) {
	                                    	uint16_t new_offset = atoi((char*)uart_inputString);
     	                                    switch (uart_ddsMode) {
	                               			  case CMD_INPUT_CH1:  dds[0].offset = new_offset;
			                                                       break;
			                                  case CMD_INPUT_CH2:  dds[1].offset = new_offset;
		                        		   	                       break;
			                                  case CMD_INPUT_SYNC: dds[0].offset = new_offset;
			                                                       dds[1].offset = new_offset;
	                                                               break;
		                            	      default:             printf("\nerr: unknown dds mode\r\n");
			                                        			   break;
		                                    }
	                                        printf("\n%d\r\n", new_offset);
	                                      }
	                                      printf("\r\nEnter dds_offset_adcsteps: ");
	                                      break;
	      case CMD_INPUT_CH1:             uart_commandIsKnown = 1;
                                          uart_ddsMode = CMD_INPUT_CH1;
	                                      printf("\r\nChannel 0 selected:\r\n");
	                                      break;
	      case CMD_INPUT_CH2:             uart_commandIsKnown = 1;
	                                      uart_ddsMode = CMD_INPUT_CH2;
	                                      printf("\r\nChannel 1 selected:\r\n");
	                                      break;
	      case CMD_INPUT_SYNC:            uart_commandIsKnown = 1;
                                          uart_ddsMode = CMD_INPUT_SYNC;
                                          uint32_t prim = __get_PRIMASK();
                                          __disable_irq();
                                          dds[1].phaseInc = dds[0].phaseInc;
                                          dds[1].phase = dds[0].phase;
                                          if (!prim) {
                                            __enable_irq();
                                          }
                                          printf("\r\nSync mode selected:\r\n");
	                                      break;
	      case CMD_INPUT_DELTA:           uart_commandIsKnown = 1;
                                          if (isdigit(uart_inputString[0])) {
	                                        uint32_t phasejump = atol((char*)uart_inputString);
                                            uint32_t prim = __get_PRIMASK();
	                                        __disable_irq();
	                                        dds[1].phase += phasejump;
                                            if (!prim) {
                                              __enable_irq();
                                            }
                                            printf("\n%"PRIu32"\r\n", phasejump);
                                          }
	                                      printf("\r\nEnter pos. phasejump delta value N: ");
	                                      break;
	      case CMD_INPUT_JUMP:           uart_commandIsKnown = 1;
                                          if (isdigit(uart_inputString[0])) {
	                                        float jumpDegrees = atof((char*)uart_inputString);
	                                        //if (jumpDegrees < 0.0){
	                                        //	jumpDegrees = 360.0 + jumpDegrees;
	                                        //}
	                                        uint32_t phasejump = jumpDegrees/360.0*DDS_PHASERES;
                                            uint32_t prim = __get_PRIMASK();
	                                        __disable_irq();
	                                        dds[1].phase += phasejump;
                                            if (!prim) {
                                              __enable_irq();
                                            }
                                            printf("\n%f\r\n", jumpDegrees);
                                          }
	                                      printf("\r\nEnter phasejump in degrees: ");
	                                      break;
	      default:                        uart_commandIsKnown = 0;
	    }
	    // clear the string:
	    uart_inputString[0] = '\0';
	    uart_stringComplete = 0;
	  }

	  if( !uart_commandIsKnown ){  // print some interesting numbers
	    for (int i=0; i<2; i++) {
	    	if (i==1) {
	    		printf("  ");
	    	}
	    	printf("f_out[%d] = %f Hz, ", i, calc_dds_fout(1.0e6/dds_samplePeriod_us,dds[i].phaseInc, DDS_PHASEBITS));
			printf("f_sample = %f Hz, ", 1.0e6/dds_samplePeriod_us);
			printf("phasereg[%d] = %"PRIu32", ", i, dds[i].phase);
			printf("phaseinc[%d] = %"PRIu32", ", i, dds[i].phaseInc);
			printf("ampl[%d] = %d, ", i, dds[i].amplitude);
			printf("offs[%d] = %d", i, dds[i].offset);
	    	if (i==0) {
	    		printf("\r\n");
	    	}
	    }
	    printf(", CPU load = %f%%\r\n",
	    		100.0*(cpuLoadNoLoadCnt-getCPULoadCntrValue(CPULOADTICKS))/cpuLoadNoLoadCnt );
	  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Supply configuration update enable 
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization 
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config 
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config 
  */
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 199;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  huart3.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  huart3.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PROF3_Pin|PROF2_Pin|LD3_Pin|PROF1_Pin 
                          |LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PROF0_GPIO_Port, PROF0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PROF3_Pin PROF2_Pin LD3_Pin PROF1_Pin 
                           LD2_Pin */
  GPIO_InitStruct.Pin = PROF3_Pin|PROF2_Pin|LD3_Pin|PROF1_Pin 
                          |LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PROF0_Pin */
  GPIO_InitStruct.Pin = PROF0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PROF0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/*
 *   enable printf for console output
 */

// Redirect standard output to the serial port
int _write(int file, char *ptr, int len)
{
    for (int i=0; i<len; i++)
    {
        while(!(USART3->ISR & USART_ISR_TXE_TXFNF)); // wait till transmit data reg empty
        USART3->TDR = *ptr++;
    }
    return len;
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

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
