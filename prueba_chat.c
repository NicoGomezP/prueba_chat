/* USER CODE BEGIN Header */
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
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "gb_ina219.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define buff 6
#define nada "\0\0\0" //misma cantidad de null que el valor de buff


#define map_in_max 100   //valor del duty maximo
#define map_out_max 120  //valor del periodo del pwm

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint16_t adcval;
uint16_t adcval2;

char data_in[buff];
char bak[buff]="d10";
float duty_actual=-1;
float duty_anterior=-1;

volatile int flag_interru=0;

uint16_t timer_val=0;
uint16_t timer_real=0;
float tiempo;
float freq;
float rpm;

uint8_t data_enviada[20]="0";


int voltage;
float voltage2;
float duty=0;


float valor=0;
float bus=0;
float pow=0;
float pow_c=0;
float pow_c_n=0;




//BUFers
char buf[16];
char buf2[16];
char A[120] ="";
char buf4[256];
char frase_inicio[] = "\nControlador MPPT iniciado correctamente \n";

/* Filtro */

// PARA FILTRO 2 voltage
uint32_t adc_val;

float ynj=0;
float xn1 = 0;
float yn1 = 0;

float ynj2=0;
float xn2 = 0;
float yn2 = 0;


//Filtro butter
float x[] = {0,0,0};
float y[] = {0,0,0};
float b[] = {0.00024132, 0.00048264, 0.00024132};
float a[] = {1.95558189, -0.95654717};

//



//

int k=0;
int cont=0;
int result=0;

int modo_automatico=0;
float duty_automatico=0;
float duty_step=1;
float duty_select=40;
int signo=1;



// SALIDAS

float shunt_value =0;
float vol_value =0;
float pow_value =0;
float bus_value =0;
float cur_value =0;
float cuf_value =0;
float buf_value =0;
float shf_value =0;


// Modos

int modo_update = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

void cleanBuff(char *,int);
void cleanBuff2(char *,int);
int map(int);
int ok=2;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

float alpha= 0.01 ;




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
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */


 /****************************************************
 *    VERSION 1:                                      *
 *    -Contiene los 2 ADC, pwm y lectura rpm    *
 *
 * 	  VERSION 2:
 * 	  Falta:	   									  *
 *
 *    -Verificar que se este haciendo bien conversion
 *
 *****************************************************/


/****************************************************
  PWM

*****************************************************/
HAL_TIM_Base_Start(&htim2);
HAL_TIM_PWM_Start (&htim2, TIM_CHANNEL_3); //A2
HAL_TIM_PWM_Start (&htim2, TIM_CHANNEL_4); //A3



/****************************************************
  INICIALIZACION 2 ADC.

*****************************************************/

HAL_ADC_Start(&hadc1);
HAL_ADC_Start(&hadc2);


/****************************************************
  TIMER PARA MEDIR VELOCIDAD.

*****************************************************/

// HAL_TIM_Base_Start(&htim2);



//***********************************************************
//
//  MEJORAS EN EL CODIGO
//
//********************************************************************

// ACOMODAR VARIABLES
// MPPT



// ************************************//
// INICIALIZACION INA219
// ******************************

ina219_init();

///BORRRAR!!!!!!
///BORRRAR!!!!!!
///BORRRAR!!!!!!
///BORRRAR!!!!!!
///BORRRAR!!!!!!
///BORRRAR!!!!!!
///BORRRAR!!!!!!
///BORRRAR!!!!!!
///BORRRAR!!!!!!
///BORRRAR!!!!!!

htim2.Instance->CCR3= map(0); //////////////////AAAA!!!!!!!1 solo para probar
///BORRRAR!!!!!!

// UART


HAL_UART_Receive_IT(&huart1, (uint8_t *)data_in, buff);
clean_uart(data_in, buff);




//INICIO - BIENVENIDA
HAL_UART_Transmit(&huart1, (uint8_t *)frase_inicio, strlen(frase_inicio),250); //transmite "s10" y se observa en la consola de la pc






/* USER CODE END 2 */

/* Infinite loop */
/* USER CODE BEGIN WHILE */


  while (1)
  {

	  	 valor=ina219_shuntvoltage();

		//LOW PASS FILTER (Lo paso por filtro pasa bajos)
		 ynj= 0.969*yn1 + 0.0155*valor + 0.0155*xn1;

		//STORE VALUES
		 yn1=ynj;
		 xn1=valor;

		//SECOND LOW PASS FILTER
		 ynj2= 0.969*yn2 + 0.0155*ynj + 0.0155*xn2;
		yn2=ynj2;
		xn2=ynj;

		bus=ina219_busvoltage();
		pow=ina219_Loadpower();









		//ActualizaciÃ³n de valores...


		shunt_value =0;
		vol_value =ina219_shuntvoltage();
		pow_value =ina219_Loadpower();
		bus_value =ina219_busvoltage();
		cur_value =ina219_shuntvoltage()/5;
		cuf_value =0;
		buf_value =0;
		shf_value =0;




		//pow_c = pow_c


		/*
if(ok==1){
//result=0;ok
for(k=0;k<sizeof(data_in);k++){


	if(data_in[0]=='\0'){

		result=k;
		k=sizeof(data_in);
	}



}


}*/


/***************************************
* Si el ingreso de datos es:
* -da = se activa el modo MPPT
* -d100



	*********************************/
	  if (data_in[0]=='d'&&data_in[1]!='a') /*strcmp:compara carÃ¡cter por carÃ¡cter , si las cadenas son iguales retornara un valor 0,
		  	  	  	  	  	  	  	  si la primera cadena es mayor retornara un valor positivo y si es menor retornara un valor negativo.*/
	  {

		  modo_automatico=0;

		  strncpy(buf2, data_in + 1, sizeof(data_in)); //Copia solo la parte numerica porque ignora



		  duty_actual=atof(buf2);  //lo pasa a numero con atof : char - float


		  if(data_in[2]=='.'){
			  duty_actual=duty_actual+atof(data_in[2])+atof(data_in[3])/100;

		 		  }




		  htim2.Instance->CCR3= map(duty_actual); //Duty 10				//modifica la cantidad de pulsos del canal 3 del timer 2 (es decir modifica el Duty Cycle)

		  snprintf(buf4, sizeof(buf4), "\nSe ha configurado un nuevo duty %s \n", buf2);



		  if(duty_anterior!=duty_actual){
		  duty_anterior=duty_actual;


		  }

		  HAL_UART_Transmit(&huart1, (uint8_t *)buf4, strlen(buf4),250); //transmite "s10" y se observa en la consola de la pc

		  clean_uart(data_in, buff);
		  //HAL_UART_Abort_IT(&huart1);
		  //HAL_UART_Receive_IT(&huart1, (uint8_t *)data_in, buff);

		  ok=0;
	  }






	  else if (data_in[0]=='v'&&data_in[1]=='o'&&data_in[2]=='l')
	  {
	  // no hace nada

		  valor=ina219_shuntvoltage();;


		  clean_array(buf4, sizeof(buf4)); //antes de imprimir borro buffer
		  snprintf(buf4, sizeof(buf4), "Voltaje shunt %f \n", valor);
		  HAL_UART_Transmit(&huart1, (uint8_t *)buf4, strlen(buf4),250); //transmite "s10" y se observa en la consola de la pc


		  ok=0;

		  clean_uart(data_in, buff);


	  }




	  else if (data_in[0]=='b'&&data_in[1]=='u'&&data_in[2]=='s')
	  {
	  // no hace nada

		  valor=ina219_busvoltage();
		  clean_array(A, sizeof(A));
		  sprintf(A, "VBus= %f \n", valor);
		  HAL_UART_Transmit(&huart1,(uint8_t *)A,strlen(A), 150);

			  clean_uart(data_in, buff);


	  }


	  else if (data_in[0]=='c'&&data_in[1]=='u'&&data_in[2]=='r')
	  {
	  // no hace nada
		  valor=ina219_shuntvoltage()/5;

		  clean_array(A, sizeof(A));
		  sprintf(A, "Vcurrent= %f \r\n", valor);
		  HAL_UART_Transmit(&huart1,(uint8_t *)A,strlen(A), 150);

		  ok=0;
		  clean_uart(data_in, buff);

	  }


	  else if (data_in[0]=='c'&&data_in[1]=='u'&&data_in[2]=='f')
	  {
	  // no hace nada
		  clean_array(A, sizeof(A));
		  valor=ynj2;
		  sprintf(A, "Vcurrent= %f", valor);
		  strcat(A,"\r\n");
			 HAL_UART_Transmit(&huart1,(uint8_t *)A,strlen(A), 150);
			 HAL_UART_Transmit(&huart1,(uint8_t *)"\n",strlen("\n"), 150);

		  ok=0;
		  clean_uart(data_in, buff);


	  }



	  else if (data_in[0]=='p'&&data_in[1]=='o'&&data_in[2]=='w')
	  {
	  // no hace nada
		  clean_array(A, sizeof(A));
		  valor=ina219_busvoltage()*ina219_shuntvoltage()/5;
		  sprintf(A, "Power= %f", valor);
		  strcat(A,"\r\n");
			 HAL_UART_Transmit(&huart1,(uint8_t *)A,strlen(A), 150);
			 HAL_UART_Transmit(&huart1,(uint8_t *)"\n",strlen("\n"), 150);

		  ok=0;
		  clean_uart(data_in, buff);


	  }




	  else if (data_in[0]=='m'&&data_in[1]=='u'&&data_in[2]=='1')
	  {
	  // no hace nada
		  clean_array(buf4, sizeof(buf4));

			  modo_update=1;
		  sprintf(buf4, "Dutaaay update...\n");
		  HAL_UART_Transmit(&huart1,(uint8_t *)buf4,sizeof(buf4), 150);
		  clean_uart(data_in, buff);




	  }


	  else if (data_in[0]=='m'&&data_in[1]=='u'&&data_in[2]=='0')
	  {
	  // no hace nada

			  modo_update=0;
		  clean_array(buf4, sizeof(buf4));
		  sprintf(buf4, "Dupdateuty ...");
		  HAL_UART_Transmit(&huart1,(uint8_t *)buf4,sizeof(buf4), 150);
		 // cleanBuff(buf4, sizeof(buf4));
		  clean_uart(data_in, buff);





	  }



	  //MPPT MODO

	  else if (data_in[0]=='d'&&data_in[1]=='a')
	  {
	  // no hace nada
		  duty_select=1;
		  modo_automatico=1;
		  clean_array(A, sizeof(A));
		  valor=ynj2;
		  sprintf(A, "Duty modo automatico...", valor);
		  strcat(A,"\r\n");
			 HAL_UART_Transmit(&huart1,(uint8_t *)A,strlen(A), 150);
			  clean_uart(data_in, buff);

			  duty_select=duty_select+duty_step; //primer perturbacion
			  								  htim2.Instance->CCR3= map(duty_select); //Duty 10
	  }


	  if(modo_automatico==1){
	  		duty_automatico=0;

	  		//pow_c_n= ina219_busvoltage()*ina219_shuntvoltage()/5 ;

	  		pow_c_n=alpha*ina219_busvoltage()*ina219_shuntvoltage()/5+(1.0-alpha)*pow_c_n;

	  		if(pow_c_n>pow_c){ //comparo el valor viejo con el nuevo..

	  			signo=signo;
	  			clean_array(A, sizeof(A));


	  			if(modo_update==0){
	  			//sprintf(A, "La potencia actual: %f ,aumentÃ³ %f,duty:%f \r\n", pow_c_n, pow_c_n-pow_c,duty_select);
	  			//strcat(A,"\r\n");
	  			HAL_UART_Transmit(&huart1,(uint8_t *)A,strlen(A), 150);
	  			}

	  		}
	  		else
	  		{
	  			signo=signo*-1;
	  			clean_array(A, sizeof(A));
	  			if(modo_update==0){
	  			//sprintf(A, "La potencia actual: %f ,disminuyo %f ,duty:%f \r\n", pow_c_n, pow_c-pow_c_n,duty_select);
	  			//strcat(A,"\r\n");
	  			HAL_UART_Transmit(&huart1,(uint8_t *)A,strlen(A), 150);
	  			}


	  					//	if(duty_select!=1){
	  						//	duty_select=duty_select-duty_step;
	  							//}
	  		}


	  		duty_select=duty_select+duty_step*signo;

	  		if(duty_select==100){
	  			duty_select=99;
	  		}

	  		htim2.Instance->CCR3= map(duty_select);

	  		pow_c=pow_c_n;  //Guardo el valor nuevo...

	  	}




	  // UPDATE SCREEN DATA
	  if(modo_update==1){

			shunt_value =0;
				vol_value =ina219_shuntvoltage();
				pow_value =ina219_shuntvoltage()/5*ina219_busvoltage();
				bus_value =ina219_busvoltage();
				//cur_value =ina219_shuntvoltage()/5;

				cur_value = alpha*(ina219_shuntvoltage()/5)+(1.0-alpha)*cur_value;


				cuf_value =0;
				buf_value =0;
				shf_value =0;


		  clean_array(buf4, sizeof(buf4));
		  //sprintf(buf4, "Bus: %f , Cur:%f , Pow=%f , Vshunt=%f\n",bus_value,cur_value,pow_value,vol_value );

		  sprintf(buf4, "%.2f,%.2f,%.2f,%.2f\n",bus_value,cur_value,pow_value,vol_value );

		  sprintf(buf4, "%.2f",bus_value,cur_value,pow_value,vol_value );


		  HAL_UART_Transmit(&huart1,(uint8_t *)buf4,strlen(buf4), 150);


	  }




	  else if (strcmp(data_in, "eof") == 0)
	  {
		  HAL_GPIO_WritePin(Emerg_off_GPIO_Port, Emerg_off_Pin, false);
		  HAL_UART_Transmit(&huart1, (uint8_t *)"Emergencia Off", sizeof("Emergencia Off"), 150);
		  ok=0;
	  }
	  else if (strcmp(data_in, "eon") == 0)
	  {
		  HAL_GPIO_WritePin(Emerg_off_GPIO_Port, Emerg_off_Pin, true);
		  HAL_UART_Transmit(&huart1, (uint8_t *)"Emergencia On", sizeof("Emergencia On"), 150);
		  ok=0;
	  }
	  else if (strcmp(data_in, "jon") == 0)
	  {
		  HAL_GPIO_WritePin(jumper_GPIO_Port, jumper_Pin, true);
		  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
		  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
		  HAL_UART_Transmit(&huart1, (uint8_t *)"Jumper On",sizeof("Jumper On"), 150);
		  ok=0;
	  }
	  else if (strcmp(data_in, "jof") == 0)
	  {
		  HAL_GPIO_WritePin(jumper_GPIO_Port, jumper_Pin, false);
		  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
		  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
		  HAL_UART_Transmit(&huart1, (uint8_t *)"Jumper Off", sizeof("Jumper Off"), 150);
		  ok=0;
	  }



	  /* more else if clauses */
	  else /* default: */
	  {

		  clean_uart(data_in, buff);
				 // HAL_UART_Abort_IT(&huart1);
				  //HAL_UART_Receive_IT(&huart1, (uint8_t *)data_in, buff);

		  if(ok==1){
		 HAL_UART_Transmit(&huart1,(uint8_t *)"comando desconocido\n",sizeof("comando desconocido\n"), 150);
		 ok=0;
				  	  	  }

	  }


		adcval =  hadc1.Instance->DR;
	adcval2 =  hadc2.Instance->DR;
	voltage = adcval * (3.3 / 4095);
	voltage2 = adcval2 * (3.3 / 4095);

	//char buffer[10];
	//valor=ina219_shuntvoltage();
	//sprintf(buffer,);

	//itoa(valor,A , 10);
	//strcat(A,"\n");
	//HAL_UART_Transmit(&huart1, (uint8_t*)A, strlen(A), 150);


//valor=ina219_shuntvoltage()/5;

	//voltage_conv= adc_val * 3.27/4095 / 0.1254752;
	//voltage_conv=adc_val   *   25/4095 ;

	 // voltage_conv=adc_val   *   3.3/4095   * 9.4e3/1.477e3 ;
    //voltage_filt_conv=voltage_filt   *   3.3/4095   *   8.18;





	//sprintf("")
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

//	HAL_UART_Transmit(&huart1,(uint8_t *)buffer, 20, 150);



	HAL_Delay(100);

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
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 12-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 120-1;
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
  sConfigOC.Pulse = 30;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 60;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  htim3.Init.Prescaler = 36000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Emerg_off_Pin|jumper_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Emerg_off_Pin jumper_Pin */
  GPIO_InitStruct.Pin = Emerg_off_Pin|jumper_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RPM_Pin */
  GPIO_InitStruct.Pin = RPM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RPM_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_pin)
{

    if(flag_interru==0)
    {
    	HAL_TIM_Base_Start(&htim3);

    }

    if(flag_interru==1)
    {
    	timer_val=TIM3->CNT;
    }


    if(flag_interru==2)
     {
    	timer_real=((TIM3->CNT)-timer_val);
    	tiempo=(float) timer_real/1000;
    	if(timer_real!=0){
        	freq=1/tiempo;
        	rpm=60*freq;
    	}
    	else
    	{
    		freq=-1;
    	}

     	flag_interru=0;
     }

    flag_interru=flag_interru+1;
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) //Si recibo dato...
{

	//;
  if (huart->Instance == USART1)
  {
	 //char data_in[buff];
    /* Receive ten bytes in interrupt mode */
    HAL_UART_Receive_IT(&huart1, (uint8_t *)data_in, buff);
	//__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

     ok=1;
    // data_in[0]='\0';
    //data_in[1]='\0';
    //data_in[2]='\0';

  }
}

/****************************************************
    cleanbuff limpia el buffer de entrada

*****************************************************/

void clean_uart(char *b,int size){
	char *buf;

	buf=b;

	for (int i=0;i<size;i++){
		 *buf='\0';
		 buf++;
	}

	  HAL_UART_Abort_IT(&huart1);            //reseteo el buffer (para que ponga el puntero en el primer elemento) para que no se desfase si se pone d500
	  HAL_UART_Receive_IT(&huart1, (uint8_t *)data_in, buff);

}



void clean_array(char *b,int size){
	char *buf;

	buf=b;

	for (int i=0;i<size;i++){
		 *buf='\0';
		 buf++;
	}

	  HAL_UART_Abort_IT(&huart1);
	  HAL_UART_Receive_IT(&huart1, (uint8_t *)data_in, buff);

}


/****************************************************
    map mapea el duty de un valor a 0 a 100 a el necesario
    para el periodo usado para el pwm (120)

    si el duty es 50% lo mapea a 60 que es el 50% del periodo 120

*****************************************************/
int map(int var){
	int var_map;

	var_map=((var *map_out_max)/map_in_max);

	return var_map;
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
