
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "pid_controller.h"
#include <math.h>
#include <stm32f4xx_hal_tim.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* ---------------- CAN ----------------- */
uint8_t CanSendMSG[8] = {};

/* ---------------- SD ------------------ */
/*
FATFS SDFatFs;
FIL MyFile;
char SD_Path[4];
uint8_t buffer[256] = "654\r\n";;
char sdError[50] = "SD not mounted!\r\n";
char msg[256] = "";
char tmpString[255] = "";
int random_number = 0;
uint8_t sdMounted = 0;
//char LogFile[256];
char nameFile[256];
char LogFile[256];
uint8_t wtext[256]  = "BMS LV started...\r\n";
uint8_t rtext[100]  = "";
*/
/* --------------- Timer ---------------- */
long count         = 0; /* Counter */
long deltaCount    = 0; /* Elapsed tics */
uint8_t exeTime    = 0; /* Execution time */
uint32_t waitTime  = 0; /* Wait time */
uint16_t cntTick   = 3000; /* Ticks after sending CAN msg */

/* ---------------- ADC ----------------- */
uint32_t value[10];
uint32_t adcBuffer[10];
uint32_t randomNumber[1];
char tmpString[255] = "";
char adc[100];
float volConv       = 3.3 / 4095.0;
float currSens      = 0.0092; /* Current sensor sensibility V/A */
float conv1         = 20.0 / 10.0;
float conv2         = 32.0 / 10.0;
float conv3         = 43.0 / 10.0;
float conv4         = 57.0 / 10.0;
uint16_t cycle      = 400;
uint16_t cycleCnt   = 0;
uint32_t tmpCurr    = 0;
float calibCurr     = 0.0;
uint8_t charging    = 0; /* 0 discharge - 1 charging */
uint8_t SoC         = 0; /* State of Charge */
double energy       = 31.2; /* Ah */
double energyDrawn  = 0; /* Ah */
/* ADCs */
float TH1      = 0.0;
float TH2      = 0.0;
float TH3      = 0.0;
float TH4      = 0.0;
float TH5      = 0.0;
float maxTemp  = 0.0;
float avgTemp  = 0.0;
float VP1      = 0.0;
float VP2      = 0.0;
float VP2temp  = 0.0;
float VP3      = 0.0;
float VP3temp  = 0.0;
float VP4      = 0.0;
float totVol   = 0.0;
float CURR     = 0.0;
float tempCurr = 0.0;
float currFIFO[10] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
double avgCurrent  = 0.0;
float filtCurrent  = 0.0;
/*------------------------*/
#define B 3380
#define RT0 10000
#define T0 1/298.15
float VTH      = 0;
float tempITH  = 0;
float RTH      = 0;
/*------------------------*/
float extTemperature = 25.0;
float calibTH1       = 0.0;
float calibTH2       = 0.0;
float calibTH3       = 0.0;
float calibTH4       = 0.0;
float calibTH5       = 0.0;

/* ---------------- CAN ------------------ */
CAN_FilterTypeDef sFilter;
CAN_RxHeaderTypeDef RxHeader;

/* ---------------- ID ------------------- */
#define BMS_LV_ASK_ID 0xFF // Foo Fighters
#define STEER_ASK_ID  0xAF // Steering wheel
#define INV_LEFT_ASK_ID  0x181 // Inverter left
#define INV_RIGHT_ASK_ID  0x182 // Inverter right
#define ACC_TEMP_ASK_ID  0xAA // Accumulator temperatures
#define ECU_ASK_ID 0xF8 // ECU initial check ID

/* ---------------- TEMP ------------------- */
uint16_t motLeftTemp     = 0;
float  motorLeftTemp     = 0.0;
uint16_t motRightTemp    = 0;
float  motorRightTemp    = 0.0;
uint16_t invLeftTemp     = 0;
float  inverterLeftTemp  = 0.0;
uint16_t invRightTemp    = 0;
float  inverterRightTemp = 0.0;
uint16_t tmpHvAvgTemp    = 0;
uint16_t tmpHvMaxTemp    = 0;
float hvAvgTemp          = 0;
float hvMaxTemp          = 0;
uint8_t pumpRequest      = 0;
uint8_t fanRequest       = 0;
uint8_t overridePID      = 0;
float motPumpOut         = 0.0;
float motPumpIn          = 0.0;
float invPumpOut         = 0.0;
float invPumpIn          = 0.0;
float invFanOut          = 0.0;
float invFanIn           = 0.0;
float accFanOut          = 0.0;
float accFanIn           = 0.0;
uint16_t timerCntPeriod  = 100;

/* -------------- ERRORS ----------------- */
uint8_t undervoltageError    = 0;
uint8_t overvoltageError     = 0;
uint8_t overtemperatureError = 0;
uint8_t overcurrentError     = 0;
uint8_t errors               = 0;
uint8_t SCScnt               = 200; /* Counts after which LV is turned off */
uint8_t shutdownCnt          = 0;
/*
 * 1 undervoltage (10.0V)
 * 2 overvoltgage (16.8V)
 * 3 overtemperature (60.0C)
 * 4 overcurrent (35A)
 */

/* ---------- Security variables ------------ */
float underVoltage      = 2.7;  /* V */
uint8_t undVolCnt       = 0;
float overVoltage       = 4.2;  /* V */
uint8_t oveVolCnt       = 0;
float overTemperature   = 60.1; /* C */
uint8_t oveTemCnt       = 0;
float overCurrent       = 36.0; /* A */
uint8_t oveCurCnt       = 0;

/* ---------- Security variables ------------ */
uint8_t fadeErrLED      = 0;
uint8_t trigErrLED      = 0;
uint8_t fadeVariableERR = 1;
uint8_t fadeStsLED      = 0;
uint8_t trigStsLED      = 0;
uint8_t fadeVariableSTS = 1;
/*------------ PID variables -----------------*/
float KpMp = 2.0;  //kp motor pump
float KiMp = 0.1;  //ki motor pump
float KdMp = 0.0;  //kd motor pump

float KpIp = 2.0;  //kp inverter pump
float KiIp = 0.1;  //ki inverter pump
float KdIp = 0.0;  //kd inverter pump

float KpAf = 2.0;  //kp accumulator fan
float KiAf = 0.1;  //ki accumulator  fan
float KdAf = 0.0;  //kd accumulator  fan

float KpIf = 2.0;  //kp inverter fan
float KiIf = 0.1;  //ki inverter fan
float KdIf = 0.0;  //kd inverter fan

float tss = 0.01;  //sample time in seconds

/*------------Input initialization----------- */
#define INMP 49.0; //input motor pump
#define INAF 49.0; //input accumulator fan
#define INIP 49.0; //input inverter pump
#define INIF 49.0; //input inverter fan

/*------------SETPOINT initialization----------- */
float SETMP = 50.0; //setpoint motor pump
float SETAF = 50.0; //setpoint accumulator fan
float SETIP = 50.0; //setpoint inverter pump
float SETIF = 50.0; //setpoint inverter fan

float motorsTempMax    = 0.0;
float invertersTempMax = 0.0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM12_Init(void);
static void MX_CAN1_Init(void);
static void MX_NVIC_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
                                
                                
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
// ---------------------------------- PID ----------------------------------- //
void PIDInit(PIDControl *pid, float kp, float ki, float kd,
             float sampleTimeSeconds, float minOutput, float maxOutput,
             PIDMode mode, PIDDirection controllerDirection);

// ---------------------------------- PID ---------------------------------- //
void user_pwm_setvalue(uint32_t value, TIM_HandleTypeDef *htim, uint32_t Channel ) {
    TIM_OC_InitTypeDef sConfigOC;
    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = value;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, Channel);
    HAL_TIM_PWM_Start(htim, Channel);
}

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM6_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  MX_TIM12_Init();
  MX_CAN1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  // ----------------------------------- PWM ----------------------------------- //

  	//HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); /* PB6  - PWM_Daniele */
  	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2); /* PB7  - PWM3 */
  	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); /* PB8  - FAN1 PID!!!*/

  	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); /* PA15 - PIEZO */
  	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); /* PB9  - PUMP1  PID!!!*/

  	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); /* PA9  - PWM2 */
  	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); /* PA10 - ERR LED */

  	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); /* PC6  - FAN2 PID!!!*/
  	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); /* PA7  - STS_LED LED */
  	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); /* PB0  - PCB LED */
  	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); /* PC9  - PUMP2 PID!!!*/

  	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2); /* PC7  - EXTRA_PWM INVERTER FAN (BJT-MOS CONTROLLER)*/

  	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1); /* PB15  - PWM1 */

  	void checkPID(void) {
  		for (int pwm = 0; pwm < 99; pwm++) {
  			htim4.Instance->CCR3 = pwm; // PUMP1
  			htim3.Instance->CCR4 = pwm; // PUMP2
  			htim8.Instance->CCR2 = pwm; // RADIATOR FAN
  			htim1.Instance->CCR2 = pwm; // ACCUMULATOR FAN
  			HAL_Delay(75);
  		}
  	}

  	// ---------------------------------- TIMER ---------------------------------- //
  	//HAL_TIM_Base_Start_IT(&htim6); /* Enable interrupt of timer 6 */
  	HAL_TIM_Base_Start(&htim6); /* Enable counter mode on timer 6 */
  	// Timer APB2 = 72Mhz / prescaler -> 3600 = 0.0025 us, 7200 = 0.05 ms

  	// ----------------------------------- SD  ----------------------------------- //
  	HAL_UART_Transmit(&huart2, (uint8_t*)"I'm alive!\r\n", strlen("I'm alive!\r\n"), 10);

  	// ----------------------------------- CAN ----------------------------------- //
  	/* CAN Setting */
  	sFilter.FilterMode           = CAN_FILTERMODE_IDMASK;
  	sFilter.FilterIdLow          = 0;
  	sFilter.FilterIdHigh         = 0;
  	sFilter.FilterMaskIdHigh     = 0;
  	sFilter.FilterMaskIdLow      = 0;
  	sFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  	sFilter.FilterBank           = 0;
  	sFilter.FilterScale          = CAN_FILTERSCALE_16BIT;
  	sFilter.FilterActivation     = ENABLE;

  	HAL_CAN_ConfigFilter(&hcan1, &sFilter);
  	//HAL_CAN_Init(&hcan1);
  	HAL_CAN_Start(&hcan1);

  	uint8_t RxData[8];
  	//uint8_t TxData[8];
  	int idsave;

  	// ----------------------------------- PID ----------------------------------- //
  	PIDControl myPid;  /* MOTOR PUMP PID */
  	PIDControl myPid1; /* INVERTER PUMP PID */
  	PIDControl myPid2; /* INVERTER FAN PID */
  	PIDControl myPid3; /* ACCUMULATOR FAN PID */

    //PID (pointer, kp,ki,kd,sampleSeconds, minOut, maxOut, mode, direction)
  	PIDInit(&myPid, KpMp, KiMp, KdMp, tss, 0, 100, AUTOMATIC, REVERSE);
  	myPid.setpoint = SETMP;
  	myPid.input    = INMP;

  	PIDInit(&myPid1, KpIp, KiIp, KdIp, tss, 0, 100, AUTOMATIC, REVERSE);
  	myPid.setpoint = SETIP;
  	myPid.input    = INIP;

  	PIDInit(&myPid2, KpIf, KiIf, KdIf, tss, 100, 0, AUTOMATIC, REVERSE);
  	myPid.setpoint = SETIF;
  	myPid.input    = INIF;

  	PIDInit(&myPid3, KpAf, KiAf, KdAf, tss, 0, 100, AUTOMATIC, REVERSE);
  	myPid.setpoint = SETAF;
  	myPid.input    = INAF;

  	HAL_Delay(200);
  	// --------------------------- INTIALIZATION END ------------------------------ //

  	// ---------------------------- CALIRATION START ------------------------------ //
	currentCalibration();
	HAL_Delay(200);

	// -------------------------------- LV CHECK ---------------------------------- //
	getAdcs(); /* Load adc values inside adcBuffer */
	adcConversion();
	avgTemp = avgTemperature(); // Average temperature
	maxTemp = maxTemperature(); // Maximum temperature
	SCSfunction(); // Check all SCS and return errors

	if (errors == 0) {
		activateLVSystems(); // Activate or keep on
	} else {
		shutdownLVSystems(); // Shutdown or keep off
	}
  	HAL_Delay(200);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // Toggle STM LED
	  	    //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
	  		/* --------------------- ADC --------------------- */
	  		getAdcs(); /* Load adc values inside adcBuffer */
	  		adcConversion();
	  		avgTemp = avgTemperature(); // Average temperature
	  		maxTemp = maxTemperature(); // Maximum temperature
	  		SCSfunction(); // Check all SCS

	  		if (errors == 0) {
	  			activateLVSystems(); // Activate or keep on
	  			htim1.Instance->CCR3 = fadeErrLED;
	  		} else {
	  			shutdownLVSystems(); // Shutdown if SCS occour for at least 200 times (400-600ms) or keep off
	  			htim1.Instance->CCR3 = trigErrLED; // Error status
	  		}

	        /* ------------------- CAN SEND ------------------- */
	        /* SEND MESSAGE EVRY 10 ms - 200 tick */
	  		if ( __HAL_TIM_GetCounter(&htim6) >= cntTick) {
	  			__HAL_TIM_SetCounter(&htim6, 0); // Reset counter
	  			canSendMsgReset(); // Clean send message
	  			// [vol*10] [SoC] [avgTmp] [maxTmp] [errors]
	  			uint8_t totV     = (int)(totVol*10);
	  			CanSendMSG[0]    = totV;
	  			CanSendMSG[1]    = SoC;
	  			float tmpAvgTemp = avgTemp * 5;
	  			uint8_t tmpVar1  = (int) tmpAvgTemp;
	  			CanSendMSG[2]    = tmpVar1;
	  			float tmpMaxTemp = maxTemp * 5;
	  			uint8_t tmpVar2  = (int) tmpMaxTemp;
	  			CanSendMSG[3]    = tmpVar2;
	  			CanSendMSG[4]    = errors;
	  			//exeTime         = cntTick * 0.05;
	  			//CanSendMSG[5]   = exeTime;
	  			CAN_Send(BMS_LV_ASK_ID , CanSendMSG, 8);
	  		}

	        /* ----------------- CAN RECEIVE ------------------ */
	  		idsave = CAN_Receive(RxData, 8);
	  		if ( idsave == STEER_ASK_ID ) { // Steering wheel override PID
	  			if (RxData[0] == 0) {
	  				overridePID = RxData[1]; // 1 - override PID
	  			} else if (RxData[0] == 1) {
	  				overridePID = RxData[1];
	  				pumpRequest = RxData[2];
	  			} else if (RxData[0] == 2) {
	  				overridePID = RxData[1];
	  				fanRequest  = RxData[2];
	  			} else if ( RxData[0] == 3 && RxData[1] == 1 ) {
	  				overridePID = 2;
	  			}
			} else if ( idsave == ACC_TEMP_ASK_ID ) { // Accumulator temperature
				if ( RxData[0] == 6 ) { // Little endian
					tmpHvAvgTemp = RxData[5] + ( RxData[4] << 8 );
					hvAvgTemp    = tmpHvAvgTemp / 100.0;
				} else if ( RxData[0] == 6 ) {
					tmpHvMaxTemp = RxData[7] + ( RxData[6] << 8 );
					hvMaxTemp    = tmpHvMaxTemp / 100.0;
				}
			} else if ( idsave == INV_LEFT_ASK_ID ) {
				if ( RxData[0] == 0x4A ) { // Inverter left temperature
					invLeftTemp       = RxData[1] + ( RxData[2] << 8 );
					inverterLeftTemp  = ( invLeftTemp - 15797 ) / 112.12;
				} else if ( RxData[0] == 0x49  ) { // Motor left temperature
					motLeftTemp       = RxData[1] + ( RxData[2] << 8 );
					motorLeftTemp     = (motLeftTemp - 9394) / 55.10;
				}
			} else if ( idsave == INV_RIGHT_ASK_ID ) {
				if ( RxData[0] == 0x4A ) { // Inverter right temperature
					invRightTemp      = RxData[1] + ( RxData[2] << 8 );
					inverterRightTemp = (invRightTemp - 15797) / 112.12;
			    } else if ( RxData[0] == 0x49 ) { // Motor right temperature
			    	motRightTemp      = RxData[1] + ( RxData[2] << 8 );
			    	motorRightTemp    = (motRightTemp - 9394) / 55.10;
				}
			} else if ( idsave == ECU_ASK_ID) {
				//canSendMsgReset(); // Clean send message
				//CAN_Send(BMS_LV_ASK_ID , CanSendMSG, 8);
			}

	  		/* --------------------- PID --------------------- */
	  		if ( overridePID == 0 ) {
	  			motorsTempMax = max(motorLeftTemp, motorRightTemp);
	  			PIDSetpointSet(&myPid, SETMP);
				PIDInputSet(&myPid, motorsTempMax); //set input to PID controller for motors' pump
	  			PIDCompute(&myPid);
	  			motPumpOut = PIDOutputGet(&myPid);
	  			user_pwm_setvalue(motPumpOut, &htim4, TIM_CHANNEL_1);
	  			motPumpIn  = PIDInputGet(&myPid);

	  			invertersTempMax = max(inverterLeftTemp, inverterRightTemp);
	  			PIDSetpointSet(&myPid1, SETIP);
				PIDInputSet(&myPid1, invertersTempMax); //set input to PID controller for inverters' pump
	  			PIDCompute(&myPid1);
	  			invPumpOut = PIDOutputGet(&myPid1);
	  			user_pwm_setvalue(invPumpOut, &htim3, TIM_CHANNEL_4);
	  			user_pwm_setvalue(invPumpOut, &htim4, TIM_CHANNEL_3);
	  			invPumpIn  = PIDInputGet(&myPid1);

	  			PIDSetpointSet(&myPid2, SETIF);
				PIDInputSet(&myPid2, invertersTempMax); //set input to PID controller for inverters' fans
	  			PIDCompute(&myPid2);
	  			invFanOut  = PIDOutputGet(&myPid2);

	  			//control to stop fan or pumps if output is below a certain limit
	  			if (invFanOut < 40 && accFanOut > 20 ){
	  				user_pwm_setvalue(40, &htim3, TIM_CHANNEL_1);
	  			}else if(invFanOut < 20){
	  				user_pwm_setvalue(0, &htim3, TIM_CHANNEL_1);
	  			}else{
		  		    user_pwm_setvalue(invFanOut, &htim3, TIM_CHANNEL_1);
	  			}
	  	        invFanIn   = PIDInputGet(&myPid2);

	  			PIDSetpointSet(&myPid3, SETAF);
				PIDInputSet(&myPid3, hvMaxTemp); //set input to PID controller for Accumulator's fans
	  			PIDCompute(&myPid3);
	  			accFanOut  = PIDOutputGet(&myPid3);

	  			//control to stop fan or pumps if output is below a certain limit
	  			if (accFanOut < 40 && accFanOut > 20 ){
	  				user_pwm_setvalue(40, &htim4, TIM_CHANNEL_3);
	  			}else if(accFanOut < 20){
	  				user_pwm_setvalue(0, &htim4, TIM_CHANNEL_3);
	  			}else{
		  		    user_pwm_setvalue(accFanOut, &htim4, TIM_CHANNEL_3);
	  			}
	  			accFanIn   = PIDInputGet(&myPid3);


	  		} else if ( overridePID == 1 ) { /* override PID */
	  			htim8.Instance -> CCR2 = 100 - fanRequest; // Radiator's fans
	  			htim4.Instance -> CCR3 = fanRequest; // Pump1
	  			htim3.Instance -> CCR4 = fanRequest; // Pump2
	  			htim1.Instance -> CCR2 = 100 - fanRequest; // Accumulator's fans
	  		} else if ( overridePID == 2 ) {
	  			checkPID();
	  			overridePID = 0;
	  		}

	  		/* ----------------- TECH INSPECTION -------------- */
	  		if ( cycleCnt >= cycle) {
	  			cycleCnt = 0;
	  			//printADC1(); /* Print all data every XXs */
	  			//printTemp();
	  			//printPID();
	  			printValues();
	  			//printTemp();
	  			if (trigErrLED == 0) {trigErrLED = 100;} else {trigErrLED = 0;} // Trigger error led
	  		}
	  		cycleCnt++;

	  		/* --------------------- FADING -------------------- */
	  		fadeErrLED = fadeErrLED + fadeVariableERR;
	  		if ((fadeErrLED < 1) || (fadeErrLED > 99)) {
	  			fadeVariableERR = -fadeVariableERR;
	  		}
	  		fadeStsLED = fadeStsLED + fadeVariableSTS;
	  		if ((fadeStsLED < 1) || (fadeStsLED > 99)) {
	  			fadeVariableSTS = -fadeVariableSTS;
	  		}
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  HAL_Delay(2);
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 288;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* PVD_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PVD_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(PVD_IRQn);
  /* FLASH_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(FLASH_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(FLASH_IRQn);
  /* RCC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(RCC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(RCC_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* ADC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADC_IRQn);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 10;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 3;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 4;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 6;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 10;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* ADC2 init function */
static void MX_ADC2_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* CAN1 init function */
static void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 2;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_5TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 45;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 45;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 45;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 45;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim4);

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 3600;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 9999;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM8 init function */
static void MX_TIM8_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 360;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 100;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim8);

}

/* TIM12 init function */
static void MX_TIM12_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;

  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 360;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 100;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim12);

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, EX_GPIO_Pin|RELAY2_Pin|SYNC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI_CS_Pin|RELAY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : EX_GPIO_Pin RELAY2_Pin SYNC_Pin */
  GPIO_InitStruct.Pin = EX_GPIO_Pin|RELAY2_Pin|SYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_CS_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RELAY_Pin */
  GPIO_InitStruct.Pin = RELAY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RELAY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RPMF1_Pin RPMF2_Pin */
  GPIO_InitStruct.Pin = RPMF1_Pin|RPMF2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
int max(int num1, int num2)
{
    return (num1 > num2 ) ? num1 : num2;
}

// ----------------------------- TECH INSPECTION ----------------------------- //
void printValues(void) {
	sprintf(tmpString, "V1: %.2f V4: %.2f V3: %.2f V4: %.2f filtCurr: %.2f errors: %d shutdown: %d \r\n", VP1, VP2, VP3, VP4, filtCurrent, errors, shutdownCnt );
	HAL_UART_Transmit(&huart2, (uint8_t*)tmpString, strlen(tmpString), 50);
}

// ------------------------------- Calibration ------------------------------- //
void currentCalibration(void) {
	tmpCurr = 0;
	for (int i = 0; i < 10; i++) {
		HAL_Delay(10);
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adcBuffer, 10); // Call DMA for ADC1
		HAL_Delay(10);// Wait for conversion
		HAL_ADC_Stop_DMA(&hadc1);
		tmpCurr += adcBuffer[2]; //( value[2] * volConv - 3.3 / 2.0 ) * currSens;
	}
	tmpCurr = tmpCurr/10.0;
	calibCurr = ( tmpCurr * volConv - 1.65 ) / currSens;
	sprintf(tmpString, "calibration current %.2f\r\n", calibCurr);
	HAL_UART_Transmit(&huart2, (uint8_t*)tmpString, strlen(tmpString), 10);
	HAL_Delay(20);
}

void temperatureCalibration(int *extTemp) {
	// Send via CAN
	calibTH1   = TH1 - extTemperature;
	calibTH2   = TH2 - extTemperature;
	calibTH3   = TH3 - extTemperature;
	calibTH4   = TH4 - extTemperature;
	calibTH5   = TH5 - extTemperature;
}

void SCSfunction(void) {
	/* -------------- Over/Under voltage ------------- */
	/* Undervoltage */
	if ((VP1 < underVoltage || VP2 < underVoltage || VP3 < underVoltage || VP4 < underVoltage)) {
		undervoltageError = 128;
		undVolCnt++;
	} else {
		undervoltageError = 0; // RESET Error
		undVolCnt = 0;
	}

	/* Overvoltage */
	if ((VP1 > overVoltage || VP2 > overVoltage || VP3 > overVoltage || VP4 > overVoltage)) {
		overvoltageError = 64;
		oveVolCnt++;
	} else {
		overvoltageError = 0; // RESET Error
		oveVolCnt = 0;
	}

	/* Overtemperature */
	if ((TH1 > overTemperature || TH2 > overTemperature || TH3 > overTemperature || TH4 > overTemperature || TH5 > overTemperature)) {
		overtemperatureError = 32;
		oveTemCnt++;
	} else {
		overtemperatureError = 0; // RESET Error
		oveTemCnt = 0;
	}

	/* Overcurrent */
	if (CURR > overCurrent) {
		overcurrentError = 16;
		oveCurCnt++;
	} else {
		overcurrentError = 0; // RESET Error
		oveCurCnt = 0;
	}

	errors = undervoltageError + overvoltageError + overtemperatureError + overcurrentError; // Error vector
}

// ----------------------------------- ADC ----------------------------------- //
void getAdcs(void) {
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adcBuffer, 10); // Call DMA for ADC1
	//HAL_ADC_Start_DMA(&hadc2, (uint32_t*) adcBuffer1, 2); // Call DMA for ADC2 - NOT USED
	HAL_Delay(0.4); // Wait for conversion
	HAL_ADC_Stop_DMA(&hadc1);
}

void adcConversion(void) {
	/* Manage all the data from ADCs */
	VTH     = adcBuffer[0] * volConv;
	tempITH = VTH/RT0; /* I across 10K res */
	RTH     = (3.3 - VTH)/tempITH;
	TH2     = ( 1 / (logf(RTH/RT0)/B + T0) ) - 273.15 + calibTH2;

	VTH     = adcBuffer[1] * volConv;
	tempITH = VTH/RT0; /* I across 10K res */
	RTH     = (3.3 - VTH)/tempITH;
	TH1     = ( 1 / (logf(RTH/RT0)/B + T0) ) - 273.15 + calibTH1;

    float vCurr     = adcBuffer[2] * volConv;
	CURR            = fabs( vCurr - 1.6500 ) / currSens - fabs(calibCurr);

	/* ----- Moving average filter ----- */
	for (int i=9; i>1; i--) { /* Shift current values */
		currFIFO[i] = currFIFO[i-1];
	}
	currFIFO[0] = CURR; // save newest value
	for (int i=0; i<10; i++) {
		avgCurrent = avgCurrent + currFIFO[i];
	}
	filtCurrent = avgCurrent/10.0;
	avgCurrent  = 0.0;
	/* --------------------------------- */

	VTH     = adcBuffer[3] * volConv;
	tempITH = VTH / RT0; /* I across 10K res */
	RTH     = (3.3 - VTH) / tempITH;
	TH5     = (1 / (logf(RTH / RT0) / B + T0)) - 273.15 + calibTH5;

	VP1       = adcBuffer[7] * volConv * conv1;
	VP2temp   = adcBuffer[4] * volConv * conv2;
	VP2       = VP2temp - VP1; // parallel voltage
	VP3temp   = adcBuffer[5] * volConv * conv3 ;
	VP3       = VP3temp - VP2temp;
	totVol    = adcBuffer[6] * volConv * conv4 ;
	VP4       = totVol - VP3temp;

	VTH = adcBuffer[8] * volConv;
	tempITH = VTH / RT0; /* I across 10K res */
	RTH = (3.3 - VTH) / tempITH;
	TH3 = (1 / (logf(RTH / RT0) / B + T0)) - 273.15 + calibTH3;

	VTH = adcBuffer[9] * volConv;
	tempITH = VTH / RT0; /* I across 10K res */
	RTH = (3.3 - VTH) / tempITH;
	TH4 = (1 / (logf(RTH / RT0) / B + T0)) - 273.15 + calibTH4;
}

float avgTemperature(void) {
	float avgTemp = TH1 + TH2 + TH3 + TH4 + TH5;
	avgTemp = avgTemp/5.0;
	return avgTemp;
}

void shutdownLVSystems(void) {
	if ( HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_SET  && ( undVolCnt > SCScnt || oveVolCnt > SCScnt || oveTemCnt > SCScnt || oveCurCnt > SCScnt ) && ( shutdownCnt <= 0 ) ) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		shutdownCnt = 10;
	} /* OPEN MAIN RELAY */
}

void activateLVSystems(void) {
	if ( HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET && shutdownCnt == 0) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	} /* CLOSE MAIN RELAY */
}

float maxTemperature(void) {
	float tempVector[5] = {TH1, TH2, TH3, TH4, TH5};
	maxTemp = TH1;
	for (int i=0; i<5; i++) {
		if ( maxTemp < tempVector[i] ) {
			maxTemp = tempVector[i];
		}
	}
	return maxTemp;
}

void printTemp(void) {
	sprintf(tmpString, "TH1: %.2f - TH2: %.2f - TH3: %.2f - TH4: %.2f - TH5: %.2f - maxTemp: %.2f - avgTemp: %.2f\r\n", TH1, TH2, TH3, TH4, TH5, maxTemp, avgTemp);
	//sprintf(tmpString, "%.2f-%.2f-%.2f-%.2f-%.2f\r\n", currFIFO[0], currFIFO[1], currFIFO[2], currFIFO[3], currFIFO[4]);
	HAL_UART_Transmit(&huart2, (uint8_t*) tmpString, strlen(tmpString), 10);
}

void printVolt(void) {
	sprintf(tmpString, "%.2f-%.2f-%.2f-%.2f-%.2f\r\n", VP1, VP2, VP3, VP4, CURR);
	HAL_UART_Transmit(&huart2, (uint8_t*) tmpString, strlen(tmpString), 10);
}

//TODO:
uint8_t SOC(void) {
    energyDrawn  = energyDrawn - (CURR*exeTime*3600.0); /* Ah */
	SoC          = (energy - energyDrawn)*100/energy;
    return SoC;
}
void printADC1(void) {
	sprintf(adc, "%lu-%lu-%lu-%lu-%lu-%lu-%lu-%lu-%lu-%lu\r\n", adcBuffer[0], adcBuffer[1],
			adcBuffer[2], adcBuffer[3], adcBuffer[4], adcBuffer[5],adcBuffer[6],adcBuffer[7],adcBuffer[8],adcBuffer[9]);
	HAL_UART_Transmit(&huart2, (uint8_t*) adc, strlen(adc), 20);
}

// ---------------------------------- RND ----------------------------------- //
void RND() {
	HAL_ADC_Start_DMA(&hadc2, (uint32_t*) randomNumber, 1); // Call DMA for ADC1
	//HAL_ADC_Start_DMA(&hadc2, (uint32_t*) adcBuffer1, 2); // Call DMA for ADC2 - NOT USED
	HAL_Delay(0.05); // Wait for conversion
	HAL_ADC_Stop_DMA(&hadc2);
}

// ---------------------------------- PID ----------------------------------- //
void printPID(void) {
	sprintf(tmpString, "MPO: %.2f IPO: %.2f IFO: %.2f AFO: %.2f AT: %.2f IT: %.2f MT: %.2f  \r\n", motPumpOut,invPumpOut,invFanOut,accFanOut,hvAvgTemp,invertersTempMax,motorsTempMax);
	HAL_UART_Transmit(&huart2, (uint8_t*)tmpString, strlen(tmpString), 50);
}

// ---------------------------------- CAN ----------------------------------- //

void canSendMsgReset(void) {
	for (int i = 0; i < 8; i++){
		CanSendMSG[i] = 0x00;
	}
}

int CAN_Send(int id, uint8_t dataTx[], int size) {
  uint32_t mailbox;
  uint8_t flag = 0;
  //char txt[50];

  CAN_TxHeaderTypeDef TxHeader;
  TxHeader.StdId = id;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = size;
  TxHeader.TransmitGlobalTime = DISABLE;

  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 0
      && HAL_CAN_IsTxMessagePending(&hcan1, CAN_TX_MAILBOX0 + CAN_TX_MAILBOX1 + CAN_TX_MAILBOX2) == 0) {
    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, dataTx, &mailbox);
    //sprintf(txt, "%d, %d, %d, %d, %d, %d, %d, %d, %d\r\n", id, dataTx[0], dataTx[1], dataTx[2], dataTx[3], dataTx[4], dataTx[5], dataTx[6], dataTx[7]);
    //HAL_UART_Transmit(&huart2, (uint8_t*)txt, strlen(txt), 10);
    flag = 1;
  }

  return flag;
}

/* old can function
int CAN_Send(int id, uint8_t dataTx[], int size){

	uint32_t mailbox;
	uint8_t flag = 0;

	CAN_TxHeaderTypeDef TxHeader;
	TxHeader.StdId = id;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = size;
	TxHeader.TransmitGlobalTime = DISABLE;

	if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 0 && HAL_CAN_IsTxMessagePending(&hcan1, CAN_TX_MAILBOX0) == 0){
		HAL_CAN_AddTxMessage(&hcan1, &TxHeader, dataTx, &mailbox);
		flag = 1;
	}

	return flag;
}
*/
int CAN_Receive(uint8_t *DataRx, int size){

	if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) != 0){
		HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, DataRx);
	}

	int id = RxHeader.StdId;

	return id;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
