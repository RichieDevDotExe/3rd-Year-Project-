#include "stm32f7xx.h"                  // Device header
#include "RTE_Components.h"             // Component selection
#include "stm32746g_discovery_sdram.h"  // Keil.STM32F746G-Discovery::Board Support:Drivers:SDRAM
#include "Board_GLCD.h"                 // ::Board Support:Graphic LCD
#include "GLCD_Config.h"                // Keil.STM32F746G-Discovery::Board Support:Graphic LCD
#include "cmsis_os.h"                   // ARM::CMSIS:RTOS:Keil RTX
#include "RTE_Device.h"                 // Keil::Device:STM32Cube Framework:Classic
#include "stm32f7xx_hal_conf.h"         // Keil::Device:STM32Cube Framework:Classic
#include "stm32f7xx_hal.h"              // Keil::Device:STM32Cube HAL:Common

#include <stdio.h>
#include <string.h>

#include "stm32f7xx_hal.h"
#include "Board_Touch.h"
#define wait_delay HAL_Delay;

extern GLCD_FONT GLCD_Font_6x8;
extern GLCD_FONT GLCD_Font_16x24;

TIM_HandleTypeDef htim2;


#ifdef __RTX
extern uint32_t os_time;
uint32_t HAL_GetTick(void) {
	return os_time;
}
#endif

/**
* @brief  initPins() - this initialises the pins for the 4 digit 7 segment display
* @param  None
* @retval None
*/
void  initPins(void){
	GPIO_InitTypeDef PinList[9];
	GPIO_InitTypeDef gpio;
	
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOI_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	

	// set mode as output, nopull
	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_HIGH;
	gpio.Pin = GPIO_PIN_8;

	// initialise the pin
	HAL_GPIO_Init(GPIOB, &gpio);
	HAL_GPIO_Init(GPIOI, &gpio);
	HAL_GPIO_Init(GPIOA, &gpio);

	// enable the segment
	HAL_GPIO_WritePin(GPIOB, gpio.Pin, GPIO_PIN_SET);
	
	PinList[0] = gpio;
	
	gpio.Pin = GPIO_PIN_9;
	HAL_GPIO_Init(GPIOB, &gpio);
	HAL_GPIO_WritePin(GPIOB, gpio.Pin, GPIO_PIN_SET);
	PinList[1] = gpio;
	
	gpio.Pin = GPIO_PIN_1;
	HAL_GPIO_Init(GPIOI, &gpio);
	HAL_GPIO_WritePin(GPIOI, gpio.Pin, GPIO_PIN_SET);
	PinList[2] = gpio;
	
	gpio.Pin = GPIO_PIN_14;
	HAL_GPIO_Init(GPIOB, &gpio);
	HAL_GPIO_WritePin(GPIOB, gpio.Pin, GPIO_PIN_SET);
	PinList[3] = gpio;
	
	gpio.Pin = GPIO_PIN_15;
	HAL_GPIO_Init(GPIOB, &gpio);
	HAL_GPIO_WritePin(GPIOB, gpio.Pin, GPIO_PIN_SET);
	PinList[4] = gpio;
	
	gpio.Pin = GPIO_PIN_8;
	HAL_GPIO_Init(GPIOA, &gpio);
	HAL_GPIO_WritePin(GPIOA, gpio.Pin, GPIO_PIN_SET);
	PinList[5] = gpio;
	
	gpio.Pin = GPIO_PIN_2;
	HAL_GPIO_Init(GPIOI, &gpio);
	HAL_GPIO_WritePin(GPIOI, gpio.Pin, GPIO_PIN_SET);
	PinList[6] = gpio;
	
	//controls digits
	gpio.Pin = GPIO_PIN_3;
	HAL_GPIO_Init(GPIOI, &gpio);
	HAL_GPIO_WritePin(GPIOI, gpio.Pin, GPIO_PIN_SET);
	PinList[7] = gpio;
	
	gpio.Pin = GPIO_PIN_4;
	HAL_GPIO_Init(GPIOB, &gpio);
	HAL_GPIO_WritePin(GPIOB, gpio.Pin, GPIO_PIN_SET);
	PinList[8] = gpio;
}





/**
* @brief  dispNum() - function to set the pins from high to low depending on what segments are needed to represent a number. 
* @param  Int num - number to display
* @retval int
* @note Some reason segment will only turn on if pin is set to low
*/
int dispNum(int num){
	GPIO_InitTypeDef gpio;
	
	// set mode as output, nopull
	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_HIGH;

	if(num == 0){
		// enable the segment
		//a
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
		
		//b
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
		
		//c
		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_1, GPIO_PIN_RESET);
		
		//d
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		
		//e
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
		
		//f
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		
		//g
		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_SET);
	}
	
	else if(num == 1){
		// enable the segment
		//a
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
		
		//b
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
		
		//c
		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_1, GPIO_PIN_RESET);
		
		//d
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
		
		//e
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
		
		//f
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
		
		//g
		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_SET);
	}
	
	else if(num == 2){
		// enable the segment
		//a
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
		
		//b
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
		
		//c
		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_1, GPIO_PIN_SET);
		
		//d
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		
		//e
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
		
		//f
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
		
		//g
		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_RESET);
	}
	else if(num == 3){
		// enable the segment
		//a
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
		
		//b
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
		
		//c
		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_1, GPIO_PIN_RESET);
		
		//d
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		
		//e
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
		
		//f
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
		
		//g
		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_RESET);
	}
	else if(num == 4){
		// enable the segment
		//a
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
		
		//b
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
		
		//c
		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_1, GPIO_PIN_RESET);
		
		//d
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
		
		//e
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
		
		//f
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		
		//g
		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_RESET);
	}
	else if(num == 5){
		// enable the segment
		//a
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
		
		//b
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
		
		//c
		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_1, GPIO_PIN_RESET);
		
		//d
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		
		//e
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
		
		//f
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		
		//g
		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_RESET);
	}
	else if(num == 6){
		// enable the segment
		//a
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
		
		//b
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
		
		//c
		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_1, GPIO_PIN_RESET);
		
		//d
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		
		//e
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
		
		//f
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		
		//g
		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_RESET);
	}
	else if(num == 7){
		// enable the segment
		//a
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
		
		//b
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
		
		//c
		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_1, GPIO_PIN_RESET);
		
		//d
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
		
		//e
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
		
		//f
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
		
		//g
		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_SET);
	}
	else if(num == 8){
		// enable the segment
		//a
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
		
		//b
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
		
		//c
		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_1, GPIO_PIN_RESET);
		
		//d
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		
		//e
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
		
		//f
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		
		//g
		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_RESET);
	}
	else if(num == 9){
		// enable the segment
		//a
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
		
		//b
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
		
		//c
		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_1, GPIO_PIN_RESET);
		
		//d
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		
		//e
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
		
		//f
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		
		//g
		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_RESET);
	}
	return 1;
}

/**
* @brief  dispNum() - function to display 2 digits on a 4 digit display
* @param  Int num -  number of cycles shuffler has done. 
* @retval None
* @note uses the dispNum() function
*/
void displayDigit(int num){
	GPIO_InitTypeDef gpio;
	int i;
	
	// set mode as output, nopull
	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_HIGH;
	for(i = 0; i <250;i++){
		//display 10s
		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
		dispNum(num/10);
		HAL_Delay(1);
		//display 1s
		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_3, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
		dispNum(num%10);
		HAL_Delay(1);
	}
}

/**
* @brief  reset() - resets the pins for the 4 digit display.
* @param  None 
* @retval None
*/
void reset(){
	GPIO_InitTypeDef gpio;
	// enable the segment
	//a
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
	
	//b
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
	
	//c
	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_1, GPIO_PIN_SET);
	
	//d
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	
	//e
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
	
	//f
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	
	//g
	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_SET);
		
	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_3, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
}

/**
* @brief  reset() - Initialises pin for the timer used for the L293D Motor Driver
* @param  None 
* @retval None
*/
void PA15_Init(){
	GPIO_InitTypeDef gpio;

	// Enable the clock for the base A
	__HAL_RCC_GPIOA_CLK_ENABLE();

	gpio.Pin = GPIO_PIN_15;
	gpio.Mode = GPIO_MODE_AF_PP;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_HIGH;
	gpio.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(GPIOA, &gpio);
}

/**
* @brief  Button_Init() - Initialises pins needed for the button
* @param  None 
* @retval None
*/
void Button_Init(){
	GPIO_InitTypeDef gpio;
	
	__HAL_RCC_GPIOH_CLK_ENABLE();
	
	gpio.Mode = GPIO_MODE_INPUT;
	gpio.Pull = GPIO_PULLUP;
	gpio.Pin = GPIO_PIN_6;
	
	HAL_GPIO_Init(GPIOH, &gpio);
}
/**
* @brief  Input_Init1() - initialises the pins for input to the L349D. used to control the motors stopping and starting and which direction the motors spin in
* @param  None 
* @retval None
*/
void Input_Init1(){
	GPIO_InitTypeDef gpio;
	
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	
	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_HIGH;
	gpio.Pin = GPIO_PIN_7;
	
	HAL_GPIO_Init(GPIOC, &gpio);
	HAL_GPIO_WritePin(GPIOC, gpio.Pin, GPIO_PIN_RESET);
	
	gpio.Pin = GPIO_PIN_6;
	HAL_GPIO_Init(GPIOC, &gpio);
	HAL_GPIO_WritePin(GPIOC, gpio.Pin, GPIO_PIN_SET);
	
	HAL_GPIO_Init(GPIOG, &gpio);
	HAL_GPIO_WritePin(GPIOG, gpio.Pin, GPIO_PIN_SET);
	
	gpio.Pin = GPIO_PIN_7;
	HAL_GPIO_Init(GPIOG, &gpio);
	HAL_GPIO_WritePin(GPIOG, gpio.Pin, GPIO_PIN_RESET);
}

/**
* @brief  Buzzer_Init() - initialises pins needed for the buzzer
* @param  None 
* @retval None
*/
void Buzzer_Init(){
	GPIO_InitTypeDef gpio;
	
	__HAL_RCC_GPIOI_CLK_ENABLE();
	
	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_HIGH;
	gpio.Pin = GPIO_PIN_0;
	
	HAL_GPIO_Init(GPIOI, &gpio);
	HAL_GPIO_WritePin(GPIOI, gpio.Pin, GPIO_PIN_RESET);
	
}

/**
* @brief  Buzzer_Input() - takes input from the board and then either turns the buzzer on or off
* @param  Int num - on/off value for the buzzer 
* @retval None
*/
void Buzzer_Input(int num){
	GPIO_InitTypeDef gpio;
	
	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_HIGH;
	
	//turn on
	if(num == 1){
		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_0, GPIO_PIN_SET);
	}
	//turn off
	else if(num == 0){
		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_0, GPIO_PIN_RESET);
	}
}


/**
* @brief  BuzzDone() - Function used to signify when the card shuffler is done shuffling. by turning it off and on periodically
* @param  None 
* @retval None
*/
void BuzzDone(){
	int i;
	for(i = 0; i<11 ; i++){
		Buzzer_Input(i%2);
		HAL_Delay(250);
	}
}

/**
* @brief  MotorState1 - depending on the input it controls which motor is spinning. used to alternate each side of the shuffler. or to turn off both sides when the shuffling is done
* @param  Int num - selects what state the motors can be in
* @retval None
*/
void MotorState1(int num){
	GPIO_InitTypeDef gpio;
	
	// set mode as output, nopull
	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_HIGH;
	
	//turn off
	if(num == 1){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_RESET);
	}
	//turn on
	else if(num == 0){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_SET);
	}
	//stop both
	else if(num ==2){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_SET);
	}
	HAL_Delay(500);
}


/**
* @brief  SystemClock_Config() - System Clock Configuration
* @param  None
* @retval None
*/
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	/* Enable Power Control clock */
	__HAL_RCC_PWR_CLK_ENABLE();
	/* The voltage scaling allows optimizing the power
	consumption when the device is clocked below the
	maximum system frequency. */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/* Enable HSE Oscillator and activate PLL
	with HSE as source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);
	/* Select PLL as system clock source and configure
	the HCLK, PCLK1 and PCLK2 clocks dividers */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | 
	RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

/**
* @brief  TIM2_Init() - initialises the timer pin
* @param  None
* @retval None
*/
static void TIM2_Init(void){
	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_OC_InitTypeDef sConfigOC;

	//Timer configuration
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 64000;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 20;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	HAL_TIM_Base_Init(&htim2);

	//Set the timer in PWM mode
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);
	HAL_TIM_PWM_Init(&htim2);

	//Channel configuration
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);

	PA15_Init();
}


/**
* @brief  Main program - the program takes user input to control components that to shuffle cards.
* @param  None
* @retval None
*/

int main(void){
	//default values
	TOUCH_STATE tsc_state;
	char buffer[128];
	int state = 0;
	int c = 0;
	int x = 1;
	GPIO_PinState ButtonInput;
	
	//inits pins
	HAL_Init();
	initPins();
	
	//inits touch screen
	SystemClock_Config();
	Touch_Initialize();
	GLCD_Initialize();
	GLCD_ClearScreen();
	GLCD_SetFont(&GLCD_Font_16x24);
	

	//Reset of all peripherals, initializes the Flash interface and the Systick
	HAL_Init();

	//Enable peripheral clock for TIM2
	__HAL_RCC_TIM2_CLK_ENABLE();

	//Configure the system clock
	SystemClock_Config();

	//Initialize TIM2, CH1 and PA15
	TIM2_Init();

	//Start PWM on TIM2_CH1 (PA15 a.k.a. D9)
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

	//Set the compare register to 1
	htim2.Instance->CCR1 = 14;
	
	Buzzer_Init();
	Input_Init1();
	MotorState1(2);
	
	//infinite for loop. needed to constantly check what state the system is in and to clear the screen
	for(;;){
		GLCD_ClearScreen();
		//state one is a screen with a button that checks if the user has pressed the button to shuffle the cards
		if(state == 0){
			while(state==0){
				Touch_GetState(&tsc_state);
				ButtonInput = HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_6);
				//if input is detected change state
				if(((tsc_state.pressed)&&(tsc_state.x < 360) && (tsc_state.x >120) && (tsc_state.y > 108) && (tsc_state.y <156)) || (ButtonInput == GPIO_PIN_RESET)){
					sprintf(buffer,"%i %i", tsc_state.x,tsc_state.y);
					GLCD_DrawString(200,5*24,buffer);
					GLCD_ClearScreen();
					state = 1;
				}
				//if no input then display main menu
				else{
					GLCD_SetForegroundColor(GLCD_COLOR_BLUE);
					GLCD_SetBackgroundColor(GLCD_COLOR_WHITE);
					GLCD_DrawString(120,122,"START SHUFFLING");
					GLCD_SetForegroundColor(GLCD_COLOR_BLUE);
					GLCD_DrawRectangle (105, 108, 270, 48);
				}
			}
		}
		//state 1 is when the cards are being shuffled
		else if(state ==1){
			int i;
			int done =0;
			char prog[100] = " ";
			while(done != 15){
				//draws an empty bar
				GLCD_SetForegroundColor(GLCD_COLOR_BLUE);
				GLCD_DrawRectangle (105, 108, 270, 48);
				GLCD_SetForegroundColor(GLCD_COLOR_WHITE);
				GLCD_SetBackgroundColor(GLCD_COLOR_BLUE);
				//the progress bar is made of " ". to make it seem the progress bar is moving in each cycle it appends another " ". so when it prints the string the bar gets longer
				for(i =0 ; i <15; i++){
					GLCD_DrawString(120,122,prog);
					strcat(prog," ");
					HAL_Delay(25);
				}
				
				//switches motor state
				MotorState1(done%2);
				
				
				GLCD_SetBackgroundColor(GLCD_COLOR_WHITE);
				done = done +1;
				//displays amount of times the cards have been shuffled
				displayDigit(done);
				GLCD_ClearScreen();
				reset();
				//reset the progress string
				strcpy(prog," ");
			}
			//switches to next state
			state = 2;
		}
		else if(state == 2){
			//turns off motor
			MotorState1(2);
			//displays that the shuffle has been finished and displays a button to shuffle again
			GLCD_SetForegroundColor(GLCD_COLOR_BLUE);
			GLCD_SetBackgroundColor(GLCD_COLOR_WHITE);
			GLCD_DrawString(95,97,"SHUFFLING FINISHED");
			GLCD_DrawString(147,142,"START AGAIN?");
			GLCD_SetForegroundColor(GLCD_COLOR_BLUE);
			GLCD_DrawRectangle (105, 128, 270, 48);
			//turns on the buzzer to notify user that the shuffle is done
			BuzzDone();
			while(state==2){
				Touch_GetState(&tsc_state);
				ButtonInput = HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_6);
				//if input to button is detected moves to state 0
				if(((tsc_state.pressed)&&(tsc_state.x < 375) && (tsc_state.x >105) && (tsc_state.y > 128) && (tsc_state.y <176)) || (ButtonInput == GPIO_PIN_RESET)){
					sprintf(buffer,"%i %i", tsc_state.x,tsc_state.y);
					GLCD_DrawString(200,5*24,buffer);
					GLCD_ClearScreen();
					state = 0;
				}
				else{
					//if no input display state 2 again
					GLCD_SetForegroundColor(GLCD_COLOR_BLUE);
					GLCD_SetBackgroundColor(GLCD_COLOR_WHITE);
					GLCD_DrawString(95,97,"SHUFFLING FINISHED");
					GLCD_DrawString(147,142,"START AGAIN?");
					GLCD_SetForegroundColor(GLCD_COLOR_BLUE);
					GLCD_DrawRectangle (105, 128, 270, 48);
				}
			}
		}
		HAL_Delay(250);
		}
}
