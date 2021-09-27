/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hmi.h"
#include "host.h"
#include "pid.h"
#include "servo.h"
#include <stdlib.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define USER_MAIN_DEBUG

#ifdef USER_MAIN_DEBUG
#define user_main_printf(format, ...) HostSendLog(LOG_COLOR_BLACK, format "\r\n" ,##__VA_ARGS__)
#define user_main_info(format, ...) HostSendLog(LOG_COLOR_BLACK, "[\tmain]info:" format "\r\n" ,##__VA_ARGS__)
#define user_main_debug(format, ...) HostSendLog(LOG_COLOR_GREEN, "[\tmain]debug:" format "\r\n" ,##__VA_ARGS__)
#define user_main_error(format, ...) HostSendLog(LOG_COLOR_RED, "[\tmain]error:" format "\r\n" , ##__VA_ARGS__)
#else
#define user_main_printf(format, ...)
#define user_main_info(format, ...)
#define user_main_debug(format, ...)
#define user_main_error(format, ...)
#endif
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ERROR_RANGE 5
#define BIAS 1 //go past bias
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


extern int32_t posX, posY;
extern u8 posBUF[4];

char strbuf[50];

const struct Waypoint wp1[] = { { 153, 116, 6 } } // 2s5
                        , wp2[] = { { 0, 0, 1 }, { 0, 0, 3 } } // 1->5s2
                        , wp3[] = { { 0, 0, 1 }, { 0, 0, 3 }, { 0, 0, 3 } } // 1->4s2->5s2
                        , wp4[] = { { 0, 0, 1 }, { 0, 0, 0 }, { 0, 0, 3 } } // 1-^->9s2
                        , wp5[] = { { 0, 0, 1 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 3 } } // 1->2->6->9s2
                        , wp7[] = { { 0, 0, 1 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 3 } };
// 4->(((5)))->9s2
const u16 coor[3][3][2] = { 
    { { 0, 0 }, { 0, 0 }, { 0, 0 } },
    { { 0, 0 }, { 0, 0 }, { 0, 0 } }, 
    { { 0, 0 }, { 0, 0 }, { 0, 0 } }
};

void PointCtr(struct Waypoint waypoints[], u8 n) // Moving through point list
{
    uint8_t j = 0;
    uint16_t out = 0;
    
    user_main_debug("waiting for ball");
    
    while (posX < 50 || posY < 50 || posX > 180 || posY > 180) ;
    
    user_main_debug("Start!");
    
    ServoSetState(&servoX, SERVO_STATE_ACTIVE);
    ServoSetState(&servoY, SERVO_STATE_ACTIVE);
    
    PIDSetState(&pidX, PID_STATE_ACTIVE);
    PIDSetState(&pidY, PID_STATE_ACTIVE);
    
    for (int i = 0; i < n; i++)
    {
        snprintf((char*)strbuf, 49, "\tTarget No.%d(%d,%d)", i, waypoints[i].x, waypoints[i].y);
        HostSendLog(LOG_COLOR_BLACK, strbuf);
        
        snprintf((char*)strbuf, 49, "%d", i);
        HMISetValue("t2.txt", strbuf);
        snprintf((char*)strbuf, 49, "%ds", waypoints[i].stopT);
        HMISetValue("t4.txt", strbuf);
        
        PIDSetTarget(&pidX, waypoints[i].x);
        PIDSetTarget(&pidY, waypoints[i].y);
        while (pidX.e  > ERROR_RANGE || pidY.e > ERROR_RANGE || pidX.e  < -ERROR_RANGE || pidY.e  < -ERROR_RANGE)
        {
            HAL_Delay(100);
        }
        
        snprintf((char*)strbuf, 49, "\tIn point No.%d(%d,%d)\tHold for %d s", i, waypoints[i].x, waypoints[i].y, waypoints[i].stopT);
        HostSendLog(LOG_COLOR_BLACK, strbuf);
        
        
        HAL_Delay(waypoints[i].stopT * 1000);
    }
    
    HMIGetOrder();
    
    PIDSetState(&pidX, PID_STATE_DEACTIVE);
    PIDSetState(&pidY, PID_STATE_DEACTIVE);
    
    ServoSetState(&servoX, SERVO_STATE_DEACTIVE);
    ServoSetState(&servoY, SERVO_STATE_DEACTIVE);
    
    HMISetValue("t2.txt", "end");
    HMISetValue("t4.txt", "end");
    HAL_Delay(1000);
    
    HMISendOrder("page", "page0");
}

void DebugServo() // Debug Servo
{
    u8 order[10] = { 0 }, servo = 2, buf[10];
    int8_t sX = 0, sY = 0;
    
    user_main_debug("Debug Mode");
    
    ServoSetState(&servoX, SERVO_STATE_ACTIVE);
    ServoSetState(&servoY, SERVO_STATE_ACTIVE);
    
    while (order[0] != 'O' || order[1] != 'D' || order[2] != 'E')
    {
        HMIGetData(order);
        if (order[1] != 'S' || order[2] != 'V')
            continue;
        if (order[3] == 'X' && order[4] == '+') sX++;
        if (order[3] == 'X' && order[4] == '-') sX--;
        if (order[3] == 'Y' && order[4] == '+') sY++;
        if (order[3] == 'Y' && order[4] == '-') sY--;
        
        ServoSetAngle(&servoX, sX);
        ServoSetAngle(&servoY, sY);
        
        itoa(sX, (char*)buf, 10);
        HMISetValue("t2.txt", (char*)buf);
        itoa(sY, (char*)buf, 10);
        HMISetValue("t4.txt", (char*)buf);
    }
    
    ServoSetState(&servoX, SERVO_STATE_DEACTIVE);
    ServoSetState(&servoY, SERVO_STATE_DEACTIVE);
    
    HMISendOrder("page", "page0");
}

int AutoPath(struct Waypoint *path)
{
    u8 node[5] = { 0 }, i = 0, n = 0;
    u8 nodeTo, r, l, last_r, last_l;
    while (node[0] != 4)
    {
        user_main_debug("Waiting for input node list");
        
        HMIGetData(node);
        snprintf((char*)strbuf, 49, "\tGet Node List:%d->%d->%d->%d", node[1], node[2], node[3], node[4]);
        HostSendLog(node[0] == 4 ? LOG_COLOR_BLACK : LOG_COLOR_RED, node[0] == 4 ? strbuf : "Error Path");
    }
    
    while (i < node[0])
    {
        nodeTo = node[i + 1] - '0' - 1;
        last_r = r;
        last_l = l;
        r = nodeTo / 3;
        l = nodeTo % 3;
        
        if (i == 0 || r == 1 || l == 1 || last_l == 1 || last_r == 1 || (last_l + last_r - l - r) % 2 == 1)
        {
            (path + n)->x = coor[r][l][0];
            (path + n)->y = coor[r][l][1];
            (path + n)->stopT = 0;
            n++;
        }
        else
        {
            (path + n + 1)->x = coor[r][l][0];
            (path + n + 1)->y = coor[r][l][1];
            (path + n + 1)->stopT = 0;
            
            (path + n)->x = ((path + n + 1)->x + (path + n - 1)->x) / 2 + BIAS;
            (path + n)->y = ((path + n + 1)->y + (path + n - 1)->y) / 2 + BIAS;
            (path + n)->stopT = 0;
            
            n += 2;
        }
        
        i++;
    }
    path->stopT = 1;
    return n;
}

void ManualServo()
{
    u8 od = 0;
    s16 ang;
    
    ServoSetState(&servoX, SERVO_STATE_ACTIVE);
    ServoSetState(&servoY, SERVO_STATE_ACTIVE);
    
    while (od != 'E')
    {
        if (od == 'X' || od == 'Y')
        {
            ang = HMIReadInt() - 500;
            
            if (ang > 500 || ang < -500) continue;
            
            ServoSetAngle((od == 'X' ? &servoX : &servoY), ang);
            
            //            snprintf((char*)strbuf, 49, "\tServo%c set %d", od, (int)ang);
            //            HostSendLog(LOG_COLOR_BLACK, strbuf);
        }
        od = HMIGetOrder();
    }
    
    ServoSetState(&servoX, SERVO_STATE_DEACTIVE);
    ServoSetState(&servoY, SERVO_STATE_DEACTIVE);
    
    user_main_debug("Exit Manual Mode");
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
    pidX.obj = &servoX;
    pidX.func = &ServoSetAngle;
    pidY.obj = &servoY;
    pidY.func = &ServoSetAngle;
    PIDInit();
    
    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_TIM3_Init();
    MX_TIM6_Init();
    MX_USART1_UART_Init();
    MX_USART3_UART_Init();
    MX_USART2_UART_Init();
    /* USER CODE BEGIN 2 */
    int mode = 0;
    struct Waypoint path[10];
    u8 nodeN = 0;
    
    HMI_Init();
    
    user_main_debug("Init Done!");
    
    HAL_UART_Receive_DMA(&huart2, posBUF, 4);
    
    HAL_TIM_Base_Start_IT(&htim6);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        user_main_debug("Waiting Order");
        
        mode = HMIGetOrder();
        switch (mode - '0')
        {
        case 1:
            user_main_debug("Go to Mode 1");
            PointCtr((struct Waypoint*)wp1, 1);
            break;
        case 2:
            user_main_debug("Go to Mode 2");
            PointCtr((struct Waypoint*)wp2, 2);
            break;
        case 3:
            user_main_debug("Go to Mode 3");
            PointCtr((struct Waypoint*)wp3, 3);
            break;
        case 4:
            user_main_debug("Go to Mode 4");
            PointCtr((struct Waypoint*)wp4, 3);
            break;
        case 5:
            user_main_debug("Go to Mode 5");
            PointCtr((struct Waypoint*)wp5, 4);
            break;
        case 6:
            user_main_debug("Go to Mode 6");
            nodeN = AutoPath(path);
            PointCtr(path, nodeN);
            break;
        case 7:
            user_main_debug("Go to Mode 7");
            PointCtr((struct Waypoint*)wp7, 11);
            break;
        case 0:
            user_main_debug("Go to Mode 0");
            DebugServo();
            break;
        case 9:
            user_main_debug("Go to Mode 9");
            ManualServo();
        default:
            break;
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
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 84;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
    /** Enables the Clock Security System
    */
    HAL_RCC_EnableCSS();
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
           ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
         /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
