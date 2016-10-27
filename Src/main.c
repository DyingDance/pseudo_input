/* vim:set ts=4 sw=4: */
/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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

/* USER CODE BEGIN Includes */
uint32_t m_ticks = 0 , m_step= 0 , m_index = 0 ;
uint32_t k_ticks = 0 , k_step= 0 , k_index = 0 ;
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
__IO uint8_t BlinkSpeed = 1 ;
IWDG_HandleTypeDef IwdgHandle ;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void) ;
void Error_Handler(void) ;
static void MX_GPIO_Init(void) ;
static void MX_USART1_UART_Init(void) ;
static void MX_WWDG_Init(void) ;

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();

    /*##-1- Check if the system has resumed from IWDG reset ####################*/
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET) {
  }
  else {
  }

  /*##-2- Get the LSI frequency: TIM14 is used to measure the LSI frequency ###*/
  /*  uwLsiFreq = GetLSIFrequency();  */

  /*##-3- Configure the IWDG peripheral ######################################*/
  /* Set counter reload value to obtain 250ms IWDG TimeOut.
     IWDG counter clock Frequency = LsiFreq / 32
     Counter Reload Value = 250ms / IWDG counter clock period
                          = 0.25s / (32/LsiFreq)
                          = LsiFreq / (32 * 4)
                          = LsiFreq / 128 */
  IwdgHandle.Instance = IWDG;

  IwdgHandle.Init.Prescaler = IWDG_PRESCALER_32;
  IwdgHandle.Init.Reload    = 43608 / 128;
  IwdgHandle.Init.Window    = IWDG_WINDOW_DISABLE;

  if (HAL_IWDG_Init(&IwdgHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-4- Start the IWDG #####################################################*/
  if (HAL_IWDG_Start(&IwdgHandle) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN 2 */
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
  
  clear_sio_env () ;
  clear_mouse_env() ;
  clear_KBD_env() ;
  /* enable clk fall interrupt ,start to receive reset(0xff) command */
  //HAL_NVIC_EnableIRQ(MOUSE_CLK_EXTI_IRQn);
  /* in 30/09/2016, I think this work should be down by clk interrupt */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      /* USER CODE END WHILE */
      BSP_LED_Toggle( LED2 ) ; 
      HAL_Delay( 4 ) ;

      /* USER CODE BEGIN 3 */
#ifndef KBD_DEBUG
      mouse_ctrl() ;
#endif
      KBD_ctrl() ;
      process_command() ;

      if ( HAL_IWDG_Refresh(&IwdgHandle) != HAL_OK ) {
          /* Refresh Error */
          Error_Handler();
      }
  }
      /* USER CODE END 3 */
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/100000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE /*UART_HWCONTROL_RTS_CTS*/ ;
  //huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  //huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if(HAL_UART_DeInit(&huart1) != HAL_OK)
  {
    Error_Handler();
  }  
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MOUSE_DATA_W_Pin MOUSE_CLK_W_Pin */
  GPIO_InitStruct.Pin = MOUSE_DATA_W_Pin|MOUSE_CLK_W_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  /* release clock and data line */
  m_write_clk(GPIO_PIN_SET) ;
  m_write_data(GPIO_PIN_SET) ;

  /*Configure GPIO pins : MOUSE_DATA_R_Pin MOUSE_CLK_R_Pin */
  GPIO_InitStruct.Pin = MOUSE_DATA_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  /* define the clk pin as a interrupt line ,but I'm not very
     convinced that it can work ,incase GPIO_MODE_INPUT is 0x0000
     so this "GPIO_MODE_INPUT | GPIO_MODE_IT_FALLING" is shit*/
  GPIO_InitStruct.Pin = MOUSE_CLK_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT | GPIO_MODE_IT_FALLING ;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : KBD_DATA_W_Pin KBD_CLK_W_Pin LD2_Pin */
  GPIO_InitStruct.Pin = KBD_DATA_W_Pin|KBD_CLK_W_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : KBD_DATA_R_Pin KBD_CLK_R_Pin */
  GPIO_InitStruct.Pin = KBD_DATA_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = KBD_CLK_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT | GPIO_MODE_IT_FALLING ;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, MOUSE_DATA_W_Pin|MOUSE_CLK_W_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, KBD_DATA_W_Pin|KBD_CLK_W_Pin|LD2_Pin, GPIO_PIN_RESET);

  /* set mouse and KBD CLK line EXTI Interruptto the lowest
     priority */
  HAL_NVIC_SetPriority(KBD_CLK_EXTI_IRQn, 0x03, 0x00);
  HAL_NVIC_SetPriority(MOUSE_CLK_EXTI_IRQn, 0x03, 0x00);
  
  /* Enable Interrupt ,wait ...not now */
#if 0
  HAL_NVIC_EnableIRQ(KBD_CLK_EXTI_IRQn);
  HAL_NVIC_EnableIRQ(MOUSE_CLK_EXTI_IRQn);
#endif
  
}

/* USER CODE BEGIN 4 */
/**
  * @brief  EXTI line detection callback.
  * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI line.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin)
    {
        case KEY_BUTTON_PIN: 
            if(BlinkSpeed >= 10)
            {
                BlinkSpeed = 1 ;
            }
            else
            {
                BlinkSpeed ++ ;
            }
            break ;
        case KBD_CLK_R_Pin:
            break ;
        case MOUSE_CLK_R_Pin:
            /* means host may send an command
             * thus data line shoule be high*/
            if( m_read_data() == GPIO_PIN_SET ) {
                if ( m_status == Power_On ) {
                    m_ticks = 10 ;
                    m_load.content = command ;
                    m_load.done = 0x00 ;
                    m_data = (data_package){ 0 ,0 , 0 ,} ;
                    m_step = 0 ;
                    //TODO: clear interrupt flag and disable mouse 
                    // clock line interrupt
                    HAL_NVIC_DisableIRQ(MOUSE_CLK_EXTI_IRQn) ;
                }
            }
            break ;
        default:
            break ;
    }
}

void HAL_SYSTICK_Callback(void)
{
    static uint32_t m_count , k_count ;
#ifndef KBD_DEBUG
    switch ( m_trans) {     /* process mouse transfer */
        case idle:
            if ( m_read_clk() == GPIO_PIN_SET ) {
                if ( m_ticks == 0 ) {
                    switch ( m_load.content ) {
                        case standby:
                            /* clear trans environment variables */
                            m_count = 0 ;
                            break ;
                        case answer:
                        case report:
                            /* ready for device to host trans */
                            if ( m_count < m_load.load_buf[8] ) {
                                m_data.d = m_load.load_buf[m_count] ;
                                m_data.parity = 0 ;
                                /*TODO: m_count should increase after an byte has 
                                 * been sent sucessful */
                                m_load.done = 0 ;
                                /* double check clk line is high */
                                m_write_clk( 1 ) ;
                                m_clk.level = HIGH ;
                                m_ticks = 4 ;
                                m_trans = start_bit ;
                            }
                            else {
                                m_count = 0 ;
                                m_load.done = 1 ;
                                m_load.load_buf[8] = 0 ;
                                /* need change out of interrupt */
                                //m_load.content = standby ;
                            }
                            break ;
                    }
                }
                else --m_ticks ;
            }
            else {
                /* host pull down the clk, communication has been inhibited*/
                m_write_data( 1 ) ;
                m_ticks = 10 ;
                m_step = 0 ;
                /*m_count = 0 ;*/
                m_trans = inhibit ;
            }
            break ;
        case inhibit:
            switch ( m_step ) {
                case 0:
                    /* m_step 0 whait for clk line keep low for at least 100 us */
                    if ( --m_ticks > 0 ) {
                        if ( m_read_clk() == GPIO_PIN_SET ) {
                            m_trans = idle ;
                            m_ticks = 0 ;
                        }
                        else if ( m_read_data() == GPIO_PIN_RESET ) {
                            m_ticks = 0 ;
                            m_step = 2 ;
                        }
                    }
                    else {
                        /*m_count = 0 ; */ /* anyway , answer & report must terminate*/
                        /* FIXME: should clean load_buf and let routine knows a 
                         * transfer fail */
                        if ( m_read_clk() == GPIO_PIN_RESET ) {
                            /* check for clk keep a low level at least 100 us */
                            m_step++ ;
                        }
                        else {
                            m_trans = idle ;
                            m_ticks = 0 ;
                        }
                    }
                    break ;
                case 1:
                    /*
                     * m_step 1 wait for data line fall to low ,and clk must keep low
                     * */
                    if ( m_read_clk() == GPIO_PIN_SET ) {
                        m_trans = idle ;
                        m_ticks = 0 ;
                        /*m_step = 0 ;*/
                    }
                    else if ( m_read_data() == GPIO_PIN_RESET ) {
                        /* got a host command start condition */
                        m_step++ ;  /* next step ,wait for clk rise to high */
                    }
                    break ; 
                case 2:
                    /*
                     * m_step 2 wait for clk rise to high , thus the start
                     * condition is meet
                     * */
                    m_count = 0 ;  /* anyway , answer & report must terminate*/
                    if (( m_read_clk() == GPIO_PIN_SET ) && 
                        ( m_read_data() == GPIO_PIN_RESET )) {
                        m_load.content = command ;
                        m_load.done = 0 ;
                        m_data = (data_package){ 0 ,0 , 0 ,} ;
                        m_trans = start_bit ;
                        m_ticks = 4 ;
                        m_step = 0 ;
                        m_clk.level = HIGH ;
                    }
                    break ;
                default:
                    break ;
            }
            break ;

        case start_bit:
            switch ( m_load.content ) {
                case command:
                    /*
                     * receiving an command
                     * */
                    if ( --m_ticks == 0 ) {
                        m_write_clk( 0 ) ;
                        m_clk.level = LOW ;
                        m_ticks = 4 ;
                        m_step = 0 ;
                        m_trans = significant_bit ;
                    }
                    break ;

                case answer:
                case report:
                    if( m_clk.level == HIGH ) { /* set start bit */
                        --m_ticks ;
                        if ( m_ticks == 1 ) m_write_data( 0 ) ; 
                        else if ( m_ticks == 0 ) {
                            m_write_clk( 0 ) ;
                            m_clk.level = LOW ;
                            m_ticks = 4 ;
                        }
                    }
                    else {  /* data line is high means start bit fault */
                        if ( --m_ticks == 0 ) {
                            m_write_clk ( 1 ) ;
                            m_clk.level = HIGH ;
                            m_ticks = 4 ;
                            m_step = 0 ;
                            m_trans = significant_bit ;
                        }
                    }
                    break ;
                default:
                    //error condition
                    break ;
            }
            break ;

        case significant_bit:
            switch ( m_load.content ) {
                case command:
                    if ( --m_ticks == 0 ) {
                        if ( m_clk.level == LOW ) {
                            m_write_clk(1) ;
                            m_clk.level = HIGH ;
                            m_ticks = 4 ;
                            m_data.d |= (((m_read_data()) & 1 ) << m_step ) ;
                        }
                        else {
                            m_write_clk( 0 ) ;
                            m_clk.level = LOW ;
                            m_ticks = 4 ;
                            if ( ++m_step >= 8 ) {
                                m_trans = parity_bit ;
                                m_step = 0 ;
                            }
                        }
                    }
                    break ;
                case answer:
                case report:
                    if( m_clk.level == HIGH ) { /* set start bit */
                        --m_ticks ;
                        if ( m_ticks == 1 ) {
                            m_write_data ( m_data.d & 1 ) ;
                            m_data.parity += m_data.d & 1 ;
                            m_data.d >>= 1 ;
                        }
                        else if ( m_ticks == 0 ) {
                            m_write_clk( 0 ) ;
                            m_clk.level = LOW ;
                            m_ticks = 4 ;
                        }
                    }
                    else {  /* data line is high means start bit fault */
                        if ( --m_ticks == 0 ) {
                            m_write_clk ( 1 ) ;
                            m_clk.level = HIGH ;
                            m_ticks = 4 ;
                            if ( ++m_step >= 8 ) {
                                m_trans = parity_bit ;
                                m_step = 0 ;
                            }
                        }
                    }
                    break ;
                default:
                    break ;
                    //error condition
            }
            break ;

        case parity_bit:
            switch ( m_load.content ) {
                case command:
                    if ( --m_ticks == 0 ) {
                        if ( m_clk.level == LOW ) {
                            m_write_clk(1) ;
                            m_clk.level = HIGH ;
                            m_ticks = 4 ;
                            m_data.parity = m_data.d ;
                            m_data.valid = 0 ;
                            while ( m_data.parity ) {
                                m_data.parity &= ( m_data.parity-1 ) ;
                                m_data.valid++ ;
                            }
                            if ((m_data.valid & 1 ) != ((m_read_data()) & 1 )) {
                                m_data.valid = 1 ;
                            }
                            else m_data.valid = 0 ;
                        }
                        else {
                            m_write_clk( 0 ) ;
                            m_clk.level = LOW ;
                            m_ticks = 4 ;
                            m_trans = stop_bit ;
                        }
                    }
                    break ; 

                case answer:
                case report:
                    if( m_clk.level == HIGH ) { /* set start bit */
                        --m_ticks ;
                        if ( m_ticks == 1 ) {
                            m_write_data (( m_data.parity & 1 ) ? 0 : 1 ) ;
                        }
                        else if ( m_ticks == 0 ) {
                            m_write_clk( 0 ) ;
                            m_clk.level = LOW ;
                            m_ticks = 4 ;
                        }
                    }
                    else {
                        if ( --m_ticks == 0 ) {
                            m_write_clk ( 1 ) ;
                            m_clk.level = HIGH ;
                            m_ticks = 4 ;
                            m_trans = stop_bit ;
                        }
                    }
                    break ;

                default:
                    //error condition
                    break ;
            }
            break ;

        case stop_bit :
            switch ( m_load.content ) {
                case command:
                    if ( m_clk.level == HIGH ) {
                        --m_ticks ;
                        if ( m_ticks == 1 ) m_write_data( 0 ) ; 
                        else if ( m_ticks == 0 ) {
                            m_write_clk( 0 ) ;
                            m_clk.level = LOW ;
                            m_ticks = 4 ;
                            m_trans = ack_bit ;
                        }
                        /*TODO: Read stop bit here ,but I don`t think it was necessary
                         *      just generate clock */
                    }
                    else {
                        if ( --m_ticks == 0 ) {
                            m_write_clk( 1 ) ;
                            m_clk.level = HIGH ;
                            m_ticks = 4 ;
                        }
                    }
                    break ;

                case answer:
                case report:
                    if( m_clk.level == HIGH ) { /* set stoop bit */
                        --m_ticks ;
                        if ( m_ticks == 1 ) m_write_data( 1 ) ; 
                        else if ( m_ticks == 0 ) {
                            m_write_clk( 0 ) ;
                            m_clk.level = LOW ;
                            m_ticks = 4 ;
                        }
                    }
                    else {
                        if ( --m_ticks == 0 ) {
                            m_write_clk ( 1 ) ;
                            m_clk.level = HIGH ;
                            m_ticks = 4 ;
                            m_step = 0 ;
                            m_trans = tail_bit ;
                        }
                    }
                    break ;

                default:
                    //error condition
                    break ;
            }
            break ;

        case ack_bit:
            if ( m_load.content == command ) {
                if ( m_clk.level == HIGH ) {
                    --m_ticks ;
                    if ( m_ticks == 1 ) m_write_data( 1 ) ;  /* ACK bit end here */
                    else if ( m_ticks == 0 ) {
                        m_write_clk( 0 ) ;
                        m_clk.level = LOW ;
                        m_ticks = 4 ;
                        m_trans = tail_bit ;
                    }
                }
                else {
                    if ( --m_ticks == 0 ) {
                        m_write_clk( 1 ) ;
                        m_clk.level = HIGH ;
                        m_ticks = 4 ;
                    }
                }
            }
            /* device report trans did not have ack bit */
            else {
                //error condition
            }
            break ;

        case tail_bit:
            switch ( m_load.content ) {
                case command:
                    if ( --m_ticks == 0 ) {
                        m_write_clk(1) ;    /* Release clock line */
                        m_clk.level = HIGH ;
                        m_write_data( 1 ) ;    /* Release data line */
                        /* an transfer completed */
                        m_ticks = 10 ;
                        m_trans = idle;
                        m_load.done = 1 ;
                    }
                    break ;

                case answer:
                case report:
                    if ( m_read_clk() == GPIO_PIN_SET ) {
                        m_count ++ ;
                        m_ticks = 10 ;
                        m_trans = idle /*silence*/ ;
                    }
                    break ;
                default:
                    break;
            }
            break ;
/*
        case silence:
            if ( m_read_clk() != GPIO_PIN_SET  )
            {
                m_ticks = 12 ;
            }
            if ( --m_ticks == 0 ) m_trans = idle ;
            break ;
*/
        default:
            break ;

    }
#endif
    switch ( k_trans ) {    /* process KBD transfer */
        case idle:
            if ( k_read_clk() == GPIO_PIN_SET ) {
                if ( k_ticks == 0 ) {
                    switch ( k_load.content ) {
                        case standby:
                            /* clear trans environment variables */
                            k_count = 0 ;
                            break ;
                        case answer:
                        case report:
                            /* ready for device to host trans */
                            if ( k_count < k_load.load_buf[8] ) {
                                k_data.d = k_load.load_buf[k_count] ;
                                k_data.parity = 0 ;
                                /*TODO: k_count should increase after an byte has 
                                 * been sent sucessful */
                                k_load.done = 0 ;
                                /* double check clk line is high */
                                k_write_clk( 1 ) ;
                                k_clk.level = HIGH ;
                                k_ticks = 4 ;
                                k_trans = start_bit ;
                            }
                            else {
                                k_count = 0 ;
                                k_load.done = 1 ;
                                k_load.load_buf[8] = 0 ;
                                /* need change out of interrupt */
                                //k_load.content = standby ;
                            }
                            break ;
                    }
                }
                else --k_ticks ;
            }
            else {
                /* host pull down the clk, communication has been inhibited*/
                k_write_data( 1 ) ;
                k_ticks = 10 ;
                k_step = 0 ;
                /*k_count = 0 ;*/
                k_trans = inhibit ;
            }
            break ;
        case inhibit:
            switch ( k_step ) {
                case 0:
                    /* k_step 0 whait for clk line keep low for at least 100 us */
                    if ( --k_ticks > 0 ) {
                        if ( k_read_clk() == GPIO_PIN_SET ) {
                            k_trans = idle ;
                            k_ticks = 0 ;
                        }
                        else if ( k_read_data() == GPIO_PIN_RESET ) {
                            k_ticks = 0 ;
                            k_step = 2 ;
                        }
                    }
                    else {
                        /*k_count = 0 ; */ /* anyway , answer & report must terminate*/
                        /* FIXME: should clean load_buf and let routine knows a 
                         * transfer fail */
                        if ( k_read_clk() == GPIO_PIN_RESET ) {
                            /* check for clk keep a low level at least 100 us */
                            k_step++ ;
                        }
                        else {
                            k_trans = idle ;
                            k_ticks = 0 ;
                        }
                    }
                    break ;
                case 1:
                    /*
                     * k_step 1 wait for data line fall to low ,and clk must keep low
                     * */
                    if ( k_read_clk() == GPIO_PIN_SET ) {
                        k_trans = idle ;
                        k_ticks = 0 ;
                        /*k_step = 0 ;*/
                    }
                    else if ( k_read_data() == GPIO_PIN_RESET ) {
                        /* got a host command start condition */
                        k_step++ ;  /* next step ,wait for clk rise to high */
                    }
                    break ; 
                case 2:
                    /*
                     * k_step 2 wait for clk rise to high , thus the start
                     * condition is meet
                     * */
                    k_count = 0 ;  /* anyway , answer & report must terminate*/
                    if (( k_read_clk() == GPIO_PIN_SET ) && 
                        ( k_read_data() == GPIO_PIN_RESET )) {
                        k_load.content = command ;
                        k_load.done = 0 ;
                        k_data = (data_package){ 0 ,0 , 0 ,} ;
                        k_trans = start_bit ;
                        k_ticks = 4 ;
                        k_step = 0 ;
                        k_clk.level = HIGH ;
                    }
                    break ;
                default:
                    break ;
            }
            break ;

        case start_bit:
            switch ( k_load.content ) {
                case command:
                    /*
                     * receiving an command
                     * */
                    if ( --k_ticks == 0 ) {
                        k_write_clk( 0 ) ;
                        k_clk.level = LOW ;
                        k_ticks = 4 ;
                        k_step = 0 ;
                        k_trans = significant_bit ;
                    }
                    break ;

                case answer:
                case report:
                    if( k_clk.level == HIGH ) { /* set start bit */
                        --k_ticks ;
                        if ( k_ticks == 1 ) k_write_data( 0 ) ; 
                        else if ( k_ticks == 0 ) {
                            k_write_clk( 0 ) ;
                            k_clk.level = LOW ;
                            k_ticks = 4 ;
                        }
                    }
                    else {  /* data line is high means start bit fault */
                        if ( --k_ticks == 0 ) {
                            k_write_clk ( 1 ) ;
                            k_clk.level = HIGH ;
                            k_ticks = 4 ;
                            k_step = 0 ;
                            k_trans = significant_bit ;
                        }
                    }
                    break ;
                default:
                    //error condition
                    break ;
            }
            break ;

        case significant_bit:
            switch ( k_load.content ) {
                case command:
                    if ( --k_ticks == 0 ) {
                        if ( k_clk.level == LOW ) {
                            k_write_clk(1) ;
                            k_clk.level = HIGH ;
                            k_ticks = 4 ;
                            k_data.d |= (((k_read_data()) & 1 ) << k_step ) ;
                        }
                        else {
                            k_write_clk( 0 ) ;
                            k_clk.level = LOW ;
                            k_ticks = 4 ;
                            if ( ++k_step >= 8 ) {
                                k_trans = parity_bit ;
                                k_step = 0 ;
                            }
                        }
                    }
                    break ;
                case answer:
                case report:
                    if( k_clk.level == HIGH ) { /* set start bit */
                        --k_ticks ;
                        if ( k_ticks == 1 ) {
                            k_write_data ( k_data.d & 1 ) ;
                            k_data.parity += k_data.d & 1 ;
                            k_data.d >>= 1 ;
                        }
                        else if ( k_ticks == 0 ) {
                            k_write_clk( 0 ) ;
                            k_clk.level = LOW ;
                            k_ticks = 4 ;
                        }
                    }
                    else {  /* data line is high means start bit fault */
                        if ( --k_ticks == 0 ) {
                            k_write_clk ( 1 ) ;
                            k_clk.level = HIGH ;
                            k_ticks = 4 ;
                            if ( ++k_step >= 8 ) {
                                k_trans = parity_bit ;
                                k_step = 0 ;
                            }
                        }
                    }
                    break ;
                default:
                    break ;
                    //error condition
            }
            break ;

        case parity_bit:
            switch ( k_load.content ) {
                case command:
                    if ( --k_ticks == 0 ) {
                        if ( k_clk.level == LOW ) {
                            k_write_clk(1) ;
                            k_clk.level = HIGH ;
                            k_ticks = 4 ;
                            k_data.parity = k_data.d ;
                            k_data.valid = 0 ;
                            while ( k_data.parity ) {
                                k_data.parity &= ( k_data.parity-1 ) ;
                                k_data.valid++ ;
                            }
                            if ((k_data.valid & 1 ) != ((k_read_data()) & 1 )) {
                                k_data.valid = 1 ;
                            }
                            else k_data.valid = 0 ;
                        }
                        else {
                            k_write_clk( 0 ) ;
                            k_clk.level = LOW ;
                            k_ticks = 4 ;
                            k_trans = stop_bit ;
                        }
                    }
                    break ; 

                case answer:
                case report:
                    if( k_clk.level == HIGH ) { /* set start bit */
                        --k_ticks ;
                        if ( k_ticks == 1 ) {
                            k_write_data (( k_data.parity & 1 ) ? 0 : 1 ) ;
                        }
                        else if ( k_ticks == 0 ) {
                            k_write_clk( 0 ) ;
                            k_clk.level = LOW ;
                            k_ticks = 4 ;
                        }
                    }
                    else {
                        if ( --k_ticks == 0 ) {
                            k_write_clk ( 1 ) ;
                            k_clk.level = HIGH ;
                            k_ticks = 4 ;
                            k_trans = stop_bit ;
                        }
                    }
                    break ;

                default:
                    //error condition
                    break ;
            }
            break ;

        case stop_bit :
            switch ( k_load.content ) {
                case command:
                    if ( k_clk.level == HIGH ) {
                        --k_ticks ;
                        if ( k_ticks == 1 ) k_write_data( 0 ) ; 
                        else if ( k_ticks == 0 ) {
                            k_write_clk( 0 ) ;
                            k_clk.level = LOW ;
                            k_ticks = 4 ;
                            k_trans = ack_bit ;
                        }
                        /*TODO: Read stop bit here ,but I don`t think it was necessary
                         *      just generate clock */
                    }
                    else {
                        if ( --k_ticks == 0 ) {
                            k_write_clk( 1 ) ;
                            k_clk.level = HIGH ;
                            k_ticks = 4 ;
                        }
                    }
                    break ;

                case answer:
                case report:
                    if( k_clk.level == HIGH ) { /* set stoop bit */
                        --k_ticks ;
                        if ( k_ticks == 1 ) k_write_data( 1 ) ; 
                        else if ( k_ticks == 0 ) {
                            k_write_clk( 0 ) ;
                            k_clk.level = LOW ;
                            k_ticks = 4 ;
                        }
                    }
                    else {
                        if ( --k_ticks == 0 ) {
                            k_write_clk ( 1 ) ;
                            k_clk.level = HIGH ;
                            k_ticks = 4 ;
                            k_step = 0 ;
                            k_trans = tail_bit ;
                        }
                    }
                    break ;

                default:
                    //error condition
                    break ;
            }
            break ;

        case ack_bit:
            if ( k_load.content == command ) {
                if ( k_clk.level == HIGH ) {
                    --k_ticks ;
                    if ( k_ticks == 1 ) k_write_data( 1 ) ;  /* ACK bit end here */
                    else if ( k_ticks == 0 ) {
                        k_write_clk( 0 ) ;
                        k_clk.level = LOW ;
                        k_ticks = 4 ;
                        k_trans = tail_bit ;
                    }
                }
                else {
                    if ( --k_ticks == 0 ) {
                        k_write_clk( 1 ) ;
                        k_clk.level = HIGH ;
                        k_ticks = 4 ;
                    }
                }
            }
            /* device report trans did not have ack bit */
            else {
                //error condition
            }
            break ;

        case tail_bit:
            switch ( k_load.content ) {
                case command:
                    if ( --k_ticks == 0 ) {
                        k_write_clk(1) ;    /* Release clock line */
                        k_clk.level = HIGH ;
                        k_write_data( 1 ) ;    /* Release data line */
                        /* an transfer completed */
                        k_ticks = 40 ;
                        k_trans = idle;
                        k_load.done = 1 ;
                    }
                    break ;

                case answer:
                case report:
                    if ( k_read_clk() == GPIO_PIN_SET ) {
                        k_count ++ ;
                        k_ticks = 10 ;
                        k_trans = idle ;
                    }
                    break ;
                default:
                    break;
            }
            break ;
        default:
            break ;
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
