/* vim:set ts=4 sw=4: */
/**
  ******************************************************************************
  * File Name          : mxconstants.h
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MXCONSTANT_H
#define __MXCONSTANT_H
  /* Includes ------------------------------------------------------------------*/
#include <stdint.h>
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
#define CP210x_RESET GPIO_PIN_12
#define CP210x_RESET_GPIO_Port GPIOB
#define CP210x_SUSPEND_N GPIO_PIN_13
#define CP210x_SUSPEND_N_GPIO_Port GPIOB
#define CP210x_SUSPEND GPIO_PIN_14
#define CP210x_SUSPEND_GPIO_Port GPIOB

#define KBD_DATA_R_Pin GPIO_PIN_0
#define KBD_DATA_R_GPIO_Port GPIOA
#define KBD_DATA_W_Pin GPIO_PIN_1
#define KBD_DATA_W_GPIO_Port GPIOA
#define KBD_CLK_R_Pin GPIO_PIN_2
#define KBD_CLK_R_GPIO_Port GPIOA
#define KBD_CLK_W_Pin GPIO_PIN_3
#define KBD_CLK_W_GPIO_Port GPIOA
#define MOUSE_DATA_R_Pin GPIO_PIN_0
#define MOUSE_DATA_R_GPIO_Port GPIOC
#define MOUSE_DATA_W_Pin GPIO_PIN_1
#define MOUSE_DATA_W_GPIO_Port GPIOC
#define MOUSE_CLK_R_Pin GPIO_PIN_2
#define MOUSE_CLK_R_GPIO_Port GPIOC
#define MOUSE_CLK_W_Pin GPIO_PIN_3
#define MOUSE_CLK_W_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */
#define wait_100ms  10000L
#define wait_500ms  50000L
#define wait_1000ms 100000L
#define wait_4000ms 400000L

/* vision info, used as response of +CMGR command */
#define version "Vanxum Pseudo inputs Ver 0.0.1\r\n"
#define ACK "OK\r\n"

/**
  * @}
  */ 
typedef enum {
    mouse = 0 ,
    KBD ,
    uart ,
    general ,
} timmer ;

typedef enum __ps2_status__ {
     Invild_status = 0 ,
     Power_On  ,
     Reseting ,
     Config ,
     other ,
     stream ,
     error_status ,
 } ps2_status ;

typedef struct __data_package__ {
    /*volatile*/ uint8_t d ;
    /*volatile*/ uint8_t parity ;
    /*volatile*/ uint8_t valid ;
} data_package ;

typedef struct __clk_line_phase__ {
    enum  {
        LOW = 0 ,
        HIGH ,
    } level ;
    uint32_t hold_time ;
} clk_line_phase ;

#if 0
typedef struct __data_line_phase__ {
    uint8_t d ;
    uint8_t parity ;
} data_line_phase ;
#endif

typedef enum __trans_phase__  {
    start_bit = 0 ,
    significant_bit ,
    parity_bit = 9 ,
    stop_bit ,
    ack_bit ,       /* only in command load */
    tail_bit ,      /* for clock complete and clk 
                       data line rise high level */
    /*silence ,*/       /* end of a translate , wait for clock level is high for at
                       least 120us */
    inhibit ,       /* Inhibit communication by pulling Clock low for at least
                       100 microseconds */
    idle = 0xff ,
} trans_phase ;

typedef struct __trans_load__ {
    enum {
        standby ,
        command ,
        answer ,
        report ,
    } content ;
    uint8_t load_buf[9] ;   /* the last byte load_buf[8] is counter */
    uint8_t done ;
} trans_load ;

typedef struct __input_event__ {
    uint16_t type ;
    uint16_t code ;
    uint32_t value ;
} event ;

typedef struct __ps2_events__ {
    uint8_t events ;
    event event_info[4] ;
} ps2_event ;
/**
  * @}
*/ 

#define  UPCASE( c ) ( ((c) >= 'a' && (c) <= 'z') ? ((c) - 0x20) : (c) )

extern uint32_t m_ticks , m_step , m_index ;
extern uint32_t k_ticks , k_step , k_index ;

extern trans_phase m_trans ;
extern trans_load m_load ;
extern data_package m_data ;
extern clk_line_phase m_clk ;
extern ps2_event m_event ;
//extern data_line_phase m_data ;      

extern trans_phase k_trans ;
extern trans_load k_load ;
extern data_package k_data ;
extern clk_line_phase k_clk ;
extern ps2_event k_event ;
#endif /* __MXCONSTANT_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
