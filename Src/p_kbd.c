/* vim:set ts=4 sw=4: */
/**
  ******************************************************************************
  * File Name          : p_KBD.c
  * Description        : pseudo KBD operation implement
  ******************************************************************************
  */
#include "main.h"
#include "stm32f0xx_it.h"

ps2_status k_status ;
ps2_event k_event ;
data_package k_data ;
ps2_package k_package ;
trans_phase k_trans ;
trans_load k_load ;
clk_line_phase k_clk ;


static void get_event( uint8_t ) ;


void clear_KBD_env( void )
{
    k_status = Power_On ;
    k_load.content = standby ;
    k_load.done = 1 ;
    k_load.load_buf[8] = 0 ;    /* clear buffer count */
    // k_data = (data_package){ 0 ,0 , 0 ,} ;
    k_trans = idle ;
    k_ticks = 0 ;
    SetTimeout( wait_4000ms, KBD ) ;
}

void KBD_ctrl (void)
{
    static uint32_t p_timeout ;
    static uint8_t k_mission , k_job ,hot ;
    switch ( k_status ) {
        case Power_On:
            switch ( k_load.content ) {
                case standby:
                    if ( GetRemainTime( KBD ) == 0 ) {
                        /* wait for a second ,without reset command, should
                         * reset actively */
                        k_status = Reseting ;
                        hot = 1 ;
                    }
                    break ;
                case command:
                    if (( k_trans == idle ) && k_load.done ) {
                        if ( k_data.valid && ( k_data.d == 0xff )) {
                            k_load.load_buf[0] = 0xfa ;
                            k_load.load_buf[8] = 1 ;
                            hot = 0 ;    /* power on reset */
                            HAL_NVIC_DisableIRQ( SysTick_IRQn ) ;
                            if( k_load.done ) k_load.content = answer ;
                            k_load.done = 0 ;
                            HAL_NVIC_EnableIRQ( SysTick_IRQn ) ;
                        }
                        else {
                            HAL_NVIC_DisableIRQ( SysTick_IRQn ) ;
                            if( k_load.content != command ) k_load.content = standby ;
                            HAL_NVIC_EnableIRQ( SysTick_IRQn ) ;
                            SetTimeout( wait_1000ms  , KBD ) ;
                        }
                    }
                    break ;
                case answer:
                    if ( k_load.done ) {
                        if ( k_load.load_buf[8] == 0 ) {
                            k_status = Reseting ;
                            HAL_NVIC_DisableIRQ( SysTick_IRQn ) ;
                            if( k_load.done ) k_load.content = standby ;
                            HAL_NVIC_EnableIRQ( SysTick_IRQn ) ;
                        }
                    }
                    break ;
                default:
                    break ;
            }
            break ;
        case Reseting:
            switch ( k_load.content ) {
                case standby:
                        /* wait for a second ,without reset command, should send
                         * 0xAA, 0x00 actively */
                        /* before reporting, disable clk interrupt */
                    if ( GetRemainTime( KBD ) == 0 ) {
                        if (( k_trans == idle ) && k_load.done ) {
                            k_load.load_buf[0] = 0xaa ;
                            if ( hot ) {
                                k_load.load_buf[8] = 1 ;
                            }
                            else {
                                k_load.load_buf[1] = 0 ;
                                k_load.load_buf[8] = 2 ;
                            }
                            HAL_NVIC_DisableIRQ( SysTick_IRQn ) ;
                            if( k_load.done ) k_load.content = report ;
                            k_load.done = 0 ;
                            HAL_NVIC_EnableIRQ( SysTick_IRQn ) ;
                        }
                    }
                    break ;
                case report:
                    if ( k_load.done ) {
                        if ( k_load.load_buf[8] == 0 ) {
                            k_status = Config ;
                            HAL_NVIC_DisableIRQ( SysTick_IRQn ) ;
                            if( k_load.done ) k_load.content = standby ;
                            HAL_NVIC_EnableIRQ( SysTick_IRQn ) ;
                        }
                    }
                    break ;
                case command:
                    /* has already received an command , Most likely to be F2 */
                    k_status = Config ;
                case answer:
                default:
                    break ;
            }
            break ;
        case Config:
            /*
             *keep receiving comand ,until F4 arrived
             * */
            switch ( k_load.content ) {
                case standby:
                    break ;
                case command:
                    if ( k_load.done ) {
                        if ( k_data.valid ) {
                            k_mission = k_data.d ;
                            if ( k_data.d == 0xF2 ) {
                                k_load.load_buf[0] = 0xfa ;
                                k_load.load_buf[1] = 0x03 ;
                                k_load.load_buf[8] = 2 ;
                            }
                            else if ( k_data.d == 0xE9 ) {
                                k_load.load_buf[0] = 0xfa ;
                                k_load.load_buf[1] = 0x00 ;
                                k_load.load_buf[2] = 0x00 ;
                                k_load.load_buf[3] = 0x64 ;
                                k_load.load_buf[8] = 4 ;
                            }
                            else {
                                k_load.load_buf[0] = 0xfa ;
                                k_load.load_buf[8] = 1 ;
                            }
                            HAL_NVIC_DisableIRQ( SysTick_IRQn ) ;
                            if( k_load.done ) k_load.content = answer ;
                            k_load.done = 0 ;
                            HAL_NVIC_EnableIRQ( SysTick_IRQn ) ;
                        }
                    }
                    break ;
                case answer:
                    if ( k_load.done ) {
                        if ( k_mission == 0xF4 ) {
                            k_job = 0 ;
                            k_status = stream  ;
                        }
                        else if ( k_mission == 0xFF ) {
                            hot = 0 ;
                            SetTimeout( wait_500ms, KBD ) ;
                            k_status = Reseting ;
                        }
                        k_mission = 0 ;
                        HAL_NVIC_DisableIRQ( SysTick_IRQn ) ;
                        if( k_load.done ) k_load.content = standby ;
                        HAL_NVIC_EnableIRQ( SysTick_IRQn ) ;
                    }
                    break ;
                case report:
                default:
                    break ;
            }
            break ;
/* Daily work routine */
        case stream:
            switch ( k_load.content ) {
                case standby:
                    if (( k_trans == idle ) && k_load.done 
                     && ( k_load.load_buf[8] == 0 )) {
                        if ( k_job < k_event.events ){
                            get_event( k_job ) ;
                            k_load.load_buf[8] = 4 ;
                            HAL_NVIC_DisableIRQ( SysTick_IRQn ) ;
                            if( k_load.done ) k_load.content = report ;
                            k_load.done = 0 ;
                            HAL_NVIC_EnableIRQ( SysTick_IRQn ) ;
                        }
                        else {  /* received event has been done */
                            k_job = 0 ;
                            /*k_load.load_buf[8] = 0 ;*/
                            k_event.events = 0 ;
                        }
                    }
                    break ;
                case command:
                    /* stat should change only by received reset command */
                    if (( k_trans == idle ) && k_load.done ) {
                        if ( k_data.valid ) {
                            k_mission = k_data.d ;
                            k_load.load_buf[0] = 0xfa ;
                            k_load.load_buf[8] = 1 ;
                            HAL_NVIC_DisableIRQ( SysTick_IRQn ) ;
                            if( k_load.done ) k_load.content = answer ;
                            k_load.done = 0 ;
                            HAL_NVIC_EnableIRQ( SysTick_IRQn ) ;
                        }
                    }
                    break ;
                case answer:
                    if ( k_load.done ) {
                        if ( k_load.load_buf[8] == 0 ) {
                            if ( k_mission == 0xff ) {
                                k_status = Reseting ;
                                hot = 0 ;
                                SetTimeout( wait_500ms, KBD ) ;
                                k_status = Reseting ;
                            }
                            k_mission = 0 ;
                            HAL_NVIC_DisableIRQ( SysTick_IRQn ) ;
                            if( k_load.done ) k_load.content = standby ;
                            HAL_NVIC_EnableIRQ( SysTick_IRQn ) ;
                        }
                    }
                    break ;
                case report:
                    /* KBD event report routine */
                    /*if ( k_load.done ) {*/
                    if (( k_trans == idle ) && k_load.done ) {
                        k_job++ ;
                        HAL_NVIC_DisableIRQ( SysTick_IRQn ) ;
                        if( k_load.done ) k_load.content = standby ;
                        HAL_NVIC_EnableIRQ( SysTick_IRQn ) ;
                    }
                default:
                    break ;
            }
            break ;

        case other:
        case Invild_status:
        default:
            break ;
    }
}

GPIO_PinState k_read_clk ()
{
    return HAL_GPIO_ReadPin ( GPIOA , KBD_CLK_R_Pin ) ;
}

GPIO_PinState k_read_data ()
{
    return HAL_GPIO_ReadPin ( GPIOA , KBD_DATA_R_Pin ) ;
}

void k_write_clk( int level)
{
    HAL_GPIO_WritePin ( GPIOA , KBD_CLK_W_Pin , level ? GPIO_PIN_RESET : GPIO_PIN_SET ) ;
}

void k_write_data( int level)
{
    HAL_GPIO_WritePin ( GPIOA , KBD_DATA_W_Pin , level ? GPIO_PIN_RESET : GPIO_PIN_SET ) ;
}

void get_event( uint8_t point )
{
    k_load.load_buf[0] = 0x8 ;      /* the first byte ,bit3 always 1 */
    switch ( k_event.event_info[point].type ) {    /* sort the type of event */
        case 0x0001:      /* button press type event */
            switch ( k_event.event_info[point].code ) {
                case 0x0110:    /* left button event */
                    if ( k_event.event_info[point].value )  /* left button pressed */
                        k_load.load_buf[0] |= 1 ;
                    break ;
                case 0x0111:    /* right button event */
                    if ( k_event.event_info[point].value )  /* right button pressed */
                        k_load.load_buf[0] |= 2 ;
                    break ;
                case 0x0112:    /* middle button event */
                    if ( k_event.event_info[point].value )  /* middle button pressed */
                        k_load.load_buf[0] |= 4 ;
                    break ;
                default:
                    break ;
            }
            /* movemont value is zero */
            k_load.load_buf[1] = 0 ;
            k_load.load_buf[2] = 0 ;
            k_load.load_buf[3] = 0 ;
            break ;
        case 0x0002:      /* movement type event  */
            switch ( k_event.event_info[point].code ) {
                case 0x0000:      /* X way movement */
                    k_load.load_buf[1] = k_event.event_info[point].value ;
                    if ( k_load.load_buf[1] & 0x80 ) k_load.load_buf[0] |= 0x10 ;
                    k_load.load_buf[2] = 0 ;
                    k_load.load_buf[3] = 0 ;
                    break ;
                case 0x0001:      /* Y way movement */
                    k_load.load_buf[1] = 0 ;
                    k_load.load_buf[2] = k_event.event_info[point].value ;
                    if ( k_load.load_buf[2] & 0x80 ) k_load.load_buf[0] |= 0x20 ;
                    k_load.load_buf[3] = 0 ;
                    break ;
                case 0x0008:      /* wheel movement */
                    k_load.load_buf[1] = 0 ;
                    k_load.load_buf[2] = 0 ;
                    k_load.load_buf[3] = k_event.event_info[point].value ;
                    break ;
                default:
                    k_load.load_buf[1] = 0 ;
                    k_load.load_buf[2] = 0 ;
                    k_load.load_buf[3] = 0 ;
                    break ;
            }
            break ;
        default:
            k_load.load_buf[1] = 0 ;
            k_load.load_buf[2] = 0 ;
            k_load.load_buf[3] = 0 ;
            break ;
    }
}

