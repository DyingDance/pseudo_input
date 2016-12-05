/* vim:set ts=4 sw=4: */
/**
  ******************************************************************************
  * File Name          : p_KBD.c
  * Description        : pseudo KBD operation implement
  ******************************************************************************
  */
#include "main.h"
#include "stm32f0xx_it.h"
#include "code.h"

ps2_status k_status ;
ps2_event k_event ;
data_package k_data ;
ps2_package k_package ;
trans_phase k_trans ;
trans_load k_load ;
clk_line_phase k_clk ;

uint8_t kbd_led = 0 ;

static void get_event( uint8_t ) ;

void clear_KBD_env( void )
{
    k_status = Power_On ;
    k_load.content = standby ;
    k_load.done = 1 ;
    k_load.load_buf[8] = 0 ;    /* clear buffer count */
    k_trans = idle ;
    k_ticks = 0 ;
    SetTimeout( wait_4000ms, KBD ) ;
}

void KBD_ctrl (void)
{
    static uint8_t k_mission , k_job , hot ;
    switch ( k_status ) {
        case Power_On:
            switch ( k_load.content ) {
                case standby:
                    if ( GetRemainTime( KBD ) == 0 ) {
                        /* wait for 4 second ,without reset command, should
                         * reset actively */
                        hot = 1 ;
                        k_status = Reseting ;
                    }
                    break ;
                case command:
                    if (( k_trans == idle ) && k_load.done ) {
                        if ( k_data.valid && ( k_data.d != 0xe0 )) {
                            k_mission = k_data.d ;
                            k_load.load_buf[0] = 0xfa ;
                            k_load.load_buf[8] = 1 ;
                            HAL_NVIC_DisableIRQ( SysTick_IRQn ) ;
                            if( k_load.done ) k_load.content = answer ;
                            k_load.done = 0 ;
                            HAL_NVIC_EnableIRQ( SysTick_IRQn ) ;
                        }
                        else {
                            HAL_NVIC_DisableIRQ( SysTick_IRQn ) ;
                            if( k_load.done ) k_load.content = standby ;
                            HAL_NVIC_EnableIRQ( SysTick_IRQn ) ;
                            SetTimeout( wait_1000ms  , KBD ) ;
                        }
                    }
                    break ;
                case answer:
                    if ( k_load.done ) {
                        if ( k_load.load_buf[8] == 0 ) {
                            if ( k_mission == 0xff ) {
                                hot = 0 ;
                            }
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
                        /* wait for 4 second ,without reset command, should send
                         * 0xAA, 0x00 actively */
                        /* before reporting, disable clk interrupt */
                    if ( GetRemainTime( KBD ) == 0 ) {
                        if (( k_trans == idle ) && k_load.done ) {
                            k_load.load_buf[0] = 0xaa ;
                            k_load.load_buf[8] = 1 ;
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
#if 0   /* I have ran out of ideas ,only can skip Config step  */
                            if ( hot ) k_status = stream ;
                            else  k_status = Config ;
#else
                            k_status = stream ;
#endif
                            HAL_NVIC_DisableIRQ( SysTick_IRQn ) ;
                            if( k_load.done ) k_load.content = standby ;
                            HAL_NVIC_EnableIRQ( SysTick_IRQn ) ;
                        }
                    }
                    break ;
                case command:
                    /* has already received an command , Most likely to be F2 */
#if 0   /* I have ran out of ideas ,only can skip Config step  */
                    if ( hot ) k_status = stream ;
                    else k_status = Config ;
#else
                    k_status = stream ;
#endif
                    break ;
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
                            /*if ( k_data.d == 0xed ) */k_mission = k_data.d ;
                            if ( k_data.d == 0xF2 ) {
                                k_load.load_buf[0] = 0xfa ;
                                k_load.load_buf[1] = 0xab ;
                                k_load.load_buf[2] = 0x83 ;
                                k_load.load_buf[8] = 3 ;
                            }
#if 0
                            else if ( k_data.d == 0xE9 ) {
                                k_load.load_buf[0] = 0xfa ;
                                k_load.load_buf[1] = 0x00 ;
                                k_load.load_buf[2] = 0x00 ;
                                k_load.load_buf[3] = 0x64 ;
                                k_load.load_buf[8] = 4 ;
                            }
#endif
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
                        if ( k_mission == 0xed ) {
                            k_job = 0 ;
                            k_status = stream  ;
                        }
                        else if ( k_mission == 0xFF ) {
                            SetTimeout( wait_500ms, KBD ) ;
                            hot = 0 ;
                            k_status = Reseting ;
                        }
                        /*k_mission = 0 ;*/
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
                            if ( k_load.load_buf[8] == 0 ) {
                                k_job++ ;
                            }
                            else {
                                HAL_NVIC_DisableIRQ( SysTick_IRQn ) ;
                                if( k_load.done ) k_load.content = report ;
                                k_load.done = 0 ;
                                HAL_NVIC_EnableIRQ( SysTick_IRQn ) ;
                            }
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
                    if (/*( k_trans == idle ) && */k_load.done ) {
                        if ( k_data.valid ) {
                            /* if has received a led command */
                            if (( k_mission == 0xed ) && 
                               (( k_data.d & 0xf0 ) == 0 )) {
                                /* do not report any KBD LED command */
                                /*  kbd_led = k_data.d | 0x80 ; */
                                  k_mission = 0 ;
                            }
                            else k_mission = k_data.d ;
                            if ( k_data.d == 0xf2 ) {
                                k_load.load_buf[0] = 0xfa ;
                                k_load.load_buf[1] = 0xab ;
                                k_load.load_buf[2] = 0x83 ;
                                k_load.load_buf[8] = 3 ;
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
                        if ( k_load.load_buf[8] == 0 ) {
                            if ( k_mission == 0xff ) {
                                SetTimeout( wait_500ms, KBD ) ;
                                hot = 0 ;
                                k_status = Reseting ;
                                k_mission = 0 ;
                            }
                            else if ( k_mission == 0xf5 ) {
                                k_status = Config ;
                                k_mission = 0 ;
                            }
                            HAL_NVIC_DisableIRQ( SysTick_IRQn ) ;
                            if( k_load.done ) k_load.content = standby ;
                            HAL_NVIC_EnableIRQ( SysTick_IRQn ) ;
                        }
                    }
                    break ;
                case report:
                    /* KBD event report routine */
                    if ( k_load.done ) {
                    /*if (( k_trans == idle ) && k_load.done ) {*/
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

#ifndef KBD_DEBUG
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
#else
GPIO_PinState k_read_clk ()
{
    return HAL_GPIO_ReadPin ( GPIOC , MOUSE_CLK_R_Pin ) ;
}

GPIO_PinState k_read_data ()
{
    return HAL_GPIO_ReadPin ( GPIOC , MOUSE_DATA_R_Pin ) ;
}

void k_write_clk( int level)
{
    HAL_GPIO_WritePin ( GPIOC , MOUSE_CLK_W_Pin , level ? GPIO_PIN_RESET : GPIO_PIN_SET ) ;
}

void k_write_data( int level)
{
    HAL_GPIO_WritePin ( GPIOC , MOUSE_DATA_W_Pin , level ? GPIO_PIN_RESET : GPIO_PIN_SET ) ;
}
#endif

void get_event( uint8_t point )
{
    static uint8_t ctrl = 0 ;
    k_load.load_buf[0] = 0x8 ;      /* the first byte ,bit3 always 1 */
    if ( k_event.event_info[point].type == 0x0001 ) {    /* sort the type of event */
        if ( k_event.event_info[point].value ) {
            if ( k_event.event_info[point].code < 0x60 ) {
                k_load.load_buf[0] = scan_code[k_event.event_info[point].code][1] ;
                /* For Normal Key , Add an release */
                if(( k_event.event_info[point].code != 0x1d ) &&  /* Left ctrl */
                   ( k_event.event_info[point].code != 0x2a ) &&  /* Left shift */
                   ( k_event.event_info[point].code != 0x36 ) &&  /* Right shift */
                   ( k_event.event_info[point].code != 0x38 )) {  /* Left alt */
                    k_load.load_buf[1] = 0xf0 ;
                    k_load.load_buf[2] = k_load.load_buf[0] ;
                    k_load.load_buf[8] = 3 ;
                }
                else k_load.load_buf[8] = 1 ;
                if ( k_event.event_info[point].code == 0x1d ) ctrl = 1 ;
            }
            else {
                if (( k_event.event_info[point].code == 0x77 ) && ( !ctrl )) {   /* Press Ctrl+Break */
                    k_load.load_buf[0] = 0xe1 ;
                    k_load.load_buf[1] = 0x14 ;
                    k_load.load_buf[2] = 0x77 ;
                    k_load.load_buf[8] = 3 ;
                }
                else {
                    k_load.load_buf[0] = 0xe0 ;
                    k_load.load_buf[1] = scan_code[k_event.event_info[point].code][1] ;
                    if (( k_event.event_info[point].code != 0x61 ) &&  /* Right ctrl */
                        ( k_event.event_info[point].code != 0x65 )) {  /* Right alt */
                    k_load.load_buf[2] = 0xe0 ;
                    k_load.load_buf[3] = 0xf0 ;
                    k_load.load_buf[4] = k_load.load_buf[1] ;
                    k_load.load_buf[8] = 5 ;
                    }
                    else k_load.load_buf[8] = 2 ;
                    if ( k_event.event_info[point].code == 0x61 ) ctrl = 1 ;
                }
            }
        }
        else {
            if ( k_event.event_info[point].code < 0x60 ) {
                k_load.load_buf[0] = 0xf0 ;
                k_load.load_buf[1] = scan_code[k_event.event_info[point].code][1] ;
                k_load.load_buf[8] = 2 ;
                if ( k_event.event_info[point].code == 0x1d ) ctrl = 0 ;
            }
            else {
                if (( k_event.event_info[point].code == 0x77 ) && ( !ctrl )) {
                    k_load.load_buf[0] = 0xe1 ;
                    k_load.load_buf[1] = 0xf0 ;
                    k_load.load_buf[2] = 0x14 ;
                    k_load.load_buf[3] = 0xf0 ;
                    k_load.load_buf[4] = 0x77 ;
                    k_load.load_buf[8] = 5 ;
                }
                else {
                    k_load.load_buf[0] = 0xe0 ;
                    k_load.load_buf[1] = 0xf0 ;
                    k_load.load_buf[2] = scan_code[k_event.event_info[point].code][1] ;
                    k_load.load_buf[8] = 3 ;
                    if ( k_event.event_info[point].code == 0x61 ) ctrl = 0 ;
                }
            }
        }
    }
}

