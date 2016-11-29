/* vim:set ts=4 sw=4: */
/**
  ******************************************************************************
  * File Name          : p_mouse.c
  * Description        : pseudo mouse operation implement
  ******************************************************************************
  */
#include "main.h"
#include "stm32f0xx_it.h"

ps2_status m_status ;
ps2_event m_event ;
data_package m_data ;
ps2_package m_package ;
trans_phase m_trans ;
trans_load m_load ;
//uint8_t m_mission[4] ;
clk_line_phase m_clk ;
/* To fix LB & Drag error */
uint8_t left_button = 0 ;
//data_line_phase m_data ;      


static void get_event( uint8_t ) ;


void clear_mouse_env( void )
{
    m_status = Power_On ;
    m_load.content = standby ;
    m_load.done = 1 ;
    m_load.load_buf[8] = 0 ;    /* clear buffer count */
    // m_data = (data_package){ 0 ,0 , 0 ,} ;
    m_trans = idle ;
    m_ticks = 0 ;
    SetTimeout( wait_4000ms, mouse ) ;
}

#if 0
inline void m_read_bits(void)
{
    m_data.d |== (((u_int8_t)(m_read_data()) & 1 ) << m_step )
}
#endif

void mouse_ctrl (void)
{
    static uint8_t m_mission , m_job ,hot ;
    switch ( m_status ) {
        case Power_On:
            switch ( m_load.content ) {
                case standby:
                    if ( GetRemainTime( mouse ) == 0 ) {
                        /* wait for a second ,without reset command, should
                         * reset actively */
                        m_status = Reseting ;
                        hot = 1 ;
                    }
                    break ;
                case command:
                    if (( m_trans == idle ) && m_load.done ) {
                        if ( m_data.valid && ( m_data.d == 0xff )) {
                            m_load.load_buf[0] = 0xfa ;
                            m_load.load_buf[8] = 1 ;
                            hot = 0 ;    /* power on reset */
                            HAL_NVIC_DisableIRQ( SysTick_IRQn ) ;
                            if( m_load.done ) m_load.content = answer ;
                            m_load.done = 0 ;
                            HAL_NVIC_EnableIRQ( SysTick_IRQn ) ;
                        }
                        else {
                            HAL_NVIC_DisableIRQ( SysTick_IRQn ) ;
                            if( m_load.done ) m_load.content = standby ;
                            HAL_NVIC_EnableIRQ( SysTick_IRQn ) ;
                            SetTimeout( wait_1000ms  , mouse ) ;
                        }
                    }
                    break ;
                case answer:
                    if ( m_load.done ) {
                        if ( m_load.load_buf[8] == 0 ) {
                            m_status = Reseting ;
                            HAL_NVIC_DisableIRQ( SysTick_IRQn ) ;
                            if( m_load.done ) m_load.content = standby ;
                            HAL_NVIC_EnableIRQ( SysTick_IRQn ) ;
                        }
                    }
                    break ;
                default:
                    break ;
            }
            break ;
        case Reseting:
            switch ( m_load.content ) {
                case standby:
                        /* wait for a second ,without reset command, should send
                         * 0xAA, 0x00 actively */
                        /* before reporting, disable clk interrupt */
                    if ( GetRemainTime( mouse ) == 0 ) {
                        if (( m_trans == idle ) && m_load.done ) {
                            m_load.load_buf[0] = 0xaa ;
                            if ( hot ) {
                                m_load.load_buf[8] = 1 ;
                            }
                            else {
                                m_load.load_buf[1] = 0 ;
                                m_load.load_buf[8] = 2 ;
                            }
                            HAL_NVIC_DisableIRQ( SysTick_IRQn ) ;
                            if( m_load.done ) m_load.content = report ;
                            m_load.done = 0 ;
                            HAL_NVIC_EnableIRQ( SysTick_IRQn ) ;
                        }
                    }
                    break ;
                case report:
                    if ( m_load.done ) {
                        if ( m_load.load_buf[8] == 0 ) {
                            m_status = Config ;
                            HAL_NVIC_DisableIRQ( SysTick_IRQn ) ;
                            if( m_load.done ) m_load.content = standby ;
                            HAL_NVIC_EnableIRQ( SysTick_IRQn ) ;
                        }
                    }
                    break ;
                case command:
                    /* has already received an command , Most likely to be F2 */
                    m_status = Config ;
                case answer:
                default:
                    break ;
            }
            break ;
        case Config:
            /*
             *keep receiving comand ,until F4 arrived
             * */
            switch ( m_load.content ) {
                case standby:
                    break ;
                case command:
                    if ( m_load.done ) {
                        if ( m_data.valid ) {
                            m_mission = m_data.d ;
                            if ( m_data.d == 0xF2 ) {
                                m_load.load_buf[0] = 0xfa ;
                                m_load.load_buf[1] = 0x03 ;
                                m_load.load_buf[8] = 2 ;
                            }
                            else if ( m_data.d == 0xE9 ) {
                                m_load.load_buf[0] = 0xfa ;
                                m_load.load_buf[1] = 0x00 ;
                                m_load.load_buf[2] = 0x00 ;
                                m_load.load_buf[3] = 0x64 ;
                                m_load.load_buf[8] = 4 ;
                            }
                            else {
                                m_load.load_buf[0] = 0xfa ;
                                m_load.load_buf[8] = 1 ;
                            }
                            HAL_NVIC_DisableIRQ( SysTick_IRQn ) ;
                            if( m_load.done ) m_load.content = answer ;
                            m_load.done = 0 ;
                            HAL_NVIC_EnableIRQ( SysTick_IRQn ) ;
                        }
                    }
                    break ;
                case answer:
                    if ( m_load.done ) {
                        if ( m_mission == 0xF4 ) {
                            m_job = 0 ;
                            m_status = stream  ;
                        }
                        else if ( m_mission == 0xFF ) {
                            hot = 0 ;
                            SetTimeout( wait_500ms, mouse ) ;
                            m_status = Reseting ;
                        }
                        m_mission = 0 ;
                        HAL_NVIC_DisableIRQ( SysTick_IRQn ) ;
                        if( m_load.done ) m_load.content = standby ;
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
            switch ( m_load.content ) {
                case standby:
                    if (( m_trans == idle ) && m_load.done 
                     && ( m_load.load_buf[8] == 0 )) {
                        if ( m_job < m_event.events ){
                            get_event( m_job ) ;
                            m_load.load_buf[8] = 4 ;
                            HAL_NVIC_DisableIRQ( SysTick_IRQn ) ;
                            if( m_load.done ) m_load.content = report ;
                            m_load.done = 0 ;
                            HAL_NVIC_EnableIRQ( SysTick_IRQn ) ;
                        }
                        else {  /* received event has been done */
                            m_job = 0 ;
                            /*m_load.load_buf[8] = 0 ;*/
                            m_event.events = 0 ;
                        }
                    }
                    break ;
                case command:
                    /* stat should change only by received reset command */
                    if (( m_trans == idle ) && m_load.done ) {
                        if ( m_data.valid ) {
                            m_mission = m_data.d ;
                            m_load.load_buf[0] = 0xfa ;
                            m_load.load_buf[8] = 1 ;
                            HAL_NVIC_DisableIRQ( SysTick_IRQn ) ;
                            if( m_load.done ) m_load.content = answer ;
                            m_load.done = 0 ;
                            HAL_NVIC_EnableIRQ( SysTick_IRQn ) ;
                        }
                    }
                    break ;
                case answer:
                    if ( m_load.done ) {
                        if ( m_load.load_buf[8] == 0 ) {
                            if ( m_mission == 0xff ) {
                                m_status = Reseting ;
                                hot = 0 ;
                                SetTimeout( wait_500ms, mouse ) ;
                                m_status = Reseting ;
                            }
                            m_mission = 0 ;
                            HAL_NVIC_DisableIRQ( SysTick_IRQn ) ;
                            if( m_load.done ) m_load.content = standby ;
                            HAL_NVIC_EnableIRQ( SysTick_IRQn ) ;
                        }
                    }
                    break ;
                case report:
                    /* mouse event report routine */
                    /*if ( m_load.done ) {*/
                    if (( m_trans == idle ) && m_load.done ) {
                        m_job++ ;
                        HAL_NVIC_DisableIRQ( SysTick_IRQn ) ;
                        if( m_load.done ) m_load.content = standby ;
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
#if 0
    static uint32_t m_step = 0 ;
    switch ( m_status )
    {
        case Power_On:
            if ( m_step == 0 ) {
                if(( m_read_clk() == GPIO_PIN_RESET ) && ( m_read_data() == GPIO_PIN_SET )) {
                    HAL_Delay( 10 ) ;
                    if (( m_read_clk() == GPIO_PIN_RESET ) && ( m_read_data() == GPIO_PIN_SET )) {
                        m_step = 1 ;
                    }
                    else {
                        m_step = 0 ;
                    }
                }
            }
            else if ( m_step == 1 ) {
                if((m_read_data() == GPIO_PIN_RESET) && (m_read_clk() == GPIO_PIN_RESET)){
                    HAL_Delay( 4 ) ;
                    if((m_read_data() == GPIO_PIN_RESET) && (m_read_clk() == GPIO_PIN_RESET)){
                        m_step = 2 ;
                    }
                    // an unnecessary move
                    //else m_step = 1 ;
                }
                else if (m_read_data() == GPIO_PIN_SET) {
                    /*in m_step 1 , as clock is low ,data line should be hold low
                      else , we fall back to m_step 0 */
                    m_step = 0 ;
                }
            }
            else if ( m_step == 2 ) {
                /*
                 * host request of send condition ready, waiting for host release
                 * clock line and hold data line,next m_step ,device should generate
                 * clock signal
                 * */
                if ((m_read_data() == GPIO_PIN_RESET) && (m_read_clk() == GPIO_PIN_SET)) {
                    /*
                     * here begin to receive command , most should be reset(0xff)
                     * */
                    
                }
                /*
                 * before device start to generate clock signal, host shouldn't
                 * release data line 
                 * */
                else if (m_read_data() == GPIO_PIN_SET) {
                    m_step = 0 ;
                } 
            }
           break ;
        case Reseting:
            if((m_read_data() == GPIO_PIN_RESET) && (m_read_data() == GPIO_PIN_SET)){
                
            }
            break ;
        case Config:
            break ;
        case stream:
            break ;
        case other:
        case Invild_status:
            break ;
        default:
            break ;
    }
#endif
}

GPIO_PinState m_read_clk ()
{
    return HAL_GPIO_ReadPin ( GPIOC , MOUSE_CLK_R_Pin ) ;
}

GPIO_PinState m_read_data ()
{
    return HAL_GPIO_ReadPin ( GPIOC , MOUSE_DATA_R_Pin ) ;
}

void m_write_clk( int level)
{
    HAL_GPIO_WritePin ( GPIOC , MOUSE_CLK_W_Pin , level ? GPIO_PIN_RESET : GPIO_PIN_SET ) ;
}

void m_write_data( int level)
{
    HAL_GPIO_WritePin ( GPIOC , MOUSE_DATA_W_Pin , level ? GPIO_PIN_RESET : GPIO_PIN_SET ) ;
}

void get_event( uint8_t point )
{
    m_load.load_buf[0] = 0x8 ;      /* the first byte ,bit3 always 1 */
    switch ( m_event.event_info[point].type ) {    /* sort the type of event */
        case 0x0001:      /* button press type event */
            switch ( m_event.event_info[point].code ) {
                case 0x0110:    /* left button event */
                    if ( m_event.event_info[point].value ){  /* left button pressed */
                        m_load.load_buf[0] |= 1 ;
                        left_button = 1 ; 
                    }
                    else left_button = 0 ;  /* left button released */
                    break ;
                case 0x0111:    /* right button event */
                    if ( m_event.event_info[point].value )  /* right button pressed */
                        m_load.load_buf[0] |= 2 ;
                    break ;
                case 0x0112:    /* middle button event */
                    if ( m_event.event_info[point].value )  /* middle button pressed */
                        m_load.load_buf[0] |= 4 ;
                    break ;
                default:
                    break ;
            }
            /* movemont value is zero */
            m_load.load_buf[1] = 0 ;
            m_load.load_buf[2] = 0 ;
            m_load.load_buf[3] = 0 ;
            break ;
        case 0x0002:      /* movement type event  */
            if( left_button ) m_load.load_buf[0] |= 1 ; /* with left button hold */
            switch ( m_event.event_info[point].code ) {
                case 0x0000:      /* X way movement */
                    m_load.load_buf[1] = m_event.event_info[point].value ;
                    if ( m_load.load_buf[1] & 0x80 ) m_load.load_buf[0] |= 0x10 ;
                    m_load.load_buf[2] = 0 ;
                    m_load.load_buf[3] = 0 ;
                    break ;
                case 0x0001:      /* Y way movement */
                    m_load.load_buf[1] = 0 ;
                    m_load.load_buf[2] = m_event.event_info[point].value ;
                    if ( m_load.load_buf[2] & 0x80 ) m_load.load_buf[0] |= 0x20 ;
                    m_load.load_buf[3] = 0 ;
                    break ;
                case 0x0008:      /* wheel movement */
                    m_load.load_buf[1] = 0 ;
                    m_load.load_buf[2] = 0 ;
                    m_load.load_buf[3] = m_event.event_info[point].value ;
                    break ;
                default:
                    m_load.load_buf[1] = 0 ;
                    m_load.load_buf[2] = 0 ;
                    m_load.load_buf[3] = 0 ;
                    break ;
            }
            break ;
        default:
            m_load.load_buf[1] = 0 ;
            m_load.load_buf[2] = 0 ;
            m_load.load_buf[3] = 0 ;
            break ;
    }
}

#if 0
int m_get_cmdding ()
{
    while ( m_read_clk() == GPIO_PIN_RESET )
    {
    }
    return 0 ;
}
#endif

