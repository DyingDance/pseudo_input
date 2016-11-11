/* vim:set ts=4 sw=4: */
/**
  ******************************************************************************
  * File Name          : c_host.c
  * Description        : communicat with host ,receive mouse and KBD events
  ******************************************************************************
  */
#include <string.h>
#include <stdlib.h>
#include "main.h"
#include "stm32f0xx_it.h"

//#define  ALGORITHM_1

uint8_t cs_phase = 0 ;

static char* KBD_LED[] = { "AT>K\"0000\"\r\n",
                           "AT>K\"0001\"\r\n",
                           "AT>K\"0010\"\r\n",
                           "AT>K\"0011\"\r\n",
                           "AT>K\"0100\"\r\n",
                           "AT>K\"0101\"\r\n",
                           "AT>K\"0110\"\r\n",
                           "AT>K\"0111\"\r\n", } ;

UART_HandleTypeDef huart1;

/* Buffer used for transmission */
uint8_t aTxBuffer[24] ;

/* Buffer used for reception 
 * the size of input_event is 24 byte , buffer can store 3 events*/
uint8_t aRxBuffer[RXBUFFERSIZE] ;

uint8_t msg_pool[72] ;     /* 3 events is 66 bytes */

#ifndef VANXUM_PS2KM 
__IO ITStatus UartReady = RESET ;
#endif

static at_state_enum_type at_state = HUNT ;
rx_queue_buffer rx_buffer ;

void clear_sio_env ( void )
{
    aRxBuffer[0] = '\0' ;
    aTxBuffer[0] = '\0' ;
    rx_buffer.en_q = 0 ;
    rx_buffer.de_q = 0 ;
    rx_buffer.count_q = 0 ;
    rx_buffer.buf = aRxBuffer ; 
    rx_buffer.pool = msg_pool ;
}
    
/*
 * main routine for at command
 * */
void process_command( void )
{
    /*as mouse and KBD finished init , report AT
     * to host ,start trans */
    /* TODO: add KBD status here */
    if (/* m_status != stream && */ k_status != stream ) return ;

    switch ( cs_phase ) {
        case 0:     /* after ps2 init done , Send an AT for check in */
            if ( GetRemainTime( uart ) == 0 ) {
                if( HAL_UART_Transmit_IT( &huart1 , "AT\r\n" , ( strlen( "AT\r\n" ))) 
                            != HAL_OK ) {
                    /*start trans fail , wait 1s then retry*/
                    SetTimeout( wait_1000ms , uart ) ;
                }
                /* XXX: start uart receive here ,never stop... 
                 * so , got the full buffer here */
                else if ( HAL_UART_Receive_IT( &huart1, (uint8_t *)aRxBuffer,
                                RXBUFFERSIZE ) != HAL_OK ) {
                    /*start receive fail , wait 1s then retry*/
                    SetTimeout( wait_1000ms , uart ) ;
                    cs_phase++ ;
                }
                else cs_phase += 2 ;
            }
            break ;

        case 1:    /* receive init fail , should reinit here */ 
            if ( GetRemainTime( uart ) == 0 ) {
                if ( HAL_UART_Receive_IT( &huart1, (uint8_t *)aRxBuffer,
                                          RXBUFFERSIZE ) != HAL_OK ) {
                    SetTimeout( wait_1000ms , uart ) ;
                }
                else cs_phase++ ;
            }
            break ;
        case 2:     /* host should reply a OK */
            if ( atproc_command() == ack ) {
                if ( kbd_led & 0x80 ) cs_phase = 5 ;
                else cs_phase++ ;
            }
            break ;
        case 3:     /* normal program flow  */
            if ( atproc_command() == cmd ) cs_phase++ ;
            else if ( kbd_led & 0x80 ) cs_phase += 2 ;
            break ;
        case 4:
            if ( m_event.events == 0  &&  k_event.events == 0 ) {
                if ( GetRemainTime( uart ) == 0 ) {
                    if( HAL_UART_Transmit_IT( &huart1 , "OK\r\n" , ( strlen( "OK\r\n" ))) 
                                              != HAL_OK ) {
                        /*start trans fail , wait 1s then retry*/
                        SetTimeout( wait_1000ms , uart ) ;
                    }
                    else {
                        if ( kbd_led & 0x80 ) cs_phase++ ;  /* received a kbd led command */
                        else cs_phase--  ; 
                    }
                }
            }
            break ;
        case 5:
            if ( GetRemainTime( uart ) == 0 ) {
                if( HAL_UART_Transmit_IT( &huart1 , (uint8_t*)KBD_LED[kbd_led & 7] , strlen( KBD_LED[0] )) 
                            != HAL_OK ) {
                    /*start trans fail , wait 1s then retry*/
                    SetTimeout( wait_1000ms , uart ) ;
                }
                else {
                    kbd_led = 0x0 ;
                    cs_phase = 2 ; 
                }
            }
            break ;

        default:
            /* error , restore to zero */
            if ( cs_phase > 5 ) cs_phase = 3 ;
            break ;
    }
}

received_data_type atproc_command ( void )
{
    unsigned char cc ;
    static received_data_type ret , pencil_ret ;
    ret = null ;
    while (( cc = rx_de_queue() ) != '\0') {
        switch ( at_state ) {
            case HUNT:
                if (( cs_phase > 2 ) &&( UPCASE( cc ) == 'A' )) at_state = FOUND_A ;
                else if ( UPCASE( cc ) == 'O' ) at_state =  FOUND_O ;
                break ;
            case FOUND_A:
                if ( UPCASE( cc ) == 'T' ) at_state = CAT ;
                else at_state = HUNT ;
                break ;
            case FOUND_O:
                if ( UPCASE( cc ) == 'K' ) at_state = COK ;
                else at_state = HUNT ;
                break ;
            case CAT:
                if ( cc == '\r' ) {
                    at_state = TAIL_1 ;
                    *(rx_buffer.pool)++ = '\0' ; 
                    rx_buffer.pool = msg_pool ;
                    pencil_ret = cmd ; 
                }
                else {
                    if ( rx_buffer.pool < &(msg_pool[69]) ) {
                        *rx_buffer.pool++ = UPCASE( cc ) ; 
                    }
                    else {  /* pool overflow , abandon all received datas
                               state machine back to HUNT */
                        rx_buffer.pool = msg_pool ;
                        at_state = HUNT ;
                    }
                }
                break ;
            case COK:
                if ( cc == '\r' ) {
                    at_state = TAIL_1 ;
                    pencil_ret = ack ; 
                }
                else at_state = Error ;
                break ;
            case TAIL_1:
                if ( cc == '\n' ) {
                    if ( pencil_ret == cmd ) {
                        if ( post_cmd() != 0 ) pencil_ret = garbage ;
                    }
                    at_state = HUNT ;
                    ret = pencil_ret ;
                }
                else at_state = Error ;
                break ;
            case Error:
                at_state = HUNT ;
                ret = garbage ;
            default:
                break ;
        }
    }
    return( ret ) ;
}

unsigned char rx_de_queue ( void )
{
    if ( rx_buffer.count_q > 0 ) {
        /* disable uart interrupt */
        __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);

        if ( rx_buffer.en_q < rx_buffer.count_q ) {  /* buffer has roll back */
            rx_buffer.de_q = RXBUFFERSIZE - ( rx_buffer.count_q - rx_buffer.en_q ) ;
        }
        else {
            rx_buffer.de_q = rx_buffer.en_q - rx_buffer.count_q ;
        }

        rx_buffer.count_q-- ;

        /* restore uart interrupt */
        __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
        
        return ( rx_buffer.buf[rx_buffer.de_q] ) ;
    }
    else return ( '\0' ) ;      /* this '\0' is not for termate string ,
                                   just used to mean buffer empty */
}

int post_cmd ( void )
{
    char *pool_scal ;
    ps2_event *event_pool ;
    pool_scal = ( char * )msg_pool ;

    if ( *pool_scal++ == '>' ) {
        switch ( *pool_scal++ ) {
            case 'M':
                if ( m_status != stream ) return -1 ;
                event_pool = &m_event ;
                break ;
            case 'K':
                if ( k_status != stream ) return -1 ;
                event_pool = &k_event ;
                break ;
        }
        return ( purge_pool( pool_scal , event_pool ) );
    }
    return -1 ;
}

inline int purge_pool ( char *load , ps2_event *event_load )
{
    char *token = NULL ;
    int i , value ,error;

    event_load->events = 0 ;

    if ((token = strtok ( load , "\"" )) != NULL ) { 
        i = 0 ;
        do {
            value = str2hex( token , &error ) ;
            if ( error != 0 ) {
                event_load->events = 0 ;
                return -1 ;
            }
            switch (i) {
                case 0 :
                    event_load->event_info[event_load->events].type = value ;
                    break ;
                case 1 :
                    event_load->event_info[event_load->events].code = value ;
                    break ;
                case 2:
                    event_load->event_info[event_load->events].value = value ;
                    event_load->events++ ;
                    break ;
            }
            i++ ; i %= 3 ;
        } while ((token = strtok ( NULL , "\"" )) != NULL ) ;
    }
    return 0 ;
}

#ifdef ALGORITHM_1
inline int str2hex ( const char *nptr , int *error )
{
    unsigned char s , i , k , p;
    union{
        unsigned int b;
        char a[8] ;
    }  hex ;
    k = strlen(nptr)/2 - 1  ;
    hex.b = 0 ;
    for ( i = 0; i <=k ; i++ ) {
        s = *nptr++ ;
        if ( s >= 'A' && s <= 'F' )  p= s-'A'+10 ;
        else  p = s - '0' ;
        p &= 0xf ; p <<= 4 ;
        s = *nptr++ ;
        if ( s >='A' && s <='F' ) p |= s-'A' + 10 ;
        else p |= s-'0' ;
        hex.a[k-i] = p ;
    }
    *error = 0 ;
    return hex.b ; 
}
#else
inline int str2hex (const char *nptr , int *error )
{
    unsigned char s , i , k ;
    union{
        unsigned int b;
        char a[8] ;
    }  hex ;

    k = strlen(nptr)/2 - 1  ;
    i = 0 ;
    hex.b = 0 ;

    while (( s = *nptr++ ) != '\0' ) {
        if ( s >= 'A' && s <= 'F' ) hex.a[k-i/2] |= s-'A'+10 ;
        else if ( s >= '0' && s <= '9' ) {
            hex.a[k-i/2] |= s - '0' ;
        }
        else {
            *error = -1 ;
            return 0 ;
        }
        if ((i & 1 ) == 0 ) {
            hex.a[k-i/2] &= 0xf ;
            hex.a[k-i/2] <<= 4 ;
        }
        if ( ++i/2 > k ) break ;
    }
    *error = 0 ;
    return hex.b ;
}
#endif

#ifdef VANXUM_PS2KM
/**
  * @brief Handle UART interrupt request.
  * @param huart: UART handle.
  * @retval None
  */
void PS2KM_UART_IRQHandler(UART_HandleTypeDef *huart)
{

#ifndef VANXUM_PS2KM    /* do not check error interrupts , given thar all error
                           interrupts has been disabled */

    /* UART parity error interrupt occurred -------------------------------------*/
    if((__HAL_UART_GET_IT(huart, UART_IT_PE) != RESET) && (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_PE) != RESET))
    {
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_PEF);

        huart->ErrorCode |= HAL_UART_ERROR_PE;
    }

    /* UART frame error interrupt occurred --------------------------------------*/
    if((__HAL_UART_GET_IT(huart, UART_IT_FE) != RESET) && (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_ERR) != RESET))
    {
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_FEF);

        huart->ErrorCode |= HAL_UART_ERROR_FE;
    }

    /* UART noise error interrupt occurred --------------------------------------*/
    if((__HAL_UART_GET_IT(huart, UART_IT_NE) != RESET) && (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_ERR) != RESET))
    {
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_NEF);

        huart->ErrorCode |= HAL_UART_ERROR_NE;
    }

    /* UART Over-Run interrupt occurred -----------------------------------------*/
    if((__HAL_UART_GET_IT(huart, UART_IT_ORE) != RESET) && (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_ERR) != RESET))
    {
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_OREF);

        huart->ErrorCode |= HAL_UART_ERROR_ORE;
    }
#endif

#if !defined(STM32F030x6) && !defined(STM32F030x8)&& !defined(STM32F070xB)&& !defined(STM32F070x6)&& !defined(STM32F030xC)
    /* UART wakeup from Stop mode interrupt occurred -------------------------------------*/
    if((__HAL_UART_GET_IT(huart, UART_IT_WUF) != RESET) && (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_WUF) != RESET))
    {
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_WUF);
        /* Set the UART state ready to be able to start again the process */
        huart->gState = HAL_UART_STATE_READY;
        huart->RxState = HAL_UART_STATE_READY;
        HAL_UARTEx_WakeupCallback(huart);
    }
#endif /* !defined(STM32F030x6) && !defined(STM32F030x8)&& !defined(STM32F070xB)&& !defined(STM32F070x6)&& !defined(STM32F030xC) */

    /* UART in mode Receiver ---------------------------------------------------*/
    if((__HAL_UART_GET_IT(huart, UART_IT_RXNE) != RESET) && (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_RXNE) != RESET))
    {
        PS2KM_Receive_IT(huart);
    }


    /* UART in mode Transmitter ------------------------------------------------*/
    if((__HAL_UART_GET_IT(huart, UART_IT_TXE) != RESET) &&(__HAL_UART_GET_IT_SOURCE(huart, UART_IT_TXE) != RESET))
    {
        UART_Transmit_IT(huart);
    }

    /* UART in mode Transmitter (transmission end) -----------------------------*/
    if((__HAL_UART_GET_IT(huart, UART_IT_TC) != RESET) &&(__HAL_UART_GET_IT_SOURCE(huart, UART_IT_TC) != RESET))
    {
        UART_EndTransmit_IT(huart);
    }

    if(huart->ErrorCode != HAL_UART_ERROR_NONE)
    {
        /* Set the UART state ready to be able to start again the Tx/Rx process */
        huart->gState = HAL_UART_STATE_READY;
        huart->RxState = HAL_UART_STATE_READY;

        HAL_UART_ErrorCallback(huart);
    }  
}
#else
inline void PS2KM_UART_IRQHandler(UART_HandleTypeDef *huart) {}
#endif

#ifdef VANXUM_PS2KM
void PS2KM_Receive_IT(UART_HandleTypeDef *huart)
{
    /* Check that a Rx process is ongoing */
    if(huart->RxState == HAL_UART_STATE_BUSY_RX)
    {
        if ( ++rx_buffer.count_q < RXBUFFERSIZE ) {
        /* if aRxBuffer is full , the current strategy is to throw away
         * this data. here I don't use huart->RxXferCount as the point of 
         * buffer, use en_q is appropriate much more, I think...
         * */
            rx_buffer.buf[rx_buffer.en_q++] = 
                          (unsigned char)(huart->Instance->RDR & (uint8_t)huart->Mask) ;
             rx_buffer.en_q %= RXBUFFERSIZE ;
        }
    }
    else
    {
        /* Clear RXNE interrupt flag */
        __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
    }
}
#else
inline void PS2KM_Receive_IT(UART_HandleTypeDef *huart) {}
#endif
#ifndef  VANXUM_PS2KM
/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle. 
  * @note   This example shows a simple way to report end of IT Tx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: trasfer complete*/
  UartReady = SET ;

  
}
/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: trasfer complete*/
  UartReady = SET ;
  
  
}
#endif

/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
 void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
    Error_Handler();
}
