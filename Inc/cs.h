/* vim:set ts=4 sw=4: */

#ifndef __CS__H__
#define  __CS__H__

#ifdef __cplusplus
 extern "C" {
#endif

typedef enum
{
    prepare ,
    listen ,
    action ,
    reward ,
    mission ,
    KBD_entry ,
    mouse_entry ,
} cs_status ;

typedef enum       
{                                                                      
    HUNT ,              /*  Initial:  looking for a or A                      */
    FOUND_A ,           /*  Found A, looking for t, T or /                    */
    FOUND_O ,           /*  Found O, Lookling for K or k                      */
    CAT ,               /*  Found AT: filling buffer and processing backspace */
    COK ,               /*  Found OK: termanting a processed command          */
    TAIL_1 ,
    Error               /*  Error!  loop until end of line                    */
} at_state_enum_type ;

typedef enum {
    null ,
    ack ,
    cmd ,
    garbage
} received_data_type ;

typedef struct {
    /*volatible*/ uint8_t count_q ;
    uint8_t de_q ;
    /*volatile*/ uint8_t en_q ;
    uint8_t *buf ;
    uint8_t *pool ;
} rx_queue_buffer ;

typedef struct __converter__ {
    int value ;
    uint8_t error ;
} converter ;

extern at_state_enum_type at_state ;

extern UART_HandleTypeDef huart1 ;

extern received_data_type at_type ;

void clear_sio_env ( void ) ;
void process_command( void ) ;

received_data_type atproc_command ( void ) ;
unsigned char rx_de_queue ( void ) ;
int post_cmd ( void ) ;
inline int purge_pool ( char *, ps2_event * ) ;
inline int str2hex ( const char *, int * ) ;

void PS2KM_UART_IRQHandler(UART_HandleTypeDef *) ;
void PS2KM_Receive_IT(UART_HandleTypeDef *) ;

#ifdef __cplusplus
}
#endif

#endif  /* __CS__H__ */
