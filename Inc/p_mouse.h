/* vim:set ts=4 sw=4: */

#ifndef __P_MOUSE_H
#define __P_MOUSE_H

#ifdef __cplusplus
 extern "C" {
#endif

extern ps2_status m_status ;

typedef struct __mouse_attr__ {
    uint8_t sample_rate ;
    uint8_t resolution ;
} mouse_attr ;

typedef struct {
    union {
        char a ;
        struct {
            int LB:1;
            int RB:1;
            int MB:1;
            int con_1:1;
            int x_sign:1;
            int y_sign:1;
            int x_ovf:1;
            int y_ovf:1;
        } byte_1 ;
    } c ;
    char x;
    char y;
    char w;
} ps2_package ;

/**
   Functions
 */
void clear_mouse_env( void ) ;
void mouse_ctrl( void ) ;

GPIO_PinState m_read_clk ( void ) ;
GPIO_PinState m_read_data ( void ) ;

void m_write_clk( int ) ;
void m_write_data( int ) ;

#ifdef __cplusplus
}
#endif

#endif  /* __P_MOUSE_H */
