/* vim:set ts=4 sw=4: */

#ifndef __P_KBD_H
#define __P_KBD_H

#ifdef __cplusplus
 extern "C" {
#endif

extern ps2_status k_status ;

typedef struct __KBD_attr__ {
    uint8_t sample_rate ;
    uint8_t resolution ;
} KBD_attr ;

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
} kbd_package ;

/**
   Functions
 */
void clear_KBD_env( void ) ;
void KBD_ctrl( void ) ;

GPIO_PinState k_read_clk ( void ) ;
GPIO_PinState k_read_data ( void ) ;

void k_write_clk( int ) ;
void k_write_data( int ) ;

#if 0
void k_read_bits(void) ;
#endif

#ifdef __cplusplus
}
#endif

#endif  /* __P_KBD_H */
