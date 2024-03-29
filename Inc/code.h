/* vim:set ts=4 sw=4: */
/**
  *****************************************************************************
  * File Name       :code.h
  * Description     :PS2 KeyBoard code set1( event info ) to set2 translate
  * ***************************************************************************
  */
#ifndef __CODE_H__
#define __CODE_H_

#ifdef __cplusplus
 extern "C" {
#endif

static const uint8_t scan_code[][2] = {
    { 0x00 , 0x00 },    /* unused */
    { 0x01 , 0x76 },    /* Escape */
    { 0x02 , 0x16 },    /* 1 ! */
    { 0x03 , 0x1e },    /* 2 @ */
    { 0x04 , 0x26 },    /* 3 # */
    { 0x05 , 0x25 },    /* 4 $ */
    { 0x06 , 0x2e },    /* 5 % */
    { 0x07 , 0x36 },    /* 6 ^ */
    { 0x08 , 0x3d },    /* 7 & */
    { 0x09 , 0x3e },    /* 8 * */
    { 0x0a , 0x46 },    /* 9 ( */
    { 0x0b , 0x45 },    /* 0 ) */
    { 0x0c , 0x4e },    /* - _ */
    { 0x0d , 0x55 },    /* = + */
    { 0x0e , 0x66 },    /* Backspace */
    { 0x0f , 0x0d },    /* Tab */
    { 0x10 , 0x15 },    /* q Q */
    { 0x11 , 0x1d },    /* w W */
    { 0x12 , 0x24 },    /* e E */
    { 0x13 , 0x2d },    /* r R */
    { 0x14 , 0x2c },    /* t T */
    { 0x15 , 0x35 },    /* y Y */
    { 0x16 , 0x3c },    /* u U */
    { 0x17 , 0x43 },    /* i I */
    { 0x18 , 0x44 },    /* o O */
    { 0x19 , 0x4d },    /* p P */
    { 0x1a , 0x54 },    /* [ { */
    { 0x1b , 0x5b },    /* ] } */
    { 0x1c , 0x5a },    /* Enter */
    { 0x1d , 0x14 },    /* Left Control */
    { 0x1e , 0x1c },    /* a A */
    { 0x1f , 0x1b },    /* s S */
    { 0x20 , 0x23 },    /* d D */
    { 0x21 , 0x2b },    /* f F */
    { 0x22 , 0x34 },    /* g G */
    { 0x23 , 0x33 },    /* h H */
    { 0x24 , 0x3b },    /* j J */
    { 0x25 , 0x42 },    /* k K*/
    { 0x26 , 0x4b },    /* l L */
    { 0x27 , 0x4c },    /* ; : */
    { 0x28 , 0x52 },    /* ' " */
    { 0x29 , 0x0e },    /* ` ~ */
    { 0x2a , 0x12 },    /* Left Shift*/
    { 0x2b , 0x5d },    /* \ | */
    { 0x2c , 0x1a },    /* z Z */
    { 0x2d , 0x22 },    /* x X */
    { 0x2e , 0x21 },    /* c C */
    { 0x2f , 0x2a },    /* v V */
    { 0x30 , 0x32 },    /* b B */
    { 0x31 , 0x31 },    /* n N */
    { 0x32 , 0x3a },    /* m M */
    { 0x33 , 0x41 },    /* , < */
    { 0x34 , 0x49 },    /* . > */
    { 0x35 , 0x4a },    /* / ? */
    { 0x36 , 0x59 },    /* Right Shift*/
    { 0x37 , 0x7c },    /* Keypad * */
    { 0x38 , 0x11 },    /* Left Alt */
    { 0x39 , 0x29 },    /* Space */
    { 0x3a , 0x58 },    /* CapsLock */
    { 0x3b , 0x05 },    /* F1 */
    { 0x3c , 0x06 },    /* F2 */
    { 0x3d , 0x04 },    /* F3 */
    { 0x3e , 0x0c },    /* F4 */
    { 0x3f , 0x03 },    /* F5 */
    { 0x40 , 0x0b },    /* F6 */
    { 0x41 , 0x83 },    /* F7 */
    { 0x42 , 0x0a },    /* F8 */
    { 0x43 , 0x01 },    /* F9 */
    { 0x44 , 0x09 },    /* F10 */
    { 0x45 , 0x77 },    /* Num Lock */
    { 0x46 , 0x7e },    /* Scroll Lock */
    { 0x47 , 0x6c },    /* Keypad 7 Home */
    { 0x48 , 0x75 },    /* Keypad 8 Up */
    { 0x49 , 0x7d },    /* Keypad 9 PgUp */
    { 0x4a , 0x7b },    /* Keypad - */
    { 0x4b , 0x6b },    /* Keypad 4 Left */
    { 0x4c , 0x73 },    /* Keypad 5 */
    { 0x4d , 0x74 },    /* Keypad 6 */
    { 0x4e , 0x79 },    /* Keypad + */
    { 0x4f , 0x69 },    /* Keypad 1 End */
    { 0x50 , 0x72 },    /* Keypad 2 Down */
    { 0x51 , 0x7a },    /* Keypad 3 PgDn */
    { 0x52 , 0x70 },    /* Keypad 0 Ins */
    { 0x53 , 0x71 },    /* Keypad . Del */
    { 0x54 , 0x7f },    /* TODO:Unknown */
    { 0x55 , 0x60 },    /* TODO:Unknown */
    { 0x56 , 0x61 },    /* TODO:Unknown */
    { 0x57 , 0x78 },    /* F11 */
    { 0x58 , 0x07 },    /* F12 */
    { 0x59 , 0x0f },    /* TODO:Unknown */
    { 0x5a , 0x17 },    /* TODO:Unknown */
    { 0x5b , 0x1f },    /* TODO:Unknown */
    { 0x5c , 0x27 },    /* TODO:Unknown */
    { 0x5d , 0x2f },    /* TODO:Unknown  use e0 5d code */
    { 0x5e , 0x37 },    /* TODO:Unknown  use e0 5e code */
    { 0x5f , 0x3f },    /* TODO:Unknown  use e0 5f code*/
    { 0x60 , 0x5a },    /* Keypad Enter */
    { 0x61 , 0x14 },    /* right Control */
    { 0x62 , 0x4a },    /* Keypad / */
    { 0x63 , 0x7c },    /* PrtSc SysRq */
    { 0x64 , 0x11 },    /* Right Alt */
    { 0x65 , 0x10 },    /* TODO:Unknown */
    { 0x66 , 0x6c },    /* Home */
    { 0x67 , 0x20 },    /* Up arrow */
    { 0x68 , 0x7d },    /* PgUp */
    { 0x69 , 0x6b },    /* Left arrow */
    { 0x6a , 0x74 },    /* Right arrow */
    { 0x6b , 0x69 },    /* End */
    { 0x6c , 0x72 },    /* Down arrow */
    { 0x6d , 0x7a },    /* PgDn */
    { 0x6e , 0x70 },    /* Insert */
    { 0x6f , 0x71 },    /* Delete */
    { 0x70 , 0x13 },    /* TODO:Unknown  use e0 70 code*/
    { 0x71 , 0x19 },    /* TODO:Unknown */
    { 0x72 , 0x39 },    /* TODO:Unknown */
    { 0x73 , 0x51 },    /* TODO:Unknown */
    { 0x74 , 0x37 },    /* Power */
    { 0x75 , 0x5c },    /* TODO:Unknown */
    { 0x76 , 0x5f },    /* TODO:Unknown */
    { 0x77 , 0x7e },    /* Pause */
    { 0x78 , 0x63 },    /* TODO:Unknown */
    { 0x79 , 0x64 },    /* TODO:Unknown */
    { 0x7a , 0x65 },    /* TODO:Unknown */
    { 0x7b , 0x67 },    /* TODO:Unknown */
    { 0x7c , 0x68 },    /* TODO:Unknown */
    { 0x7d , 0x1f },    /* Left Window */
    { 0x7e , 0x27 },    /* Right Window */
    { 0x7f , 0x2f },    /* menu Key */
} ;

#ifdef __cplusplus
}
#endif

#endif
