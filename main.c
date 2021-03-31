/*
 * File:   main.c
 * Author: raimu
 *
 * Created on 2021/03/30, 20:31
 */

// CONFIG1
#pragma config FEXTOSC = OFF    // External Oscillator mode selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINT1  // Power-up default value for COSC bits (HFINTOSC (1MHz))
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; i/o or oscillator function on OSC2)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (FSCM timer enabled)

// CONFIG2
#pragma config MCLRE = OFF      // Master Clear Enable bit (MCLR pin function is port defined function)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config LPBOREN = OFF    // Low-Power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = ON       // Brown-out reset enable bits (Brown-out Reset Enabled, SBOREN bit is ignored)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (VBOR) set to 1.9V on LF, and 2.45V on F Devices)
#pragma config ZCD = OFF        // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR.)
#pragma config PPS1WAY = ON     // Peripheral Pin Select one-way control (The PPSLOCK bit can be cleared and set only once in software)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a reset)

// CONFIG3
#pragma config WDTCPS = WDTCPS_31// WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF       // WDT operating mode (WDT Disabled, SWDTEN is ignored)
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4
#pragma config WRT = OFF        // UserNVM self-write protection bits (Write protection off)
#pragma config SCANE = available// Scanner Enable bit (Scanner module is available for use)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (High Voltage on MCLR/Vpp must be used for programming)

// CONFIG5
#pragma config CP = OFF         // UserNVM Program memory code protection bit (Program Memory code protection disabled)
#pragma config CPD = OFF        // DataNVM code protection bit (Data EEPROM code protection disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

#define _XTAL_FREQ 1000000

void goForward(void);
void goBack(void);
void goRight(void);
void goLeft(void);

void main(void) {
    // 動作周波数設定
    OSCCON1bits.NOSC = 0b110;   // 内部クロック使用
    OSCCON1bits.NDIV = 0b0000;  // 分周1:1
    OSCFRQbits.HFFRQ = 0b000;   // 1MHz
    
    // ピン属性設定
    ANSELA = 0b00000000;
    ANSELB = 0b00000000;
    ANSELC = 0b00000000;
    TRISA  = 0b00000000;
    TRISB  = 0b00000000;
    TRISC  = 0b00000000;
    
    //PWMの設定
    PPSLOCKbits.PPSLOCKED = 0;      // PPS設定ロックの解除
    RC1PPS = 0x09;                  //PWMの割り当て
    RC2PPS = 0x0A;
    PPSLOCKbits.PPSLOCKED = 1;      // PPS設定ロック
    
    T2CLKCONbits.CS = 0b0010;       //タイマ2に内部クロックを使用
    
    //CCP1の設定
    CCP1CONbits.EN   = 1;             //PWMを許可
    CCP1CONbits.FMT  = 1;         //right-aligned formatに設定
    CCP1CONbits.MODE = 0b1111;    //PWMモードに設定
    //CCP2の設定
    CCP2CONbits.EN   = 1;
    CCP2CONbits.FMT  = 1;
    CCP2CONbits.MODE = 0b1111;
    
    //CCP1と2にタイマ2を使用
    CCPTMRS0bits.C1TSEL = 0b01;
    CCPTMRS0bits.C2TSEL = 0b01;
    
    T2CONbits.CKPS     = 0b110;     //プリスケーラを64に設定
    PR2                = 77;        //50Hzに設定
    
    //CCP1のデューティーサイクルを設定
    CCPR1H = (uint8_t)(100 >> 2);
    CCPR1L = (uint8_t)(100 << 6);
    
    //CCP2のデューティーサイクルを設定
    CCPR2H = (uint8_t)(33 >> 2);
    CCPR2L = (uint8_t)(33 << 6);
    
    TMR2ON = 1;
    while(1) {
        goForward();
        __delay_ms(2000);
        goBack();
        __delay_ms(2000);
    }
    
    return;
}

void goForward() {
    CCPR1H = (uint8_t)(60 >> 2);
    CCPR1L = (uint8_t)(60 << 6);
    
    CCPR2H = (uint8_t)(60 >> 2);
    CCPR2L = (uint8_t)(60 << 6);

    return;
}

void goBack() {
    CCPR1H = (uint8_t)(130 >> 2);
    CCPR1L = (uint8_t)(130 << 6);
    
    CCPR2H = (uint8_t)(130 >> 2);
    CCPR2L = (uint8_t)(130 << 6);

    return;
}

void goRight() {
    CCPR1H = (uint8_t)(60 >> 2);
    CCPR1L = (uint8_t)(60 << 6);
    
    CCPR2H = (uint8_t)(60 >> 2);
    CCPR2L = (uint8_t)(60 << 6);

    return;
}

void goLeft() {
    CCPR1H = (uint8_t)(60 >> 2);
    CCPR1L = (uint8_t)(60 << 6);
    
    CCPR2H = (uint8_t)(60 >> 2);
    CCPR2L = (uint8_t)(60 << 6);

    return;
}