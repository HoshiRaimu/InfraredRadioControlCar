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
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "AQM0802.h"

#define _XTAL_FREQ 1000000

void goForward(void);
void goBack(void);
void goRight(void);
void goLeft(void);

void dispInt(uint8_t pos_x, uint8_t pos_y, uint8_t data) {
    uint8_t data1, data2;
    
    //16進数表記にするために上位4ビットと下位4ビットを取り出す
    data1 = (data & 0b11110000) >> 4;
    data2 = data & 0b00001111;
    
    //0から9のときとAからFのときの場合分け
    //上位4ビットの表示
    lcdLocateCursor(pos_x, pos_y);
    if(data1 >= 0 && data1 <= 9) {
        data1 |= 0b00110000;
        lcdSendCharacterData(data1);
    }else {
        data1 -= 9;
        data1 |= 0b01000000;
        lcdSendCharacterData(data1);
    }
    
    //下位4ビットの表示
    lcdLocateCursor(pos_x + 1, pos_y);
    if(data2 >= 0 && data2 <= 9) {
        data2 |= 0b00110000;
        lcdSendCharacterData(data2);
    }else {
        data2 -= 9;
        data2 |= 0b01000000;
        lcdSendCharacterData(data2);
    }
    
    return;
}

void main(void) {
    // 動作周波数設定
    OSCCON1bits.NOSC = 0b110;   // 内部クロック使用
    OSCCON1bits.NDIV = 0b0000;  // 分周1:1
    OSCFRQbits.HFFRQ = 0b000;   // 1MHz
    
    // ピン属性設定
    ANSELA = 0b00000000;
    ANSELB = 0b00000000;
    ANSELC = 0b00000000;
    TRISA  = 0b00000011;        //RA0とRA1を入力に設定
    TRISB  = 0b00000011;        //RB0とRB1を入力に設定
    TRISC  = 0b00000000;
    
    //I2Cの設定
    SSP1STAT = 0x80;   // クロック信号は100kHzを使用
    SSP1CON1 = 0x28;   // I2C通信のマスターモードを有効化
    SSP1CON3 = 0x00;   // CON3はデフォルト設定
    SSP1ADD  = 0x09;   //クロック信号速度を100kHzに設定
    
    //Timer1の設定
    T1CLKbits.CS   = 0b0010;        //システムクロックを使用
    T1CONbits.CKPS = 0b00;          //プリスケール値は1:1
    //T1CONbits.RD16 = 0;             //16ビットの値を読めるように許可
    T1CONbits.ON   = 0;             //タイマー1を停止
    
    //PPSの設定
    PPSLOCKbits.PPSLOCKED = 0;      //PPS設定ロックの解除
    //PWMの割り当て
    RC0PPS = 0x09;                  //RC0にCCP1を割り当て     
    RC1PPS = 0x0A;                  //RC1にCCP2を割り当て
    
    //I2Cの設定
    SSP1DATPPS = 0x08;              //SDA入力部をRB0に割り当て
    SSP1CLKPPS = 0x09;              //SCL入力部をRB1に割り当て
    RB0PPS     = 0x15;              //RB0をSDAに割り当て
    RB1PPS     = 0x14;              //RB1をSCLに割り当て
    PPSLOCKbits.PPSLOCKED = 1;      //PPS設定ロック
    
    //PWMの設定
    T2CLKCONbits.CS = 0b0010;       //タイマ2に内部クロックを使用
    
    //CCP1の設定
    CCP1CONbits.EN   = 1;           //PWMを許可
    CCP1CONbits.FMT  = 1;           //right-aligned formatに設定
    CCP1CONbits.MODE = 0b1111;      //PWMモードに設定
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
    
    uint8_t rcv_data[4] = {255, 0, 0, 0};
    
    //TMR2ON = 1;
    
    LATA4 = 0;
    bool led = false;
    
    
    
        // LCDモジュール電源安定化時間待ち
    __delay_ms(100);
     
    // LCD初期化
    lcdInitialize();
 
    // LCD表示位置を左上に設定
  
    
    
    
    while(1) {
        
        if(RA1) {
            __delay_ms(50);     //安定化待ち
            LATA2 = 1;          //LED点灯
            __delay_ms(1000);
            
            //リーダーコードの待ち
            while(RA0);
            
            //リーダーコードの長さを測定
            //Timer1を開始
            TMR1H  = 0;
            TMR1L  = 0;
            TMR1ON = 1;
            while(!RA0);
            TMR1ON = 0;
            
            
            //LOWになるのを待つ
            while(RA0);

            for(int i = 0; i < 4; i++) {
                rcv_data[i] = 0;
                for(int j = 7; j >= 0; j--) {
                    //HIGHの時間つぶし
                    while(!RA0);

                    //LOWの時間を測定
                    TMR1H  = 0;
                    TMR1L  = 0;
                    TMR1ON = 1;
                    while(RA0);
                    TMR1ON = 0;

                    //LOWの時間が0x04よりも長ければ1と判断
                    if(TMR1H >= 0x04) {
                        rcv_data[i] = rcv_data[i] | (uint8_t)(0b00000001 << j);
                    }
                }
            }
            TMR1ON = 0;
            LATA2 = 0; 
            
            /*
            if((rcv_data[0] == 0xE7) && (rcv_data[1] == 0x30) && (rcv_data[2] == 0xD1) && (rcv_data[3] == 0x2E)) {
                if(~led) {
                    LATA4 = 1;
                    led = true;
                }else {
                    LATA4 = 0; 
                    led = false;
                } 
            }
            */
        }
        
        dispInt(1, 1, rcv_data[0]);
        dispInt(5, 1, rcv_data[1]);
        dispInt(1, 2, rcv_data[2]);
        dispInt(5, 2, rcv_data[3]);
        //lcdLocateCursor(1, 1);
        //printf("Hello");
        //lcdSendCharacterData(rcv_data[0]);
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