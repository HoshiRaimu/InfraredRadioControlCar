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

#define _XTAL_FREQ 32000000

#define TARGET     512

void init(void);
uint16_t changePID(uint8_t, uint8_t, uint8_t, char, uint16_t);

void goForward(void);
void goBack(void);
void goRight(void);
void goLeft(void);

//フォトリフレクタからの入力用変数
uint16_t in_pr1 = 0, in_pr2 = 0;

uint16_t p, i, d;

uint16_t diff_r[2] = {0, 0};
uint16_t diff_l[2] = {0, 0};
float integral_r = 0, integral_l = 0;
uint16_t pid_r = 0, pid_l = 0;

uint16_t cnt = 0;

void __interrupt() isr(void) {      //タイマー4の割り込み
    float val_p, val_i, val_d;
    GIE = 0;
    if(TMR4IF == 1) {
        diff_r[0]  = diff_r[1];
        diff_r[1]  = in_pr1 - TARGET;
        integral_r = ((diff_r[0] + diff_r[1]) >> 2) * 0.1;      //本当は"/ 2"だが高速化のため">> 2"
        
        val_p = p / 100 * diff_r[1];
        val_i = i / 100 * integral_r;
        val_d = d / 1000 * (diff_r[1] - diff_r[0]) * 10;               //本当は"/ 0.1"だが高速化のため"* 10"
        
        pid_r = (uint16_t)(val_p + val_i + val_d);
        
        diff_l[0]  = diff_l[1];
        diff_l[1]  = in_pr2 - TARGET;
        integral_l = ((diff_l[0] + diff_l[1]) >> 2) * 0.1;      //本当は"/ 2"だが高速化のため">> 2"
        
        val_p = p / 100 * diff_l[1];
        val_i = i / 100 * integral_l;
        val_d = d / 1000 * (diff_l[1] - diff_l[0]) * 10;               //本当は"/ 0.1"だが高速化のため"* 10"
        
        pid_l = (uint16_t)(val_p + val_i + val_d);
        
        cnt++;
    }
    TMR4IF = 0;
    GIE = 1;
}

void main(void) {
    
    init();
    
    uint8_t rcv_data[4];
     
    // LCDモジュール電源安定化時間待ち
    __delay_ms(100);
     
    // LCD初期化
    lcdInitialize();
    
    TMR2ON = 1;
    CCPR1H = (uint8_t)(1000 >> 2);
    CCPR1L = (uint8_t)(1000 << 6);
    
    
    uint16_t cnt = 0;
    while(1) {
        __delay_ms(1000);
        lcdLocateCursor(1, 1);
        printf("%d", cnt);
        cnt++;
    }
    /*
    //リーダーコードの長さに応じて赤外線を受信するかしないか判断
    bool receive;
   
    
    //p、i、dの設定
    uint8_t data1 = eeprom_read(0);
    uint8_t data2 = eeprom_read(1);
    p             = (data1 << 4) | data2;
    
    data1 = eeprom_read(2);
    data2 = eeprom_read(3);
    i     = (data1 << 4) | data2;
    
    data1 = eeprom_read(4);
    data2 = eeprom_read(5);
    d     = (data1 << 4) | data2;
    
    while(1) {
        if(RA1) {
            //ライントレースモード
            __delay_ms(100);        //チャタリング対策
            LATA2 = 1;              //LEDを点灯
            
            TMR4ON = 1;             //タイマー4を開始
                
            while(RA1) {    
                //ボタンが押されたらPIDを設定
                if(RA3 == 1) {
                    GIE = 0;                //割り込みを禁止
                    
                    __delay_ms(100);        //チャタリング対策
                    while(RA3);
                    lcdClearDisplay();
                    
                    p = changePID(0, 1, 0b00001100, 'P', p);       //B4のアナログ値を読み取り
                    i = changePID(2, 3, 0b00001101, 'I', i);       //B5のアナログ値を読み取り
                    d = changePID(4, 5, 0b00010011, 'D', d);       //C3のアナログ値を読み取り
                        
                    //LCDをまっさらにして終了
                    lcdClearDisplay();
                    
                    GIE = 1;                //割り込みを許可
                }
                
                lcdLocateCursor(1, 1);////
                printf("%d", cnt);////
                lcdLocateCursor(1, 2);////
                printf("%d", pid_l);//// 
           
                ADPCHbits.ADPCH = 0b001010;         //B2のアナログ値を読み取り
                ADGO = 1;
                while(ADGO);
                in_pr1 = ADRES;

                ADPCHbits.ADPCH = 0b001011;         //B3のアナログ値を読み取り
                ADGO = 1;
                while(ADGO);
                in_pr2 = ADRES;
            
                //lcdLocateCursor(1, 1);
                //printf("%4d", in_PR1);
                //lcdLocateCursor(1, 2);
                //printf("%4d", in_PR2);
                
            }
            
            TMR4ON = 0;            //タイマー4を停止
            LATA2 = 0;              //LEDを消灯
            //LCDをまっさらにして終了
            lcdClearDisplay();         
        } else {
            //赤外線ラジコンモード
            //初期化
            receive = false;
            for(uint8_t i = 0; i < 4; i++) rcv_data[i] = 0;

            //何も受信していない間の時間つぶし
            while(RA0) {
                if(RA1) break;      //モード変更スイッチが押されてたらbreak
            }

            //受信し始めたらカウンタの値を初期化し、タイマをスタート
            if(!RA0) {
                TMR1H  = 0;
                TMR1L  = 0;
                TMR1ON = 1;
            }
            //HIGHの時間を計測
            while(!RA0) {
                if(RA1) break;      //モード変更スイッチが押されてたらbreak
            }
            TMR1ON = 0;

            //HIGHの時間が8ms-10.0msだとデータを受信するようにする
            if(TMR1H >= 0x1F && TMR1H <= 0x27) receive = true;

            //LOWの時間つぶし
            while(RA0) {
                if(RA1) break;      //モード変更スイッチが押されてたらbreak
            }

            //データの受信
            if(receive) {
                for(int i = 0; i < 4; i++) {
                    rcv_data[i] = 0;
                    for(int j = 7; j >= 0; j--) {
                        //HIGHの時間つぶし
                        while(!RA0) {
                            if(RA1) break;      //モード変更スイッチが押されてたらbreak
                        }

                        //LOWの時間を測定
                        TMR1H  = 0;
                        TMR1L  = 0;
                        TMR1ON = 1;
                        while(RA0) {
                            if(RA1) break;      //モード変更スイッチが押されてたらbreak
                        }
                        TMR1ON = 0;

                        //LOWの時間が0x04よりも長ければ1と判断
                        if(TMR1H >= 0x04) {
                            rcv_data[i] = rcv_data[i] | (uint8_t)(0b00000001 << j);
                        }
                    }
                }
            }
            
            if(rcv_data[0] == 'S' && rcv_data[1] == 'C') {
                if(rcv_data[3] == 0x00)      goForward();
                else if(rcv_data[3] == 0x01) goLeft();
                else if(rcv_data[3] == 0x02) goBack();
                else if(rcv_data[3] == 0x03) goRight();
            }
        }
    }
    */
        
    return;
}

void init() {
    // 動作周波数設定
    OSCCON1bits.NOSC = 0b110;   // 内部クロック使用
    OSCCON1bits.NDIV = 0b0000;  // 分周1:1
    OSCFRQbits.HFFRQ = 0b110;   // 32MHz
    
    // ピン属性設定
    ANSELA = 0b00000000;
    ANSELB = 0b00111100;        //RB2、RB3RB4、RB5をアナログに設定
    ANSELC = 0b00001000;        //RC3をアナログに設定
    TRISA  = 0b00001011;        //RA0、RA1、RA3を入力に設定
    TRISB  = 0b00111111;        //RB0、RB1、RA2、RA3、RB4、RB5を入力に設定
    TRISC  = 0b00001000;        //RC3を入力に設定
    
    //I2Cの設定
    SSP1STAT = 0x80;   // クロック信号は100kHzを使用
    SSP1CON1 = 0x28;   // I2C通信のマスターモードを有効化
    SSP1CON3 = 0x00;   // CON3はデフォルト設定
    SSP1ADD  = 0x09;   //クロック信号速度を100kHzに設定
    
    
    //Timer1の設定
    //T1CLKbits.CS   = 0b0010;        //システムクロックを使用
    //T1CONbits.CKPS = 0b00;          //プリスケール値は1:1
    ////T1CONbits.RD16 = 0;             //16ビットの値を読めるように許可
    //T1CONbits.ON   = 0;             //タイマー1を停止
    
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
    T2CLKCONbits.CS = 0b0010;       //タイマー2に内部クロックを使用
    
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
    
    T2CLKCONbits.CS = 0b0001;       //Fosc / 4を使用
    T2CONbits.CKPS  = 0b101;     //プリスケーラを1:32に設定
    T2CONbits.OUTPS = 0b1001;       //ポストスケーラ1:10を使用
    PR2             = 124;        //50Hzに設定
    
    /*
    //CCP1のデューティーサイクルを設定
    CCPR1H = (uint8_t)(100 >> 2);
    CCPR1L = (uint8_t)(100 << 6);
    
    //CCP2のデューティーサイクルを設定
    CCPR2H = (uint8_t)(33 >> 2);
    CCPR2L = (uint8_t)(33 << 6);
    
    ADCON0bits.ADON   = 1;
    ADCON0bits.ADFRM0 = 1;
    ADCLKbits.ADCCS   = 0b000000;
    ADREFbits.ADPREF  = 0b00;
    
    //EEPROMの初期化
    //P、I、Dはそれぞれ2バイトなため最初の6バイトに初期値である0を入れる
    __EEPROM_DATA(0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF);
    
    //タイマー4の設定
    T4CLKCONbits.CS = 0b0001;       //Fosc / 4を使用
    T4CONbits.CKPS  = 0b110;        //プリスケーラ1:64を使用
    T4CONbits.OUTPS = 0b1001;       //ポストスケーラ1:10を使用
    T4CONbits.ON    = 0;            //タイマー4を停止
    PR4             = 39;           //39までカウント(100ms)されたら割り込みするように設定
    TMR4IF          = 0;            //タイマー4の割り込みフラグを0に設定
    TMR4IE          = 1;            //タイマー4の割り込みを許可
    PEIE            = 1;            //周辺機器の割り込みを許可
    GIE             = 1;            //全体の割り込みを許可
    */
}

uint16_t                                                                                                                                                                                                                                                                                                changePID(uint8_t addr1, uint8_t addr2, uint8_t channel, char pid, uint16_t pre_val) {
    //前回の設定値を表示
    lcdLocateCursor(5, 1);
    printf("%4d", pre_val);
    
    ADPCHbits.ADPCH = channel;     //B4のアナログ値を読み取り
        
    //Pの値を設定する
    lcdLocateCursor(1, 1);
    printf("%c", pid);
    
    uint16_t val;
    while(1) {
        ADGO = 1;
        while(ADGO);
        lcdLocateCursor(1, 2);
        val = ADRES;
        printf("%4d", val);
        
        //RA3が押されて、離されたら抜ける
        if(RA3) {
            __delay_ms(100);        //チャタリング対策
            while(RA3);
            break;
        }
    }
    
    //設定されたPをEEPROMに保存
    uint8_t data3 = val >> 4;
    uint8_t data4 = val & 0x0F;
    eeprom_write(addr1, data3);
    eeprom_write(addr2, data4);
    
    return val;
}

void goForward() {
    TMR2ON = 1;
    
    CCPR1H = (uint8_t)(60 >> 2);
    CCPR1L = (uint8_t)(60 << 6);
    
    CCPR2H = (uint8_t)(130 >> 2);
    CCPR2L = (uint8_t)(130 << 6);
    
    __delay_ms(100);
    TMR2ON = 0;
    
    return;
}

void goBack() {
    TMR2ON = 1;
    
    CCPR1H = (uint8_t)(130 >> 2);
    CCPR1L = (uint8_t)(130 << 6);
    
    CCPR2H = (uint8_t)(60 >> 2);
    CCPR2L = (uint8_t)(60 << 6);
                                                                                                                                                                                  
    __delay_ms(100);
    TMR2ON = 0;

    return;
}

void goRight() {
    TMR2ON = 1;
    
    CCPR1H = (uint8_t)(130 >> 2);
    CCPR1L = (uint8_t)(130 << 6);
    
    CCPR2H = (uint8_t)(130 >> 2);
    CCPR2L = (uint8_t)(130 << 6);
    
    __delay_ms(100);
    TMR2ON = 0;

    return;
}

void goLeft() {
    TMR2ON = 1;
    
    CCPR1H = (uint8_t)(60 >> 2);
    CCPR1L = (uint8_t)(60 << 6);
    
    CCPR2H = (uint8_t)(60 >> 2);
    CCPR2L = (uint8_t)(60 << 6);
    
    __delay_ms(100);
    TMR2ON = 0;

    return;
}