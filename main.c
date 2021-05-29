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

#define TARGET     300

void init(void);
uint16_t changePID(uint8_t, uint8_t, uint8_t, char, uint16_t);

void goForward(void);
void goBack(void);
void goRight(void);
void goLeft(void);

//フォトリフレクタからの入力用変数
uint16_t in_pr1 = 0, in_pr2 = 0;

uint16_t p, i, d;

int diff_r[2] = {0, 0};
int diff_l[2] = {0, 0};
float integral_r = 0, integral_l = 0;
uint32_t pid_r = 0, pid_l = 0;
float val_p, val_i, val_d;

uint32_t millis = 0;
float dt = 0;
uint32_t now = 0, prev = 0;

void __interrupt() isr(void) {
    GIE = 0;
    if(TMR3IF == 1) {
        millis++;
        TMR3H = (64536 >>8);            //タイマー3の初期化(65536 - 1000 = 64536);
        TMR3L = (64536 & 0x00FF);
        TMR3IF = 0;
    } else if(TMR2IF == 1) {    //Timer2の割り込み、20msの周期
        LATC0 = 1;              //RC0をHIGH
        LATC1 = 1;              //RC1をHIGH

        T4CONbits.ON = 1;       //Timer4を開始
        T6CONbits.ON = 1;       //Timer6を開始
        
        TMR2IF = 0;             //割り込みフラグをクリア
        
    } else if(TMR4IF == 1) {    //Timer4の割り込み
        LATC0 = 0;              //RC0をLOW
        T4CONbits.ON = 0;       //Timer4を停止
        TMR4IF = 0;             //割り込みフラグをクリア
        
    } else if(TMR6IF == 1) {    //Timer6の割り込み
        LATC1 = 0;              //RC1をLOW
        T6CONbits.ON = 0;       //Timer6を停止
        TMR6IF = 0;             //割り込みフラグをクリア
    }
    GIE = 1;
}


void main(void) {
    
    init();
    
    uint8_t rcv_data[4];
     
    // LCDモジュール電源安定化時間待ち
    __delay_ms(100);
     
    // LCD初期化
    lcdInitialize();
    
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
            
            TMR3ON = 1;             //タイマー3を開始
            
            TMR2ON = 1;
                
            while(RA1) {    
                //ボタンが押されたらPIDを設定
                if(RA3 == 1) {
                    TMR3ON = 0;
                    TMR2ON = 0;
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
                    TMR3ON = 1;
                    TMR2ON = 1;
                }
                __delay_ms(100);
                
                lcdLocateCursor(1, 1);////
                printf("%8d", pid_r);////
                lcdLocateCursor(1, 2);////
                printf("%8d", pid_l);//// 

                
                
 
                ADPCHbits.ADPCH = 0b001010;         //B2のアナログ値を読み取り
                ADGO = 1;
                while(ADGO);
                in_pr1 = ADRES;

                ADPCHbits.ADPCH = 0b001011;         //B3のアナログ値を読み取り
                ADGO = 1;
                while(ADGO);
                in_pr2 = ADRES;
                
                prev = now;
                now = millis;
                dt = (now - prev) * 0.001;
               
                
                diff_r[0]  = diff_r[1];
                diff_r[1]  = in_pr1 - TARGET;
                integral_r += ((diff_r[0] + diff_r[1]) / 2.0) * dt;      //本当は"/ 2"だが高速化のため">> 2"

                val_p = (p * 0.0001) * diff_r[1];
                val_i = (i * 0.0001) * integral_r;
                val_d = (d * 0.00001) * (diff_r[1] - diff_r[0]) / dt;               //本当は"/ 0.1"だが高速化のため"* 10"

                pid_r = (uint16_t)(val_p + val_i + val_d);

                diff_l[0]  = diff_l[1];
                diff_l[1]  = in_pr2 - TARGET;
                integral_l += ((diff_l[0] + diff_l[1]) / 2.0) * dt;      //本当は"/ 2"だが高速化のため">> 2"

                val_p = p * 0.0001 * diff_l[1];
                val_i = i * 0.0001 * integral_l;
                val_d = d * 0.00001 * (diff_l[1] - diff_l[0]) / dt;               //本当は"/ 0.1"だが高速化のため"* 10"

                pid_l = (uint16_t)(val_p + val_i + val_d);
                
                /*
                uint8_t tmp1, tmp2;
                if(pid_r < 0) tmp1 = 0;
                else if(pid_r > 40) tmp1 = 40;
                
                if(pid_l < 0) tmp2 = 0;
                else if(pid_l > 40) tmp2 = 40;
                */
                //PR4 = 70 - pid_r;
                //PR6 = 80 + pid_l;
            }
            
            TMR3ON = 0;            //タイマー4を停止
            TMR2ON = 0;
            
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
    SSP1STAT = 0x80;            // クロック信号は100kHzを使用
    SSP1CON1 = 0x28;            // I2C通信のマスターモードを有効化
    SSP1CON3 = 0x00;            // CON3はデフォルト設定
    SSP1ADD  = 0x09;            //クロック信号速度を100kHzに設定
    
    
    //Timer1の設定
    T1CLKbits.CS   = 0b0001;        //Fosc/4を使用
    T1CONbits.CKPS = 0b11;          //プリスケール値は1:8
    T1CONbits.ON   = 0;             //タイマー1を停止
    
    //Timer3の設定
    T3CLKbits.CS   = 0b0001;        //Fosc/4を使用
    T3CONbits.CKPS = 0b11;          //プリスケール値は1:8
    T3CONbits.ON   = 0;             //タイマー3を停止
    TMR3H = (64536 >>8);            //タイマー3の初期化(65536 - 1000 = 64536);
    TMR3L = (64536 & 0x00FF);
    TMR3IF          = 0;            //タイマー3の割り込みフラグを0に設定
    TMR3IE          = 1;            //タイマー3の割り込みを許可
    PEIE            = 1;            //周辺機器の割り込みを許可
    GIE             = 1;            //全体の割り込みを許可
    
    //Timer2の設定
    T2CLKCONbits.CS = 0b0001;       //Fosc / 4を使用
    T2CONbits.CKPS  = 0b111;        //プリスケーラを1:128
    T2CONbits.OUTPS = 0b1001;       //ポストスケーラ1:10を使用
    PR2             = 25;           //50Hzに設定
    T2CONbits.ON    = 0;            //タイマー2停止
    TMR2IF          = 0;            //タイマー2の割り込みフラグを0に設定
    TMR2IE          = 1;            //タイマー2の割り込みを許可
    
    //Timer4の設定
    T4CLKCONbits.CS = 0b0001;       //Fosc / 4を使用
    T4CONbits.CKPS  = 0b100;        //プリスケーラ1:16を使用
    T4CONbits.OUTPS = 0b1001;       //ポストスケーラ1:10を使用
    PR4             = 85;           //この値を変化させ、サーボモーターを制御
    T4CONbits.ON    = 0;            //タイマー4を停止
    TMR4IF          = 0;            //タイマー4の割り込みフラグを0に設定
    TMR4IE          = 1;            //タイマー4の割り込みを許可
    
    //Timer6設定
    T6CLKCONbits.CS = 0b0001;       //Fosc / 4を使用
    T6CONbits.CKPS  = 0b100;        //プリスケーラ1:16を使用
    T6CONbits.OUTPS = 0b1001;       //ポストスケーラ1:10を使用
    PR6             = 85;           //この値を変化させ、サーボモーターを制御
    T6CONbits.ON    = 0;            //タイマー6を停止
    TMR6IF          = 0;            //タイマー6の割り込みフラグを0に設定
    TMR6IE          = 1;            //タイマー6の割り込みを許可
    
    //PPSの設定
    PPSLOCKbits.PPSLOCKED = 0;      //PPS設定ロックの解除
    //I2Cの設定
    SSP1DATPPS = 0x08;              //SDA入力部をRB0に割り当て
    SSP1CLKPPS = 0x09;              //SCL入力部をRB1に割り当て
    RB0PPS     = 0x15;              //RB0をSDAに割り当て
    RB1PPS     = 0x14;              //RB1をSCLに割り当て
    PPSLOCKbits.PPSLOCKED = 1;      //PPS設定ロック

    
    ADCON0bits.ADON   = 1;          //ADCを許可
    ADCON0bits.ADFRM0 = 1;          //結果数値を右詰めに設定
    ADCLKbits.ADCCS   = 0b111111;   //Fosc / 128を使用
    ADREFbits.ADPREF  = 0b00;       //基準電圧は電源電圧
    
    //EEPROMの初期化
    //P、I、Dはそれぞれ2バイトなため最初の6バイトに初期値である0を入れる
    __EEPROM_DATA(0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF);
}

uint16_t changePID(uint8_t addr1, uint8_t addr2, uint8_t channel, char pid, uint16_t pre_val) {
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
    uint8_t data1 = val >> 4;
    uint8_t data2 = val & 0x0F;
    eeprom_write(addr1, data1);
    eeprom_write(addr2, data2);
    
    return val;
}

void goForward() {
    PR4 = 40;
    PR6 = 110;
    
    TMR2ON = 1;
    __delay_ms(100);
    TMR2ON = 0;
    
    return;
}

void goBack() {
    PR4 = 110;
    PR6 =40;
    
    TMR2ON = 1;
    __delay_ms(100);
    TMR2ON = 0;
    
    return;
}

void goRight() {
    PR4 = 110;
    PR6 = 110;
    
    TMR2ON = 1;
    __delay_ms(100);
    TMR2ON = 0;
    
    return;
}

void goLeft() {
    PR4 = 40;
    PR6 = 40;
    
    TMR2ON = 1;
    __delay_ms(100);
    TMR2ON = 0;
    
    return;
}