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

#define _XTAL_FREQ 32000000
#define TRIG RB0
#define ECHO RB1

typedef struct
{
    uint16_t data;
    uint8_t  index;
} Pair;

void init(void);

void goForward(uint8_t);
void goBack(uint8_t);
void goRight(uint8_t);
void goLeft(uint8_t);

uint16_t measureDist(void);
void avoidObs(void);
void bubbleSort(Pair *);

void putch(uint8_t data) 
{
    while(!TRMT);
    TX1REG = data;
    
    return;
}

void __interrupt() isr(void) 
{
    GIE = 0;
    if(TMR2IF == 1)             //Timer2の割り込み、20msの周期
    {   
        LATC0 = 1;              //RC0をHIGH
        LATC1 = 1;              //RC1をHIGH

        T4CONbits.ON = 1;       //Timer4を開始
        T6CONbits.ON = 1;       //Timer6を開始
        
        TMR2IF = 0;             //割り込みフラグをクリア
        
    }
    else if(TMR4IF == 1)      //Timer4の割り込み
    {    
        LATC0 = 0;              //RC0をLOW
        T4CONbits.ON = 0;       //Timer4を停止
        TMR4IF = 0;             //割り込みフラグをクリア
        
    } 
    else if(TMR6IF == 1)      //Timer6の割り込み
    {    
        LATC1 = 0;              //RC1をLOW
        T6CONbits.ON = 0;       //Timer6を停止
        TMR6IF = 0;             //割り込みフラグをクリア
    }
    GIE = 1;
}


void main(void) 
{
    
    init();
    
    uint8_t rcv_data[4];
     
    //リーダーコードの長さに応じて赤外線を受信するかしないか判断
    bool receive;
    
    uint32_t echo_time, dist;
    TMR3 = 0;
    
    
    ADPCHbits.ADPCH = 0b00001100; 
    uint16_t val;
    
    while(1) 
    {
        if(RA1) 
        {
            LATA2 = 1;          //LEDを点灯
            
            while(RA1) 
            {
                dist = measureDist();

                if(dist < 400) 
                {
                    if(dist > 15) goForward(0);
                    else avoidObs();
                }
                //printf("%d\n", dist);
                TMR3 = 0;
                 __delay_ms(100);
            }

            LATA2 = 0;              //LEDを消灯   
        } else 
        {
            //赤外線ラジコンモード
            //初期化
            receive = false;
            for(uint8_t i = 0; i < 4; i++) rcv_data[i] = 0;

            //何も受信していない間の時間つぶし
            while(RA0) 
            {
                if(RA1) break;      //モード変更スイッチが押されてたらbreak
            }

            //受信し始めたらカウンタの値を初期化し、タイマをスタート
            if(!RA0) 
            {
                TMR1H  = 0;
                TMR1L  = 0;
                TMR1ON = 1;
            }
            //HIGHの時間を計測
            while(!RA0) 
            {
                if(RA1) break;      //モード変更スイッチが押されてたらbreak
            }
            TMR1ON = 0;

            //HIGHの時間が8ms-10.0msだとデータを受信するようにする
            if(TMR1H >= 0x1F && TMR1H <= 0x27) receive = true;

            //LOWの時間つぶし
            while(RA0) 
            {
                if(RA1) break;      //モード変更スイッチが押されてたらbreak
            }

            //データの受信
            if(receive) 
            {
                for(int i = 0; i < 4; i++) 
                {
                    rcv_data[i] = 0;
                    for(int j = 7; j >= 0; j--) 
                    {
                        //HIGHの時間つぶし
                        while(!RA0) 
                        {
                            if(RA1) break;      //モード変更スイッチが押されてたらbreak
                        }

                        //LOWの時間を測定
                        TMR1H  = 0;
                        TMR1L  = 0;
                        TMR1ON = 1;
                        while(RA0) 
                        {
                            if(RA1) break;      //モード変更スイッチが押されてたらbreak
                        }
                        TMR1ON = 0;

                        //LOWの時間が0x04よりも長ければ1と判断
                        if(TMR1H >= 0x04) 
                        {
                            rcv_data[i] = rcv_data[i] | (uint8_t)(0b00000001 << j);
                        }
                    }
                }
            }
            
            if(rcv_data[0] == 'S' && rcv_data[1] == 'C') 
            {
                if(rcv_data[3] == 0x00)      goForward(rcv_data[2] * 19 / 31);          //0~31の値を0~19の値に変換してあげる
                else if(rcv_data[3] == 0x01) goLeft(rcv_data[2] * 19 / 31);
                else if(rcv_data[3] == 0x02) goBack(rcv_data[2] * 19 / 31);
                else if(rcv_data[3] == 0x03) goRight(rcv_data[2] * 19 / 31);
            }
        }
    }
    
    return;
}

void init() 
{
    // 動作周波数設定
    OSCCON1bits.NOSC = 0b110;   // 内部クロック使用
    OSCCON1bits.NDIV = 0b0000;  // 分周1:1
    OSCFRQbits.HFFRQ = 0b110;   // 32MHz
    
    // ピン属性設定
    ANSELA = 0b00000000;
    ANSELB = 0b00000000;        //RB2、RB3RB4、RB5をアナログに設定
    ANSELC = 0b00000000;        //RC3をアナログに設定
    TRISA  = 0b00000011;        //RA0、RA1、RA3を入力に設定
    TRISB  = 0b00000010;        //RB1を入力に設定
    TRISC  = 0b00000000;        //RC3を入力に設定
    
    //Timer1の設定
    T1CLKbits.CS   = 0b0001;        //Fosc/4を使用
    T1CONbits.CKPS = 0b11;          //プリスケール値は1:8
    T1CONbits.ON   = 0;             //タイマー1を停止
    
    //Timer3の設定
    T3CLKbits.CS   = 0b0001;        //Fosc/4を使用
    T3CONbits.CKPS = 0b11;          //プリスケール値は1:8
    T3CONbits.ON   = 0;             //タイマー3を停止

    //Timer2の設定
    T2CLKCONbits.CS = 0b0001;       //Fosc / 4を使用
    T2CONbits.CKPS  = 0b111;        //プリスケーラを1:128
    T2CONbits.OUTPS = 0b1001;       //ポストスケーラ1:10を使用
    PR2             = 125;          //50Hzに設定
    T2CONbits.ON    = 0;            //タイマー2停止
    TMR2IF          = 0;            //タイマー2の割り込みフラグを0に設定
    TMR2IE          = 1;            //タイマー2の割り込みを許可
    PEIE            = 1;            //周辺機器の割り込みを許可
    GIE             = 1;            //全体の割り込みを許可
    
    //Timer4の設定
    T4CLKCONbits.CS = 0b0001;       //Fosc / 4を使用
    T4CONbits.CKPS  = 0b100;        //プリスケーラ1:16を使用
    T4CONbits.OUTPS = 0b1001;       //ポストスケーラ1:10を使用
    PR4             = 0;            //この値を変化させ、サーボモーターを制御
    T4CONbits.ON    = 0;            //タイマー4を停止
    TMR4IF          = 0;            //タイマー4の割り込みフラグを0に設定
    TMR4IE          = 1;            //タイマー4の割り込みを許可
    
    //Timer6設定
    T6CLKCONbits.CS = 0b0001;       //Fosc / 4を使用
    T6CONbits.CKPS  = 0b100;        //プリスケーラ1:16を使用
    T6CONbits.OUTPS = 0b1001;       //ポストスケーラ1:10を使用
    PR6             = 0;            //この値を変化させ、サーボモーターを制御
    T6CONbits.ON    = 0;            //タイマー6を停止
    TMR6IF          = 0;            //タイマー6の割り込みフラグを0に設定
    TMR6IE          = 1;            //タイマー6の割り込みを許可
    
    ADCON0bits.ADON   = 1;          //ADCを許可
    ADCON0bits.ADFRM0 = 1;          //結果数値を右詰めに設定
    ADCLKbits.ADCCS   = 0b111111;   //Fosc / 128を使用
    ADREFbits.ADPREF  = 0b00;       //基準電圧は電源電圧
    
    //EUSARTの設定
    RXPPS  = 0x17;              //RC7をRXに設定
    RC6PPS = 0x10;              //RC6をTXに設定
    
    TX1STA   = 0x20;              //非同期モードで動作し、で1バイトを送信する
    TX1STAbits.BRGH = 1;
    RC1STA   = 0x90;              //シリアルポートを許可し、CRENビットがクリアになるまで継続的な受信を許可する
    BAUD1CON = 0x08;
    SP1BRG = 832;
}

void goForward(uint8_t speed) 
{
    PR4 = 70 - speed;
    PR6 = 77 + speed;
    
    TMR2ON = 1;
    __delay_ms(50);
    TMR2ON = 0;
    
    return;
}

void goBack(uint8_t speed) 
{
    PR4 = 77 + speed;
    PR6 = 70 - speed;
    
    TMR2ON = 1;
    __delay_ms(50);
    TMR2ON = 0;
    
    return;
}

void goRight(uint8_t speed) 
{
    PR4 = 80 + speed;
    PR6 = 81 + speed;               //どちらも80にするとモーターが止まらない
                                    //おそらく値を同じにすると同時に割り込みが発生するため
    
    TMR2ON = 1;
    __delay_ms(50);
    TMR2ON = 0;
    
    return;
}

void goLeft(uint8_t speed) 
{
    PR4 = 70 - speed;
    PR6 = 71 - speed;
    
    TMR2ON = 1;
    __delay_ms(50);
    TMR2ON = 0;
    
    return;
}

uint16_t measureDist() 
{
    //TRIGを10usHIGHにする
    TRIG = 1;
    __delay_us(10);
    TRIG = 0;

    //ECHOが返ってくるまでの時間を計測
    TMR3ON = 1;
    while(!ECHO);
    while(ECHO);
    TMR3ON = 0;

    //距離を計算
    uint32_t echo_time = TMR3;
    echo_time = echo_time * 5 / 10;
    uint16_t dist = echo_time * 34 / 1000;    
    
    return dist;
}

void avoidObs()
{
    Pair dist[10];
    for(uint8_t i = 0; i < 5; i++)
    {
        goLeft(20);
        dist[i].data = measureDist();
        dist[i].index = i;
        if(dist[i].data > 400) dist[i].data = 400;
    }
    for(uint8_t i = 0; i < 5; i++) goRight(20);             //元に戻す
    for(uint8_t i = 0; i < 5; i++)
    {
        goRight(20);
        dist[i + 5].data = measureDist();
        dist[i + 5].index = i + 5;
        if(dist[i + 5].data > 400) dist[i + 5].data = 400;
    }
    for(uint8_t i = 0; i < 5; i++) goLeft(20);             //元に戻す
    
    
    if(dist[9].data < 15) 
    {
        for(uint8_t i = 0; i < 10; i++) goRight(20);
    } 
    else 
    {
        if(dist[9].index < 5) 
        {
            for(uint8_t i = 0; i < dist[9].index; i++) goLeft(20);
        }
        else 
        {
            for(uint8_t i = 0; i < dist[9].index - 5; i++) goRight(20);
        }
    }
    
    return;
}

void bubbleSort(Pair *p)
{
    for(uint8_t i = 0; i < 10; i++) 
    {
        for(uint8_t j = 0; j < 10 - i - 1; j++) 
        {
            if(p[j].data > p[j + 1].data) 
            {
                Pair tmp = p[j];
                p[j] = p[j + 1];
                p[j + 1] = tmp;
            }
        }
    }
    return;
}