#include <xc.h>
#include <stdint.h>
#include <stdio.h>
#include "AQM0802.h"
#define _XTAL_FREQ 32000000

#define I2C_ACK 0x00
#define I2C_NACK 0xff
#define LCD_I2C_ADDRESS 0x7c

void lcdInitialize() {
    // 初期化コマンド送信
    lcdSendCommandData(0x38); // 2行モードに設定
    lcdSendCommandData(0x39); // 拡張コマンド選択
    lcdSendCommandData(0x14); // 内部クロック周波数設定
    lcdSendCommandData(0x70); // コントラスト設定(C3:C0 = 0b0000に設定)
    lcdSendCommandData(0x56); // 電源電圧が3.3VなのでBooster=ONに設定。コントラスト設定はC5:C4 = 0b10
    lcdSendCommandData(0x6c); // オペアンプのゲイン設定
    
    // モジュール内電源安定化のための時間待ち
    __delay_ms(200);
    
    // 初期化コマンド続き
    lcdSendCommandData(0x38); // 通常コマンド選択
    lcdSendCommandData(0x0c); // ディスプレイ表示
    lcdSendCommandData(0x01); // ディスプレイ表示内容クリア
    
    return;
}

void lcdClearDisplay() {
    lcdSendCommandData(0x01);
    
    return;
}

void lcdSendCommandData(uint8_t command) {
    // コマンドを送信する場合の制御コードは0x00
    lcdI2CProtocol(LCD_I2C_ADDRESS, 0x00, command);
    
    // ウエイト
    //   データシートではウエイト時間は26.3us以上になっているが、
    //   それより長くしないと初期化できないケースがあるため1msのウエイトを入れる
    __delay_ms(1);
    
    return;
}

void lcdSendCharacterData(uint8_t data) {
    // 表示文字のデータを送信する場合の制御コードは0x40
    lcdI2CProtocol(LCD_I2C_ADDRESS, 0x40, data);
    
    // ウエイト
    //   文字表示の場合はウエイトを入れなくても動作しているが
    //   表示されない場合は1ms程度のウエイトを入れる
    // __delay_ms(1);
    
    return;
}

void lcdLocateCursor(uint8_t pos_x, uint8_t pos_y) {
    lcdSendCommandData(0x80 + 0x40 * (pos_y - 1) + (pos_x - 1));
    
    return;
}

void lcdI2CProtocol(uint8_t address, uint8_t cont_code, uint8_t data) {
    i2cProtocolStart();                 //スタートコンディション
    i2cProtocolSendData(address);       //アドレス送信
    i2cProtocolSendData(cont_code);     //制御コード送信
    i2cProtocolSendData(data);          //データ送信
    i2cProtocolStop();                  //ストップコンディション
    
    return;
}

void i2cProtocolStart() {
    // SSP1CON2レジスタのSENビットを1に設定すると
    // スタートコンディションが生成される
    // 発行が完了するとSSP1IFが1になるのでwhile文で待つ
    
    SSP1IF = 0;
    SSP1CON2bits.SEN = 1;
    while(SSP1IF == 0) {};
    SSP1IF = 0;
    
    return;
}

void i2cProtocolStop() {
    // SSP1CON2レジスタのPENビットを1に設定すると
    // ストップコンディションが生成される
    // 発行が完了するとSSP1IFが1になるのでwhile文で待つ
    
    SSP1IF = 0;
    SSP1CON2bits.PEN = 1;
    while(SSP1IF == 0) {};
    SSP1IF = 0;
    
    return;
}

void i2cProtocolSendData(uint8_t data) {
    // SSP1BUFに送信したいデータをセットすると、そのデータが送信される
    // 発行が完了するとSSP1IFが1になるのでwhile文で待つ
    
    SSP1IF = 0;
    SSP1BUF = data;
    while(SSP1IF == 0) {};
    SSP1IF = 0;
    
    return;
}

uint8_t i2cProtocolCheckAck() {
    uint8_t ack_status;
    
    if(SSP1CON2bits.ACKSTAT) {
        ack_status = I2C_NACK;
    }else {
        ack_status = I2C_ACK;
    }
    
    return ack_status;
}

void putch(char character) {
    lcdSendCharacterData(character);
    
    return;
}