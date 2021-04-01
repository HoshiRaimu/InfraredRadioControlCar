/* 
 * File:   AQM0802.h
 * Author: raimu
 *
 * Created on 2021/04/01, 15:40
 */

#ifndef AQM0802_H
#define	AQM0802_H

// LCDモジュール表示制御関数
void lcdInitialize(void);               // LCD初期化
void lcdClearDisplay(void);             // ディスプレイ全消去
void lcdSendCommandData(uint8_t);       // コマンド送信
void lcdSendCharacterData(uint8_t);     // 1文字表示
void lcdLocateCursor(uint8_t,uint8_t);  // カーソル位置指定
 
// LCDモジュールI2Cプロトコル関数
void lcdI2CProtocol(uint8_t, uint8_t, uint8_t);
 
// I2Cプロトコル各信号の生成関数
void    i2cProtocolStart(void);        // スタートビット生成
void    i2cProtocolStop(void);         // ストップビット生成
void    i2cProtocolSendData(uint8_t);  // 1バイトデータ送信
uint8_t i2cProtocolCheckAck(void);     // ACK信号チェック

#ifdef	__cplusplus
extern "C" {
#endif


#ifdef	__cplusplus
}
#endif

#endif	/* AQM0802_H */

