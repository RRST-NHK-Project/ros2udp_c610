#include <mbed.h>

// CAN1を使用
CAN can{PD_0, PD_1, (int)1e6}; // 通信速度を1MHzに設定

int main() {
  printf("CAN受信開始\n");

  while (1) {
    CANMessage msg; // 受信メッセージ用

    if (can.read(msg)) { // メッセージを受信した場合
      printf("メッセージ受信: ID=0x%x データ=%d %d %d %d %d %d %d %d\n",
             msg.id,
             msg.data[0], msg.data[1], msg.data[2], msg.data[3],
             msg.data[4], msg.data[5], msg.data[6], msg.data[7]);
    }

    ThisThread::sleep_for(1ms); // 必要に応じてスリープを挿入
  }
}
