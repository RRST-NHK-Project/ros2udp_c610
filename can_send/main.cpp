#include <mbed.h>

// CAN1を使用
CAN can{PD_0, PD_1, (int)1e6}; // 通信速度を1MHzに設定

int main() {
  printf("CAN送信開始\n");

  auto pre = HighResClock::now(); // 前回送信時刻

  while (1) {
    auto now = HighResClock::now();

    // 10msごとにデータを送信
    if (now - pre > 10ms) {
      uint32_t id = 0x200; // メッセージID
      uint8_t data[8] = {1, 2, 3, 4, 5, 6, 7, 8}; // 送信するデータ

      CANMessage msg{id, data, sizeof(data)}; // CANメッセージ作成

      if (can.write(msg)) {
        printf("メッセージ送信: ID=0x%x データ=%d %d %d %d %d %d %d %d\n",
               id,
               data[0], data[1], data[2], data[3],
               data[4], data[5], data[6], data[7]);
      } else {
        printf("メッセージ送信失敗\n");
      }

      pre = now;
    }
  }
}
