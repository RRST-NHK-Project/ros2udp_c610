#include <mbed.h>
#include "C610.hpp"
#include "QEI.h"

CAN can{PD_0, PD_1, static_cast<int>(1e6)};
DigitalIn button{BUTTON1};
C610Array c610{};

// エンコーダーの設定 (2048 PPR, X4_ENCODING)
QEI ENC1(PC_0, PG_1, NC, 2048, QEI::X4_ENCODING);

float Pulse[6];                         // エンコーダーのパルス格納用
float v[5] = {0.0, 0.0, 0.0, 0.0, 0.0}; // 速度の格納[mm/s]
float d[5] = {0.0, 0.0, 0.0, 0.0, 0.0}; // 変位[m]
int Angle;

int main()
{
    while (1)
    {
        auto now = Kernel::Clock::now();
        static auto pre = now;

        // CANメッセージを読み取り、C610オブジェクトにパース
        if (CANMessage msg; can.read(msg))
        {
            c610.parse_packet(msg);
        }

        // 10msごとに実行
        if (now - pre > 10ms)
        {
            int16_t out = 0;
            if (!button)
                out = 0; // ボタンが押されていないとき、6000

            // エンコーダーの角度を計算 (度単位)
            //m2006のエンコーダーの現在のパルス*(10/8192) = 現在のangle

           
            printf("%d", c610[0].get_absolute_angle()); // 先頭の C610 の角度を取得
            printf("%d\n", c610[1].get_Angle());

            // 各モーターの電流を設定
            for (auto &e : c610)
            {
                e.set_raw_current(out);
            }

            // CANメッセージを生成して送信
            auto msg = c610.to_msgs();
            if (!can.write(msg))
            {
                printf("failed to write c610 msg\n");
            }

            pre = now;
        }
    }
}
