////////////////////////////copy////////////////////////
/*
RRST NHK2025
f7_udpパッケージのフィードフォワード制御用
IPアドレスは適宜変更すること
垂直MD基板用にピンを変更
2024/12/03
*/

#include "EthernetInterface.h"
#include "QEI.h"
#include "mbed.h"
#include "rtos.h"
#include <cstdint>
#include "C610.hpp"
//---------------------------QEI---------------------------//
QEI ENC1(PC_0, PG_1, NC, 2048, QEI::X4_ENCODING);
QEI ENC2(PF_2, PC_3, NC, 2048, QEI::X4_ENCODING);
QEI ENC3(PD_4, PF_5, NC, 2048, QEI::X4_ENCODING);
QEI ENC4(PA_6, PF_7, NC, 2048, QEI::X4_ENCODING);
QEI ENC5(PE_8, PF_9, NC, 2048, QEI::X4_ENCODING);
QEI ENC6(PF_10, PD_11, NC, 2048, QEI::X4_ENCODING);

/*
QEI (A_ch, B_ch, index, int pulsesPerRev, QEI::X2_ENCODING)
index -> Xピン, １回転ごとに１パルス出力される？ 使わない場合はNCでok
pulsePerRev -> Resolution (PPR)を指す
X4も可,X4のほうが細かく取れる
データシート: https://jp.cuidevices.com/product/resource/amt10-v.pdf
*/
C610Array c610;


void receive(UDPSocket *receiver);

// マッピング関数
int map(int value, int inMin, int inMax, int outMin, int outMax) {
  return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

// PWM
PwmOut MD1P(PA_0);
PwmOut MD2P(PA_3);
PwmOut MD3P(PC_7);
PwmOut MD4P(PC_6);
PwmOut MD5P(PC_8);
PwmOut MD6P(PC_9);

// DIR
DigitalOut MD1D(PD_2);
DigitalOut MD2D(PG_2);
DigitalOut MD3D(PD_5);
DigitalOut MD4D(PD_6);
DigitalOut MD5D(PD_7);
DigitalOut MD6D(PC_10);

//サーボ
PwmOut SERVO1(PB_1);
// PwmOut SERVO2(PB_6);
// PwmOut SERVO3(PD_13);
// PwmOut SERVO4(PD_12);

//電磁弁(サーボから借りてる)
// DigitalOut SV1(PB_1);
DigitalOut SV2(PB_6);
DigitalOut SV3(PD_13);
DigitalOut SV4(PD_12);

//スイッチ
DigitalIn SW1(PF_15);
DigitalIn SW2(PG_14);
DigitalIn SW3(PG_9);
DigitalIn SW4(PE_7);

//パイロットランプ（普通のデジタルIOとしても使用可）
DigitalOut PL_1(PF_12);
DigitalOut PL_2(PF_13);

// CAN
CAN can{PD_0, PD_1, (int)1e6}; // rd,td,1Mhz

int Pulse[7];      // エンコーダーのパルス格納用
float period = 10; // 制御周期[ms]
float R = 0.05;    // オムニ直径[mm]
int PPRx4 = 8192;  // エンコーダーのResolution

// MD出力を格納する配列
double mdd[9]; // dir
double mdp[9]; // pwm

const char *recievefromIP = nullptr; //ネットワーク切断検知用

int main() {
  // 送信データ
  char sendData[32];
 for (int i = 0; i < 3; ++i) // 2台のモーターにIDを設定
    {
        c610[i].set_motor_id(i + 1);
    }
  // PWM周期の設定
  MD1P.period_us(50);
  MD2P.period_us(50);
  MD3P.period_us(50);
  MD4P.period_us(50);
  MD5P.period_us(50);
  MD6P.period_us(50);
  /*
  50(us) = 1000(ms) / 20000(Hz) * 10^3
  MDに合わせて調整
  CytronのMDはPWM周波数が20kHzなので上式になる
  */

  // 送信先情報(PCに何か返すときは設定する)
  const char *destinationIP = "192.168.8.195";
  const uint16_t destinationPort = 4000;

  // 自機情報
  const char *myIP = "192.168.0.218";
  const char *myNetMask = "255.255.255.0";
  const uint16_t receivePort = 5000;

  // イーサネット経由でインターネットに接続するクラス
  EthernetInterface net;
  // IPアドレスとPortの組み合わせを格納しておくクラス（構造体でいいのでは？）
  SocketAddress destination, source, myData;
  // UDP通信関係のクラス
  UDPSocket udp;
  // 受信用スレッド
  Thread receiveThread;

  /* マイコンのネットワーク設定 */
  // DHCPはオフにする（静的にIPなどを設定するため）
  net.set_dhcp(false);
  // IPなど設定
  net.set_network(myIP, myNetMask, "");

  printf("Start\n");

  // マイコンをネットワークに接続
  if (net.connect() != 0) {
    printf("Network connection Error >_<\n");
    return -1;
  } else {
    printf("Network connection success ^_^\n");
  }

  // UDPソケットをオープン
  udp.open(&net);

  // portをバインドする
  udp.bind(receivePort);

  // 送信先の情報を入力
  destination.set_ip_address(destinationIP);
  destination.set_port(destinationPort);

  // 受信用のスレッドをスタート
  receiveThread.start(callback(receive, &udp));

  // メインループ(ネットワーク切断検知)
  while (1) {

    if (const int result =
            udp.sendto(recievefromIP, sendData, sizeof(sendData)) < 0) {
      while (1) {
        printf("Connection lost\n");
        MD1P = 0;
        MD2P = 0;
        MD3P = 0;
        MD4P = 0;
        MD5P = 0;
        MD6P = 0;
      }
    }

    //以下エンコーダー送信用
    /*
     using namespace std::chrono;

     // エンコーダーの値を取得
     Pulse[1] = ENC1.getPulses();
     Pulse[2] = ENC2.getPulses();
     Pulse[3] = ENC3.getPulses();
     Pulse[4] = ENC4.getPulses();
     Pulse[5] = ENC5.getPulses();
     Pulse[6] = ENC6.getPulses();

     // エンコーダーをリセット
     ENC1.reset();
     ENC2.reset();
     ENC3.reset();
     ENC4.reset();
     ENC5.reset();
     ENC6.reset();

     // 速度データを文字列に変換
     for (int i = 0; i <= 7; i++) {
       char temp[32];             // 一時的なバッファ
       sprintf(temp, "%d", Pulse[i]); // floatを文字列に変換

       // sendDataにtempをペーストする
       if (i == 0) {
         strcpy(sendData, temp); // 最初はそのままペースト
       } else {
         strcat(sendData, ","); // 以降カンマ区切りに文字列を連結する
         strcat(sendData, temp);
       }
     }

     // ROS2ノードに現在の速度を送信
     if (const int result =
             udp.sendto(destination, sendData, sizeof(sendData)) < 0) {
       printf("send Error: %d\n", result); // エラーの処理
     }
     osDelay(period); // 一定時間待機（制御周期）
     */
  }

  receiveThread.join();

  udp.close();
  net.disconnect();
  return 0;
}

void receive(UDPSocket *receiver) { // UDP受信スレッド

  using namespace std::chrono;

  SocketAddress source;
  char buffer[64];

  int data[9] = {0, -1, -1, -1, -1, -1, 0, 0, 0};

  while (1) {
    memset(buffer, 0, sizeof(buffer));
    if (const int result =
            receiver->recvfrom(&source, buffer, sizeof(buffer)) < 0) {
      printf("Receive Error : %d", result);
    } else {

      recievefromIP = source.get_ip_address();

      //以下受信した文字列をカンマ区切りでintに変換
      char *ptr;
      int ptr_counter = 1;
      // カンマを区切りに文字列を分割
      // 1回目
      ptr = strtok(buffer, ",");
      data[1] = atoi(ptr); // intに変換

      // 2回目以降
      while (ptr != NULL) {
        ptr_counter++;
        // strtok関数により変更されたNULLのポインタが先頭
        ptr = strtok(NULL, ",");
        data[ptr_counter] = atoi(ptr); // intに変換

        // ptrがNULLの場合エラーが発生するので対処
        if (ptr != NULL) {
          // printf("%s\n", ptr);
        }
      }
      //ここまで

   //c610のコード追加
 
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
        int16_t current_0 = data[6] * 100; // data[6] の値をスケーリングして出力
        int16_t current_1 = data[7] * 100;           // モーター2用の電流値
        int16_t current_2 = data[8] * 100;          // モーター3用の電流値 
            c610[0].set_raw_current(current_0);
            c610[1].set_raw_current(current_1);
            c610[2].set_raw_current(current_2);
           // printf("data[6]: %d, darta[7]: %d, data[8]: %d\n",data[6],data[7],data[8]);
            printf("Actual Current -> ID: %d, %d | ID: %d, %d | ID: %d, %d\n",
            c610[0].get_motor_id(), c610[0].get_rpm(),
            c610[1].get_motor_id(), c610[1].get_rpm(),
            c610[2].get_motor_id(), c610[2].get_rpm());
        // //printf("RPM: %d\n", c610[0].get_rpm());
            // エンコーダーの角度を計算 (度単位)
            //m2006のエンコーダーの現在のパルス*(10/8192) = 現在のangle

           //printf("%d\n",c610[0].get_rpm());
           // printf("%d", c610[0].get_absolute_angle()); // 先頭の C610 の角度を取得
           // printf("%d\n", c610[1].get_Angle());

            // 各モーターの電流を設定
            // for (auto &e : c610)
            // {
            //   //  e.set_raw_current(out);
            //     e.set_raw_current(current_0);
            //     e.set_raw_current(current_1);
            //     e.set_raw_current(current_2);
            // }

            // CANメッセージを生成して送信
            auto msg = c610.to_msgs();
            if (!can.write(msg))
            {
                printf("failed to write c610 msg\n");
            }

            pre = now;
        
    }
     //ここまで

      // printf("%d\n",data[7]);

      //方向成分と速度成分を分離
      for (int i = 1; i <= 5; i++) {
        if (data[i] >= 0) {
          mdd[i] = 1;
        } else {
          mdd[i] = 0;
        }
      }
    }

    // printf("%f\n",mdd[6]);
    // printf("%lf, %lf, %lf, %lf\n", mdp[1], mdp[2], mdp[3], mdp[4]);
    // 
    //printf("%d, %d, %d, %d, %d\n", data[1], data[2], data[3], data[4], data[5]);
   // printf("%d\n",data[6]);//ボタン押したときのデータの値を表示
    // MDに出力

    MD1D = mdd[1]; //電磁弁に転用
    MD2D = mdd[2]; //電磁弁に転用
    MD3D = mdd[3]; //電磁弁に転用
    MD4D = mdd[4]; //電磁弁に転用
    MD5D = mdd[5]; //電磁弁に転用
    //MD6D = mdd[4];

    // MD1P = mdp[1];
    // MD2P = mdp[2];
    // MD3P = mdp[3];
    // MD4P = mdp[4];
    // MD5P = mdp[3];
    // MD6P = mdp[4];

    SERVO1.pulsewidth_us(map(data[8], 0, 180, 500, 2500));

    // SV1 = data[5];
    // SV2 = mdd[6];
    // SV3 = mdd[7];
    // SV4 = mdd[8];
  }
}