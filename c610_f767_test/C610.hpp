#ifndef RCT_C610_HPP
#define RCT_C610_HPP

#include <mbed.h>
#include <array>

// C610モーターのデータを管理する構造体
struct C610
{
    // モーターの電流制限（-10000 〜 10000）
    static constexpr int16_t current_limit = 10000;
    // C610のエンコーダ最大値（1回転のカウント数）
    static constexpr int16_t angle_max = 8192;
    // 角度換算用の係数（絶対角度を算出する際に使用）
    static constexpr float angle_divisor = 146.5f;

    // 電流を設定（0〜10の値を 0〜10000 にスケーリング）
    void set_current(float current)
    {
        raw_current_ = static_cast<int16_t>(current / 10.0f * current_limit);
    }

    // 生の電流値を直接設定
    void set_raw_current(int16_t raw_current)
    {
        raw_current_ = raw_current;
    }

 // RPM制御のための関数を追加
    void set_rpm(int16_t rpm)
    {
        target_rpm_ = rpm;
    }
    // モーターの角度（エンコーダ値）を取得
    uint16_t get_angle() const { return rx_.angle; }
    // モーターの回転速度（RPM）を取得
    int16_t get_rpm() const { return rx_.rpm; }
    // 実際に流れている電流値を取得（スケーリング済み）
    float get_actual_current() const
    {
        return static_cast<float>(rx_.actual_current) / current_limit * 10.0f;
    }
    // 実際に流れている生の電流値を取得
    int16_t get_actual_raw_current() const { return rx_.actual_current; }
    // モーターの温度を取得
    uint8_t get_temp() const { return rx_.temp; }
    // 設定された生の電流値を取得
    int16_t get_raw_current() const { return raw_current_; }

    /// @brief 角度を更新（循環なしの累積値を保持）
    void update_angle()
    {
        int16_t diff = rx_.angle - last_angle_;

        // 角度のオーバーフロー補正（エンコーダ値が 0 から 8192 へ循環する問題の対策）
        if (diff > angle_max / 2)
        {
            diff -= angle_max;
        }
        else if (diff < -angle_max / 2)
        {
            diff += angle_max;
        }

        // 累積角度を更新
        angle_ += diff;
        last_angle_ = rx_.angle;
    }

    // 累積角度（循環なしの値）を取得
    int32_t get_absolute_angle() const { return angle_; }

    /// @brief `get_absolute_angle()` の値を 146.5 で割った角度を返す
    float get_Angle() const
    {
        return static_cast<float>(angle_) / angle_divisor;
    }

    // 受信データパケットを解析し、各値を更新
    void parse_packet(const uint8_t data[8])
    {
        rx_.angle = uint16_t(data[0] << 8 | data[1]); // 角度データ
        rx_.rpm = int16_t(data[2] << 8 | data[3]);   // RPMデータ
        rx_.actual_current = int16_t(data[4] << 8 | data[5]); // 実際の電流値
        rx_.temp = data[6]; // 温度データ

        update_angle(); // 角度を更新
    }

private:
    // CAN通信で受信したデータを格納する構造体
    struct C610Packet
    {
        uint16_t angle;         // エンコーダ角度値
        int16_t rpm;            // RPM値
        int16_t actual_current; // 実際の電流値
        uint8_t temp;           // 温度値
    } rx_ = {};

    int16_t raw_current_ = 0; // 設定電流値（生データ）
    int16_t last_angle_ = 0;  // 前回の角度値
    int32_t angle_ = 0;       // 累積角度値
};

// 複数の C610 モーターを管理する構造体
struct C610Array
{
    // CANメッセージを解析し、該当するモーターのデータを更新
    void parse_packet(const CANMessage &msg)
    {
        if (msg.format == CANStandard && msg.type == CANData &&
            msg.len == 8 && 0x201 <= msg.id && msg.id <= 0x208)
        {
            arr_[msg.id - 0x201].parse_packet(msg.data);
        }
    }

    // 現在の各モーターの電流値をCANメッセージとして送信可能な形式に変換
    auto to_msgs() -> CANMessage
    {
        uint8_t buf[8] = {};
        for (int i = 0; i < 4; ++i)
        {
            buf[i * 2] = static_cast<uint8_t>(arr_[i].get_raw_current() >> 8);
            buf[i * 2 + 1] = static_cast<uint8_t>(arr_[i].get_raw_current() & 0xFF);
        }
        return CANMessage{0x200, buf, 8};
    }
    // RPM制御のためのメッセージ送信関数を追加
    // auto to_rpm_msgs() -> CANMessage
    // {
    //     uint8_t buf[8] = {};
    //     for (int i = 0; i < 4; ++i)
    //     {
    //         buf[i * 2] = static_cast<uint8_t>(arr_[i].get_rpm() >> 8);
    //         buf[i * 2 + 1] = static_cast<uint8_t>(arr_[i].get_rpm() & 0xFF);
    //     }
    //     return CANMessage{0x200, buf, 8};
    // }

    // インデックスでアクセスできるようにする
    auto &operator[](int index) { return arr_[index]; }
    // イテレータを取得
    auto begin() { return std::begin(arr_); }
    auto end() { return std::end(arr_); }

private:
    C610 arr_[8] = {}; // C610 モーターの配列（最大8台）
};

#endif // RCT_C610_HPP
