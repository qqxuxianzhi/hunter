/******************************************************************************
 ** 定义报警灯的接口
 ******************************************************************************
 *
 *  定义报警灯的接口
 *
 *  @file       alarm_lamp.h
 *
 *  @author     xianzhix
 *  @date       2022.10.9
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#ifndef ALARM_LAMP_H_
#define ALARM_LAMP_H_
#include <boost/thread.hpp>
#include <iostream>

#include "pc/util.h"
#include "sensor/servo/servo_control.h"
#include "utils/log.h"
namespace phoenix {
namespace framework {
/******************************************************************************
SERVO_SERVO_PWM设置PWM
 1: 控制卡自带的PWM    0:为通过延时的方式实现PWM(效果不好)
******************************************************************************/
#define SERVO_SERVO_PWM (1)

#define PWM_BREATHING_FREQ 140  //呼吸灯　PWM 60HZ
#define PWM_STROBOSCOPIC_FREQ 1 //频闪
class AlarmLamp {
  public:
    //报警灯的颜色
    enum AlarmColor {
        ALARM_COLOR_BLACK = 0, //黑色为不亮
        ALARM_COLOR_RED = 1,
        ALARM_COLOR_GREEN = 2,
        ALARM_COLOR_RED_GREEN = 3,
        ALARM_COLOR_YELLOW = 4,
        ALARM_COLOR_RED_YELLOW = 5,
        ALARM_COLOR_YELLOW_GREEN = 6,
        ALARM_COLOR_WHITE = 7
    };
    //报警灯的闪烁模式
    enum FlashingMode { 
        FLASHING_MODE_BREATHING, 
        FLASHING_MODE_STROBOSCOPIC,
        FLASHING_MODE_STEADY_ON };
    //报警灯的引脚设置
    struct AlarmLampPin {
        int red_pin_;
        int green_pin_;
        int yellow_pin_;

        AlarmLampPin() = default;
        AlarmLampPin(int red_pin, int green_pin, int yellow_pin)
            : red_pin_(red_pin), green_pin_(green_pin),
              yellow_pin_(yellow_pin) {}
    };

    AlarmLamp();
    AlarmLamp(AlarmLampPin alarmPin);
    ~AlarmLamp();

    bool Start();
    bool Stop();
    void SetLampParam(FlashingMode mode, AlarmColor color,
                      AlarmLampPin alarm_pin);
    void SetAlarmColor(AlarmColor color);
    void SetAlarmFlashingMode(FlashingMode mode);
    void SetAlarmLampPin(AlarmLampPin alarm_pin);

  private:
    void ThreadAlarmLamp(); 
    void ThreadBreathingChangeDuty();

    //处理报警灯控制
    void StroboscopicFlashing(float duty);
    void BreathingFlashing(float duty);
    void SteadyOnFlashing();
  private:
    sensor::servo::ServoControl *servo_control_;
    AlarmColor alarm_color_;
    FlashingMode flashing_mode_;
    FlashingMode cur_flashing_mode_;
    AlarmLampPin alarm_pin_;

    boost::thread thread_alarm_lamp;
    boost::thread thread_change_duty;

    float duty_; // 占空比(0---1)
    float step_; // 呼吸灯占空比变化的step 默认值为0.05
    boost::atomic_bool running_flag_alarm_lamp_;
};

} // namespace framework
} // namespace phoenix
#endif