
#include "alarm_lamp.h"

#include "math/math_utils.h"

namespace phoenix {
namespace framework {

AlarmLamp::AlarmLamp() : duty_(0), step_(0.01) {
    servo_control_ = sensor::servo::ServoControl::instance();
    alarm_color_ = AlarmColor::ALARM_COLOR_BLACK;
    flashing_mode_ = FlashingMode::FLASHING_MODE_BREATHING;
    running_flag_alarm_lamp_ = false;
}

AlarmLamp::AlarmLamp(AlarmLampPin alarmPin)
    : duty_(0), step_(0.01), alarm_pin_(alarmPin) {
    servo_control_ = sensor::servo::ServoControl::instance();
    alarm_color_ = AlarmColor::ALARM_COLOR_BLACK;
    flashing_mode_ = FlashingMode::FLASHING_MODE_BREATHING;
    running_flag_alarm_lamp_ = false;
}
AlarmLamp::~AlarmLamp() { Stop(); }

bool AlarmLamp::Start() {
    if (running_flag_alarm_lamp_) {
        return (true);
    }
    running_flag_alarm_lamp_ = true;
    thread_alarm_lamp= boost::thread(
            boost::bind(&AlarmLamp::ThreadAlarmLamp, this));
    thread_change_duty= boost::thread(
            boost::bind(&AlarmLamp::ThreadBreathingChangeDuty, this));
  LOG_INFO(3) << "Create thread of task alarm lamp.... [OK]";
  std::cout << "Start id:" << boost::this_thread::get_id() << std::endl;
  return (true);
}

bool AlarmLamp::Stop() {
    running_flag_alarm_lamp_ = false;
    alarm_color_ = AlarmColor::ALARM_COLOR_BLACK;
    LOG_INFO(3) << "Stop thread of task alarm lamp...";

    bool ret =
    thread_alarm_lamp.timed_join(boost::posix_time::seconds(1));
    ret =thread_change_duty.timed_join(boost::posix_time::seconds(1));

    servo_control_->SetPwmOutput(alarm_pin_.red_pin_, 0, 0);
    servo_control_->SetPwmOutput(alarm_pin_.green_pin_, 0, 0);
    servo_control_->SetPwmOutput(alarm_pin_.yellow_pin_, 0, 0);
    servo_control_->SetOutPort(alarm_pin_.red_pin_, 0);    //关
    servo_control_->SetOutPort(alarm_pin_.green_pin_, 0);  //关
    servo_control_->SetOutPort(alarm_pin_.yellow_pin_, 0); //关

    if (false == ret) {
        LOG_ERR << "Failed to wait thread of task alarm lamp to stop.";
    }
    if (ret) {
        LOG_INFO(3) << "Stop thread of task alarm lamp. ... [OK]";
    } else {
        LOG_INFO(3) << "Stop thread of task alarm lamp. ... [NG]";
    }
    return ret;
}
void AlarmLamp::SetLampParam(FlashingMode mode, AlarmColor color,
                             AlarmLampPin alarm_pin) {
    alarm_color_ = color;
    flashing_mode_ = mode;
    alarm_pin_ = alarm_pin;
}
void AlarmLamp::SetAlarmColor(AlarmColor color) { alarm_color_ = color; }
void AlarmLamp::SetAlarmFlashingMode(FlashingMode mode) {
    flashing_mode_ = mode;
}
void AlarmLamp::SetAlarmLampPin(AlarmLampPin alarm_pin) {
    alarm_pin_ = alarm_pin;
}

void  AlarmLamp::ThreadAlarmLamp(){
    Dormancy dormancy(5);
    while (running_flag_alarm_lamp_){
        if(flashing_mode_ != cur_flashing_mode_){
            cur_flashing_mode_ = flashing_mode_;
            servo_control_->SetPwmOutput(alarm_pin_.red_pin_, 0, 0);
            servo_control_->SetPwmOutput(alarm_pin_.green_pin_, 0, 0);
            servo_control_->SetPwmOutput(alarm_pin_.yellow_pin_, 0, 0);
            servo_control_->SetOutPort(alarm_pin_.red_pin_, 0);    //关
            servo_control_->SetOutPort(alarm_pin_.green_pin_, 0);  //关
            servo_control_->SetOutPort(alarm_pin_.yellow_pin_, 0); //关
        }
        switch (flashing_mode_) {
        case FlashingMode::FLASHING_MODE_BREATHING:
            BreathingFlashing(duty_);
            dormancy.Sleep(); 
            break;
        case FlashingMode::FLASHING_MODE_STROBOSCOPIC:
            StroboscopicFlashing(0.4);
            break;
        case FlashingMode::FLASHING_MODE_STEADY_ON:
            SteadyOnFlashing();
            break;
        default:
            break;
        }
    }
}
void AlarmLamp::ThreadBreathingChangeDuty() {
    Dormancy dormancy(22);
    float min_duty = 0.005;
    float max_duty = 0.995;
    float boundary_duty = 0.025;
    float init_step = common::com_abs(step_);
    float dev_step =  common::com_abs(step_) / 2.0;
    while (running_flag_alarm_lamp_)
    {
         if (duty_ <= min_duty) {
            step_ = common::com_abs(step_);
        }
        if (duty_ >= max_duty) {
            step_ = -common::com_abs(step_);
        }
        if (duty_ <= boundary_duty) {
            step_ = step_ > 0 ? dev_step : -dev_step;
        } else {
            step_ = step_ > 0 ? init_step : -init_step;
        }
        if (duty_ + step_ >= max_duty) {
            duty_ = max_duty;
        } else if (duty_ + step_ <= min_duty) {
            duty_ = min_duty;
        } else {
            duty_ = duty_ + step_;
        }
        dormancy.Sleep(); 
    }
}

void AlarmLamp::StroboscopicFlashing(float duty) {
    unsigned int on_time = static_cast<unsigned int>(
        1000000.0 / PWM_STROBOSCOPIC_FREQ * duty);
    unsigned int off_time = static_cast<unsigned int>(
        1000000.0 / PWM_STROBOSCOPIC_FREQ * (1.0 - duty)); 
    if ((alarm_color_ & 0x01) == 0x01) {
        servo_control_->SetOutPort(alarm_pin_.red_pin_, 1); //开
        //LOG_INFO(3) << "Red Stroboscopic";              
    }
    if ((alarm_color_ & 0x02) == 0x02) {
        servo_control_->SetOutPort(alarm_pin_.green_pin_, 1); //开
        //LOG_INFO(3) << "Green Stroboscopic";       
    }
    if ((alarm_color_ & 0x04) == 0x04) {
        servo_control_->SetOutPort(alarm_pin_.yellow_pin_, 1); //开
        //LOG_INFO(3) << "Yellow Stroboscopic";   
    }
    usleep(on_time);
    servo_control_->SetOutPort(alarm_pin_.red_pin_, 0); //关
    servo_control_->SetOutPort(alarm_pin_.green_pin_, 0); //关
    servo_control_->SetOutPort(alarm_pin_.yellow_pin_, 0); //关
    usleep(off_time);
  }

void AlarmLamp::BreathingFlashing(float duty) {
#if SERVO_SERVO_PWM
    if ((alarm_color_ & 0x01) == 0x01) {
        servo_control_->SetPwmOutput(alarm_pin_.red_pin_, PWM_BREATHING_FREQ,
                                     duty);                
    } else {
        servo_control_->SetPwmOutput(alarm_pin_.red_pin_, PWM_BREATHING_FREQ,
                                     0);
        servo_control_->SetOutPort(alarm_pin_.red_pin_, 0);    //关
    }
    if ((alarm_color_ & 0x02) == 0x02) {
        servo_control_->SetPwmOutput(alarm_pin_.green_pin_, PWM_BREATHING_FREQ,
                                     duty);
    } else {
        servo_control_->SetPwmOutput(alarm_pin_.green_pin_, PWM_BREATHING_FREQ,
                                     0);
        servo_control_->SetOutPort(alarm_pin_.green_pin_, 0);  //关                             
    }
    if ((alarm_color_ & 0x04) == 0x04) {
        servo_control_->SetPwmOutput(alarm_pin_.yellow_pin_, PWM_BREATHING_FREQ,
                                     duty);               
    } else {
        servo_control_->SetPwmOutput(alarm_pin_.yellow_pin_, PWM_BREATHING_FREQ,
                                     0);
        servo_control_->SetOutPort(alarm_pin_.yellow_pin_, 0); //关                                 
    }
#else
    unsigned int on_time = static_cast<unsigned int>(
        1000000.0 / PWM_BREATHING_FREQ * duty);
    unsigned int off_time = static_cast<unsigned int>(
        1000000.0 / PWM_BREATHING_FREQ * (1.0 - duty));

    if ((alarm_color_ & 0x01) == 0x01) {
        servo_control_->SetOutPort(alarm_pin_.red_pin_, 1); //开
    }
    if ((alarm_color_ & 0x02) == 0x02) {
        servo_control_->SetOutPort(alarm_pin_.green_pin_, 1); //开
    }
    if ((alarm_color_ & 0x04) == 0x04) {
        servo_control_->SetOutPort(alarm_pin_.yellow_pin_, 1); //开
    }
    usleep(on_time);
    servo_control_->SetOutPort(alarm_pin_.red_pin_, 0); //关
    servo_control_->SetOutPort(alarm_pin_.green_pin_, 0); //关
    servo_control_->SetOutPort(alarm_pin_.yellow_pin_, 0); //关
    usleep(off_time);
#endif
}

void  AlarmLamp::SteadyOnFlashing(){
     if ((alarm_color_ & 0x01) == 0x01) {
        servo_control_->SetOutPort(alarm_pin_.red_pin_, 1); //开
        //LOG_INFO(3) << "Red SteadyOn";              
    }else{
        servo_control_->SetOutPort(alarm_pin_.red_pin_, 0); //关
    }
    if ((alarm_color_ & 0x02) == 0x02) {
        servo_control_->SetOutPort(alarm_pin_.green_pin_, 1); //开
        //LOG_INFO(3) << "Green SteadyOn";              
    }else{
        servo_control_->SetOutPort(alarm_pin_.green_pin_, 0); //关
    }
    if ((alarm_color_ & 0x04) == 0x04) {
        servo_control_->SetOutPort(alarm_pin_.yellow_pin_, 1); //开
        //LOG_INFO(3) << "Yellow SteadyOn";              
    }else{
         servo_control_->SetOutPort(alarm_pin_.yellow_pin_, 0); //关
    }
    usleep(3000000);
}
}  // namespace framework
}  // namespace phoenix