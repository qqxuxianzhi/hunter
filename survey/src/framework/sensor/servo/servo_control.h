#ifndef TASK_SERVO_CONTROL_H
#define TASK_SERVO_CONTROL_H

// #include <boost/atomic.hpp>
// #include <boost/thread.hpp>
#include <memory>
#include <queue>

#include "utils/macros.h"
#include "zmcaux.h"
namespace phoenix {
namespace sensor {
namespace servo {
#define AXIS_X_COOR_MIN_ -26
#define AXIS_X_COOR_MAX_ 216

#define AXIS_Y_COOR_MIN_ -24
#define AXIS_Y_COOR_MAX_ 494
class ServoControl {
  public:
    // ServoControl(framework::Task* manager);
    ~ServoControl();

    bool Start();
    bool Stop();
    bool CheckAxisXCoor(Float32_t x) {
        return (x >= AXIS_X_COOR_MIN_ && x <= AXIS_X_COOR_MAX_);
    }
    bool CheckAxisYCoor(Float32_t y) {
        return (y >= AXIS_Y_COOR_MIN_ && y <= AXIS_Y_COOR_MAX_);
    }
    Float32_t GetMaxCoorX() { return (Float32_t)AXIS_X_COOR_MAX_; }
    Float32_t GetMaxCoorY() { return (Float32_t)AXIS_Y_COOR_MAX_; }
    Float32_t GetMinCoorX() { return (Float32_t)AXIS_X_COOR_MIN_; }
    Float32_t GetMinCoorY() { return (Float32_t)AXIS_Y_COOR_MIN_; }
    //两轴直线插补
    void MoveToPos(Float32_t x, Float32_t y);
    void MoveToPosAbs(Float32_t x, Float32_t y);
    //圆弧插补
    void MoveCircle();
    //只移动Y轴，画线时使用
    void MoveYAxis(Float32_t y);
    //只移动X轴
    void MoveXAxis(Float32_t x);
    //书写
    void Write(std::string words);
    // IO控制
    void SetOutPort(int ionum, uint32 value);
    void GetOutPort(int ionum,uint32* value);
    //获取当前轴位置
    void GetCurrentPos(int axisnum, float *value) {
        ZAux_Direct_GetMpos(zmc_handle_, axisnum, value);
    }

    bool WaitAxisMoveDone(Int32_t axis_num);

    /*************************************************************
    Description:    //获取轴运行状态
    Input:
                    axis_num	轴
    Output:         //　
    Return:         //轴运行状态　true: 正在运行  false:停止运行
    *************************************************************/
    bool GetAxisIsRunning(Int32_t axis_num);

    /*************************************************************
     Description:    //获取轴当前的位置坐标
     Input:
                     axis_num	轴
     Output:         //　
     Return:         //轴的位置坐标　
     *************************************************************/
    float GetAxisMpos(Int32_t axis_num);

    /*************************************************************
    Description:    //获取轴当前的位置坐标
    Input:
                    axis_num	轴
    Output:         //　
    Return:         //轴的位置坐标　
    *************************************************************/
    float GetAxisDpos(Int32_t axis_num);

    /*************************************************************
    Description:    //获取轴的脉冲当量
    Input:
                    axis_num	轴
    Output:         //　
    Return:         //轴的脉冲当量
    *************************************************************/
    float GetAxisUnits(Int32_t axis_num);

    /*************************************************************
    Description:    //写table
    Input:
                    tabstart	写入的TABLE起始编号
                    numes		写入的数量
                    pfValue		写入的数据值
    Output:         //
    Return:         //错误码
    *************************************************************/
    void SetTable(int tabstart, int numes, float *pfValue);
    /*************************************************************
    Description:    //设置PWM 输出　
    Input:
                    ionum	io输出引脚
                    freq		pwm频率         频率 硬件PWM1M   软PWM 2K
                    duty		pwm占空比   占空比	(0-1)  0表示关闭PWM口
    Output:         //
    Return:         //错误码
    *************************************************************/
    void SetPwmOutput(int ionum, float freq, float duty);

    /*************************************************************
    Description:    //电子凸轮 同步运动
    Input:                    iaxis			轴号
                                    istartpoint		起始点TABLE编号
                                    iendpoint		结束点TABLE编号
                                    ftablemulti　位置比例，一般设为脉冲当量值
                                    fDistance　　参考运动的距离，用来计算总运动时间
      Output:         // Return:         //错误码
    *************************************************************/
    void CamMove(int iaxis, int istartpoint, int iendpoint, float ftablemulti,
                 float fDistance);

    /*************************************************************
    Description:    ////获取TABLE的Size
    Output:
                              piValue  TABLE的Size
    Return:         //错误码
    *************************************************************/
    Int32_t GetTableSize(int *piValue);

    /*************************************************************
   Description:    //读取输入信号
   Input:
                              ionum IN编号
   Output:         //piValue 输入口状态
   Return:         //错误码
   *************************************************************/
    Int32_t GetInPort(int ionum, uint32 *piValue);

    /*************************************************************
    Description:    //单轴运动停止
    Input:
                                    iaxis 轴号
                                    imode 模式
                                            0（缺省）取消当前运动
                                            1	取消缓冲的运动
                                            2	取消当前运动和缓冲运动。
                                            3	立即中断脉冲发送。
    Output:         //
    Return:
    *************************************************************/
    void CancelMovement(Int32_t axis_num, int imode) {
        ZAux_Direct_Single_Cancel(zmc_handle_, axis_num, imode);
    }

    //使能料仓出料
    void EnableRepositoryVibrate(bool enable) {
        if (enable) {
            ZAux_Direct_SetOp(zmc_handle_, 3, 1); //螺旋电机开启,电磁阀开启
            ZAux_Direct_SetOp(zmc_handle_, 5, 1); //料仓震动
        } else {
            ZAux_Direct_SetOp(zmc_handle_, 3, 0); //螺旋电机关闭,电磁阀关闭
            ZAux_Direct_SetOp(zmc_handle_, 5, 0); //料仓震动
        }
    }

    //漏斗出料
    void EnableHopperVibrate(bool enable) {
        if (enable) {
            //开启漏斗震动
            ZAux_Direct_SetOp(zmc_handle_, 8, 1);
            ZAux_Direct_SetOp(zmc_handle_, 4, 1); //漏斗震动
        } else {
            //关闭漏斗震动
            ZAux_Direct_SetOp(zmc_handle_, 8, 0);
            ZAux_Direct_SetOp(zmc_handle_, 4, 0); //漏斗震动
        }
    }

    void SetMerge(int axis, int value) {
        ZAux_Direct_SetMerge(zmc_handle_, axis, value);
    }
    void SetParameter(Int32_t argv[]);

  private:
    ZMC_HANDLE zmc_handle_;
    Int32_t axis_list_[2] = {0, 1};
    bool is_connected_;
    Int32_t pulse_;

  private:
    DECLARE_SINGLETON(ServoControl);
};
} // namespace servo
}  // namespace sensor
}  // namespace phoenix
#endif