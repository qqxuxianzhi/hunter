#include "servo_control.h"

#include "pc/util.h"
#include "utils/log.h"
namespace phoenix {
namespace sensor {
namespace servo {

ServoControl::ServoControl() { pulse_ = 1000; }
ServoControl::~ServoControl() { Stop(); }

void ServoControl::SetParameter(Int32_t argv[]) { pulse_ = argv[0]; }

bool ServoControl::Start() {
  Int32_t ret = 0;  //= ZAux_SearchEth("192.168.1.11", 5000);
  if (ret == ERR_OK) {
    Int32_t ret = ZAux_OpenEth("192.168.0.11", &zmc_handle_);
    if (ret != 0) {
      LOG_ERR << "Connect to Zmotion Error:" << ret;
      return (false);
    }
    is_connected_ = true;
  } else {
    LOG_ERR << "Can not find Zmotion at '192.168.0.11':" << ret;
    return (false);
  }

  //设置原点信号
  ZAux_Direct_SetDatumIn(zmc_handle_, 0, 0);  // 300mm行程
  ZAux_Direct_SetDatumIn(zmc_handle_, 1, 5);  // 500mm行程
  ZAux_Direct_SetFwdIn(zmc_handle_, 0, 2);
  ZAux_Direct_SetRevIn(zmc_handle_, 0, 1);
  ZAux_Direct_SetFwdIn(zmc_handle_, 1, 4);
  ZAux_Direct_SetRevIn(zmc_handle_, 1, 3);

  ZAux_Direct_SetDatumIn(zmc_handle_, 2, -1);
  ZAux_Direct_SetDatumIn(zmc_handle_, 3, -1);
  ZAux_Direct_SetFwdIn(zmc_handle_, 2, -1);
  ZAux_Direct_SetRevIn(zmc_handle_, 2, -1);
  ZAux_Direct_SetFwdIn(zmc_handle_, 3, -1);
  ZAux_Direct_SetRevIn(zmc_handle_, 3, -1);

  //初始化轴参数
  LOG_INFO(3) << "Servo Start go back to home.";
  for (int i = 0; i < 2; i++) {
    ZAux_Direct_SetAtype(zmc_handle_, i, 1);     //轴类型  脉冲轴
    ZAux_Direct_SetUnits(zmc_handle_, i, pulse_); //脉冲当量 1 脉冲为单位
    ZAux_Direct_SetSpeed(zmc_handle_, i, 50);   //速度	mm/ S
    ZAux_Direct_SetAccel(zmc_handle_, i, 1000); //加速度
    ZAux_Direct_SetDecel(zmc_handle_, i, 1000); //减速度
    ZAux_Direct_SetSramp(zmc_handle_, i, 100);  //设置 S 曲线,单位 ms
    ZAux_Direct_SetCreep(zmc_handle_, i, 1);    //反向查找速度为10mm/s
    //开始回零，碰到限位反转，回零方式：原点+反找模式
    Uint32_t in = 0, revin = 0;
    if (i == 0) {
        ZAux_Direct_GetIn(zmc_handle_, 2, &in);
        ZAux_Direct_GetIn(zmc_handle_, 1, &revin);
    } else if (i == 1) {
        ZAux_Direct_GetIn(zmc_handle_, 4, &in);
        ZAux_Direct_GetIn(zmc_handle_, 3, &revin);
    }
    if (in || revin) { //已经在原点，前移50mm后再找零
        ZAux_Direct_Single_MoveAbs(zmc_handle_, i, 50);
        WaitAxisMoveDone(i);
    }
    ZAux_Direct_Single_Datum(zmc_handle_, i, 14);
  }
  WaitAxisMoveDone(0);
  WaitAxisMoveDone(1);

//喷粉设备初始化
#if 0
  ZAux_Direct_SetAtype(zmc_handle_, 2, 1);     //轴类型  脉冲轴
  ZAux_Direct_SetUnits(zmc_handle_, 2, 1);     //脉冲当量 1 脉冲为单位
  ZAux_Direct_SetSpeed(zmc_handle_, 2, 500);   //速度	mm/ S
  ZAux_Direct_SetAccel(zmc_handle_, 2, 1000);  //加速度
  ZAux_Direct_SetDecel(zmc_handle_, 2, 1000);  //减速度
  ZAux_Direct_SetDpos(zmc_handle_, 2, 0.0);
  ZAux_Direct_SetMpos(zmc_handle_, 2, 0.0);

  ZAux_Direct_SetAtype(zmc_handle_, 3, 1);     //轴类型  脉冲轴
  ZAux_Direct_SetUnits(zmc_handle_, 3, 1);     //脉冲当量 1 脉冲为单位
  ZAux_Direct_SetSpeed(zmc_handle_, 3, 500);   //速度	mm/ S
  ZAux_Direct_SetAccel(zmc_handle_, 3, 1000);  //加速度
  ZAux_Direct_SetDecel(zmc_handle_, 3, 1000);  //减速度
  ZAux_Direct_SetDpos(zmc_handle_, 3, 0.0);
  ZAux_Direct_SetMpos(zmc_handle_, 3, 0.0);
#endif

    ZAux_Direct_SetSpeed(zmc_handle_, 0, 150);  //速度	mm/ S
    ZAux_Direct_SetAccel(zmc_handle_, 0, 4500); //加速度
    ZAux_Direct_SetDecel(zmc_handle_, 0, 8000); //减速度
    ZAux_Direct_SetSramp(zmc_handle_, 0, 500);  //设置 S 曲线,单位 ms

    ZAux_Direct_SetSpeed(zmc_handle_, 1, 150);  //速度	mm/ S
    ZAux_Direct_SetAccel(zmc_handle_, 1, 4500); //加速度
    ZAux_Direct_SetDecel(zmc_handle_, 1, 8000); //减速度
    ZAux_Direct_SetSramp(zmc_handle_, 1, 100);  //设置 S 曲线,单位 ms

#if 0
  ZAux_Direct_Single_MoveAbs(zmc_handle_, 0, -40);
  ZAux_Direct_Single_MoveAbs(zmc_handle_, 1, -40);
  WaitAxisMoveDone(0);
  WaitAxisMoveDone(1);
  LOG_INFO(3) <<"x min:" << GetAxisDpos(0)<< "y min:" << GetAxisDpos(1);

  ZAux_Direct_Single_MoveAbs(zmc_handle_, 0, 250);
  ZAux_Direct_Single_MoveAbs(zmc_handle_, 1, 520);
  WaitAxisMoveDone(0);
  WaitAxisMoveDone(1);
  LOG_INFO(3) <<"x max:" << GetAxisDpos(0)<< "y max:" << GetAxisDpos(1);
#endif

    ZAux_Direct_Single_MoveAbs(zmc_handle_, 0, 100);
    ZAux_Direct_Single_MoveAbs(zmc_handle_, 1, 300);
    return (true);
}

bool ServoControl::Stop() {
  if (is_connected_) {
    CancelMovement(0,2);//取消轴0运动
    CancelMovement(1,2); //取消轴1运动
    EnableHopperVibrate(false);
    EnableRepositoryVibrate(false);
    MoveToPosAbs(100, 300.0);
    WaitAxisMoveDone(0);
    ZAux_Close(zmc_handle_);
    zmc_handle_ = Nullptr_t;
    std::cout << "Servo Control Stop" << std::endl;
  }
  return (true);
}

void ServoControl::MoveToPos(Float32_t x, Float32_t y) {
  Float32_t disance_list[2] = {x, y};
  ZAux_Direct_Move(zmc_handle_, 2, axis_list_, disance_list);
}
void ServoControl::MoveToPosAbs(Float32_t x, Float32_t y) {
    if (CheckAxisXCoor(x) && CheckAxisYCoor(y)) {
        Float32_t disance_list[2] = {x, y};
        ZAux_Direct_MoveAbs(zmc_handle_, 2, axis_list_, disance_list);
    } else if (CheckAxisXCoor(x)) {
        ZAux_Direct_Single_MoveAbs(zmc_handle_, 0, x);
    } else if (CheckAxisYCoor(y)) {
        ZAux_Direct_Single_MoveAbs(zmc_handle_, 1, y);
    } else {
        LOG_ERR << "Out of Axis move range.";
    }
}
//只移动X轴
void ServoControl::MoveXAxis(Float32_t x) {
    if (CheckAxisXCoor(x)) {
        ZAux_Direct_Single_MoveAbs(zmc_handle_, 0, x);
    } else {
        LOG_ERR << "Out of X Axis move range.";
    }
}
void ServoControl::MoveYAxis(Float32_t y) {
    if (CheckAxisYCoor(y)) {
        ZAux_Direct_Single_MoveAbs(zmc_handle_, 1, y);
    } else {
        LOG_ERR << "Out of Y Axis move range.";
    }
}

void ServoControl::SetOutPort(int ionum, uint32 value) {
  ZAux_Direct_SetOp(zmc_handle_, ionum, value);
}

void ServoControl::GetOutPort(int ionum,uint32* value){
  ZAux_Direct_GetOp(zmc_handle_, ionum, value);
}

/*************************************************************
Description:    //写table
Input:    　tabstart	写入的TABLE起始编号
                      numes		写入的数量
                      pfValue		写入的数据值
Output:         //
Return:         //错误码
*************************************************************/
void ServoControl::SetTable(int tabstart, int numes, float *pfValue) {
    ZAux_Direct_SetTable(zmc_handle_, tabstart, numes, pfValue);
}
/*************************************************************
Description:    //设置PWM 输出　
  Input:       
                  ionum	io输出引脚
                  freq		pwm频率
                  duty		pwm占空比　占空比	(0-1)  0表示关闭PWM口
  *************************************************************/
void  ServoControl::SetPwmOutput(int ionum,float freq,float duty) {
  ZAux_Direct_SetPwmFreq(zmc_handle_,ionum,freq);
  ZAux_Direct_SetPwmDuty(zmc_handle_,ionum,duty);
}
/*************************************************************
Description:    //电子凸轮 同步运动
Input:    　  iaxis			轴号
                      istartpoint : 起始点 TABLE 编号,存储第一个点的位置
                      iendpoint:  结束点 TABLE 编号
                      ftablemulti:位置乘以这个比例,一般设为脉冲当量值
                      fDistance: 参考运动的距离
                      总时间=distance/轴 speed
*************************************************************/
void ServoControl::CamMove(int iaxis, int istartpoint, int iendpoint,
                           float ftablemulti, float fDistance) {
    ZAux_Direct_Cam(zmc_handle_, iaxis, istartpoint, iendpoint, ftablemulti,
                    fDistance);
}
/*************************************************************
Description:    ////获取TABLE的Size
Output:
                          piValue  TABLE的Size
Return:         //错误码
*************************************************************/
Int32_t ServoControl::GetTableSize(int *piValue) {
    int32 iresult;
    char cmdbuff[2048];
    char cmdbuffAck[2048];
    //生成命令
    strcpy(cmdbuff, "?TSIZE");
    //调用命令执行函数
    iresult = ZAux_Execute(zmc_handle_, cmdbuff, cmdbuffAck, 2048);
    if (ERR_OK != iresult) {
        return iresult;
    }
    if (0 == strlen(cmdbuffAck)) {
        return ERR_NOACK;
    }
    sscanf(cmdbuffAck, "%d", piValue);
    return ERR_OK;
}

Int32_t ServoControl::GetInPort(int ionum, uint32 *piValue) {
    return ZAux_Direct_GetIn(zmc_handle_, ionum, piValue);
}
bool ServoControl::WaitAxisMoveDone(Int32_t axis_num) {
  //查询轴运动状态 status 0:没有完成, -1:运动完成
  Int32_t status = 0;
  framework::Dormancy dormancy(100);
  while (status == 0) {
    ZAux_Direct_GetIfIdle(zmc_handle_, axis_num, &status);
    dormancy.Sleep();
  }
  return (true);
}
/*************************************************************
 Description:    //获取轴当前的运动状态
 *************************************************************/
bool ServoControl::GetAxisIsRunning(Int32_t axis_num) {
    //查询轴运动状态 status 0:没有完成, -1:运动完成
    Int32_t status = 0;
    bool is_running = false;
    ZAux_Direct_GetIfIdle(zmc_handle_, axis_num, &status);
    is_running = (status != -1);
    return is_running;
}
/*************************************************************
 Description:    //获取轴当前的位置坐标
 *************************************************************/
float ServoControl::GetAxisMpos(Int32_t axis_num) {
    float m_pos = 0;
    ZAux_Direct_GetMpos(zmc_handle_, axis_num, &m_pos);
    return m_pos;
}

/*************************************************************
 Description:    //获取轴当前的位置坐标
 *************************************************************/
float ServoControl::GetAxisDpos(Int32_t axis_num) {
    float d_pos = 0;
    ZAux_Direct_GetDpos(zmc_handle_, axis_num, &d_pos);
    return d_pos;
}

  /*************************************************************
  Description:    //获取轴的脉冲当量
  *************************************************************/
  float ServoControl::GetAxisUnits(Int32_t axis_num){
    float units=0;
    ZAux_Direct_GetUnits(zmc_handle_, axis_num, &units);
    return units;
  }


}  // namespace servo
}  // namespace sensor
}  // namespace phoenix