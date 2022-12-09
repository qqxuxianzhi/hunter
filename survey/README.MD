**运动控制卡IO表**

**输入IO**

    300mm行程滑轨IO  0轴  车头方向为正
    In2   正限位
    In0   原点
    In1   负限位

    500mm行程滑轨IO  1轴  车辆右方向为正
    In3   负限位
    In5   原点
    In4   正限位

    In6   料斗料满
    //In9   挡板关闭到位 未启用
    //In10  挡板开启到位 未启用

**输出IO**

    Out8  料斗出料
    Out3  料仓电磁阀、料仓旋转出料
    Out4  料斗震动
    //Out5  料仓旋转出料
    Out0  LED 黄
    Out1  LED 绿
    Out2  LED 红
