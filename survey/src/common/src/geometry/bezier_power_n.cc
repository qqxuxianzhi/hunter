#include "geometry/bezier_power_n.h"

#include <cmath>
#include <iostream>
#include <sstream>

#include "math/math_utils.h"
#include "utils/log.h"

namespace phoenix {
namespace common {

/******************************************************************************/
/*
 *设 Bt 为要计算的贝塞尔曲线上的坐标, N 为控制点个数, P0,P1,P2..Pn
 * 为贝塞尔曲线控制点的坐标,当 N 值不同时有如下计算公式: 如 N 为 3
 * 表示贝塞尔曲线的控制点有 3 个点,这时 n 为 2, 这三个点分别用 P0,P1,P2 表示。
 *
 * N = 3: P = (1-t)^2*P0 + 2*(1-t)*t*P1 + t^2*P2
 * N = 4: P = (1-t)^3*P0 + 3*(1-t)^2*t*P1 + 3(1-t)*t^2*P2 + t^3*P3
 * N = 5: P = (1-t)^4*P0 + 4*(1-t)^3*t*P1 + 6(1-t)^2*t^2*P2 + 4*(1-t)*t^3*P3
 * +t^4*P4
 *
 * 表达式可统一表示为如下形: 常数 a,b 和 c。
 * a * (1 - t)^b * t^c * Pn
 *
 * a: 在 N 分别为 1,2,3,4,5 时将其值用如下形式表示：
 * N=1:———1
 * N=2:——–1  1
 * N=3:——1  2  1
 * N=4:—–1  3  3  1
 * N=5:—1  4  6  4  1
 * a 值的改变规则为： 杨辉三角
 * b: (N - 1) 递减到 0 (b 为 1-t 的幂)
 * c: 0 递增到 (N - 1) (c 为 t 的幂)
 */

/******************************************************************************/

StaticVector<Vec2d, BezierPowerN::kBezierMax> BezierPowerN::CalcBezierPoints(
    const StaticVector<Vec2d, kWayPointMax>& way_points, Int32_t resolution) {
  if (resolution > kBezierMax) {
    resolution = kBezierMax;
  }
  StaticVector<Vec2d, BezierPowerN::kBezierMax> bezier_points;
  Int32_t number = way_points.Size();
  if (number >= 2) {
    //计算杨辉三角
    std::vector<Int32_t> a_para;
    a_para.resize(number);
    a_para[0] = a_para[1] = 1;

    for (Int32_t i = 3; i <= number; i++) {
      std::vector<Int32_t> tmp;
      tmp.resize(i - 1);
      for (Int32_t j = 0; j < tmp.size(); j++) {
        tmp[j] = a_para[j];
      }

      a_para[0] = a_para[i - 1] = 1;
      for (Int32_t j = 0; j < i - 2; j++) {
        a_para[j + 1] = tmp[j] + tmp[j + 1];
      }
    }
    bezier_points.Resize(resolution);

    //计算坐标点
    Float32_t temp = 0.0f;
    for (Int32_t i = 0; i < resolution; i++) {
      Float32_t t = (Float32_t)i / resolution;

      temp = 0.0f;
      for (Int32_t k = 0; k < number; k++) {
        temp += std::pow(1 - t, number - k - 1) * way_points[k].x() *
                std::pow(t, k) * a_para[k];
      }
      bezier_points[i].set_x(temp);

      temp = 0.0f;
      for (Int32_t k = 0; k < number; k++) {
        temp += std::pow(1 - t, number - k - 1) * way_points[k].y() *
                std::pow(t, k) * a_para[k];
      }
      bezier_points[i].set_y(temp);
    }
  }

  return bezier_points;
}

}  // namespace common
}  // namespace phoenix
