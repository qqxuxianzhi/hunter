#ifndef PHOENIX_COMMON_BEZIER_POWER_N_H_
#define PHOENIX_COMMON_BEZIER_POWER_N_H_

#include <cmath>
#include <string>

#include "container/static_vector.h"
#include "geometry/vec2d.h"
#include "utils/log.h"
#include "utils/macros.h"

namespace phoenix {
namespace common {

/**
 * @class BezierPowerN
 * @brief Bezier曲线N阶
 */
class BezierPowerN {
 public:
  /// 最大控制点个数
  static const Int32_t kWayPointMax = 32;
  /// 最大解析度
  static const Int32_t kBezierMax = 1024;

  BezierPowerN() {}

  ~BezierPowerN() {}

  /**
   * @brief 计算Bezier点
   */
  static StaticVector<Vec2d, kBezierMax> CalcBezierPoints(
      const StaticVector<Vec2d, kWayPointMax>& way_points, Int32_t resolution);
};
}  // namespace common
}  // namespace phoenix

#endif  // PHOENIX_COMMON_BEZIER_POWER_N_H_
