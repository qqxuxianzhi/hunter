#ifndef PHOENIX_FRAMEWORK_SVG_TRANSFORM_H_
#define PHOENIX_FRAMEWORK_SVG_TRANSFORM_H_

#include <boost/array.hpp>
#include <svgpp/parser/transform_list.hpp>

#include "geometry/vec2d.h"
#include "utils/log.h"
#include "utils/macros.h"

namespace phoenix {
namespace framework {
class SvgTransformContext {
 public:
  SvgTransformContext(std::vector<phoenix::common::Vec2d>* points)
      : points_(points){};
  ~SvgTransformContext(){};

  void transform_matrix(const boost::array<double, 6>& matrix);

  void transform_translate(double tx, double ty);

  void transform_translate(double tx);

  void transform_scale(double sx, double sy);

  void transform_scale(double scale);

  void transform_rotate(double angle);

  void transform_rotate(double angle, double cx, double cy);

  void transform_skew_x(double angle);

  void transform_skew_y(double angle);

 private:
  std::vector<phoenix::common::Vec2d>* points_;
};
}  // namespace framework
}  // namespace phoenix

#endif  // PHOENIX_FRAMEWORK_SVG_TRANSFORM_H_