#ifndef PHOENIX_FRAMEWORK_SVG_PATH_H_
#define PHOENIX_FRAMEWORK_SVG_PATH_H_
#include <svgpp/parser/path_data.hpp>

#include "geometry/vec2d.h"
#include "utils/log.h"
#include "utils/macros.h"

// using namespace svgpp;

namespace phoenix {
namespace framework {
class SvgPathContext {
 public:
  enum PathPointType {
    PATH_POINT_TYPE_START = 0,
    PATH_POINT_TYPE_LINE,
    PATH_POINT_TYPE_BEZIER_Q,
    PATH_POINT_TYPE_BEZIER_T,
    PATH_POINT_TYPE_BEZIER_C,
    PATH_POINT_TYPE_BEZIER_S,
  };

  struct PathPoints {
    PathPointType point_type;
    std::vector<phoenix::common::Vec2d> points;
  };

  SvgPathContext(){};
  ~SvgPathContext(){};
  void path_move_to(double x, double y, svgpp::tag::coordinate::absolute);

  void path_line_to(double x, double y, svgpp::tag::coordinate::absolute);

  void path_line_to_ortho(double coord, bool horizontal,
                          svgpp::tag::coordinate::absolute);

  void path_cubic_bezier_to(double x1, double y1, double x2, double y2,
                            double x, double y,
                            svgpp::tag::coordinate::absolute);

  void path_cubic_bezier_to(double x2, double y2, double x, double y,
                            svgpp::tag::coordinate::absolute);

  void path_quadratic_bezier_to(double x1, double y1, double x, double y,
                                svgpp::tag::coordinate::absolute);

  void path_quadratic_bezier_to(double x, double y,
                                svgpp::tag::coordinate::absolute);

  void path_elliptical_arc_to(double rx, double ry, double x_axis_rotation,
                              bool large_arc_flag, bool sweep_flag, double x,
                              double y, svgpp::tag::coordinate::absolute);

  void path_close_subpath();

  void path_exit() {}

  std::vector<PathPoints> GetPathPoints() { return path_points_; }

  bool GetNextSubPathPoints(std::vector<phoenix::common::Vec2d>& sub_path_points);

 private:
  std::vector<PathPoints> path_points_;
  phoenix::common::Vec2d current_pos_;
  phoenix::common::Vec2d start_pos_;
};
}  // namespace framework
}  // namespace phoenix

#endif  // PHOENIX_FRAMEWORK_SVG_PATH_H_