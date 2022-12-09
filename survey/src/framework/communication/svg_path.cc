#include "communication/svg_path.h"

#include <thread>

#include "geometry/bezier_power_n.h"
namespace phoenix {
namespace framework {
void SvgPathContext::path_move_to(double x, double y,
                                  svgpp::tag::coordinate::absolute) {
  phoenix::common::Vec2d pos;
  PathPoints path_points;

  path_points.point_type = PATH_POINT_TYPE_START;

  pos.set_x(x);
  pos.set_y(y);
  path_points.points.push_back(pos);

  path_points_.push_back(path_points);

  start_pos_.set_x(x);
  start_pos_.set_y(y);
  current_pos_.set_x(x);
  current_pos_.set_y(y);
}

void SvgPathContext::path_line_to(double x, double y,
                                  svgpp::tag::coordinate::absolute) {
  phoenix::common::Vec2d pos;
  PathPoints path_points;

  path_points.point_type = PATH_POINT_TYPE_LINE;

  pos.set_x(current_pos_.x());
  pos.set_y(current_pos_.y());
  path_points.points.push_back(pos);

  pos.set_x(x);
  pos.set_y(y);
  path_points.points.push_back(pos);

  path_points_.push_back(path_points);

  current_pos_.set_x(x);
  current_pos_.set_y(y);
}

void SvgPathContext::path_line_to_ortho(double coord, bool horizontal,
                                        svgpp::tag::coordinate::absolute) {
  phoenix::common::Vec2d pos;
  PathPoints path_points;

  path_points.point_type = PATH_POINT_TYPE_LINE;

  pos.set_x(current_pos_.x());
  pos.set_y(current_pos_.y());
  path_points.points.push_back(pos);

  if (horizontal) {
    pos.set_x(current_pos_.x() + coord);
    pos.set_y(current_pos_.y());
  } else {
    pos.set_x(current_pos_.x());
    pos.set_y(current_pos_.y() + coord);
  }
  path_points.points.push_back(pos);

  path_points_.push_back(path_points);
}

void SvgPathContext::path_cubic_bezier_to(double x1, double y1, double x2,
                                          double y2, double x, double y,
                                          svgpp::tag::coordinate::absolute) {
  phoenix::common::Vec2d pos;
  PathPoints path_points;

  path_points.point_type = PATH_POINT_TYPE_BEZIER_C;

  pos.set_x(current_pos_.x());
  pos.set_y(current_pos_.y());
  path_points.points.push_back(pos);

  pos.set_x(x1);
  pos.set_y(y1);
  path_points.points.push_back(pos);

  pos.set_x(x2);
  pos.set_y(y2);
  path_points.points.push_back(pos);

  pos.set_x(x);
  pos.set_y(y);
  path_points.points.push_back(pos);

  path_points_.push_back(path_points);

  current_pos_.set_x(x);
  current_pos_.set_y(y);
}

void SvgPathContext::path_cubic_bezier_to(double x2, double y2, double x,
                                          double y,
                                          svgpp::tag::coordinate::absolute) {
  if (!path_points_.empty()) {
    LOG_ERR << "Failed to path_quadratic_bezier_to [path points is empty]";
    return;
  }
  PathPoints previous_path_points = path_points_[path_points_.size() - 1];
  if (previous_path_points.point_type != PATH_POINT_TYPE_BEZIER_C) {
    LOG_ERR << "Failed to path_quadratic_bezier_to [previous path point "
               "type invalid]";
    return;
  }
  if (previous_path_points.points.size() < 4) {
    LOG_ERR << "Failed to path_quadratic_bezier_to [previous path point "
               "size invalid]";
    return;
  }
  phoenix::common::Vec2d pos;
  PathPoints path_points;

  path_points.point_type = PATH_POINT_TYPE_BEZIER_S;

  pos.set_x(current_pos_.x());
  pos.set_y(current_pos_.y());
  path_points.points.push_back(pos);

  double x1 =
      previous_path_points.points[3].x() +
      (previous_path_points.points[3].x() - previous_path_points.points[2].x());
  double y1 =
      previous_path_points.points[3].y() +
      (previous_path_points.points[3].y() - previous_path_points.points[2].y());

  pos.set_x(x1);
  pos.set_y(y1);
  path_points.points.push_back(pos);

  pos.set_x(x2);
  pos.set_y(y2);
  path_points.points.push_back(pos);

  pos.set_x(x);
  pos.set_y(y);
  path_points.points.push_back(pos);

  path_points_.push_back(path_points);

  current_pos_.set_x(x);
  current_pos_.set_y(y);
}

void SvgPathContext::path_quadratic_bezier_to(
    double x1, double y1, double x, double y,
    svgpp::tag::coordinate::absolute) {
  phoenix::common::Vec2d pos;
  PathPoints path_points;

  path_points.point_type = PATH_POINT_TYPE_BEZIER_Q;

  pos.set_x(current_pos_.x());
  pos.set_y(current_pos_.y());
  path_points.points.push_back(pos);

  pos.set_x(x1);
  pos.set_y(y1);
  path_points.points.push_back(pos);

  pos.set_x(x);
  pos.set_y(y);
  path_points.points.push_back(pos);

  path_points_.push_back(path_points);

  current_pos_.set_x(x);
  current_pos_.set_y(y);
}

void SvgPathContext::path_quadratic_bezier_to(
    double x, double y, svgpp::tag::coordinate::absolute) {
  if (!path_points_.empty()) {
    LOG_ERR << "Failed to path_quadratic_bezier_to [path points is empty]";
    return;
  }
  PathPoints previous_path_points = path_points_[path_points_.size() - 1];
  if (previous_path_points.point_type != PATH_POINT_TYPE_BEZIER_S) {
    LOG_ERR << "Failed to path_quadratic_bezier_to [previous path point "
               "type invalid]";
    return;
  }
  if (previous_path_points.points.size() < 3) {
    LOG_ERR << "Failed to path_quadratic_bezier_to [previous path point "
               "size invalid]";
    return;
  }

  phoenix::common::Vec2d pos;
  PathPoints path_points;

  path_points.point_type = PATH_POINT_TYPE_BEZIER_T;

  pos.set_x(current_pos_.x());
  pos.set_y(current_pos_.y());
  path_points.points.push_back(pos);

  double x1 =
      previous_path_points.points[2].x() +
      (previous_path_points.points[2].x() - previous_path_points.points[1].x());
  double y1 =
      previous_path_points.points[2].y() +
      (previous_path_points.points[2].y() - previous_path_points.points[1].y());

  pos.set_x(x1);
  pos.set_y(y1);
  path_points.points.push_back(pos);

  pos.set_x(x);
  pos.set_y(y);
  path_points.points.push_back(pos);

  path_points_.push_back(path_points);

  current_pos_.set_x(x);
  current_pos_.set_y(y);
}

void SvgPathContext::path_elliptical_arc_to(double rx, double ry,
                                            double x_axis_rotation,
                                            bool large_arc_flag,
                                            bool sweep_flag, double x, double y,
                                            svgpp::tag::coordinate::absolute) {
  LOG_INFO(3) << "###########"
              << (svgpp::tag::coordinate::absolute::is_absolute ? "A" : "a")
              << " [" << rx << "," << ry << "," << x_axis_rotation << ","
              << large_arc_flag << "," << sweep_flag << "," << x << "," << y
              << "] ########### ";
}

void SvgPathContext::path_close_subpath() {
  phoenix::common::Vec2d pos;
  PathPoints path_points;

  path_points.point_type = PATH_POINT_TYPE_LINE;

  pos.set_x(current_pos_.x());
  pos.set_y(current_pos_.y());
  path_points.points.push_back(pos);

  pos.set_x(start_pos_.x());
  pos.set_y(start_pos_.y());
  path_points.points.push_back(pos);

  path_points_.push_back(path_points);

  current_pos_.set_x(start_pos_.x());
  current_pos_.set_y(start_pos_.y());
}

bool SvgPathContext::GetNextSubPathPoints(
    std::vector<phoenix::common::Vec2d>& sub_path_points) {
  sub_path_points.clear();
  if (path_points_.empty()) {
    return false;
  }
  framework::SvgPathContext::PathPoints points = path_points_[0];
  if (points.point_type == framework::SvgPathContext::PATH_POINT_TYPE_LINE) {
    sub_path_points.push_back(points.points[0]);
    sub_path_points.push_back(points.points[1]);
  } else if (points.point_type ==
                 framework::SvgPathContext::PATH_POINT_TYPE_BEZIER_Q ||
             points.point_type ==
                 framework::SvgPathContext::PATH_POINT_TYPE_BEZIER_T ||
             points.point_type ==
                 framework::SvgPathContext::PATH_POINT_TYPE_BEZIER_C ||
             points.point_type ==
                 framework::SvgPathContext::PATH_POINT_TYPE_BEZIER_S) {
    common::StaticVector<common::Vec2d, common::BezierPowerN::kBezierMax>
        bezier_points;
    common::StaticVector<common::Vec2d, common::BezierPowerN::kWayPointMax>
        way_points;
    for (Int32_t i = 0; i < points.points.size(); i++) {
      way_points.PushBack(points.points[i]);
    }
    bezier_points = common::BezierPowerN::CalcBezierPoints(way_points, 10);
    for (Int32_t i = 0; i < bezier_points.Size(); i++) {
      sub_path_points.push_back(bezier_points[i]);
    }
  }
  path_points_.erase(path_points_.begin());

  return true;
}

}  // namespace framework
}  // namespace phoenix