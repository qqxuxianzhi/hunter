#include "communication/svg_transform.h"

namespace phoenix {
namespace framework {
void SvgTransformContext::transform_matrix(
    const boost::array<double, 6>& matrix) {
  if (points_ == Nullptr_t) {
    return;
  }
  Float32_t a = matrix[0];
  Float32_t b = matrix[1];
  Float32_t c = matrix[2];
  Float32_t d = matrix[3];
  Float32_t e = matrix[4];
  Float32_t f = matrix[5];
  for (Int32_t i = 0; i < points_->size(); i++) {
    (*points_)[i].set_x((a * (*points_)[i].x()) + (c * (*points_)[i].y()) + e);
    (*points_)[i].set_y((b * (*points_)[i].x()) + (d * (*points_)[i].y()) + f);
  }
  // LOG_INFO(3) << "matrix(";
  // for (int i = 0; i < 6; ++i) LOG_INFO(3) << matrix[i] << (i == 5 ? ")" :
  // ",");
}

void SvgTransformContext::transform_translate(double tx, double ty) {
  if (points_ == Nullptr_t) {
    return;
  }
  for (Int32_t i = 0; i < points_->size(); i++) {
    (*points_)[i].set_x((*points_)[i].x() + tx);
    (*points_)[i].set_y((*points_)[i].y() + ty);
  }
  LOG_INFO(3) << "translate(" << tx << "," << ty << ")";
}

void SvgTransformContext::transform_translate(double tx) {
  if (points_ == Nullptr_t) {
    return;
  }
  for (Int32_t i = 0; i < points_->size(); i++) {
    (*points_)[i].set_x((*points_)[i].x() + tx);
  }
  LOG_INFO(3) << "translate(" << tx << ")";
}

void SvgTransformContext::transform_scale(double sx, double sy) {
  if (points_ == Nullptr_t) {
    return;
  }

  for (Int32_t i = 0; i < points_->size(); i++) {
    (*points_)[i].set_x((*points_)[i].x() * sx);
    (*points_)[i].set_y((*points_)[i].y() * sy);
  }
  LOG_INFO(3) << "scale(" << sx << "," << sy << ")";
}

void SvgTransformContext::transform_scale(double scale) {
  if (points_ == Nullptr_t) {
    return;
  }
  for (Int32_t i = 0; i < points_->size(); i++) {
    (*points_)[i].set_x((*points_)[i].x() * scale);
    (*points_)[i].set_y((*points_)[i].y() * scale);
  }
  LOG_INFO(3) << "scale(" << scale << ")";
}

void SvgTransformContext::transform_rotate(double angle) {
  if (points_ == Nullptr_t) {
    return;
  }
  LOG_INFO(3) << "rotate(" << angle << ")";
}

void SvgTransformContext::transform_rotate(double angle, double cx, double cy) {
  if (points_ == Nullptr_t) {
    return;
  }
  LOG_INFO(3) << "rotate(" << angle << "," << cx << "," << cy << ")";
}

void SvgTransformContext::transform_skew_x(double angle) {
  if (points_ == Nullptr_t) {
    return;
  }
  LOG_INFO(3) << "skewX(" << angle << ")";
}

void SvgTransformContext::transform_skew_y(double angle) {
  if (points_ == Nullptr_t) {
    return;
  }
  LOG_INFO(3) << "skewY(" << angle << ")";
}
}  // namespace framework
}  // namespace phoenix