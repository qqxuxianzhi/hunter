/******************************************************************************
 ** 图形显示窗口
 ******************************************************************************
 *
 *  图形显示窗口
 *
 *  @file       widget_map.h
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#ifndef PHOENIX_HMI_WIDGET_MAP_H_
#define PHOENIX_HMI_WIDGET_MAP_H_

#include <vector>
#include <QVector3D>

#include "GL/gl.h"
#include "GL/glu.h"
#include "GL/glut.h"
#include "QtGui"
#include "QtOpenGL/QGLWidget"
#include "communication/shared_data.h"
#include "geometry/aabbox2d.h"
#include "geometry/geometry_utils.h"
#include "geometry/line_segment2d.h"
#include "geometry/obbox2d.h"
#include "geometry/quaternion.h"
#include "geometry/sphere2d.h"
#include "geometry/vec2d.h"
#include "math/math_utils.h"
#include "math/matrix.h"

namespace phoenix {
namespace hmi {
#define WORD_COUNT 14 

class WidgetMap : public QGLWidget {
  Q_OBJECT

 public:
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit WidgetMap(QWidget* parent = 0);
  ~WidgetMap();

  void Update();

 protected:
  void wheelEvent(QWheelEvent* event);
  void mousePressEvent(QMouseEvent* event);
  void mouseMoveEvent(QMouseEvent* event);
  void resizeGL(Int32_t w, Int32_t h);
  void initializeGL();
  void paintGL();

 private:
  void LoadIcon();
  void ChangeView();
  void DrawViewSwitchButton();

  void SetDisplayOriginCoor(Float64_t x, Float64_t y, Float64_t z) {
    view_param_.origin_coor.x = x;
    view_param_.origin_coor.y = y;
    view_param_.origin_coor.z = z;
  }

  inline void DrawVertex(Float64_t x, Float64_t y, Float64_t z) {
    glVertex3d(x - view_param_.origin_coor.x, y - view_param_.origin_coor.y,
               z - view_param_.origin_coor.z);
  }

  void DrawCoordinateAxis(Float64_t x, Float64_t y, Float64_t z, Float64_t len);
  void DrawGrid(const phoenix::common::AABBox2d& range, Float64_t interval);
  void DrawArrow(Float64_t x, Float64_t y, Float64_t heading, Float64_t len,
                 Float64_t height = 0.0);
  void DrawAABB_2D(const phoenix::common::AABBox2d& aabb,
                   Float64_t height = 0.0);
  void DrawOBB_2D(const phoenix::common::OBBox2d& obb, Float64_t height = 0.0);
  void DrawBarrier(Float64_t x, Float64_t y, Float64_t heading,
                   Float64_t width = 6.0, Float64_t height = 2.0);
  void DrawEllipse(Float64_t x, Float64_t y, Float64_t a, Float64_t b,
                   Float64_t heading = 0.0, Float64_t z = 0.0,
                   Int32_t count = 20);
  void DrawVehicle();
  void DrawWord();
  void DrawSVG();

  void DrawGNSSInfo();
  void DrawLaserInfo();
  void DrawRTKInfo();
  void DrawImuInfo();

  void DrawChassisInfo();
  void DrawStatusInfo();
  void DrawCar();
  void DrawCarPath();
  void DrawLable(int x,int y,Char_t* str,int size);
  
  void UpdateModuleInfo();

 private:
  struct {
    common::Quaternion<Float64_t> qu_rot;
    common::Matrix<Float64_t, 3, 3> mat_rot;
    common::Vector3d vec_move;

    struct {
      Int32_t x = 0;
      Int32_t y = 0;
    } prev_mouse_pos;

    struct {
      Int32_t x = 0;
      Int32_t y = 0;
    } first_click_mouse_pos;

    struct {
      Float64_t x = 0;
      Float64_t y = 0;
      Float64_t z = 0;
    } origin_coor;

    Float64_t scaling = 1.0;
  } view_param_;



  struct ModuleInfo {
    ad_msg::ModuleStatusList module_status_list;
    ad_msg::Gnss gnss_info_;
    ad_msg::Imu imu_info_;
    ad_msg::Chassis chassis_info;
    ad_msg::LaserResult laser_result;
    ad_msg::RtkResult rtk_result;
    ad_msg::WriteWordResult write_word_result[WORD_COUNT];
    
    ModuleInfo() {}
  } module_info_;

  std::vector<QVector3D> car_path;
  QVector3D car_src_point;

  enum { VIEW_MODE_1, VIEW_MODE_2 };
  Int32_t vew_mode_ = VIEW_MODE_1;
  common::AABBox2d view_switch_button_;
  GLuint texture_id_of_icon_aerial_view_;
  GLuint texture_id_of_icon_3d_view_;

  // 字符串缓存，用来在窗口显示字符信息
  enum { MAX_STRING_BUFF_SIZE = 1024 * 2 };
  Char_t string_buffer_[MAX_STRING_BUFF_SIZE];
};

}  // namespace hmi
}  // namespace phoenix

#endif  // PHOENIX_HMI_WIDGET_MAP_H_
