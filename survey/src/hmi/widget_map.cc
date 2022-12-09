/******************************************************************************
 ** 图形显示窗口
 ******************************************************************************
 *
 *  图形显示窗口
 *
 *  @file       widget_map.cc
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#include "widget_map.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

#include "GL/gl.h"
#include "GL/glu.h"
#include "GL/glut.h"
#include "communication/svg_path.h"
#include "communication/svg_transform.h"
#include "curve/cubic_polynomial_curve1d.h"
#include "geometry/bezier_power_n.h"
#include "math/math_utils.h"
#include "utils/com_utils.h"

namespace phoenix {
namespace hmi {

/**
 * @class DrawConcavePolygon
 * @brief 画凹多边形
 *
 * @par Note:
 * @code
 *     通常可以采用一种叫做"分格化"的方法来画复杂的多边形。
 *     非凸多边形最简单的填充方法最简单的应该是GLU 网格化对象GLUtesselator
 *     (GLUT库或者libTess库)。要用分格化的方法画多边形，步骤如下：
 *     1. gluNewTess(); //创建一个新的分格化对象
 *     2. gluTessCallback();
 * //注册回调函数，完成分格化的一些操作，照着写就行了。
 *     3. gluTessProperty(); //设置一些分格化的属性值，如环绕数和环绕规则，
 *                             用来确定多边形的内部和外部
 *     4. gluTessBeginPolygon(); //开始画多边形
 *        gluTessBeginContour(tessobj);//设置多边形的边线 1
 *        gluTessEndContour(tessobj);//结束设置边线1
 *        gluTessBeginContour(tessobj);//，如果有边线2，设置多边形的边线 2
 *        gluTessEndContour(tessobj);//结束设置边线2
 *     5. gluTessEdnPolygon(); //结束画多边形
 *     6. gluDeleteTess(); //删除分格化对象
 *     当然也可以利用回调函数记录分格化的顶点和绘制类型，
 *     然后利用数组的绘制函数进行一次性的绘制，以提高绘制效率。
 * @endcode
 */
enum { MAX_CONCAVE_POLYGON_POINT_NUM = 30720 };
GLdouble s_concave_polygon_points_buff[MAX_CONCAVE_POLYGON_POINT_NUM][3];
class DrawConcavePolygon {
 public:
  DrawConcavePolygon() {
    point_index_ = 0;
    points_buff_ = &s_concave_polygon_points_buff[0];
  }

  ~DrawConcavePolygon() {}

  void Paint() {
    tess_obj_ = gluNewTess();
    gluTessCallback(tess_obj_, GLU_TESS_BEGIN, (_GLUfuncptr)&PolyLine3DBegin);
    gluTessCallback(tess_obj_, GLU_TESS_VERTEX, (_GLUfuncptr)&PolyLine3DVertex);
    gluTessCallback(tess_obj_, GLU_TESS_END, (_GLUfuncptr)&PolyLine3DEnd);
    gluTessCallback(tess_obj_, GLU_TESS_ERROR, (_GLUfuncptr)&HandleErr);

    gluTessBeginPolygon(tess_obj_, NULL);
    gluTessBeginContour(tess_obj_);
  }

  void DrawVertex(GLdouble x, GLdouble y, GLdouble z = 0) {
    points_buff_[point_index_][0] = x;
    points_buff_[point_index_][1] = y;
    points_buff_[point_index_][2] = z;

    if (IsSamePoint(point_index_)) {
      return;
    }

    gluTessVertex(tess_obj_, points_buff_[point_index_],
                  points_buff_[point_index_]);

    point_index_++;
  }

  void EndPaint() {
    gluTessEndContour(tess_obj_);
    gluTessEndPolygon(tess_obj_);
    gluDeleteTess(tess_obj_);
  }

 private:
  bool IsSamePoint(Int32_t index) {
    if (index > 0) {
      GLdouble dx = fabs(points_buff_[index][0] - points_buff_[index - 1][0]);
      GLdouble dy = fabs(points_buff_[index][1] - points_buff_[index - 1][1]);
      if (dx < 1e-5 && dy < 1e-5) {
        return true;
      }
    }

    return false;
  }

  static void PolyLine3DBegin(GLenum type) { glBegin(type); }

  static void PolyLine3DVertex(GLdouble* vertex) {
    const GLdouble* pointer = (GLdouble*)vertex;
    glVertex3dv(pointer);
  }

  static void PolyLine3DEnd() { glEnd(); }

  static void HandleErr(GLenum error_code) {
    const GLubyte* estring;
    // 打印错误类型
    estring = gluErrorString(error_code);
    // std::cout << "Tessellation Error: " << estring << std::endl;
    LOG_ERR << "Tessellation Error: " << (Char_t*)estring;
  }

 private:
  GLUtesselator* tess_obj_;
  Int32_t point_index_;
  GLdouble (*points_buff_)[3];
};

WidgetMap::WidgetMap(QWidget* parent) : QGLWidget(parent) {
  view_param_.qu_rot.SetIdentity();
  view_param_.mat_rot.SetIdentity();
  view_param_.vec_move.SetZeros();
  view_param_.prev_mouse_pos.x = 0;
  view_param_.prev_mouse_pos.y = 0;
  view_param_.first_click_mouse_pos.x = 0;
  view_param_.first_click_mouse_pos.y = 0;
  view_param_.scaling = 1.0;

  SetDisplayOriginCoor(0, 0, 0);

  // Transform
#if 0
  view_param_.v_eye_center_pos += -10 * view_param_.v_eye_center_y;
#endif
  // Rotate
#if 1
  common::Quaternion<Float64_t> qu_rot1;
  qu_rot1.ConstructFromAngleAxis(common::com_deg2rad(90.0),
                                 common::Vector3d(0.0, 0.0, 1.0));
  common::Quaternion<Float64_t> qu_rot2;
  qu_rot2.ConstructFromAngleAxis(common::com_deg2rad(-76.0),
                                 common::Vector3d(1.0, 0.0, 0.0));
  view_param_.qu_rot = qu_rot2 * qu_rot1;
  view_param_.qu_rot.NormalizeInplace();
  view_param_.qu_rot.ConvertToRotationMatrix(&view_param_.mat_rot);
#endif

  view_switch_button_.min().set_x(200);
  view_switch_button_.min().set_y(20);
  view_switch_button_.max().set_x(240);
  view_switch_button_.max().set_y(60);
  vew_mode_ = VIEW_MODE_2;
}

WidgetMap::~WidgetMap() {}

void WidgetMap::LoadIcon() {
  texture_id_of_icon_aerial_view_ = 0;
  texture_id_of_icon_3d_view_ = 0;

  // view icon
  QImage icon_aerial_view;
  bool ret = icon_aerial_view.load(":/images/icon_aerial_view.png");
  if (!ret) {
    return;
  }
  // icon_aerial_view = icon_aerial_view.scaled(icon_aerial_view.width(),
  //                                            icon_aerial_view.height());
  // std::cout << "### The size of aerial view icon is ("
  //           << icon_aerial_view.width()
  //           << ", " << icon_aerial_view.height()
  //           << "). ###" << std::endl;

  QImage icon_3d_view;
  ret = icon_3d_view.load(":/images/icon_3d_view.png");
  if (!ret) {
    return;
  }
  // icon_3d_view = icon_3d_view.scaled(icon_3d_view.width(),
  //                                    icon_3d_view.height());
  // std::cout << "### The size of 3d view icon is ("
  //           << icon_3d_view.width()
  //           << ", " << icon_3d_view.height()
  //           << "). ###" << std::endl;

  // 生成纹理
  GLenum gl_err = 0;
  const Char_t* gl_err_str = Nullptr_t;

  // glGenTextures(GLsizei n, GLuint *textures)函数说明
  // n：用来生成纹理的数量
  // textures：存储纹理索引
  // glGenTextures函数根据纹理参数返回n个纹理索引。纹理名称集合不必是一个连续的整数集合。
  // glGenTextures就是用来产生你要操作的纹理对象的索引的，比如你告诉OpenGL，
  // 我需要5个纹理对象，它会从没有用到的整数里返回5个给你）
  // glBindTexture实际上是改变了OpenGL的这个状态，
  // 它告诉OpenGL下面对纹理的任何操作都是对它所绑定的纹理对象的，
  // 比如glBindTexture(GL_TEXTURE_2D,1)告诉OpenGL下面代码中对2D纹理的
  // 任何设置都是针对索引为1的纹理的。
  // 产生纹理函数假定目标纹理的面积是由glBindTexture函数限制的。
  // 先前调用glGenTextures产生的纹理索引集不会由后面调用的glGenTextures得到，
  // 除非他们首先被glDeleteTextures删除。你不可以在显示列表中包含glGenTextures。
  glGenTextures(1, &texture_id_of_icon_aerial_view_);
  if (texture_id_of_icon_aerial_view_ > 0) {
    glBindTexture(GL_TEXTURE_2D, texture_id_of_icon_aerial_view_);
    /* 设置纹理过滤方式 */
    // 纹理图象被使用到一个小于它的形状上，应该如何处理。
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    // 设置如果纹理图象被使用到一个大于它的形状上时，像素如何填充，
    // 可选择的设置有GL_NEAREST和GL_LINEAR，
    // 前者表示“使用纹理中坐标最接近的一个像素的颜色作为需要绘制的像素颜色”，
    // 后者表示“使用纹理中坐标最接近的若干个颜色，
    // 通过加权平均算法得到需要绘制的像素颜色",后者效果要比前者好
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    // 指当纹理坐标的第一维坐标值大于1.0或小于0.0时，应该如何处理。
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    // 指当纹理坐标的第二维坐标值大于1.0或小于0.0时，应该如何处理。
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    // 指定二维纹理
    glTexImage2D(GL_TEXTURE_2D, 0, 4, icon_aerial_view.width(),
                 icon_aerial_view.height(), 0, GL_BGRA_EXT, GL_UNSIGNED_BYTE,
                 icon_aerial_view.bits());
  }
  gl_err = glGetError();
  gl_err_str = (const Char_t*)gluErrorString(gl_err);
  // std::cout << "### After generating texture for aerial view icon,
  // texture_id="
  //           << texture_id_of_icon_aerial_view_
  //           << ", err_code=" << gl_err
  //           << ", and err_str=\"" << gl_err_str << "\"."
  //           << " ###" << std::endl;

  // 生成纹理
  glGenTextures(1, &texture_id_of_icon_3d_view_);
  if (texture_id_of_icon_3d_view_ > 0) {
    glBindTexture(GL_TEXTURE_2D, texture_id_of_icon_3d_view_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexImage2D(GL_TEXTURE_2D, 0, 4, icon_3d_view.width(),
                 icon_3d_view.height(), 0, GL_BGRA_EXT, GL_UNSIGNED_BYTE,
                 icon_3d_view.bits());
  }

  gl_err = glGetError();
  gl_err_str = (const Char_t*)gluErrorString(gl_err);
  // std::cout << "### After generating texture for 3d view icon, texture_id="
  //           << texture_id_of_icon_3d_view_
  //           << ", err_code=" << gl_err
  //           << ", and err_str=\"" << gl_err_str << "\"."
  //           << " ###" << std::endl;
}

void WidgetMap::ChangeView() {
  switch (vew_mode_) {
    case (VIEW_MODE_1):
      vew_mode_ = VIEW_MODE_2;
      break;

    case (VIEW_MODE_2):
      vew_mode_ = VIEW_MODE_1;
      break;

    default:
      vew_mode_ = VIEW_MODE_1;
      break;
  }

  if (VIEW_MODE_2 == vew_mode_) {
    view_param_.qu_rot.SetIdentity();
    view_param_.mat_rot.SetIdentity();
    view_param_.vec_move.SetZeros();
    view_param_.prev_mouse_pos.x = 0;
    view_param_.prev_mouse_pos.y = 0;
    view_param_.first_click_mouse_pos.x = 0;
    view_param_.first_click_mouse_pos.y = 0;
    view_param_.scaling = 1.0;

    // Rotate
    common::Quaternion<Float64_t> qu_rot1;
    qu_rot1.ConstructFromAngleAxis(common::com_deg2rad(90.0),
                                   common::Vector3d(0.0, 0.0, 1.0));
    common::Quaternion<Float64_t> qu_rot2;
    qu_rot2.ConstructFromAngleAxis(common::com_deg2rad(-76.0),
                                   common::Vector3d(1.0, 0.0, 0.0));
    view_param_.qu_rot = qu_rot2 * qu_rot1;
    view_param_.qu_rot.NormalizeInplace();
    view_param_.qu_rot.ConvertToRotationMatrix(&view_param_.mat_rot);
  } else {
    view_param_.qu_rot.SetIdentity();
    view_param_.mat_rot.SetIdentity();
    view_param_.vec_move.SetZeros();
    view_param_.prev_mouse_pos.x = 0;
    view_param_.prev_mouse_pos.y = 0;
    view_param_.first_click_mouse_pos.x = 0;
    view_param_.first_click_mouse_pos.y = 0;
    view_param_.scaling = 1.0;

    // Rotate
    common::Quaternion<Float64_t> qu_rot1;
    qu_rot1.ConstructFromAngleAxis(common::com_deg2rad(90.0),
                                   common::Vector3d(0.0, 0.0, 1.0));
    view_param_.qu_rot = qu_rot1;
    view_param_.qu_rot.NormalizeInplace();
    view_param_.qu_rot.ConvertToRotationMatrix(&view_param_.mat_rot);
  }
}

void WidgetMap::DrawViewSwitchButton() {
  GLuint texture_id = 0;
  switch (vew_mode_) {
    case (VIEW_MODE_1):
      texture_id = texture_id_of_icon_aerial_view_;
      // std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "1");
      break;

    case (VIEW_MODE_2):
      texture_id = texture_id_of_icon_3d_view_;
      // std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "2");
      break;

    default:
      texture_id = texture_id_of_icon_aerial_view_;
      // std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "1");
      break;
  }

#if 0
  glLineWidth(1.0);
  qglColor(QColor(150, 150, 150));

  glBegin(GL_LINE_LOOP);
  {
    glVertex3d(view_switch_button_.min().x(),
               height() - view_switch_button_.min().y(), 0.0);
    glVertex3d(view_switch_button_.max().x(),
               height() - view_switch_button_.min().y(), 0.0);
    glVertex3d(view_switch_button_.max().x(),
               height() - view_switch_button_.max().y(), 0.0);
    glVertex3d(view_switch_button_.min().x(),
               height() - view_switch_button_.max().y(), 0.0);
  }
  glEnd();
#else
  // 激活二维纹理贴图
  glEnable(GL_TEXTURE_2D);
  // 设置纹理贴图方式(直接覆盖原来颜色或与原来颜色混合)
  // 直接使用纹理覆盖模式
  glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
  // 指定当前使用的纹理(第一次使用和第二次使用含义不同)
  glBindTexture(GL_TEXTURE_2D, texture_id);

  // std::cout << "#### texture_id=" << texture_id << "####" << std::endl;

  //清除背景颜色干扰
  glColor3ub(255, 255, 255);
  // 为每个顶点指定纹理坐标，类似指定法线一样glNormal()
  glBegin(GL_QUADS);
  {
    // glTexCoord2d(0.3, 0.1);
    glTexCoord2d(0.0, 0.0);
    glVertex2d(view_switch_button_.min().x(),
               height() - view_switch_button_.min().y());

    // glTexCoord2d(0.8, 0.1);
    glTexCoord2d(1.0, 0.0);
    glVertex2d(view_switch_button_.max().x(),
               height() - view_switch_button_.min().y());

    // glTexCoord2d(0.8, 0.7);
    glTexCoord2d(1.0, 1.0);
    glVertex2d(view_switch_button_.max().x(),
               height() - view_switch_button_.max().y());

    // glTexCoord2d(0.3, 0.7);
    glTexCoord2d(0.0, 1.0);
    glVertex2d(view_switch_button_.min().x(),
               height() - view_switch_button_.max().y());
  }
  glEnd();
  // 关闭二维纹理贴图
  glDisable(GL_TEXTURE_2D);
#endif

  // renderText(static_cast<int>(0.5*(view_switch_button_.min().x() +
  //                               view_switch_button_.max().x()))-5
  //            , static_cast<int>(0.5*(view_switch_button_.min().y() +
  //                                 view_switch_button_.max().y()))+5
  //            , string_buffer_, QFont("AnyStyle", 15));
}

void WidgetMap::wheelEvent(QWheelEvent* event) {
  Float64_t num_degrees = -event->delta() / 8.0;
  Float64_t num_steps = num_degrees / 15.0;
  view_param_.scaling *= std::pow(1.125f, num_steps);
  if (view_param_.scaling < 0.0001) {
    view_param_.scaling = 0.0001;
  } else if (view_param_.scaling > 10000.0) {
    view_param_.scaling = 10000.0;
  }
  updateGL();
}

void WidgetMap::mousePressEvent(QMouseEvent* event) {
  if ((event->button() == Qt::LeftButton) ||
      (event->button() == Qt::RightButton)) {
    view_param_.prev_mouse_pos.x = event->pos().x();
    view_param_.prev_mouse_pos.y = event->pos().y();

    view_param_.first_click_mouse_pos.x = event->pos().x();
    view_param_.first_click_mouse_pos.y = event->pos().y();
  }

  if ((event->button() == Qt::LeftButton) &&
      (event->flags() | QEvent::MouseButtonPress)) {
    if (view_switch_button_.IsPointIn(event->pos().x(), event->pos().y())) {
      ChangeView();
    }
  }
}

void WidgetMap::mouseMoveEvent(QMouseEvent* event) {
  Float64_t dx =
      static_cast<Float64_t>(event->x() - view_param_.prev_mouse_pos.x) /
      width();
  Float64_t dy =
      -static_cast<Float64_t>(event->y() - view_param_.prev_mouse_pos.y) /
      height();

  if (event->buttons() & Qt::LeftButton) {
    // translation
    Float64_t scale = 40 * view_param_.scaling;
    view_param_.vec_move(0) += scale * dx;
    view_param_.vec_move(1) += scale * dy;
    updateGL();
  } else if (event->buttons() & Qt::RightButton) {
#if 0
    // rotation
    Float64_t theta = common::com_atan2(
          common::com_abs(dy), common::com_abs(dx));
    Float64_t rotation_angle = 2.0 * common::com_sqrt(dx*dx + dy*dy);
    if (dy >= 0 && dx >= 0) {
      // First phase limit
      rotation_angle = -rotation_angle;
    } else if (dy >= 0 && dx < 0) {
      // Second phase limit
      theta = -theta;
    } else if (dy < 0 && dx < 0) {
      // Third phase limit
    } else {
      // Fourth phase limit
      theta = -theta;
      rotation_angle = -rotation_angle;
    }
    theta = common::NormalizeAngle(theta-common::com_deg2rad(90.0));

    common::Vector3d rot_axis;
    rot_axis(0) = common::com_cos(theta);
    rot_axis(1) = common::com_sin(theta);
    rot_axis(2) = 0;
    common::Quaternion<Float64_t> qu_rot1;
    qu_rot1.ConstructFromAngleAxis(rotation_angle, rot_axis);
    view_param_.qu_rot = qu_rot1 * view_param_.qu_rot;
    view_param_.qu_rot.NormalizeInplace();
    view_param_.qu_rot.ConvertToRotationMatrix(&view_param_.mat_rot);

    updateGL();
#endif
  }
  view_param_.prev_mouse_pos.x = event->pos().x();
  view_param_.prev_mouse_pos.y = event->pos().y();
}

void WidgetMap::resizeGL(Int32_t w, Int32_t h) {
  (void)w;
  (void)h;
  updateGL();
}

void WidgetMap::initializeGL() {
  makeCurrent();

  // qglClearColor(QColor(50, 50, 50, 0));
  qglClearColor(QColor(210, 210, 210, 0));
  glShadeModel(GL_FLAT);

  // 开启抗锯齿
  glEnable(GL_POINT_SMOOTH);
  glEnable(GL_LINE_SMOOTH);

  glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
  glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  glEnable(GL_DEPTH_TEST);

  LoadIcon();

#if 0
  // 开启深度Z缓存
  glEnable(GL_DEPTH_TEST);
  // 设置只有正面多边形进行光照计算
  glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);

  //启用光照
  glEnable(GL_LIGHTING);
  //1.0：光源位置；
  GLfloat ambient_light[] = { 1.0f, 1.0f, 1.0f, 1.0f };
  //设置光照模型，提供环境光
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient_light);
  //启用颜色追踪
  glEnable(GL_COLOR_MATERIAL);
  // 设置正面的环境光和散射光属性以便追踪glColor所设置的颜色：
  // 材料的环境和散射反射属性就和glColor函数所设置的当前颜色相同
  glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
  // 镜面光成分：非常亮的白色光源
  GLfloat specular[] = { 1.0f, 1.0f, 1.0f, 1.0f};
  // 为光源添加镜面光成分
  glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
  // 镜面反射的RGBA，1：反射几乎所有的入射光
  GLfloat specref[] = { 1.0f, 1.0f, 1.0f, 1.0f };
  // 为材料添加镜面反射成分
  glMaterialfv(GL_FRONT, GL_SPECULAR, specref);
  //0~128 设置镜面指数，指定了镜面加亮的大小和集中性。0：均匀加亮；值越大，加亮越明显
  glMateriali(GL_FRONT, GL_SHININESS, 128);
  // OpenGL支持至少8种独立光源，
  //GLfloat ambient_light_2[] = { 0.3f, 0.3f, 0.3f, 1.0f };
  // GLfloat diffuse_light[] = { 0.7f, 0.7f, 0.7f, 1.0f };
  //1.0：光源位置；0.0：光源在无限远处，沿向量指定方向照射过来
  GLfloat light_pos[] = { -50.f, 50.0f, 100.0f, 0.0f };
  //设置光照0
  //glLightfv(GL_LIGHT0, GL_DIFFUSE, ambient_light_2);
  //glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
  //指定位置和照射方向：(ChangeSize内)
  glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
  //从光点发散出来的光锥的发散角度
  glLightf(GL_LIGHT0, GL_SPOT_CUTOFF, 50.0f);
  GLfloat spot_dir[] = { 0.0f, 0.0f, -1.0f };
  glLightfv(GL_LIGHT0, GL_SPOT_DIRECTION, spot_dir);
#endif
#if 0
  glEnable(GL_CULL_FACE);

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);

  //启用颜色追踪
  glEnable(GL_COLOR_MATERIAL);
  // 设置正面的环境光和散射光属性以便追踪glColor所设置的颜色：
  // 材料的环境和散射反射属性就和glColor函数所设置的当前颜色相同
  glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

  GLfloat ambient[] = { 0.3f, 0.3f, 0.3f, 1.0f };
  GLfloat diffuse[] = { 0.7f, 0.7f, 0.7f, 1.0f };
  GLfloat specular[] = { 0.5f, 0.5f, 0.5f, 1.0f };

  glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
  glLightfv(GL_LIGHT0, GL_SPECULAR, specular);

  GLfloat position[] = { 2.0f, 2.0f, 2.0f, 1.0f };
  glLightfv(GL_LIGHT0, GL_POSITION, position);

  GLfloat green[] = { 0.0f, 0.2f, 0.1f, 1.0f };
  GLfloat yellow[] = { 0.7f, 0.6f, 0.1f, 1.0f };
  GLfloat white[] = { 1.0f, 1.0f, 1.0f, 1.0f };
  GLfloat gray[] = { 0.2f, 0.2f, 0.2f, 1.0f };
  // 自发光
  //glMaterialfv(GL_FRONT, GL_EMISSION, green);
  // 环境光
  glMaterialfv(GL_FRONT, GL_AMBIENT, white);
  // 漫反射光
  glMaterialfv(GL_FRONT, GL_DIFFUSE , white);
  // 镜面反射光
  glMaterialfv(GL_FRONT, GL_SPECULAR, white);
  glMaterialf(GL_FRONT, GL_SHININESS, 10.0f);
#endif
}

// 绘制窗口
void WidgetMap::paintGL() {
  Int32_t w = width();
  Int32_t h = height();

  // Get module status
  UpdateModuleInfo();

  // SetDisplayOriginCoor(module_info_.filtered_pose.x,
  //     module_info_.filtered_pose.y, 0.0);

  makeCurrent();

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  Float64_t range = 0.2;
  if (w <= h) {
    Float64_t ratio = static_cast<Float64_t>(h) / static_cast<Float64_t>(w);
    Float64_t range_mul_ratio = range * ratio;
    // glOrtho(-range, range, -range_mul_ratio, range_mul_ratio, -100.0, 100.0);
    glFrustum(-range, range, -range_mul_ratio, range_mul_ratio, 1.0, 2000.0);
  } else {
    Float64_t ratio = static_cast<Float64_t>(w) / static_cast<Float64_t>(h);
    Float64_t range_mul_ratio = range * ratio;
    // glOrtho(-range_mul_ratio, range_mul_ratio, -range, range, -100.0, 100.0);
    glFrustum(-range_mul_ratio, range_mul_ratio, -range, range, 1.0, 2000.0);
  }

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  GLdouble mat_modelview[16];
  mat_modelview[0] = view_param_.mat_rot(0, 0);
  mat_modelview[1] = view_param_.mat_rot(1, 0);
  mat_modelview[2] = view_param_.mat_rot(2, 0);
  mat_modelview[3] = 0.0;
  mat_modelview[4] = view_param_.mat_rot(0, 1);
  mat_modelview[5] = view_param_.mat_rot(1, 1);
  mat_modelview[6] = view_param_.mat_rot(2, 1);
  mat_modelview[7] = 0.0;
  mat_modelview[8] = view_param_.mat_rot(0, 2);
  mat_modelview[9] = view_param_.mat_rot(1, 2);
  mat_modelview[10] = view_param_.mat_rot(2, 2);
  mat_modelview[11] = 0.0;
  mat_modelview[12] = view_param_.vec_move(0);
  mat_modelview[13] = view_param_.vec_move(1);
  mat_modelview[14] = -100.0 * view_param_.scaling;
  mat_modelview[15] = 1.0;
  glLoadMatrixd(mat_modelview);

  // Draw vehicle
  // DrawVehicle();

  // 绘制网格
  phoenix::common::AABBox2d grid_range;
  grid_range.min().set_x(-100);
  grid_range.min().set_y(-100);
  grid_range.max().set_x(100);
  grid_range.max().set_y(100);
  DrawGrid(grid_range, 1.0);

  // 绘制坐标
  DrawCoordinateAxis(0.0, 0.0, 0.1, 5.0);
  DrawCarPath();
  DrawCar();

  // 视图变换
  // 定义视口
  glViewport(0, 0, width(), height());
  // 投影变换
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  // 定义正投裁剪区域)
  // gluOrtho2D(/*left*/, /*right*/, /*bottom*/, /*top*/);
  gluOrtho2D(0, w, 0, h);
  // glOrtho(0, w, 0, h, 0, 100);
  // 在执行模型或视图变换之前，必须以GL_MODELVIEW为参数调用glMatrixMode()函数
  glMatrixMode(GL_MODELVIEW);

  glLoadIdentity();
  DrawViewSwitchButton();
  DrawChassisInfo();
  DrawStatusInfo();
  //DrawSVG();
  DrawWord();
  // DrawBarrier(1,2,0.9);
  DrawGNSSInfo();
  DrawLaserInfo();
  DrawRTKInfo();
  DrawImuInfo();
}

void WidgetMap::DrawCoordinateAxis(Float64_t x, Float64_t y, Float64_t z,
                                   Float64_t len) {
  glLineWidth(3.0f);
  glBegin(GL_LINES);
  {
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(x, y, z);
    glVertex3d(x + len, y, z);

    glColor3d(0.0, 1.0, 0.0);
    glVertex3d(x, y, z);
    glVertex3d(x, y + len, z);

    glColor3d(0.0, 0.0, 1.0);
    glVertex3d(x, y, z);
    glVertex3d(x, y, z + len);
  }
  glEnd();
  glColor3d(1.0, 0.0, 0.0);
  renderText(x + len, y, z, "x");
  glColor3d(0.0, 1.0, 0.0);
  renderText(x, y + len, z, "y");
  glColor3d(0.0, 0.0, 1.0);
  renderText(x, y, z + len, "z");
}

void WidgetMap::DrawWord(){
    glLineWidth(3.0);

    qglColor(QColor(0, 0, 255, 255));
    for(int word_Index=0;word_Index<WORD_COUNT;word_Index++){
      ad_msg::WriteWordResult word_result = module_info_.write_word_result[word_Index];
      for(int index=0;index< word_result.stroke_path_points.size();index++){
        Float32_t div = 3.0;
        int step =word_result.stroke_path_points[index].size()>60?3:1;
        glBegin(GL_LINE_STRIP);
        for (Int32_t i = 0; i < word_result.stroke_path_points[index].size();) {
          Float32_t x = word_Index *75 + 80.0-word_result.stroke_path_points[index][i].x()/div;
          Float32_t y = 150.0-word_result.stroke_path_points[index][i].y()/div;
          Float32_t z =0;
          glVertex3d(x,y,z); 
          i = i+step;     
          if(i>= word_result.stroke_path_points[index].size()){
            i = word_result.stroke_path_points[index].size() -1;
            x =word_Index *75+ 80.0-word_result.stroke_path_points[index][i].x()/div;
            y = 150.0-word_result.stroke_path_points[index][i].y()/div;
            glVertex3d(x,y,z); 
            break;
          }
        }
        glEnd();
      }
  }
}

void WidgetMap::DrawSVG() {
  std::string path = "M 0,-12 -8,9 M 0,-12 8,9 M -5,2 H 5";
  std::string transform =
      "translate(0,133)matrix(12.99124,0,0,12.99124,3.078837,167.7658)"
      "translate(9)";

  framework::SvgPathContext path_ctx;
  svgpp::value_parser<svgpp::tag::type::path_data>::parse(
      svgpp::tag::attribute::d(), path_ctx, path,
      svgpp::tag::source::attribute());

  std::vector<phoenix::common::Vec2d> sub_path_points;
  while (path_ctx.GetNextSubPathPoints(sub_path_points)) {
    if (sub_path_points.size() <= 0) {
      continue;
    }

    framework::SvgTransformContext transform_ctx(&sub_path_points);
    svgpp::value_parser<svgpp::tag::type::transform_list>::parse(
        svgpp::tag::attribute::transform(), transform_ctx, transform,
        svgpp::tag::source::attribute());

    {
      const Float32_t y_max = 712;
      for (Int32_t i = 0; i < sub_path_points.size(); i++) {
        sub_path_points[i].set_x(sub_path_points[i].x() + 1);
        sub_path_points[i].set_y(y_max - sub_path_points[i].y());
      }
    }

    glLineWidth(1.0);
    qglColor(QColor(255, 0, 0, 255));
    glBegin(GL_LINES);
    for (Int32_t i = 0; i < sub_path_points.size(); i++) {
      glVertex3d(sub_path_points[i].x(), sub_path_points[i].y(), 0.0);
    }
    glEnd();
  }
}

void WidgetMap::DrawGrid(const phoenix::common::AABBox2d& range,
                         Float64_t interval) {
  glLineWidth(1.0);
  // glColor3d(0.3, 0.3, 0.3);
  qglColor(QColor(160, 160, 160, 100));

  Float64_t tmp = range.min().y();
  glBegin(GL_LINES);
  while (tmp < range.max().y()) {
    glVertex3d(range.min().x(), tmp, 0.0);
    glVertex3d(range.max().x(), tmp, 0.0);
    tmp += interval;
  }
  glVertex3d(range.min().x(), tmp, 0.0);
  glVertex3d(range.max().x(), tmp, 0.0);
  glEnd();

  tmp = range.min().x();
  glBegin(GL_LINES);
  while (tmp < range.max().x()) {
    glVertex3d(tmp, range.min().y(), 0.0);
    glVertex3d(tmp, range.max().y(), 0.0);
    tmp += interval;
  }
  glVertex3d(tmp, range.min().y(), 0.0);
  glVertex3d(tmp, range.max().y(), 0.0);
  glEnd();
}

// 画箭
void WidgetMap::DrawArrow(Float64_t x, Float64_t y, Float64_t heading,
                          Float64_t len, Float64_t height) {
  phoenix::common::Vec2d end_p;
  phoenix::common::Vec2d arrow_p1;
  phoenix::common::Vec2d arrow_p2;
  end_p(0) = x + len * common::com_cos(heading);
  end_p(1) = y + len * common::com_sin(heading);
  arrow_p1(0) =
      end_p(0) + 0.5f * common::com_cos(phoenix::common::NormalizeAngle(
                            heading + COM_PI * 0.8));
  arrow_p1(1) =
      end_p(1) + 0.5f * common::com_sin(phoenix::common::NormalizeAngle(
                            heading + COM_PI * 0.8));
  arrow_p2(0) =
      end_p(0) + 0.5f * common::com_cos(phoenix::common::NormalizeAngle(
                            heading - COM_PI * 0.8));
  arrow_p2(1) =
      end_p(1) + 0.5f * common::com_sin(phoenix::common::NormalizeAngle(
                            heading - COM_PI * 0.8));

  glBegin(GL_LINES);
  {
    DrawVertex(x, y, height);
    DrawVertex(end_p(0), end_p(1), height);

    DrawVertex(end_p(0), end_p(1), height);
    DrawVertex(arrow_p1(0), arrow_p1(1), height);

    DrawVertex(end_p(0), end_p(1), height);
    DrawVertex(arrow_p2(0), arrow_p2(1), height);
  }
  glEnd();
}

void WidgetMap::DrawAABB_2D(const phoenix::common::AABBox2d& aabb,
                            Float64_t height) {
  phoenix::common::Vec2d corner[4];
  corner[0] = aabb.min();
  corner[1].set_x(aabb.max().x());
  corner[1].set_y(aabb.min().y());
  corner[2] = aabb.max();
  corner[3].set_x(aabb.min().x());
  corner[3].set_y(aabb.max().y());

  if (common::com_abs(height) < 0.1) {
    glBegin(GL_LINE_LOOP);
    {
      DrawVertex(corner[0](0), corner[0](1), 0.0);
      DrawVertex(corner[1](0), corner[1](1), 0.0);
      DrawVertex(corner[2](0), corner[2](1), 0.0);
      DrawVertex(corner[3](0), corner[3](1), 0.0);
    }
    glEnd();
  } else {
#if 1
    glBegin(GL_LINE_LOOP);
    {
      DrawVertex(corner[0](0), corner[0](1), 0.0);
      DrawVertex(corner[1](0), corner[1](1), 0.0);
      DrawVertex(corner[2](0), corner[2](1), 0.0);
      DrawVertex(corner[3](0), corner[3](1), 0.0);
    }
    glEnd();

    glBegin(GL_LINE_LOOP);
    {
      DrawVertex(corner[0](0), corner[0](1), height);
      DrawVertex(corner[1](0), corner[1](1), height);
      DrawVertex(corner[2](0), corner[2](1), height);
      DrawVertex(corner[3](0), corner[3](1), height);
    }
    glEnd();

    glBegin(GL_LINES);
    {
      DrawVertex(corner[0](0), corner[0](1), 0.0);
      DrawVertex(corner[0](0), corner[0](1), height);

      DrawVertex(corner[1](0), corner[1](1), 0.0);
      DrawVertex(corner[1](0), corner[1](1), height);

      DrawVertex(corner[2](0), corner[2](1), 0.0);
      DrawVertex(corner[2](0), corner[2](1), height);

      DrawVertex(corner[3](0), corner[3](1), 0.0);
      DrawVertex(corner[3](0), corner[3](1), height);
    }
    glEnd();
#else
    glBegin(GL_QUADS);
    {
      DrawVertex(corner[0](0), corner[0](1), 0.0);
      DrawVertex(corner[1](0), corner[1](1), 0.0);
      DrawVertex(corner[1](0), corner[1](1), height);
      DrawVertex(corner[0](0), corner[0](1), height);

      DrawVertex(corner[3](0), corner[3](1), height);
      DrawVertex(corner[2](0), corner[2](1), height);
      DrawVertex(corner[2](0), corner[2](1), 0.0);
      DrawVertex(corner[3](0), corner[3](1), 0.0);

      DrawVertex(corner[0](0), corner[0](1), height);
      DrawVertex(corner[1](0), corner[1](1), height);
      DrawVertex(corner[2](0), corner[2](1), height);
      DrawVertex(corner[3](0), corner[3](1), height);

      DrawVertex(corner[3](0), corner[3](1), 0.0);
      DrawVertex(corner[2](0), corner[2](1), 0.0);
      DrawVertex(corner[1](0), corner[1](1), 0.0);
      DrawVertex(corner[0](0), corner[0](1), 0.0);

      DrawVertex(corner[1](0), corner[1](1), height);
      DrawVertex(corner[1](0), corner[1](1), 0.0);
      DrawVertex(corner[2](0), corner[2](1), 0.0);
      DrawVertex(corner[2](0), corner[2](1), height);

      DrawVertex(corner[0](0), corner[0](1), 0.0);
      DrawVertex(corner[0](0), corner[0](1), height);
      DrawVertex(corner[3](0), corner[3](1), height);
      DrawVertex(corner[3](0), corner[3](1), 0.0);
    }
    glEnd();
#endif
  }
}

void WidgetMap::DrawOBB_2D(const phoenix::common::OBBox2d& obb,
                           Float64_t height) {
  phoenix::common::Vec2d corner[4];
  corner[0] = obb.center() + obb.extents().x() * obb.unit_direction_x() +
              obb.extents().y() * obb.unit_direction_y();
  corner[1] = obb.center() - obb.extents().x() * obb.unit_direction_x() +
              obb.extents().y() * obb.unit_direction_y();
  corner[2] = obb.center() - obb.extents().x() * obb.unit_direction_x() -
              obb.extents().y() * obb.unit_direction_y();
  corner[3] = obb.center() + obb.extents().x() * obb.unit_direction_x() -
              obb.extents().y() * obb.unit_direction_y();

  if (common::com_abs(height) < 0.1) {
    glBegin(GL_LINE_LOOP);
    {
      DrawVertex(corner[0](0), corner[0](1), 0.0);
      DrawVertex(corner[1](0), corner[1](1), 0.0);
      DrawVertex(corner[2](0), corner[2](1), 0.0);
      DrawVertex(corner[3](0), corner[3](1), 0.0);
    }
    glEnd();
  } else {
#if 1
    glBegin(GL_LINE_LOOP);
    {
      DrawVertex(corner[0](0), corner[0](1), 0.0);
      DrawVertex(corner[1](0), corner[1](1), 0.0);
      DrawVertex(corner[2](0), corner[2](1), 0.0);
      DrawVertex(corner[3](0), corner[3](1), 0.0);
    }
    glEnd();

    glBegin(GL_LINE_LOOP);
    {
      DrawVertex(corner[0](0), corner[0](1), height);
      DrawVertex(corner[1](0), corner[1](1), height);
      DrawVertex(corner[2](0), corner[2](1), height);
      DrawVertex(corner[3](0), corner[3](1), height);
    }
    glEnd();

    glBegin(GL_LINES);
    {
      DrawVertex(corner[0](0), corner[0](1), 0.0);
      DrawVertex(corner[0](0), corner[0](1), height);

      DrawVertex(corner[1](0), corner[1](1), 0.0);
      DrawVertex(corner[1](0), corner[1](1), height);

      DrawVertex(corner[2](0), corner[2](1), 0.0);
      DrawVertex(corner[2](0), corner[2](1), height);

      DrawVertex(corner[3](0), corner[3](1), 0.0);
      DrawVertex(corner[3](0), corner[3](1), height);
    }
    glEnd();
#else
    glBegin(GL_QUADS);
    {
      DrawVertex(corner[0](0), corner[0](1), 0.0);
      DrawVertex(corner[1](0), corner[1](1), 0.0);
      DrawVertex(corner[1](0), corner[1](1), height);
      DrawVertex(corner[0](0), corner[0](1), height);

      DrawVertex(corner[3](0), corner[3](1), height);
      DrawVertex(corner[2](0), corner[2](1), height);
      DrawVertex(corner[2](0), corner[2](1), 0.0);
      DrawVertex(corner[3](0), corner[3](1), 0.0);

      DrawVertex(corner[0](0), corner[0](1), height);
      DrawVertex(corner[1](0), corner[1](1), height);
      DrawVertex(corner[2](0), corner[2](1), height);
      DrawVertex(corner[3](0), corner[3](1), height);

      DrawVertex(corner[3](0), corner[3](1), 0.0);
      DrawVertex(corner[2](0), corner[2](1), 0.0);
      DrawVertex(corner[1](0), corner[1](1), 0.0);
      DrawVertex(corner[0](0), corner[0](1), 0.0);

      DrawVertex(corner[1](0), corner[1](1), height);
      DrawVertex(corner[1](0), corner[1](1), 0.0);
      DrawVertex(corner[2](0), corner[2](1), 0.0);
      DrawVertex(corner[2](0), corner[2](1), height);

      DrawVertex(corner[0](0), corner[0](1), 0.0);
      DrawVertex(corner[0](0), corner[0](1), height);
      DrawVertex(corner[3](0), corner[3](1), height);
      DrawVertex(corner[3](0), corner[3](1), 0.0);
    }
    glEnd();
#endif
  }
}

void WidgetMap::Update() { updateGL(); }

void WidgetMap::DrawBarrier(Float64_t x, Float64_t y, Float64_t heading,
                            Float64_t width, Float64_t height) {
  Float64_t cos_heading = common::com_cos(heading);
  Float64_t sin_heading = common::com_sin(heading);

  phoenix::common::Vec2d certer_pt(x, y);
  phoenix::common::Vec2d direction_vec(sin_heading, -cos_heading);
  phoenix::common::Vec2d left_pt = certer_pt + 0.5 * width * direction_vec;
  phoenix::common::Vec2d right_pt = certer_pt - 0.5 * width * direction_vec;

  glLineWidth(2.0);
  glBegin(GL_LINES);
  {
    DrawVertex(left_pt.x(), left_pt.y(), 0.0);
    DrawVertex(left_pt.x(), left_pt.y(), height);
    DrawVertex(right_pt.x(), right_pt.y(), 0.0);
    DrawVertex(right_pt.x(), right_pt.y(), height);

    DrawVertex(left_pt.x(), left_pt.y(), 0.0);
    DrawVertex(right_pt.x(), right_pt.y(), 0.0);

    DrawVertex(left_pt.x(), left_pt.y(), height);
    DrawVertex(right_pt.x(), right_pt.y(), height);

    DrawVertex(left_pt.x(), left_pt.y(), 0.0);
    DrawVertex(right_pt.x(), right_pt.y(), height);

    DrawVertex(left_pt.x(), left_pt.y(), height);
    DrawVertex(right_pt.x(), right_pt.y(), 0.0);
  }
  glEnd();
}

void WidgetMap::DrawEllipse(Float64_t x, Float64_t y, Float64_t a, Float64_t b,
                            Float64_t heading, Float64_t z, Int32_t count) {
  Float64_t delta_theta = 2.0 * COM_PI / count;

  Float64_t cos_heading = common::com_cos(heading);
  Float64_t sin_heading = common::com_sin(heading);

  glBegin(GL_LINE_LOOP);
  for (Int32_t i = 0; i < count; ++i) {
    Float64_t theta = i * delta_theta;
    Float64_t px = a * common::com_cos(theta);
    Float64_t py = b * common::com_sin(theta);

    Float64_t ppx = x + px * cos_heading - py * sin_heading;
    Float64_t ppy = y + px * sin_heading + py * cos_heading;

    DrawVertex(ppx, ppy, z);
  }
  glEnd();
}

void WidgetMap::DrawVehicle() {
#if 0
  Float64_t heading = module_info_.filtered_pose.heading;
  phoenix::common::Vec2d pos(
        module_info_.filtered_pose.x, module_info_.filtered_pose.y);
  phoenix::common::OBB_2D obb;

  const Float64_t half_veh_width =
      module_info_.motion_plan_config.vehicle_width * 0.5;
  const Float64_t half_veh_length =
      module_info_.motion_plan_config.vehicle_length * 0.5;
#else
  Float64_t heading = 0;
  phoenix::common::Vec2d pos(0, 0);
  phoenix::common::OBBox2d obb;

  const Float64_t half_veh_width = 1.25;
  const Float64_t half_veh_length = 3.0;
  const Float64_t veh_height = 2.0;
#endif

  obb.set_unit_direction(phoenix::common::CosLookUp(heading),
                         phoenix::common::SinLookUp(heading));
  obb.set_center(pos + 0.5 * obb.unit_direction_x());
  obb.set_extents(half_veh_length, half_veh_width);

  glLineWidth(1.0);

  // DrawOBB_2D(obb);
  phoenix::common::Vec2d corner[4];
  corner[0] = obb.center() + obb.extents().x() * obb.unit_direction_x() +
              obb.extents().y() * obb.unit_direction_y();
  corner[1] = obb.center() - obb.extents().x() * obb.unit_direction_x() +
              obb.extents().y() * obb.unit_direction_y();
  corner[2] = obb.center() - obb.extents().x() * obb.unit_direction_x() -
              obb.extents().y() * obb.unit_direction_y();
  corner[3] = obb.center() + obb.extents().x() * obb.unit_direction_x() -
              obb.extents().y() * obb.unit_direction_y();

  glColor3d(1.0, 1.0, 0.3);
  glBegin(GL_LINE_LOOP);
  {
    DrawVertex(corner[0](0), corner[0](1), 0.0);
    DrawVertex(corner[1](0), corner[1](1), 0.0);
    DrawVertex(corner[2](0), corner[2](1), 0.0);
    DrawVertex(corner[3](0), corner[3](1), 0.0);
  }
  glEnd();

  glColor3d(1.0, 0.5, 0.3);
  glBegin(GL_LINE_LOOP);
  {
    DrawVertex(corner[0](0), corner[0](1), veh_height);
    DrawVertex(corner[1](0), corner[1](1), veh_height);
    DrawVertex(corner[2](0), corner[2](1), veh_height);
    DrawVertex(corner[3](0), corner[3](1), veh_height);
  }
  glEnd();

  glColor3d(0.0, 0.5, 1.0);
  glBegin(GL_LINES);
  {
    DrawVertex(corner[0](0), corner[0](1), 0.0);
    DrawVertex(corner[0](0), corner[0](1), veh_height);

    DrawVertex(corner[1](0), corner[1](1), 0.0);
    DrawVertex(corner[1](0), corner[1](1), veh_height);

    DrawVertex(corner[2](0), corner[2](1), 0.0);
    DrawVertex(corner[2](0), corner[2](1), veh_height);

    DrawVertex(corner[3](0), corner[3](1), 0.0);
    DrawVertex(corner[3](0), corner[3](1), veh_height);
  }
  glEnd();

  glColor3d(1.0, 1.0, 0.3);
  DrawEllipse(pos.x(), pos.y(), 0.5, 0.5, heading, 0.0, 50);
}

void WidgetMap::UpdateModuleInfo() {
  phoenix::framework::SharedData* shared_data =
      phoenix::framework::SharedData::instance();
  static Int64_t word_Index =0;
  // shared_data->GetChassis(&module_info_.chassis_info);
  shared_data->GetGnssData(&module_info_.gnss_info_);
  shared_data->GetImuData(&module_info_.imu_info_);
  shared_data->GetLaserResult(&module_info_.laser_result);
  shared_data->GetRtkResult(&module_info_.rtk_result);
  shared_data->GetModuleStatusList(&module_info_.module_status_list);

  ad_msg::WriteWordResult data;
  shared_data->GetWriteWordResult(&data);
  ad_msg::WriteWordResult temp;
  shared_data->SetWriteWordResult(temp);

  if(data.is_complete){
    word_Index++;
    std::cout << "Update:" << word_Index <<std::endl;
    if(word_Index >= WORD_COUNT){
        for(int i=0;i<WORD_COUNT-1;i++){
        module_info_.write_word_result[i] = module_info_.write_word_result[i+1];
      }
      module_info_.write_word_result[WORD_COUNT-1] = temp;
    }
  }else if(!data.isEmpty()){
     if(word_Index >= WORD_COUNT){
      module_info_.write_word_result[WORD_COUNT -1] = data;
     }
     else{
      module_info_.write_word_result[word_Index] = data;
     }
  }

  QVector3D car_pos(module_info_.gnss_info_.x_utm,
                    module_info_.gnss_info_.y_utm,
                    module_info_.gnss_info_.z_utm);
  if (car_path.size() > 0) {
    if (car_path[car_path.size() - 1] + car_src_point != car_pos &&
        car_pos != QVector3D(0, 0, 0)) {
      car_path.push_back(car_pos - car_src_point);
    }
  } else {
    if (car_pos != QVector3D(0.0, 0.0, 0.0)) {
      car_src_point = car_pos;
      car_path.push_back(car_pos - car_src_point);
    }
  }
}

void WidgetMap::DrawChassisInfo() {
  qglColor(QColor(0, 100, 255));
  com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE - 1, "RTK_UTM_X:  %f",
               module_info_.rtk_result.utm_result.x);
  renderText(10, 20, string_buffer_, QFont("AnyStyle", 10));
  com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE - 1, "RTK_UTM_Y:  %f",
               module_info_.rtk_result.utm_result.y);
  renderText(10, 40, string_buffer_, QFont("AnyStyle", 10));
  com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE - 1, "RTK_UTM_Z:  %f",
               module_info_.rtk_result.utm_result.z);
  renderText(10, 60, string_buffer_, QFont("AnyStyle", 10));
  com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE - 1, "Laser Hight: %f",
               module_info_.laser_result.height);
  renderText(10, 80, string_buffer_, QFont("AnyStyle", 10));
}

void WidgetMap::DrawGNSSInfo() {
  int startX = 12;
  int startY = 110;
  char str_buff[256] = {0};
  QColor background_normal(82, 235, 233, 200);
  std::snprintf(str_buff, sizeof(str_buff) - 1, "纬度,经度,高度(%f,%f,%f)",
                module_info_.gnss_info_.latitude,
                module_info_.gnss_info_.longitude,
                module_info_.gnss_info_.altitude);
  qglColor(background_normal);
  DrawLable(startX, startY, str_buff, qstrlen(str_buff) - 9);

  std::snprintf(str_buff, sizeof(str_buff) - 1, "utm坐标(x,y,z):   (%f,%f,%f) ",
                module_info_.gnss_info_.x_utm, module_info_.gnss_info_.y_utm,
                module_info_.gnss_info_.z_utm);
  qglColor(background_normal);
  DrawLable(startX, startY + 25, str_buff, qstrlen(str_buff) - 5);

  std::snprintf(str_buff, sizeof(str_buff) - 1,
                "odom坐标(x,y,z):   (%f,%f,%f) ",
                module_info_.gnss_info_.x_odom, module_info_.gnss_info_.y_odom,
                module_info_.gnss_info_.z_odom);
  qglColor(background_normal);
  DrawLable(startX, startY + 50, str_buff, qstrlen(str_buff) - 5);

  std::snprintf(str_buff, sizeof(str_buff) - 1,
                "偏航角:  %f, utm 偏航角 : %f,  odom 偏航角: %f",
                module_info_.gnss_info_.heading_gnss,
                module_info_.gnss_info_.heading_utm,
                module_info_.gnss_info_.heading_odom);
  qglColor(background_normal);
  DrawLable(startX, startY + 75, str_buff, qstrlen(str_buff) - 15);

  std::snprintf(str_buff, sizeof(str_buff) - 1, "俯仰角,横滚角:   (%f,%f) ",
                module_info_.gnss_info_.pitch, module_info_.gnss_info_.roll);
  qglColor(background_normal);
  DrawLable(startX, startY + 100, str_buff, qstrlen(str_buff) - 8);

  std::snprintf(str_buff, sizeof(str_buff) - 1,
                "东向速度,北向速度,天向速度: (%f,%f,%f) ",
                module_info_.gnss_info_.v_e, module_info_.gnss_info_.v_n,
                module_info_.gnss_info_.v_u);
  qglColor(background_normal);
  DrawLable(startX, startY + 125, str_buff, qstrlen(str_buff) - 15);

  std::snprintf(
      str_buff, sizeof(str_buff) - 1, "utm x轴,y轴,z轴方向速度: (%f,%f,%f) ",
      module_info_.gnss_info_.v_x_utm, module_info_.gnss_info_.v_y_utm,
      module_info_.gnss_info_.v_z_utm);
  qglColor(background_normal);
  DrawLable(startX, startY + 150, str_buff, qstrlen(str_buff) - 12);

  std::snprintf(
      str_buff, sizeof(str_buff) - 1, "odom x轴,y轴,z轴方向速度: (%f,%f,%f) ",
      module_info_.gnss_info_.v_x_odom, module_info_.gnss_info_.v_y_odom,
      module_info_.gnss_info_.v_z_odom);
  qglColor(background_normal);
  DrawLable(startX, startY + 175, str_buff, qstrlen(str_buff) - 12);
}

void WidgetMap::DrawStatusInfo() {
  QColor background_normal(82, 235, 233, 200);
  QColor background_warn(255, 255, 0, 200);
  QColor background_err(255, 0, 0, 200);
  QColor background_timeout(255, 128, 128, 200);
  char str_buff[256] = {0};
  int startX = width() - 320;
  int startY = 20;
  for (Int32_t i = 0; i < module_info_.module_status_list.module_status_num;
       ++i) {
    const ad_msg::ModuleStatus& module =
        module_info_.module_status_list.module_status_list[i];
    switch (module.sub_module_id) {
      case (framework::INTERNAL_MODULE_ID_MSG_RECV_GNSS): {
        if (module.timeout) {
          std::snprintf(str_buff, sizeof(str_buff) - 1, "GNSS: Timeout,[%dms]",
                        module.param[0]);
          qglColor(background_timeout);
        } else if (ad_msg::MODULE_STATUS_OK != module.status) {
          std::snprintf(str_buff, sizeof(str_buff) - 1, "GNSS: Err(%d),[%dms]",
                        module.status, module.param[0]);
          qglColor(background_err);
        } else if (ad_msg::Gnss::STATUS_GOOD !=
                   module_info_.gnss_info_.gnss_status) {
          std::snprintf(str_buff, sizeof(str_buff) - 1,
                        "GNSS: Bad Signal(%d),[%dms]",
                        module_info_.gnss_info_.gnss_status, module.param[0]);
          qglColor(background_warn);
        } else {
          std::snprintf(str_buff, sizeof(str_buff) - 1, "GNSS: OK,[%dms]",
                        module.param[0]);
          qglColor(background_normal);
        }
      } break;

      case (framework::INTERNAL_MODULE_ID_MSG_RECV_IMU): {
        if (module.timeout) {
          std::snprintf(str_buff, sizeof(str_buff) - 1, "IMU: Timeout,[%dms]",
                        module.param[0]);
          qglColor(background_timeout);
        } else if (ad_msg::MODULE_STATUS_OK != module.status) {
          std::snprintf(str_buff, sizeof(str_buff) - 1, "IMU: Err(%d),[%dms]",
                        module.status, module.param[0]);
          qglColor(background_err);
        } else {
          std::snprintf(str_buff, sizeof(str_buff) - 1, "IMU: OK,[%dms]",
                        module.param[0]);
          qglColor(background_normal);
        }
      } break;

      case (framework::INTERNAL_MODULE_ID_MSG_RECV_LASER): {
        if (module.timeout) {
          std::snprintf(str_buff, sizeof(str_buff) - 1, "Laser: Timeout,[%dms]",
                        module.param[0]);
          qglColor(background_timeout);
        } else if (ad_msg::MODULE_STATUS_OK != module.status) {
          std::snprintf(str_buff, sizeof(str_buff) - 1, "Laser: Err(%d),[%dms]",
                        module.status, module.param[0]);
          qglColor(background_err);
        } else {
          std::snprintf(str_buff, sizeof(str_buff) - 1, "Laser: OK,[%dms]",
                        module.param[0]);
          qglColor(background_normal);
        }
      } break;

      case (framework::INTERNAL_MODULE_ID_MSG_RECV_RTK): {
        if (module.timeout) {
          std::snprintf(str_buff, sizeof(str_buff) - 1, "RTK: Timeout,[%dms]",
                        module.param[0]);
          qglColor(background_timeout);
        } else if (ad_msg::MODULE_STATUS_OK != module.status) {
          std::snprintf(str_buff, sizeof(str_buff) - 1, "RTK: Err(%d),[%dms]",
                        module.status, module.param[0]);
          qglColor(background_err);
        } else {
          std::snprintf(str_buff, sizeof(str_buff) - 1, "RTK: OK,[%dms]",
                        module.param[0]);
          qglColor(background_normal);
        }
      } break;
      case (framework::INTERNAL_MODULE_ID_MSG_RECV_CHASSIS): {
        if (module.timeout) {
          std::snprintf(str_buff, sizeof(str_buff) - 1,
                        "CHASSIS: Timeout,[%dms]", module.param[0]);
          qglColor(background_timeout);
        } else if (ad_msg::MODULE_STATUS_OK != module.status) {
          std::snprintf(str_buff, sizeof(str_buff) - 1,
                        "CHASSIS: Err(%d),[%dms]", module.status,
                        module.param[0]);
          qglColor(background_err);
        } else {
          std::snprintf(str_buff, sizeof(str_buff) - 1, "CHASSIS: OK,[%dms]",
                        module.param[0]);
          qglColor(background_normal);
        }
      } break;
      default:
        break;
    }
    DrawLable(startX, startY + i * 23, str_buff, qstrlen(str_buff));
  }
}

void WidgetMap::DrawLaserInfo() {
  char str_buff[256] = {0};
  int startX = width() - 320;
  int startY = 140;
  QColor background_normal(82, 235, 233, 200);
  std::snprintf(str_buff, sizeof(str_buff) - 1, "激光高层(%f)",
                module_info_.laser_result.height);
  qglColor(background_normal);
  DrawLable(startX, startY, str_buff, qstrlen(str_buff) - 4);
}

void WidgetMap::DrawRTKInfo() {
  char str_buff[256] = {0};
  int startX = width() - 320;
  int startY = 170;
  QColor background_normal(82, 235, 233, 200);
  std::snprintf(str_buff, sizeof(str_buff) - 1, "RTK  ENU(x,y,z) : (%f,%f,%f)",
                module_info_.rtk_result.utm_result.x,
                module_info_.rtk_result.utm_result.y,
                module_info_.rtk_result.utm_result.z);
  qglColor(background_normal);
  DrawLable(startX, startY, str_buff, qstrlen(str_buff));
  std::snprintf(str_buff, sizeof(str_buff) - 1,
                "RTK LLH(纬度, 经度, 高度):(%f,%f,%f)",
                module_info_.rtk_result.gnss_result.lon,
                module_info_.rtk_result.gnss_result.lat,
                module_info_.rtk_result.gnss_result.height);
  qglColor(background_normal);
  DrawLable(startX, startY + 25, str_buff, qstrlen(str_buff) - 8);
}
void WidgetMap::DrawImuInfo() {
  QColor background_normal(82, 235, 233, 200);
  char str_buff[256] = {0};
  int startX = width() - 320;
  int startY = 230;
  std::snprintf(str_buff, sizeof(str_buff) - 1,
                "航向角,俯仰角, 横滚角的角速度:(%f,%f,%f)",
                module_info_.imu_info_.yaw_rate,
                module_info_.imu_info_.pitch_rate,
                module_info_.imu_info_.roll_rate);
  qglColor(background_normal);
  DrawLable(startX, startY, str_buff, qstrlen(str_buff) - 15);
  std::snprintf(str_buff, sizeof(str_buff) - 1, "x轴,y轴,z轴加速度: (%f,%f,%f)",
                module_info_.imu_info_.accel_x, module_info_.imu_info_.accel_y,
                module_info_.imu_info_.accel_z);
  qglColor(background_normal);
  DrawLable(startX, startY + 25, str_buff, qstrlen(str_buff) - 9);
  std::snprintf(str_buff, sizeof(str_buff) - 1, "x轴,y轴,z轴的角度: (%f,%f,%f)",
                module_info_.imu_info_.yaw, module_info_.imu_info_.pitch,
                module_info_.imu_info_.roll);
  qglColor(background_normal);
  DrawLable(startX, startY + 50, str_buff, qstrlen(str_buff) - 9);
}

void WidgetMap::DrawCar() {
  int n = 360;
  float R = 0.6f;
  float z_rot = 0.1;
  if (car_path.size() > 0) {
    z_rot = car_path[car_path.size() - 1][2] + 0.1;
    glTranslatef(car_path[car_path.size() - 1][0],
                 car_path[car_path.size() - 1][1], 0);
  }
  qglColor(QColor(0, 160, 0, 200));
  glBegin(GL_POLYGON);
  glVertex3d(-1.5f, 2.0f, z_rot);
  glVertex3d(-1.5f, -2.0f, z_rot);
  glVertex3d(1.5f, -2.0f, z_rot);
  glVertex3d(1.5f, 2.0f, z_rot);
  glEnd();

  glBegin(GL_POLYGON);
  for (int i = 0; i < n; i++) {
    glVertex3d(R * cos(2 * COM_PI / n * i) - 1.8,
               R * sin(2 * COM_PI / n * i) - 0.9, z_rot);
  }
  glEnd();

  glBegin(GL_POLYGON);
  for (int i = 0; i < n; i++) {
    glVertex3d(R * cos(2 * COM_PI / n * i) - 1.8,
               R * sin(2 * COM_PI / n * i) + 0.9, z_rot);
  }
  glEnd();
  glBegin(GL_POLYGON);
  glVertex3d(1.5f, -1.2f, z_rot);
  glVertex3d(1.5f, 0.2f, z_rot);
  glVertex3d(3.0f, 0.2f, z_rot);
  glVertex3d(3.0f, -1.2f, z_rot);
  glEnd();
}
void WidgetMap::DrawCarPath() {
  qglColor(QColor(255, 0, 0, 255));
  glLineWidth(10.0);
  glBegin(GL_LINE_STRIP);
  for (int i = 0; i < car_path.size(); i++) {
    glVertex3d(car_path[i][0], car_path[i][1], car_path[i][2] + 0.1);
  }
  glEnd();
}
void WidgetMap::DrawLable(int x, int y, Char_t* str, int size) {
  Float64_t lable_width = 20;
  Float64_t lable_length = size * 6.1;
  Float64_t lable_x_left = x - 3.0;

  Float64_t lable_y_up = height() - y + 15;
  Int32_t text_x = x;
  Int32_t text_y = y;

  glBegin(GL_QUADS);
  {
    glVertex3d(lable_x_left, lable_y_up, 0.0);
    glVertex3d(lable_x_left, lable_y_up - lable_width, 0.0);
    glVertex3d(lable_x_left + lable_length, lable_y_up - lable_width, 0.0);
    glVertex3d(lable_x_left + lable_length, lable_y_up, 0.0);
  }
  glEnd();
  qglColor(QColor(0, 0, 0, 200));
  renderText(text_x, text_y, str, QFont("AnyStyle", 8));
}

}  // namespace hmi
}  // namespace phoenix
