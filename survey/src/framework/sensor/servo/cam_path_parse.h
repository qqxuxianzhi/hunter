/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file          cam_data_parse.h
 * @brief      写字坐标点数据转 换成 电子凸轮数据
 * @details    定义电子凸轮数据
 *
 * @author    xianzhix
 * @date       2022.09.16
 * @version    v1.0
 *
 ******************************************************************************/

#ifndef  PHOENIX_COMMON_CAM_DATA_PARSE_H_
#define PHOENIX_COMMON_CAM_DATA_PARSE_H_

#include <string>
#include <map>
#include "geometry/vec2d.h"

namespace phoenix {
namespace common {
/**
 * @class 
 * @brief 定义电子凸轮数据
 */
class CamPathPrase {
 public:
  /**
   * @brief 构造函数
   */
  CamPathPrase();
  CamPathPrase(const std::vector<Vec2d>& points);

  /**
   * @brief 清除所有数据点
   * @param[in] 
   */
 inline void Clear(){
    path_points.clear();
    stroke_path_points.clear();
 }
  /**
   * @brief 删除数据点
   * @param[in] 
   */
  inline void RemovePointBack() { 
    path_points.pop_back();
  }

  /**
   * @brief 添加数据点
   * @param[in] path_point 数据点
   */
 inline void AddPathPoint(const Vec2d& path_point) { 
    path_points.push_back(path_point);
 }
 inline int32_t GetStrokeSize(){
    return stroke_path_points.size();
 }
 inline std::map<int32_t, std::vector<phoenix::common::Vec2d>> GetStrokePathPoints(){
    return stroke_path_points;
  }
/**
   * @brief 获取笔画的所有绝对x坐标数据
   * @param[in] stroke_index　笔画索引
   */
  std::vector<Float32_t> GetAbsCoordinatesX(const int32_t stroke_index);
  /**
   * @brief 获取笔画的所有绝对y坐标数据
   * @param[in] stroke_index　笔画索引
   */
  std::vector<Float32_t> GetAbsCoordinatesY(const int32_t stroke_index);
  /**

 /**
   * @brief 获取笔画的所有凸轮x坐标数据 { 以起点(GetStrokeStartingPoint函数获得)为原点,建立的相对坐标 }
   * @param[in] stroke_index　笔画索引
   */
  std::vector<Float32_t> GetCamCoordinatesX(const int32_t stroke_index);
  /**
   * @brief 获取笔画的所有凸轮y坐标数据(以起点(GetStrokeStartingPoint函数获得)为原点,建立的相对坐标)
   * @param[in] stroke_index　笔画索引
   */
  std::vector<Float32_t> GetCamCoordinatesY(const int32_t stroke_index);
  /**
   * @brief 获取笔画的起始点坐标
   * @param[in] stroke_index　笔画索引
   */
 phoenix::common::Vec2d GetStrokeStartingPoint(const int32_t stroke_index);
  

    /**
   * @brief 将所有路径点转成电子凸轮数据
   * @param[in] 
   */
  bool PointsToCamPath();

    /**
   * @brief 将长直线点添加点数据，以保证长直线和短直线的速度相差不大
   * @param[in] 
   */
  void AddPointsToLinePath();

    /**
   * @brief 写字笔画的顺序
   * @param[in] 
   */
  void StrokePointSort();


 private:
   /**
   * @brief 检查插入点(src_stroke_points[i] )是否为当前笔画(stroke_points)中的连续点，
   *  是连续点: 选择当前笔画插入位置，并返回true
   * 不是连续点 : 返回false
   * 
   * @param[in] src_stroke_points: 所有的路径点
   *                            i :　路径点的索引
   *                            stroke_points：当前笔画的所有路径点
   */
   bool CheckInsertPoint(const std::vector<phoenix::common::Vec2d>& src_stroke_points,
                      int i,std::vector<phoenix::common::Vec2d>&stroke_points);
 private:
  // 保存字的所有路径点
  std::vector<phoenix::common::Vec2d> path_points;

  // 字的不连贯的笔画索引
  // int32_t stroke_index;
  // 字的不连贯的笔画对应的路径点
  std::map<int32_t, std::vector<phoenix::common::Vec2d>> stroke_path_points;

  //电子凸轮数据x,y坐标
 //   std::vector<Float32_t> coordinates_x;
 //   std::vector<Float32_t> coordinates_y;
};

}  // namespace common
}  // namespace phoenix
#endif