#include <algorithm>
#include "cam_path_parse.h"
#include "utils/log.h"


namespace phoenix {
namespace common {

  CamPathPrase::CamPathPrase(){
  }
  CamPathPrase::CamPathPrase(const std::vector<Vec2d>& points){
    path_points = points;
  }

/**
   * @brief 获取笔画的所有绝对x坐标数据
   * @param[in] stroke_index　笔画索引
   */
  std::vector<Float32_t> CamPathPrase::GetAbsCoordinatesX(const int32_t stroke_index){
      COM_CHECK(stroke_path_points.find(stroke_index)!= stroke_path_points.end());
     std::vector<phoenix::common::Vec2d> points = stroke_path_points[stroke_index];
     std::vector<Float32_t> coordinates_x;
     for(int32_t i=0;i<points.size();++i){
        coordinates_x.push_back(points[i].x() );
     }
     return coordinates_x;
  }
  /**
   * @brief 获取笔画的所有绝对y坐标数据
   * @param[in] stroke_index　笔画索引
   */
  std::vector<Float32_t> CamPathPrase::GetAbsCoordinatesY(const int32_t stroke_index){
      COM_CHECK(stroke_path_points.find(stroke_index)!= stroke_path_points.end());
     std::vector<phoenix::common::Vec2d> points = stroke_path_points[stroke_index];
     std::vector<Float32_t> coordinates_y;
     for(int32_t i=0;i<points.size();++i){
         coordinates_y.push_back(points[i].y());
     }
     return coordinates_y;
  }


 /**
   * @brief 获取笔画的所有凸轮x坐标数据
   * @param[in] stroke_index　笔画索引
   */
  std::vector<Float32_t>  CamPathPrase::GetCamCoordinatesX(const int32_t stroke_index){
      COM_CHECK(stroke_path_points.find(stroke_index)!= stroke_path_points.end());
     std::vector<phoenix::common::Vec2d> points = stroke_path_points[stroke_index];
     std::vector<Float32_t> coordinates_x;
     phoenix::common::Vec2d starting_point = GetStrokeStartingPoint(stroke_index);
     size_t pointSize = points.size();
     for(int32_t i=0;i<pointSize;++i){
        coordinates_x.push_back(points[i].x() - starting_point.x());
     }
     return coordinates_x;
  }
  /**
   * @brief 获取笔画的所有凸轮y坐标数据
   * @param[in] stroke_index　笔画索引
   */
  std::vector<Float32_t>  CamPathPrase::GetCamCoordinatesY(const  int32_t stroke_index){
     COM_CHECK(stroke_path_points.find(stroke_index)!= stroke_path_points.end());
     std::vector<phoenix::common::Vec2d> points = stroke_path_points[stroke_index];
     phoenix::common::Vec2d starting_point = GetStrokeStartingPoint(stroke_index);
     std::vector<Float32_t> coordinates_y;
     size_t pointSize = points.size();
     for(int32_t i=0;i<pointSize;++i){
        coordinates_y.push_back(points[i].y()- starting_point.y());
     }
     return coordinates_y;
  }
  /**
   * @brief 获取笔画的起始点坐标
   * @param[in] stroke_index　笔画索引
   */
 phoenix::common::Vec2d CamPathPrase::GetStrokeStartingPoint(const  int32_t stroke_index){
     COM_CHECK(stroke_path_points.find(stroke_index)!= stroke_path_points.end()); 
     std::vector<phoenix::common::Vec2d> points = stroke_path_points[stroke_index];
     COM_CHECK(points.size()>0); 
     return points[0];
 }
  
   /**
   * @brief 检查插入点(src_stroke_points[i] )是否为当前笔画(stroke_points)中的连续点，
   *  是连续点: 选择当前笔画插入位置，并返回true
   * 不是连续点 : 返回false
   * 
   * @param[in]   src_stroke_points: 所有的路径点
   *                            i :　路径点的索引
   *                            stroke_points：当前笔画的所有路径点
   */
  bool CamPathPrase::CheckInsertPoint(const std::vector<phoenix::common::Vec2d>& src_stroke_points,
                      int i,std::vector<phoenix::common::Vec2d>&stroke_points){
    COM_CHECK(src_stroke_points.size()>=2 && stroke_points.size() >=2); 
    bool is_next_stroke = true;  //检测是否存为下一笔画的路径点
    //检查路径点是从尾部插入
    for(int32_t index =0;index<2;index++){
       if(src_stroke_points[i] ==stroke_points[stroke_points.size()-1-index]){
          stroke_points.push_back(src_stroke_points[i]);
          stroke_points.push_back(src_stroke_points[i +1]);
          is_next_stroke = false;
          break;
       }
       if(src_stroke_points[i +1] ==stroke_points[stroke_points.size()-1-index]){
          stroke_points.push_back(src_stroke_points[i+1]);
          stroke_points.push_back(src_stroke_points[i]);
          is_next_stroke = false;
          break;
       }
    }
    //检查点是从头部插入
    for(int32_t index =0;index<2&&is_next_stroke;index++){
       if(src_stroke_points[i] ==stroke_points[index]){
           if(stroke_points.size() > 2 &&stroke_points[0] == stroke_points[2]){
                 phoenix::common::Vec2d point = stroke_points[1];
                 stroke_points[1] = stroke_points[0];
                 stroke_points[0] = point;
          }
          stroke_points.insert(stroke_points.begin(),src_stroke_points[i]);
          stroke_points.insert(stroke_points.begin(),src_stroke_points[i +1]);
          is_next_stroke = false;
          break;
       }
       if(src_stroke_points[i+1] ==stroke_points[index]){
           if(stroke_points.size() > 2 &&stroke_points[0] == stroke_points[2]){
                 phoenix::common::Vec2d point = stroke_points[1];
                 stroke_points[1] = stroke_points[0];
                 stroke_points[0] = point;
          }
          stroke_points.insert(stroke_points.begin(),src_stroke_points[i+1]);
          stroke_points.insert(stroke_points.begin(),src_stroke_points[i]);
          is_next_stroke = false;
          break;
       }
    }
    return is_next_stroke;
  }

   bool CamPathPrase::PointsToCamPath(){
      COM_CHECK(path_points.size()>0 && path_points.size()%2 ==0); 
      std::vector<phoenix::common::Vec2d> stroke_points; // 用于存储一笔画的所有路径点
      std::vector<phoenix::common::Vec2d> src_stroke_points  =  path_points; // src_stroke_points 用来存储笔画路径所有的点
      std::vector<phoenix::common::Vec2d> next_stroke_points; //下一笔画的路径点
      int32_t stroke_index =0; // 字当中的笔画索引
      stroke_points.push_back(path_points[0]); // 初始化笔画路径点
      stroke_points.push_back(path_points[1]); // 初始化笔画路径点
      do{  
         for(int32_t i=2;i<src_stroke_points.size(); i+=2){
            bool is_next_stroke= CheckInsertPoint(src_stroke_points,i,stroke_points); //检测是否存为下一笔画的路径点
             if(is_next_stroke){
               next_stroke_points.push_back(src_stroke_points[i]);
               next_stroke_points.push_back(src_stroke_points[i +1]);
           }
         }
         if(stroke_points.size() > 2 && stroke_points[0] == stroke_points[2]){
               Vec2d point = stroke_points[1];
               stroke_points[1] = stroke_points[0];
               stroke_points[0] = point;
         }
         //移除相邻的重复的点坐标
         for(std::vector<phoenix::common::Vec2d>::iterator iter=stroke_points.begin();
               iter!=stroke_points.end()-1; ){
                 if(*iter == *(iter+1)){
                     iter = stroke_points.erase(iter);
                 }
                 else {
                     iter++;
                 }
         }
         //当有多个线段共用多个点( >=3)时(如Ｙ) ,将其分解成多个笔画
         int splipIndex =0;
           for(int i=1;i<stroke_points.size()-1;i++){
               for(int index = i+1;index<stroke_points.size()-1;index++){
                  if(stroke_points[index] == stroke_points[i]){
                     splipIndex = i;
                  }        
               }
           }
         if(splipIndex == 0) {
             stroke_path_points.insert(std::make_pair(stroke_index, stroke_points));
         }
         else {
            std::vector<phoenix::common::Vec2d> temp_stroke_points;
            for(int i=0;i<stroke_points.size();i++){
                  temp_stroke_points.push_back(stroke_points[i]);
                  if(i == splipIndex){
                     stroke_path_points.insert(std::make_pair(stroke_index, temp_stroke_points));
                     temp_stroke_points.clear();
                     stroke_index ++;
                  }
            }
            stroke_path_points.insert(std::make_pair(stroke_index, temp_stroke_points));
         }

         if(next_stroke_points.size() ==0)
            break;
         stroke_points.clear();   
         stroke_points.push_back(next_stroke_points[0]);
         stroke_points.push_back(next_stroke_points[1]);
         src_stroke_points = next_stroke_points;
         next_stroke_points.clear();
         stroke_index ++;
      } while (true);
      StrokePointSort();
      AddPointsToLinePath();
   }

   void CamPathPrase::AddPointsToLinePath(){
      COM_CHECK(stroke_path_points.size()>0); 
      float max_line_length =40.0;
      for(int i=0;i<stroke_path_points.size();i++){
         int offset =1;
         std::vector<phoenix::common::Vec2d> stroke_path = stroke_path_points[i];         
         std::vector<phoenix::common::Vec2d>::iterator iter = stroke_path_points[i].begin();
         for(int index=0;index<stroke_path.size()-1;index++){
            iter = stroke_path_points[i].begin() + offset;
            float dis = stroke_path[index].DistanceTo(stroke_path[index+1]);
            if(dis > max_line_length){
               int addNum = static_cast<int>(dis/max_line_length)+1;
               for(int k=1;k<addNum;k++){
                float x = stroke_path[index].x() + (static_cast<float>(k)*(stroke_path[index+1].x()- stroke_path[index].x()))/static_cast<float>(addNum);
                float y = stroke_path[index].y() + (static_cast<float>(k)*(stroke_path[index+1].y()- stroke_path[index].y()))/static_cast<float>(addNum);
               iter = stroke_path_points[i].insert(iter,phoenix::common::Vec2d(x,y))+1;
               }
               offset += addNum;
            }
            else{
               offset=offset+1;
           }
         }
      }
   }
   /**
   * @brief 写字笔画的顺序
   * @param[in] 
   */
   void CamPathPrase::StrokePointSort(){
     COM_CHECK(stroke_path_points.size()>0); 
    for(int i=0;i<stroke_path_points.size();i++){
       int size = stroke_path_points[i].size();
       if (stroke_path_points[i][0].x() - stroke_path_points[i][0].y() <
           stroke_path_points[i][size - 1].x() -
               stroke_path_points[i][size - 1].y()) {
           std::reverse(stroke_path_points[i].begin(),
                        stroke_path_points[i].end()); //实现vector反转
       }
     }
  }
  }  // namespace common
}  // namespace phoenix