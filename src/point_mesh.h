#ifndef _POINT_MESH_
#define _POINT_MESH_
#include <pcl/point_types.h>          //PCL中所有点类型定义的头文件
#include <pcl/io/pcd_io.h>            //打开关闭pcd文件的类定义的头文件
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>//视觉化工具函式库（VTK，Visualization Toolkit）　模型
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>  //kd-tree搜索对象的类定义的头文件
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h> //PCL的PCD格式文件的输入输出头文件
#include <pcl/point_types.h> //PCL对各种格式的点的支持头文件
#include <pcl/search/kdtree.h>//kdtree搜索对象的类定义的头文件
#include <pcl/features/normal_3d.h>//法向量特征估计相关类定义的头文件
//重构
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/mls.h>          //最小二乘法平滑处理类定义头文件
#include <pcl/surface/marching_cubes_hoppe.h>// 移动立方体算法
#include <pcl/surface/marching_cubes_rbf.h>

typedef pcl::PointXYZ PoinT;
typedef pcl::PointNormal PoinTNormal;

class MeshPoint{
public:

    MeshPoint();
    ~MeshPoint();

private:


};

#endif //_POINT_MESH_