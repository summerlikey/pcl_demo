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

#include <time.h>
typedef pcl::PointXYZ PoinT;
typedef pcl::PointNormal PoinTNormal;
using namespace std;

int main(int argc, char** argv)
{
    clock_t start,end;//定义clock_t变量
    start = clock();//开始时间
    string name = "/home/ganggang/Pictures/images0107/O/o_2/taijie.ply";
    pcl::PointCloud<PoinT>::Ptr cloud(new pcl::PointCloud<PoinT>());
    // 加载pcd文件
    pcl::io::loadPLYFile(name, *cloud);
    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;

    // for(int nIndex = 0; nIndex < cloud->points.size(); nIndex++)
    // {
    //     cloud->points[nIndex].x *= 100;
    //     cloud->points[nIndex].y *= 100;
    //     cloud->points[nIndex].z *= 100;
    // }
    //点云离群滤波
    // Create the filtering object
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(50);
	sor.setStddevMulThresh(1.0);
	sor.filter(*cloud_filtered);
	cout << "there are " << cloud_filtered->points.size() << " points after filtering." << endl;
    cloud = cloud_filtered;

    //体素滤波
    pcl::VoxelGrid<pcl::PointXYZ> sorvg;
    sorvg.setInputCloud(cloud);
    sorvg.setLeafSize (0.01f, 0.01f, 0.01f);// 单位：m
    sorvg.filter (*cloud_filtered);
	cout << "there are " << cloud_filtered->points.size() << " points after filtering." << endl;
    cloud = cloud_filtered;
    // 计算法向量
    pcl::NormalEstimation<PoinT, pcl::Normal> n;//法线估计对象
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//存储估计的法线
    pcl::search::KdTree<PoinT>::Ptr tree(new pcl::search::KdTree<PoinT>);//定义kd树指针
    tree->setInputCloud(cloud);                        //用cloud构建tree对象
    n.setInputCloud(cloud);                            //为法线估计对象设置输入点云
    n.setSearchMethod(tree);                          //设置搜索方法
    n.setKSearch(20);                                 //设置k搜索的k值为20
    n.compute(*normals);                              //估计法线存储结果到normals中
    //将点云和法线放到一起
    pcl::PointCloud<PoinTNormal>::Ptr cloud_with_normals(new pcl::PointCloud<PoinTNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);//连接字段，cloud_with_normals存储有向点云
    std::cerr << "法线计算   完成" << std::endl;
    end = clock();//结束时间
    cout<<"法线计算  完成 time = "<<double(end-start)/CLOCKS_PER_SEC<<"s"<<endl;  //输出时间（单位：ｓ）
    //创建搜索树
    pcl::search::KdTree<PoinTNormal>::Ptr tree2(new pcl::search::KdTree<PoinTNormal>);
    tree2->setInputCloud(cloud_with_normals);

    // 曲面重建,此段为是否使用最小二乘估计法线
    pcl::MovingLeastSquares<PoinTNormal, PoinTNormal> mls;
    mls.setComputeNormals(true);  //设置在最小二乘计算中需要进行法线估计
    mls.setInputCloud(cloud_with_normals);//设置参数
    mls.setPolynomialFit(true);
    mls.setSearchMethod(tree2);
    mls.setSearchRadius(5);//影响计算时间，值越大，耗时越久
    pcl::PointCloud<PoinTNormal>::Ptr cloud_with_normals_msl(new pcl::PointCloud<PoinTNormal>);
    mls.process(*cloud_with_normals_msl);
    cloud_with_normals = cloud_with_normals_msl;
    std::cerr << "法线最小二乘法  完成" << std::endl;
    end = clock();//结束时间
    cout<<"法线最小二乘法  完成 time = "<<double(end-start)/CLOCKS_PER_SEC<<"s"<<endl;  //输出时间（单位：ｓ）
    // 开始表面重建 ********************************************************************

    //创建Poisson对象，并设置参数
    // pcl::Poisson<pcl::PointNormal> pn;
    // pn.setConfidence(false); //是否使用法向量的大小作为置信信息。如果false，所有法向量均归一化。
    // pn.setDegree(2); //设置参数degree[1,5],值越大越精细，耗时越久。
    // pn.setDepth(8);
    // //树的最大深度，求解2^d x 2^d x 2^d立方体元。
    // // 由于八叉树自适应采样密度，指定值仅为最大深度。
    // pn.setIsoDivide(8); //用于提取ISO等值面的算法的深度
    // pn.setManifold(false); //是否添加多边形的重心，当多边形三角化时。 
    // // 设置流行标志，如果设置为true，则对多边形进行细分三角话时添加重心，设置false则不添加
    // pn.setOutputPolygons(false); //是否输出多边形网格（而不是三角化移动立方体的结果）
    // pn.setSamplesPerNode(3.0); //设置落入一个八叉树结点中的样本点的最小数量。无噪声，[1.0-5.0],有噪声[15.-20.]平滑
    // pn.setScale(1.25); //设置用于重构的立方体直径和样本边界立方体直径的比率。
    // pn.setSolverDivide(8); //设置求解线性方程组的Gauss-Seidel迭代方法的深度
    // //pn.setIndices();
    // //设置搜索方法和输入点云
    // pn.setSearchMethod(tree2);
    // pn.setInputCloud(cloud_with_normals);
    // //创建多变形网格，用于存储结果
    // pcl::PolygonMesh mesh;
    // //执行重构
    // pn.performReconstruction(mesh);
    // //保存网格图
    // pcl::io::savePLYFile(name + "-poisson.ply", mesh);
    // std::cerr << "泊松重建   完成" << std::endl;


    // // 贪婪投影三角化算法
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;   // 定义三角化对象
    pcl::PolygonMesh mesh;                // 存储最终三角化的网络模型
    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius(5);  // 设置连接点之间的最大距离，（即是三角形最大边长）
    // 设置各参数值
    gp3.setMu(3);  // 设置被样本点搜索其近邻点的最远距离为2.5，为了使用点云密度的变化
    gp3.setMaximumNearestNeighbors(100);// 设置样本点可搜索的邻域个数
    gp3.setMaximumSurfaceAngle(M_PI / 4);  // 设置某点法线方向偏离样本点法线的最大角度45
    gp3.setMinimumAngle(M_PI / 18);        // 设置三角化后得到的三角形内角的最小的角度为10
    gp3.setMaximumAngle(2 * M_PI / 3);       // 设置三角化后得到的三角形内角的最大角度为120
    gp3.setNormalConsistency(false);     // 设置该参数保证法线朝向一致
    // Get result
    gp3.setInputCloud(cloud_with_normals);// 设置输入点云为有向点云
    gp3.setSearchMethod(tree2);               // 设置搜索方式
    gp3.reconstruct(mesh);               // 重建提取三角化
    // 附加顶点信息
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();
    // 保存mesh文件
    pcl::io::savePLYFile(name + "-quick.ply", mesh);
    std::cerr << "快速三角化 完成" << std::endl;


    // //移动立方体算法
    // pcl::MarchingCubes<pcl::PointNormal> *mc;
    // mc = new pcl::MarchingCubesHoppe<pcl::PointNormal>();
    // /*
    //   if (hoppe_or_rbf == 0)
    //     mc = new pcl::MarchingCubesHoppe<pcl::PointNormal> ();
    //   else
    //   {
    //     mc = new pcl::MarchingCubesRBF<pcl::PointNormal> ();
    //     (reinterpret_cast<pcl::MarchingCubesRBF<pcl::PointNormal>*> (mc))->setOffSurfaceDisplacement (off_surface_displacement);
    //   }
    // */
    // //创建多变形网格，用于存储结果
    // pcl::PolygonMesh mesh;
    // //设置MarchingCubes对象的参数
    // mc->setIsoLevel(0.0f);
    // mc->setGridResolution(50, 50, 50);
    // mc->setPercentageExtendGrid(0.0f);
    // //设置搜索方法
    // mc->setInputCloud(cloud_with_normals);
    // //执行重构，结果保存在mesh中
    // mc->reconstruct(mesh);
    // //保存网格图
    // pcl::io::savePLYFile(name + "-cubes.ply", mesh);
    // std::cerr << "移动立方体 完成" << std::endl;


    end = clock();//结束时间
    cout<<"表面重建 完成 time = "<<double(end-start)/CLOCKS_PER_SEC<<"s"<<endl;  //输出时间（单位：ｓ）
    // 显示结果图
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPolygonMesh(mesh, "my");
    viewer->addCoordinateSystem(50.0);
    viewer->initCameraParameters();
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return 0;

}

