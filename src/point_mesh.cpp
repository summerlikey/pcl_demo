#include "point_mesh.h"

////////////////////////////////////////////////////////////////////////////////
/** \brief viewer PointXYZ
  * \param v1_cloud left cloud viewer
  * \param viewer_name the viewer name
  */
void viewerFunction(pcl::PointCloud<pcl::PointXYZ>::Ptr v1_cloud,
                    std::string viewer_name)
{
    // Visualization
    //pcl::visualization::PCLVisualizer::Ptr viewer = (viewer_name);
    std::unique_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer (viewer_name));
    // Create two vertically separated viewports
    int v1(0);
    viewer->createViewPort(0.0, 0.0, 1.0, 1.0, v1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb1(v1_cloud,255,255,255);
    viewer->addPointCloud(v1_cloud, rgb1, "before filter", v1);
    viewer->addCoordinateSystem (1.0);
    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }
}

////////////////////////////////////////////////////////////////////////////////
/** \brief viewer PointXYZ
  * \param v1_cloud left cloud viewer
  * \param v2_cloud right cloud viewer
  * \param viewer_name the viewer name
  */
void viewerFunction(pcl::PointCloud<pcl::PointXYZ>::Ptr v1_cloud,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr v2_cloud,
                    std::string viewer_name)
{
    // Visualization
    //pcl::visualization::PCLVisualizer::Ptr viewer = (viewer_name);
    std::unique_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer (viewer_name));
    // Create two vertically separated viewports
    int v1(0);
    int v2(1);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb1(v1_cloud,255,0,0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb2(v2_cloud,0,0,255);
    viewer->addPointCloud(v1_cloud, rgb1, "before filter", v1);
    viewer->addPointCloud(v2_cloud, rgb2, "after filter", v2);
    viewer->addCoordinateSystem (1.0);
    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }
}


PointMesh::PointMesh(){
    std::cout<<"Point mesh"<<std::endl;
}

PointMesh::~PointMesh(){

}

pcl::PointCloud<PoinT>::Ptr PointMesh::getCloud(){
    return _cloud;
}

////////////////////////////////////////////////////////////////////////////////
/** \brief read ply file to _cloud
  * \param path [in] read ply file path
  */
void PointMesh::readPointFile(std::string path){
    //pcl::PointCloud<PoinT>::Ptr cloud(new pcl::PointCloud<PoinT>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>);

    // load ply file

    pcl::io::loadPLYFile(path, *cloud);
    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;
    _cloud = cloud;
}

////////////////////////////////////////////////////////////////////////////////
/** \brief calculate depth
  * \param mat [in] convert cv::mat data to pcl::PointXYZ
  */
void PointMesh::cvMatToPointCloud(cv::Mat &mat){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>);
    std::cout << "Convert cv::Mat data to pcl::PointXYZ" << std::endl;
    for (int u = 0; u < mat.rows; u++) {
        for (int v = 0; v < mat.cols; v++) {
            pcl::PointXYZ pointXYZ;

            pointXYZ.x = u;
            pointXYZ.y = v;
            pointXYZ.z = mat.at<float>(u, v);
            cloud->points.push_back(pointXYZ);
        }
    }
    _cloud = cloud;
    std::cout<<"convert success"<<std::endl;
}

