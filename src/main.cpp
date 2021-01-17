#include <string>
#include <time.h>
#include "point_mesh.h"
#include "phase_process.h"

int main(int argc, char** argv)
{
	int step = 4;
	int period_num = 8;
	int leftupv = 630;//左上角->v
	int leftupu = 350;//u
	int height = 500;//row height
	int width = 500;//col width
    double calib_interval = 15.0;//calib interval 15mm

	//**fringe projection profilometry**/
    std::cout<<"fringe projection profilometry"<<std::endl;
    //opencv 图像坐标系和ｍatlab图像坐标系不同
    PhaseProcess phaseProcess;//fringe projector process

    std::string work_path = "/home/ganggang/Pictures/images0106_ONE/O";
    std::string object_path = "/home/ganggang/Pictures/images0106_ONE/O/o_1";
    
    std::vector<std::string> calib_images_path;
    for(int i=1;i<=4;i++){
        std::string res;
        std::stringstream ss;
        ss << i;
        ss >> res;//或者 res = ss.str();
        std::string item_path = work_path + "/v_" + res;
        calib_images_path.push_back(item_path);
    }
    phaseProcess.setCalibImagesPath(calib_images_path);
    phaseProcess.setInitialParameters(leftupv,leftupu,height,width,period_num,step,calib_interval);
    phaseProcess.setWorkPath(object_path);
    phaseProcess.setCalibParam(calib_images_path,calib_interval,step,period_num);

	cv::Mat object_unphase = cv::Mat::zeros(height, width, CV_32F);//object unphase
	cv::Mat object_height = cv::Mat::zeros(height, width, CV_32F);//object unphase

	phaseProcess.unphaseByTime(object_path,step,period_num,object_unphase);
	phaseProcess.calculateDepth(object_unphase,object_height);//fringe projection profilometry

	PointMesh pointMesh;
	//std::string path = "/home/ganggang/Pictures/images0107/O/o_12/fadongji.ply";
	//pointMesh.readPointFile(path);//ch
	pointMesh.cvMatToPointCloud(object_height);//cv mat to pcl data

	viewerFunction(pointMesh.getCloud(),"3D cloud view");

	return 0;
}