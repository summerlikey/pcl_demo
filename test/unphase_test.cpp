#include<iostream>
#include"phase_process.h"
int main(){
    std::cout<<"unphase_test"<<std::endl;
    //opencv 图像坐标系和ｍatlab图像坐标系不同
    PhaseProcess phaseProcess;

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
    phaseProcess.setInitialParameters(630,350,500,500,8,4,15);
    phaseProcess.setWorkPath(object_path);
/************test*********************************/
    //phaseProcess.processing(object_path,4,8);//processing (dir_path,4,v,u,width,height)
    // cv::Mat unphase = cv::Mat::zeros(500, 500, CV_32F);
    // phaseProcess.unphaseByTime(object_path,4,8,unphase);
    // cv::imshow("test",unphase);
    // cv::waitKey();
/***********test**********************************/
    phaseProcess.setCalibParam(calib_images_path,15.0,4,8);
    phaseProcess.calcuateHeightFromPath(object_path,-10,100,4,8);
    phaseProcess.setObjectUnphase(object_path,4,8);
    phaseProcess.processing(-10,300);

    return 0;
}