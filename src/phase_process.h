#ifndef _PHASE_PROCESS_
#define _PHASE_PROCESS_

#include<iostream>
#include<cmath>
#include<opencv2/opencv.hpp>

class PhaseProcess{
public:

    PhaseProcess();
    ~PhaseProcess();

    void setInitialParameters(int x,int y,int width,int height,int period_num,int step,double calib_interval);//initial parameters
    void setWorkPath(std::string work_path);//workPath
    void setCalibImagesPath(std::vector<std::string> calib_images_path);
    void setObjectUnphase(std::string images_path,int step,int period_num);//object unphase

    void unphaseByTime(std::string images_path,int step,int period_num,cv::Mat &UnPhase);

    void setCalibParam(std::vector<std::string> calib_images_path,double calib_interval,int step ,int period);//calib unphase
    void calculateDepth(std::vector<cv::Mat> calib_param,float height_low,float height_up,cv::Mat object_unphase,cv::Mat &object_height);//calib depth
    void calculateDepth(cv::Mat &object_unphase,cv::Mat &object_height);
    void calcuateHeightFromPath(std::string images_path,float height_low,float height_up,int step,int period_num);

    

    void processing(float height_low,float height_up);//possessing

private:
    std::vector<cv::String> _images_path;//创建容器存放读取图像路径
    int _x;//opencv图像坐标系和matlab图像坐标系不同，x相当于matlab中v
    int _y;
    int _width,_height;

    int _period_num;// time unphase period num
    int _read_num_of_pics;//phase shifting step

    std::string _work_path;//work path

    std::vector<cv::Mat> _phase;
    std::vector<cv::Mat> _unphase;
    std::vector<cv::Mat> _sum;

    cv::Mat _image;
    cv::Mat _img_gray;

    std::vector<std::string> _calib_images_path;//calib images path
    int _calib_num;//calib num
    double _calib_interval;//calib interval 15mm
    std::vector<cv::Mat> _calib_unphase;//标定板位置,一般取4个,间隔15mm
    std::vector<cv::Mat> _calib_param;//标定系统参数C1,C2,C3,C4,一般取4个,间隔15mm
    cv::Mat _object_unphase;//object unphase;
    cv::Mat _object_height;//object height;

};


CvMat* cvAtan2Mat(CvMat *a, CvMat *b);
cv::Mat cvAtan2Mat(cv::Mat a,cv::Mat b);
cv::Mat cvRoundMat(cv::Mat a);
void writeMatToFile(cv::Mat& m, const char* filename);

#endif //_PHASE_PROCESS_