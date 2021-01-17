#include"phase_process.h"

PhaseProcess::PhaseProcess(){
    std::cout<<"Phase Process constrort"<<std::endl;
}

PhaseProcess::~PhaseProcess(){

}

void PhaseProcess::setInitialParameters(int x,int y,int width,int height,int period_num,int step,double calib_interval)
{
    _x = x;//opencv图像坐标系和matlab图像坐标系不同，x相当于matlab中v
    _y = y;
    _width = width;
    _height = height;
    _period_num = period_num;// time unphase period number
    _read_num_of_pics = step;//phase shifting step
    _calib_interval = calib_interval;//calib interval
    std::cout<<"(u,v): "<<y<<","<<x<<std::endl;
    std::cout<<"col width　＝　"<<width<<std::endl;
    std::cout<<"row height　＝　"<<height<<std::endl;
    std::cout<<"period_num: "<<period_num<<std::endl;
    std::cout<<"phase shifting step: "<<step<<std::endl;
    std::cout<<"calib interval: "<<calib_interval<<std::endl;
}

void PhaseProcess::setCalibImagesPath(std::vector<std::string> calib_images_path){
    _calib_images_path = calib_images_path;
}
void PhaseProcess::setWorkPath(std::string work_path){
    _work_path = work_path;
}

////////////////////////////////////////////////////////////////////////////////
/** \brief unphase by time
  * \param images_path [in] images dir path
  * \param step [in] phase shifting step 
  * \param period [in] period 1,2,4,8,16,32,64
  * \param UnPhase [out] unphase
  */
void PhaseProcess::unphaseByTime(std::string images_path,int step,int period_num,cv::Mat &UnPhase){

    //glob是按照字典序排序
    // cv::glob(images_path, _images_path);//读取指定文件夹下图像
    // std::sort(_images_path.begin(),_images_path.end(),[](cv::String a,cv::String b){
    //     return a>b;
    // });
    // for(int i = 0;i<_images_path.size();i++){
    //     std::cout<<_images_path[i]<<std::endl;
    // }

    /**********/

    std::vector<cv::Mat> all_unphase;
    std::vector<cv::Mat> phase;
    std::vector<cv::Mat> Sum;

    cv::Rect focus_rect(_x,_y,_width,_height);
    int count_img = 0;
    for(int i = 0;i<_period_num;i++){
        /*1....7*/
        cv::Mat SumP = cv::Mat::zeros(_height, _width, CV_32F);
        cv::Mat SumQ = cv::Mat::zeros(_height, _width, CV_32F);
        cv::Mat SumI = cv::Mat::zeros(_height, _width, CV_32F);
        int n_period = powl(2,i);
        for(int k = 1;k<=_read_num_of_pics;k++){
            std::string image_path = images_path + '/' + std::to_string(n_period)+"_"+std::to_string(k-1)+".bmp";
            cv::Mat _image = cv::imread(image_path);
		    cv::cvtColor(_image, _img_gray, cv::COLOR_BGR2GRAY);

            cv::Mat img =  _img_gray(focus_rect);
            //cv::imshow("img",img);//show images
            //cv::waitKey(100);
            img.convertTo(img,CV_32F);
            //normalize(img,img,1.0,0.0,cv::NORM_MINMAX);
            add( SumP , img * sin(2*M_PI*(k)/_read_num_of_pics),SumP);
            add( SumQ , img * cos(2*M_PI*(k)/_read_num_of_pics),SumQ);
            add( SumI , img , SumI);
        }
        //包裹位相求解
        phase.push_back(-cvAtan2Mat(SumP,SumQ));
        Sum.push_back(SumI/3);

        int n = phase.size();
        if(n == 1){
            all_unphase.push_back(phase.back());
        }
        else{
            cv::Mat unphase;
            add(phase.back(),cvRoundMat((all_unphase[n-2]*2 - phase.back())/(2*M_PI))*2*M_PI,unphase);

            all_unphase.push_back(unphase);
            //_unphase.push_back(_phase.back() + cvRoundMat(_unphase[n-2]*2 - _phase.back())/(2*M_PI)*2*M_PI);
        }

    }
    std::cout<<"phase num: "<<all_unphase.size()<<std::endl;
    std::cout<<"unphase success"<<std::endl;
    //std::cout<<_unphase.back()<<std::endl;

    UnPhase = all_unphase.back().clone();//out the last one
    //show
    // cv::Mat unphase_show;
    // normalize(all_unphase.back(),unphase_show,1.0,0.0,cv::NORM_MINMAX);
    // cv::imshow("unphase by time",unphase_show);//show
    // cv::waitKey();
}
////////////////////////////////////////////////////////////////////////////////
/** \brief unphase by time 
  * \param height_low [in] height low
  * \param height_up [in] height up
  */
void PhaseProcess::processing(float height_low,float height_up){

    calculateDepth(_calib_param,height_low,height_up,_object_unphase,_object_height);

    std::cout<<"calcut depth success"<<std::endl;
    //std::cout<<_unphase.back()<<std::endl;
    cv::Mat result_show;
    normalize(_object_height,result_show,1.0,0.0,cv::NORM_MINMAX);
    cv::imshow("depth",result_show);
    cv::waitKey();
}

////////////////////////////////////////////////////////////////////////////////
/** \brief calute height from object_path 
  * \param images_path [in] the images dir path have 
  * \param height_low [in]
  * \param height_up [in]
  * \param step [in] the phase shifting step
  * \param period [in] period 1,2,4,8,16,32,64,128
  */
void PhaseProcess::calcuateHeightFromPath(std::string images_path,float height_low,float height_up,int step,int period_num){
    cv::Mat object_unphase = cv::Mat::zeros(_height, _width, CV_32F);
    unphaseByTime(images_path,step,period_num,object_unphase);
    cv::Mat object_height = cv::Mat::zeros(_height, _width, CV_32F);
    calculateDepth(_calib_param,height_low,height_up,object_unphase,object_height);

    std::cout<<"calcut depth success from object images path"<<std::endl;
    //std::cout<<_unphase.back()<<std::endl;
    cv::Mat result_show;
    normalize(object_height,result_show,1.0,0.0,cv::NORM_MINMAX);
    cv::imshow("depth",result_show);
    cv::waitKey();
}

////////////////////////////////////////////////////////////////////////////////
/** \brief compute object unphase and set the system calib param c1 c2 c3 c4
  * \param images_path [in] images dir path
  * \param step [in] phase shifting step 
  * \param period [in] period 1,2,4,8,16,32,64
  */
void PhaseProcess::setCalibParam(std::vector<std::string> calib_images_path,double calib_interval,int step ,int period)
{
    _calib_unphase.clear();
    int calib_num = calib_images_path.size();
    _calib_num = calib_num;
    for(int i=0;i<calib_num;i++){
        cv::Mat unphase = cv::Mat::zeros(_height, _width, CV_32F);
        unphaseByTime(calib_images_path[i],_read_num_of_pics,_period_num,unphase);
        _calib_unphase.push_back(unphase);
    }
    //calib depth
    float h0 = 0;
    float h1 = calib_interval*1;
    float h2 = calib_interval*2;
    float h3 = calib_interval*3;
    cv::Mat C1 = cv::Mat::zeros(_height, _width, CV_32F);
    cv::Mat C2 = cv::Mat::zeros(_height, _width, CV_32F);
    cv::Mat C3 = cv::Mat::zeros(_height, _width, CV_32F);
    cv::Mat C4 = cv::Mat::zeros(_height, _width, CV_32F);

    cv::Mat uwp0 = cv::Mat::zeros(_height, _width, CV_32F); 
    cv::Mat uwp1 = cv::Mat::zeros(_height, _width, CV_32F); 
    cv::Mat uwp2 = cv::Mat::zeros(_height, _width, CV_32F); 
    cv::Mat uwp3 = cv::Mat::zeros(_height, _width, CV_32F); 

    uwp0 = _calib_unphase[0];
    uwp1 = _calib_unphase[1];
    uwp2 = _calib_unphase[2];
    uwp3 = _calib_unphase[3];

    C1 = h3*(h1-h2)*(uwp1.mul(uwp2))+h1*(h2-h3)*(uwp2.mul(uwp3))+h2*(h3-h1)*(uwp3.mul(uwp1));
    C2 = h3*(h1-h2)*uwp3+h1*(h2-h3)*uwp1+h2*(h3-h1)*uwp2;
    C3 = (h1-h2)*(uwp1.mul(uwp2))+(h2-h3)*(uwp2.mul(uwp3))+(h3-h1)*(uwp3.mul(uwp1));
    C4 = (h1-h2)*uwp3+(h2-h3)*uwp1+(h3-h1)*uwp2;

    _calib_param.push_back(C1);
    _calib_param.push_back(C2);
    _calib_param.push_back(C3);
    _calib_param.push_back(C4);

    // cv::imshow("calib_unphase",_calib_unphase.back());
    // cv::waitKey();
}

////////////////////////////////////////////////////////////////////////////////
/** \brief calculate depth
  * \param calib_unphase [in] calib unphase
  * \param height_low [in]
  * \param height_up [in]
  * \param calib_interval [in] 
  * \param object_unphase [in] object_unphase
  */
void PhaseProcess::calculateDepth(std::vector<cv::Mat> calib_param,float height_low,float height_up,cv::Mat object_unphase,cv::Mat &object_height)
{
    object_height = (calib_param[0] + calib_param[1].mul(object_unphase))/(calib_param[2]+calib_param[3].mul(object_unphase));


    // filter the data
    for(int i = 0;i<_width;i++){
        for(int j=0;j<_height;j++){
            float item = object_height.at<float>(i, j);
            if(item>height_up||item<height_low){
                object_height.at<float>(i, j) = 0;
            }
        }
    }

    //save point txt
    // const char* filename = "./object_height.txt";
    // writeMatToFile(object_height, filename);

    //show result
    // cv::Mat result_show;
    // normalize(object_height,result_show,1.0,0.0,cv::NORM_MINMAX);
    // cv::imshow("depth_show",result_show);
}

////////////////////////////////////////////////////////////////////////////////
/** \brief calculate depth
  * \param calib_interval [in] 
  * \param object_unphase [in] object_unphase
  */
void PhaseProcess::calculateDepth(cv::Mat &object_unphase,cv::Mat &object_height)
{
    object_height = (_calib_param[0] + _calib_param[1].mul(object_unphase))/(_calib_param[2]+_calib_param[3].mul(object_unphase));
    // filter the data
    // for(int i = 0;i<_width;i++){
    //     for(int j=0;j<_height;j++){
    //         float item = object_height.at<float>(i, j);
    //         if(item>height_up||item<height_low){
    //             object_height.at<float>(i, j) = 0;
    //         }
    //     }
    // }
}

////////////////////////////////////////////////////////////////////////////////
/** \brief compute object unphase
  * \param images_path [in] images dir path
  * \param step [in] phase shifting step 
  * \param period [in] period 1,2,4,8,16,32,64
  */
void PhaseProcess::setObjectUnphase(std::string images_path,int step,int period_num)
{
    unphaseByTime(images_path,step,period_num,_object_unphase);
    // const char* filename = "./object_unphase.txt";
    // writeMatToFile(_object_unphase, filename);
    std::cout<<"object_unhase: "<<std::endl;
    cv::imshow("object_unphase",_object_unphase);
    cv::waitKey();
}

CvMat* cvAtan2Mat(CvMat *a, CvMat *b)
{
    int rows = a->rows;
    int cols = a->cols;
    CvMat *out = cvCreateMat(rows, cols, a->type);
    for(int i=0; i<rows; i++)
    {
        float* ptra = ( float*)(a->data.ptr+i*a->step);
        float* ptrb = ( float*)(b->data.ptr+i*b->step);
        float* ptrout = ( float*)(out->data.ptr+i*out->step);
        for(int j=0; j<cols; j++)
        {
            *ptrout = atan2(*ptra,*ptrb);
            ptra++;
            ptrb++;
            ptrout++;
        }
    }
    return out;
}

////////////////////////////////////////////////////////////////////////////////
/** \brief compute the mat atan2.
  * \param a [in] mat a
  * \param b [in] mat b 
  * \return cv::Mat
  */
cv::Mat cvAtan2Mat(cv::Mat a,cv::Mat b){
    int rows = a.rows;
    int cols = a.cols;
    cv::Mat out = cv::Mat::zeros(rows, cols, a.type());
    for(int i=0; i<rows; i++)
    {

        for(int j=0; j<cols; j++)
        {
            out.at<float>(i,j) = atan2(a.at<float>(i,j),b.at<float>(i,j));
        }
    }
    return out;
}

////////////////////////////////////////////////////////////////////////////////
/** \brief round mat .
  * \param a [in] the src mat
  * \return cv::Mat
  */
cv::Mat cvRoundMat(cv::Mat a){
    int rows = a.rows;
    int cols = a.cols;
    cv::Mat out = cv::Mat::zeros(rows, cols, a.type());
    for(int i=0; i<rows; i++)
    {

        for(int j=0; j<cols; j++)
        {
            out.at<float>(i,j) = cvRound(a.at<float>(i,j));
        }
    }
    return out;
}

void writeMatToFile(cv::Mat& m, const char* filename) 
{ 
    std::ofstream fout(filename); 
    if (!fout) 
    {
        std::cout << "File Not Opened" << std::endl;
        return;
    }
    
    for (int i = 0; i<m.rows; i++) 
    {
        for (int j = 0; j<m.cols; j++) 
        {
            fout <<i<<" "<<j << " " << m.at<float>(i, j)<<std::endl;
        }
    }
    fout.close(); 
    std::cout<<"save mat"<<std::endl;
}