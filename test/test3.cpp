// #include "stdafx.h"  
#include <pcl/io/ply_io.h>  
#include<iostream>  
using namespace std;  
int numofPoints(char* fname){  
    int n=0;  
    int c=0;  
    FILE *fp;  
    fp = fopen(fname,"r");  
    do{  
        c = fgetc(fp);  
        if(c == '\n'){  
            ++n;  
        }  
    }  
    while(c != EOF);  
    fclose(fp);  
    return n;  
}  
int main()  
{  
    int n = 0; //n用来计文件中点个数      
    FILE *fp_1;  
    fp_1 = fopen("/home/ganggang/Documents/pcl/pcl_demo/doc/object_height.txt","r");  
    n = numofPoints("/home/ganggang/Documents/pcl/pcl_demo/doc/object_height.txt");//使用numofPoints函数计算文件中点个数  
    std::cout << "there are "<<n<<" points in the file..." <<std::endl;  
    //新建一个点云文件，然后将结构中获取的xyz值传递到点云指针cloud中。  
    pcl::PointCloud<pcl::PointXYZ> cloud;  
    cloud.width    = n;  
    cloud.height   = 1;  
    cloud.is_dense = false;  
    cloud.points.resize (cloud.width * cloud.height);  
    //将点云读入并赋给新建点云指针的xyz      
    double x,y,z;  
    int i = 0;  
    while(3 ==fscanf(fp_1,"%lf %lf %lf\n",&x,&y,&z)){  
        cout<<x<<" "<<y<<" "<<z<<endl;  
        cloud.points[i].x = x;  
        cloud.points[i].y = y;  
        cloud.points[i].z = z;  
        ++i;  
    }  
    fclose(fp_1);  
    //将点云指针指向的内容传给pcd文件  
    
    pcl::io::savePLYFileASCII ("object_height.ply", cloud);  
    
    std::cerr <<"Saved " << cloud.points.size () <<" data points to test_ply." << std::endl;  
    return 0;  
}