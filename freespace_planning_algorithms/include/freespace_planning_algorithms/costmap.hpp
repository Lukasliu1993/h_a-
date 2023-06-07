#include<string.h>
#include<stdio.h>
class Costmap {
public:
    Costmap(unsigned char *input_data,int width,int height){
        data = new unsigned char[height*width];
        memcpy(data,input_data,height*width*sizeof(unsigned char));
        m_width = width;
        m_height = height;
        
    };
    Costmap(){
    };
    ~Costmap(){
        // delete data;
    };
    void updateGrid(unsigned char *input_data, int width, int height){
        delete data;
        m_width = width;
        m_height = height;
        data = new unsigned char[height*width];
        memcpy(data,input_data,height*width*sizeof(unsigned char));
    };
    void setpose(double x, double y){
        pos_x = x;
        pos_y = y;
    };
    int m_width;
    int m_height;
    double pos_x;
    double pos_y;
    double resolution = 1.0;
    unsigned char * data;
};
