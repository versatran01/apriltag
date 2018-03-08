#include <vector>
#include <iostream>
#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "markerdetector.h"
#include "dictionary.h"
using namespace std;

struct Stats{

    double K=0, n=0, Ex=0, Ex2 = 0.0;
     void add(float x) {
        if (n == 0) K = x;
            n = n + 1;
            Ex += x - K;
            Ex2 += (x - K) * (x - K);
    }

    double getStdDev(){
       if (n>1) return sqrt((Ex2 - (Ex*Ex)/n) / (n-1));
               return 0;

//        if (n>1) return   sqrt( (sum2 - ((sum*sum)/n))/(n-1) );
//        return 0;
    }
};

struct MarkerStats{

    Stats cx[4];
    Stats cy[4];
    void add(const vector<cv::Point2f> &corners){

        for(int i=0;i<4;i++){
            cx[i].add(corners[i].x);
            cy[i].add(corners[i].y);
        }
    }

    double getStats( ){
        double sum=0;
        for(int i=0;i<4;i++){
             sum+=cx[i].getStdDev();
             sum+=cy[i].getStdDev();
        }
        return sum/8.;
    }
};
int main(int argc,char **argv)
{


    cv::VideoCapture vcap(1);
    vcap.set(CV_CAP_PROP_FRAME_WIDTH,1280);
    vcap.set(CV_CAP_PROP_FRAME_HEIGHT,720);

    aruco::MarkerDetector mdetector;
    mdetector.setDictionary(aruco::Dictionary::ARUCO_MIP_36h12);
    cv::Mat im;
    std::map<uint32_t,MarkerStats>  markerStats;
    char key=0;
    //grab some initial frames to clean

for(int i=0;i<10;i++) vcap.grab();
    int ntimes=0;
    while(vcap.grab() && key!=27){
        ntimes++;
        vcap.retrieve(im);

        mdetector.detectEnclosedMarkers(false);
        auto m1=mdetector.detect(im);
       for(auto m:m1)
            if (m.id==1) markerStats[m.id].add(m);
       mdetector.detectEnclosedMarkers(true);
       auto m0=mdetector.detect(im);
      for(auto m:m0)
          if (m.id==0) markerStats[m.id].add(m);


      for(auto m:m1)
          if (m.id==1)  m.draw(im);
      for(auto m:m0)
          if (m.id==0)  m.draw(im);


//        for(auto m:mdetector.detect(im)){
//            if (m.id==0){
//                markerStats[m.id].add(m);
//                m.draw(im);
//            }
//        }


        //print stats
        for(auto &ms:markerStats){
            cout<<"Error in marker "<<ms.first<<" = "<<ms.second.getStats()<<" pix"<<endl;
            cv::putText(im,"Error marker "+to_string(ms.first)+"="+to_string(ms.second.getStats() )+" pix",cv::Point(10,20+20*float(ms.first)),cv::FONT_HERSHEY_SIMPLEX, 0.5f,cv::Scalar(125,255,255),1,CV_AA);
        }

        cv::imshow("image",im);
        key=cv::waitKey(10);
    }

}
