#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <numeric>
#include<math.h>

const double ref_dist=10;
const double ref_size=400;
double ratiodepth=ref_dist/ref_size;
int lowThreshold = 80;
const int max_lowThreshold = 200;
const int ratio = 3;
const int kernel_size = 3;
const double scale=1;
const char* window_name = "Edge Map";
const int corner_threshold=100;
const double epsilon=1;

double pixel_dist(int x1,int y1,int x2,int y2)
{
    double d=sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
    return d*ratiodepth;
}

int main() {

    //std::string image_path = "/home/pratyush/Desktop/CV/frame.png";
    //std::string image_path = "/home/pratyush/Desktop/CV/tilt.png";
    std::string image_path = "/home/pratyush/Desktop/CV/pic9.png";
    cv::Mat img,thresh,corn;
    img = cv::imread(image_path, CV_LOAD_IMAGE_ANYCOLOR);
    if(img.empty()) 
    {
        std::cout << "Could not read the image: " << image_path << std::endl;
        return 1;
    }
cv::resize(img, img, cv::Size(img.cols*scale, img.rows*scale));
    cv::imshow("Original", img);
    cv::Mat hsv;
    cv::cvtColor(img,hsv,CV_BGR2HSV);

    std::vector<cv::Mat> channels;
    cv::split(hsv, channels);

    cv::Mat H = channels[0];
    cv::Mat S = channels[1];
    cv::Mat V = channels[2];
    cv::GaussianBlur(S, S, cv::Size(15,15), 0, 0);
    cv::Mat grey,dst,detected_edges;
    cv::cvtColor(img, grey, CV_BGR2GRAY);
    dst.create( img.size(), img.type() );
    dst = cv::Scalar::all(0);
    cv::imshow("S channel", S);
    cv::Mat canny_output;
    cv::Canny( S, canny_output, lowThreshold, lowThreshold*ratio, kernel_size );
    cv::GaussianBlur(canny_output, canny_output, cv::Size(3,3), 0, 0);

    cv::imshow("Canny Output", canny_output);
    cv::Mat final;

    std::vector< std::vector <cv::Point> > contours; // Vector for storing contour

    std::vector< cv::Vec4i > hierarchy;
    cv::findContours( canny_output, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
    if(contours.size()==0)
    {
        cv::threshold(S,S,20,255,0);
        cv::Canny(S,canny_output,lowThreshold,lowThreshold*ratio,kernel_size);
        cv::findContours( canny_output, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
    }
    std::vector< std::vector<cv::Point> > hull(contours.size());
	for(int i = 0; i < contours.size(); i++)
	    cv::convexHull(cv::Mat(contours[i]), hull[i], false);
    //
    cv::drawContours(dst, hull,2, 255, -1);
    //
    cv::imshow("Hull work",dst);
    double maxArea=0;
    double maxAreaContourId = 0;
    std::cout<<contours.size()<<std::endl;
    for( int i = 0; i< contours.size(); i++ ) // iterate through each contour.
    //for( int i = 0; i< hull.size(); i++ )
    {
        double newArea=cv::contourArea(contours[i]);
        double convexArea=cv::contourArea(hull[i]);
        if(convexArea/newArea>1.2)
            continue;
        if (newArea > maxArea) {
            maxArea = newArea;
            maxAreaContourId = i;
        } 
    }

    final.create( S.size(), S.type() );
    final = cv::Scalar::all(0);
    cv::drawContours(final, contours, maxAreaContourId, 255, -1);

    std::vector<cv::Vec4i> linesP; // will hold the results of the detection
    cv::HoughLinesP(final, linesP, 1, CV_PI/180, 200, 50, 10 ); // runs the actual detection
     
    if(linesP.size()==0)
    {
        cv::HoughLinesP(final, linesP, 1, CV_PI/180, 10, 10, 10 ); 
    }
    int vec_len=linesP.size();
    double depth=0;
    for( size_t i = 0; i < vec_len-1; i++ )
    {
        depth+=pixel_dist(linesP[i][0],linesP[i][2],linesP[i+1][1],linesP[i+1][3]);
    }
    depth+=pixel_dist(linesP[0][0],linesP[0][2],linesP[vec_len-1][1],linesP[vec_len-1][3]);
    depth=depth/(2*vec_len);
    std::cout<<depth<<std::endl;
    cv::waitKey(0);
    cv::destroyAllWindows();
    return 0;
}