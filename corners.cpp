#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <numeric>
int lowThreshold = 100;
const int max_lowThreshold = 200;
const int ratio = 3;
const int kernel_size = 3;
const double scale=1;
const char* window_name = "Edge Map";
const int corner_threshold=100;
int main() {

    //std::string image_path = "/home/pratyush/Desktop/CV/frame.png";
    //std::string image_path = "/home/pratyush/Desktop/CV/tilt.png";
    std::string image_path = "/home/pratyush/Desktop/CV/pic.png";
    cv::Mat img,thresh,corn;
    img = cv::imread(image_path, CV_LOAD_IMAGE_ANYCOLOR);
    if(img.empty()) {
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
    cv::imshow("S channel", S);
    cv::Mat canny_output;
    cv::Canny( S, canny_output, lowThreshold, lowThreshold*ratio, kernel_size );
    cv::imshow("Canny Output", canny_output);
    // std::vector<std::vector<cv::Point> > contours;
    // cv::findContours(canny_output, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    cv::Mat final;
    //cv::threshold(tmp,thr,100,255,THRESH_BINARY_INV);

    std::vector< std::vector <cv::Point> > contours; // Vector for storing contour
    std::vector< cv::Vec4i > hierarchy;
    cv::findContours( canny_output, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
    double maxArea=0;
    int maxAreaContourId = 0;
    for( int i = 0; i< contours.size(); i++ ) // iterate through each contour.
    {
        int newArea=cv::contourArea(contours[i]);
        // Rect r= boundingRect(contours[i]);
        // if(hierarchy[i][2]<0) //Check if there is a child contour
        //   rectangle(src,Point(r.x-10,r.y-10), Point(r.x+r.width+10,r.y+r.height+10), Scalar(0,0,255),2,8,0); //Opened contour
        // else
        //   rectangle(src,Point(r.x-10,r.y-10), Point(r.x+r.width+10,r.y+r.height+10), Scalar(0,255,0),2,8,0); //closed contour
        if (newArea > maxArea) {
            maxArea = newArea;
            maxAreaContourId = i;
        } 
    }
    
    for (int j = 0; j < contours.size(); j++) {
        double newArea = cv::contourArea(contours.at(j));
        if (newArea > maxArea) {
            maxArea = newArea;
            maxAreaContourId = j;
        } // End if
    } 

    for(int i=0; i<contours.size(); ++i)
    {
        if(cv::contourArea(contours[i])<1)
            continue;
       //cv::drawContours(canny_output, contours, i, 0, -1);
        if(cv::isContourConvex(contours[i])!=false)
        {
            cv::RotatedRect rotatedRect = cv::minAreaRect(contours[maxAreaContourId]);
            cv::Point2f center = rotatedRect.center; 
            cv::circle( canny_output,center, 5,  cv::Scalar(255), 2, 8, 0 );
        }
    }
    final.create( S.size(), S.type() );
    final = cv::Scalar::all(0);
    cv::drawContours(final, contours, maxAreaContourId, 255, -1);

    std::vector<cv::Vec4i> linesP; // will hold the results of the detection
    cv::HoughLinesP(final, linesP, 1, CV_PI/180, 50, 50, 10 ); // runs the actual detection

    int xtot=0,ytot=0,vec_len=linesP.size();
    for( size_t i = 0; i < vec_len; i++ )
    {
        xtot+=(linesP[i])[0]+(linesP[i])[2];
        ytot+=(linesP[i])[1]+(linesP[i])[3];
    }
    xtot=xtot/(2*vec_len);
    ytot=ytot/(2*vec_len);
    cv::circle( canny_output,cv::Point(xtot,ytot), 5,  cv::Scalar(255), 2, 8, 0 );
    cv::imshow("Contour", final);
    cv::imshow("Edges with center", canny_output);
    cv::waitKey(0);
    cv::destroyAllWindows();
    return 0;
}