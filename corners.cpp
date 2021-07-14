#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <numeric>
int lowThreshold = 80;
const int max_lowThreshold = 200;
const int ratio = 3;
const int kernel_size = 3;
const double scale=1.5;
const char* window_name = "Edge Map";
const int corner_threshold=100;
int main() {

    //std::string image_path = "/home/pratyush/Desktop/cv2/frame.png";
    //std::string image_path = "/home/pratyush/Desktop/cv2/tilt.png";
    std::string image_path = "/home/pratyush/Desktop/cv2/img.png";
    cv::Mat img,thresh,corn;
    img = cv::imread(image_path, CV_LOAD_IMAGE_ANYCOLOR);
    if(img.empty()) {
        std::cout << "Could not read the image: " << image_path << std::endl;
        return 1;
    }
    cv::resize(img, img, cv::Size(img.cols*scale, img.rows*scale));
    cv::imshow("Original", img);
    cv::Mat channels[3];
    cv::split(img, channels);
    cv::GaussianBlur(img, img, cv::Size(15,15), 0, 0);
    cv::Mat grey,dst,detected_edges;
    cv::cvtColor(img, grey, CV_BGR2GRAY);
    dst.create( img.size(), img.type() );
    cv::fastNlMeansDenoising(grey,grey,3,7,21);
    cv::imshow("Denoised", grey);
   
    cv::Mat canny_output;
    cv::Canny( grey, canny_output, lowThreshold, lowThreshold*ratio, kernel_size );
    cv::imshow("Canny Ouptput", canny_output);
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(canny_output, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    double maxArea=0;
    int maxAreaContourId = 0;
    for (int j = 0; j < contours.size(); j++) {
        double newArea = cv::contourArea(contours.at(j));
        if (newArea > maxArea) {
            maxArea = newArea;
            maxAreaContourId = j;
        } // End if
    } 

    // for(int i=0; i<contours.size(); ++i)
    // {
    //     if(cv::contourArea(contours[i])<1)
    //         continue;
    //    //cv::drawContours(canny_output, contours, i, 0, -1);
    //     if(cv::isContourConvex(contours[i])!=false)
    //     {
            cv::RotatedRect rotatedRect = cv::minAreaRect(contours[maxAreaContourId]);
            cv::Point2f center = rotatedRect.center; 
            cv::circle( canny_output,center, 5,  cv::Scalar(255), 2, 8, 0 );
    //     }
    // }
    cv::imshow("Edges with center", canny_output);
    cv::waitKey(0);
    cv::destroyAllWindows();
    return 0;
}