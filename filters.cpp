#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <numeric>
int lowThreshold = 10;
const int max_lowThreshold = 200;
const int ratio = 3;
const int kernel_size = 3;
const double scale=2.5;
const char* window_name = "Edge Map";
const int corner_threshold=100;
int main() {

    // std::string image_path = "/home/pratyush/Desktop/cv2/frame.png";
    std::string image_path = "/home/pratyush/Desktop/cv2/frame.png";
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
   
    cv::Canny( grey, grey, lowThreshold, lowThreshold*ratio, kernel_size );
    dst = cv::Scalar::all(0);
    detected_edges= cv::Scalar::all(0);
    cv::imshow("Greyscale", grey);
    //cv::fastNlMeansDenoising(grey,grey,3,7,21);
    cv::threshold( grey, thresh, 100,255,CV_THRESH_BINARY );
    cv::cornerHarris(thresh,dst,8,11,0.01);
    //cv::circle(grey, p, 5, cv::Scalar(128,0,0), -1);
    cv::normalize( dst, dst, 0, 255, CV_MINMAX, CV_32FC1, cv::Mat() );
    //corn.create( grey.size(), grey.type() );
    //BUG ALERT IN CV_MINMAX , IT WAS ORIGINALLY NORM_MINMAX
    cv::convertScaleAbs( dst, dst );
    std::vector<int> xs,ys;
    for( int i = 0; i < dst.rows ; i++ )
    {
        for( int j = 0; j < dst.cols; j++ )
        {
            if( (int) dst.at<float>(i,j) > corner_threshold )
            {
                xs.push_back(j);
                ys.push_back(i);
                cv::circle( grey, cv::Point(j,i), 5,  cv::Scalar(255), 2, 8, 0 );
            }
        }
    }
    cv::imshow("Image with corners",grey);
    int x_sum=std::accumulate(xs.begin(), xs.end(), 0);
    int y_sum=std::accumulate(ys.begin(), ys.end(), 0);
    int x_center=x_sum/xs.size();
    int y_center=y_sum/ys.size();
    cv::circle(grey,cv::Point(x_center,y_center),10,cv::Scalar(255),2,8,0);
    cv::imshow("Image with center",grey);
    cv::waitKey(0);
    cv::destroyAllWindows();
    return 0;
}