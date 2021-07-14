#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

int main() {

    // LOADING AND DISPLAYING IMAGES
    std::string image_path = "/home/pratyush/Desktop/cv2/hot-air-balloon.jpg";
    cv::Mat img;
    img = cv::imread(image_path, CV_LOAD_IMAGE_ANYCOLOR);
    if(img.empty()) {
        std::cout << "Could not read the image: " << image_path << std::endl;
        return 1;
    }

    cv::imshow("Display Window", img);
    cv::waitKey(0);
    cv::destroyWindow("Display Window");

    // SIMPLE COPYING DOES NOT MEAN A NEW IMAGE IS CREATED. POINTS TO SAME IMAGE IN MEMORY

    // cv::Mat copy = img;
    // int thickness = 3;
    // cv::rectangle(copy, cv::Point(30,50), cv::Point(80,100), cv::Scalar(255,0,0), thickness, cv::LINE_8);


    // cv::imshow("change", img);
    // cv::waitKey(0);
    // cv::destroyWindow("change");

    // SPLITTING CHANNELS 
    cv::Mat channels[3];
    cv::split(img, channels);
    cv::imshow("Blue Channel", channels[0]);
    cv::imshow("Green Channel", channels[1]);
    cv::imshow("Red Channel", channels[2]);
    cv::waitKey(0);
    cv::destroyAllWindows();

    //Accessing pixel values 
    int x = 30, y = 50;
    cv::Vec3b intensity = img.at<cv::Vec3b>(y,x);
    int blue = intensity.val[0];
    int green = intensity.val[1];
    int red = intensity.val[2];

    std::cout<<"blue :"<<blue<<" green : "<<green<<" red : "<<red<<std::endl;


    return 0;
}