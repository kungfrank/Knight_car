#include <iostream>
#include <ros/ros.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
using namespace cv;
using namespace std;
void MatchingMethod( int, void* ); 
 Mat imgOriginal;
 Mat imgHSV;
 Mat imgLines;
 int iLowH = 0;
 int iHighH =22;

 int iLowS = 169;
 int iHighS = 255;

 int iLowV = 181;
 int iHighV = 255;

 int iLastX = -1; 
 int iLastY = -1;
 
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
        imgOriginal = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
        if (imgLines.empty()){
            imgLines = Mat::zeros( imgOriginal.size(), CV_8UC3 );;
        }
        MatchingMethod(0,0);
}

int main(int argc, char **argv)
{
    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
    namedWindow("Original", CV_WINDOW_AUTOSIZE); //create a window called "Control"
    namedWindow("Thresholded Image", CV_WINDOW_AUTOSIZE); //create a window called "Control"
 //Create trackbars in "Control" window
 createTrackbar("LowH", "Control", &iLowH, 179, MatchingMethod); //Hue (0 - 179)
createTrackbar("HighH", "Control", &iHighH, 179, MatchingMethod);

 createTrackbar("LowS", "Control", &iLowS, 255, MatchingMethod); //Saturation (0 - 255)
 createTrackbar("HighS", "Control", &iHighS, 255, MatchingMethod);

 createTrackbar("LowV", "Control", &iLowV, 255, MatchingMethod); //Value (0 - 255)
 createTrackbar("HighV", "Control", &iHighV, 255, MatchingMethod);

    cv::startWindowThread();
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/image_raw", 1, imageCallback);
    ros::spin();

  return 0;


 }


/**
 * @function MatchingMethod
 * @brief Trackbar callback
 */
void MatchingMethod( int, void* )
{
 Mat imgThresholded;

  inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
      
  //morphological opening (remove small objects from the foreground)
  erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

  //morphological closing (fill small holes in the foreground)
  dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
  erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

Moments oMoments = moments(imgThresholded);

  double dM01 = oMoments.m01;
  double dM10 = oMoments.m10;
  double dArea = oMoments.m00;
// if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
  if (dArea > 10000)
  {
   //calculate the position of the ball
   int posX = dM10 / dArea;
   int posY = dM01 / dArea;        
        
   if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0)
   {
    //Draw a red line from the previous point to the current point
    line(imgLines, Point(posX, posY), Point(iLastX, iLastY), Scalar(0,0,255), 2);
   }

   iLastX = posX;
   iLastY = posY;
  rectangle( imgOriginal, Point(posX, posY), Point( posX + 20 , posY + 20 ), Scalar::all(0), 2, 8, 0 );
  }

  imshow("Thresholded Image", imgThresholded); //show the thresholded image

  imgOriginal = imgOriginal + imgLines;
  imshow("Original", imgOriginal); //show the original image
  



}

