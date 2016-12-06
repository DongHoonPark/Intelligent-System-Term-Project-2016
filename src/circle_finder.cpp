//
// Created by donghoonpark on 16. 12. 5.
//

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <pwd.h>
#include <unistd.h>

using namespace cv;

/** @function main */
int main(int argc, char** argv)
{
  Mat map, src_gray;
// Load Map
    auto userpw = getpwuid(getuid());
//    char* user = getlogin();
    char* user = userpw->pw_name;
  /// Read the image

 map = imread((std::string("/home/")+
                      std::string(user)+
                      std::string("/catkin_ws/src/project4/src/ground_truth_map_sin1.pgm")).c_str(), CV_LOAD_IMAGE_GRAYSCALE);
//  if( !src.data )
//    { return -1; }

  /// Convert it to gray
  //cvtColor( src, src_gray, CV_BGR2GRAY );

  /// Reduce the noise so we avoid false circle detection
//  cv::GaussianBlur( src, src, Size(3, 3), 2, 2 );
//
//  std::vector<Vec3f> circles;
//
//  /// Apply the Hough Transform to find the circles
//  cv::HoughCircles( src, circles, CV_HOUGH_GRADIENT, 1, 8, 50, 15, 0, 10 );
//
//  /// Draw the circles detected
//  for( size_t i = 0; i < circles.size(); i++ )
//  {
//      cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
//      int radius = cvRound(circles[i][2]);
//      // circle center
//      circle( src, center, 3, Scalar(0,255,0), -1, 8, 0 );
//      // circle outline
//      circle( src, center, radius, Scalar(0,0,255), 3, 8, 0 );
//   }
//
//  /// Show your results
//  namedWindow( "Hough Circle Transform Demo", CV_WINDOW_AUTOSIZE );
//  imshow( "Hough Circle Transform Demo", src );
//
//  waitKey(0);
//  return 0;

    auto dynamic_map = map.clone();
    cv::Mat dynamic_map_circledetection = dynamic_map.clone();
    cv::Mat dynamic_map_linedetection = dynamic_map.clone();

    cv::GaussianBlur(dynamic_map, dynamic_map_circledetection, cv::Size(3, 3), 2, 2 );


    std::vector<cv::Vec3f> circles;
    cv::HoughCircles( dynamic_map_circledetection, circles, CV_HOUGH_GRADIENT, 1, 8, 50, 15, 0, 10 );


    for( size_t i = 0; i < circles.size(); i++ )
    {
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));

        // circle center
        cv::circle( dynamic_map, center, 10, cv::Scalar(255,255,255), CV_FILLED);
    }

    cv::Canny(dynamic_map, dynamic_map_linedetection, 200, 1200, 3);
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(dynamic_map_linedetection, lines, 1, CV_PI/180, 40, 20, 10 );
    for( size_t i = 0; i < lines.size(); i++ )
    {
        cv::Point pt1, pt2;
        pt1.x = lines[i][0];
        pt1.y = lines[i][1];
        pt2.x = lines[i][2];
        pt2.y = lines[i][3];

        cv::line( dynamic_map, pt1, pt2, cv::Scalar(0,0,0),20);
    }

  namedWindow( "Hough Circle Transform Demo", CV_WINDOW_AUTOSIZE );
  imshow( "Hough Circle Transform Demo", dynamic_map );
    waitKey(0);
}