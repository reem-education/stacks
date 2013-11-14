/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Jordi Pages. */

// ROS headers
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

// OpenCV headers
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>

// Std C++ headers
#include <vector>

/**
 * @brief The PersonDetector class encapsulating an image subscriber and the OpenCV's CPU HOG person detector
 *
 * @example rosrun person_detector_opencv person_detector image:=/stereo/right/image
 *
 */
class PersonDetector
{
public:

  PersonDetector(ros::NodeHandle& nh);
  virtual ~PersonDetector();

protected:

  ros::NodeHandle _nh;

  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  void detectPersons(const cv::Mat& img,
                     std::vector<cv::Rect>& detections);

  void showDetections(cv::Mat& img,
                      const std::vector<cv::Rect>& detections);

  cv::HOGDescriptor _hog;

  image_transport::ImageTransport _imageTransport;
  image_transport::Subscriber _imageSub;

};

PersonDetector::PersonDetector(ros::NodeHandle& nh):
  _nh(nh),
  _imageTransport(nh)
{
  _hog.setSVMDetector( cv::HOGDescriptor::getDefaultPeopleDetector());

  image_transport::TransportHints transportHint("raw");

  _imageSub = _imageTransport.subscribe("image", 1, &PersonDetector::imageCallback, this, transportHint);

  cv::namedWindow("person detections");
}

PersonDetector::~PersonDetector()
{
  cv::destroyWindow("person detections");
}

void PersonDetector::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  std::cout << "detecting persons..." << std::endl;

  cv::Mat img;

  cv_bridge::CvImageConstPtr cvImgPtr;
  cvImgPtr = cv_bridge::toCvShare(msg);
  cvImgPtr->image.copyTo(img);

  cv::vector<cv::Rect> detections;

  detectPersons(img, detections);

  showDetections(img, detections);
}

void PersonDetector::detectPersons(const cv::Mat& img,
                                   std::vector<cv::Rect>& detections)
{
  _hog.detectMultiScale(img,
                        detections,
                        0,                //hit threshold: decrease in order to increase number of detections but also false alarms
                        cv::Size(8,8),    //win stride
                        cv::Size(24,16),  //padding
                        1.02,             //scaling
                        1,                //final threshold
                        false);            //use mean-shift to fuse detections
}

void PersonDetector::showDetections(cv::Mat& img,
                                    const std::vector<cv::Rect>& detections)
{
  for (unsigned int i = 0; i < detections.size(); ++i)
    cv::rectangle(img, detections[i], CV_RGB(0,255,0), 2);

  cv::imshow("person detections", img);
  cv::waitKey(15);
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"person_detector"); // Create and name the Node
  ros::NodeHandle nh;

  std::cout << "Creating person detector" << std::endl;

  PersonDetector detector(nh);

  std::cout << "Spinning to serve callbacks ..." << std::endl;

  ros::spin();

  return 0;
}
