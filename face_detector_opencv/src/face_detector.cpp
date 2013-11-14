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
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Std C++ headers
#include <vector>
#include <stdexcept>

/**
 * @brief The FaceDetector class encapsulating an image subscriber and the OpenCV's cascade classifier
 *        for face detection
 *
 * @example rosrun face_detector_opencv face_detector image:=/stereo/right/image
 *
 */
class FaceDetector
{
public:

  FaceDetector(ros::NodeHandle& nh);
  virtual ~FaceDetector();

protected:

  ros::NodeHandle _nh;

  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  void detectFaces(const cv::Mat& img,
                     std::vector<cv::Rect>& detections);

  void showDetections(cv::Mat& img,
                      const std::vector<cv::Rect>& detections);

  cv::CascadeClassifier _faceClassifier;

  image_transport::ImageTransport _imageTransport;
  image_transport::Subscriber _imageSub;

};

FaceDetector::FaceDetector(ros::NodeHandle& nh):
  _nh(nh),
  _imageTransport(nh)
{
  if ( !_faceClassifier.load("/opt/ros/fuerte/share/OpenCV/haarcascades/haarcascade_frontalface_alt.xml") )
    throw std::runtime_error("Error loading classifier");

  image_transport::TransportHints transportHint("raw");

  _imageSub = _imageTransport.subscribe("image", 1, &FaceDetector::imageCallback, this, transportHint);

  cv::namedWindow("face detections");
}

FaceDetector::~FaceDetector()
{
  cv::destroyWindow("face detections");
}

void FaceDetector::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  std::cout << "detecting faces..." << std::endl;

  cv::Mat img;

  cv_bridge::CvImageConstPtr cvImgPtr;
  cvImgPtr = cv_bridge::toCvShare(msg);
  cvImgPtr->image.copyTo(img);

  cv::vector<cv::Rect> detections;

  detectFaces(img, detections);

  showDetections(img, detections);
}

void FaceDetector::detectFaces(const cv::Mat& img,
                               std::vector<cv::Rect>& detections)
{
  cv::Mat imgGray;

  cv::cvtColor(img, imgGray, CV_BGR2GRAY);

  std::cout << "before multiscale" << std::endl;

  _faceClassifier.detectMultiScale(imgGray,
                                   detections,
                                   1.05,           //scale factor
                                   2,             //min neighbors
                                   cv::CASCADE_DO_CANNY_PRUNING | CV_HAAR_SCALE_IMAGE);
}

void FaceDetector::showDetections(cv::Mat& img,
                                  const std::vector<cv::Rect>& detections)
{
  for (unsigned int i = 0; i < detections.size(); ++i)
    cv::rectangle(img, detections[i], CV_RGB(0,255,0), 2);

  cv::imshow("face detections", img);
  cv::waitKey(15);
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"face_detector"); // Create and name the Node
  ros::NodeHandle nh;

  std::cout << "Creating face detector" << std::endl;

  FaceDetector detector(nh);

  std::cout << "Spinning to serve callbacks ..." << std::endl;

  ros::spin();

  return 0;
}
