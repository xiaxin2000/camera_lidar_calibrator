/*
 * camera_lidar_calibration.cpp
 *
 *  Created on: Sep 4, 2017
 *      Author: amc-jp
 */

#include <string>
#include <vector>
#include <ctime>
#include <sstream>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <boost/filesystem.hpp>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#if (CV_MAJOR_VERSION == 3)
  #include <opencv2/imgcodecs.hpp>
#else
  #include <opencv2/contrib/contrib.hpp>
#endif

#define __APP_NAME__ "autoware_camera_lidar_calibration_node"

std::ofstream image_points_file,lidar_points_file,results_normal_file,results_ransac_file;
std::string path_calibration_filename_str,path_calibration2_filename_str,path_camera_filename_str,path_lidar_filename_str;
int index_save_results=120;

class ROSCameraLidarApp

{
  ros::NodeHandle   node_handle_;

  ros::Subscriber   subscriber_image_raw_;
  ros::Subscriber   subscriber_points_raw_;
  ros::Subscriber     subscriber_intrinsics_;
  ros::Subscriber     subscriber_clicked_point_;
  ros::Subscriber     subscriber_image_point_;

  cv::Size            image_size_;
  cv::Mat             camera_instrinsics_;
  cv::Mat             distortion_coefficients_;

  cv::Mat             lidar_camera_rotation_matrix_;
  cv::Mat             lidar_camera_translation_vector_;

  std::vector<cv::Point2f> clicked_image_points_;
  std::vector<cv::Point3f> clicked_velodyne_points_;

  std::vector<cv::Point3f> euler_angle_normal_vec_,euler_angle_ransac_vec_;
  std::vector<cv::Point3f> translation_normal_vec_,translation_ransac_vec_;


  bool is_rotation_matrix(const cv::Mat& in_matrix)
  {
    cv::Mat rotation_matrix;
    transpose(in_matrix, rotation_matrix);

    cv::Mat test_matrix = rotation_matrix * in_matrix;
    cv::Mat test_result = cv::Mat::eye(3,3, test_matrix.type());

    return cv::norm(test_result, test_matrix) < 1e-6;

  }

  cv::Point3f get_rpy_from_matrix(const cv::Mat& in_rotation)
  {

    assert(is_rotation_matrix(in_rotation));

    float sy = sqrt(in_rotation.at<double>(0,0) * in_rotation.at<double>(0,0) +
                        in_rotation.at<double>(1,0) * in_rotation.at<double>(1,0) );

    bool is_singular = sy < 1e-6;

    float x, y, z;
    if (!is_singular)
    {
      x = atan2(in_rotation.at<double>(2,1), in_rotation.at<double>(2,2));
      y = atan2(-in_rotation.at<double>(2,0), sy);
      z = atan2(in_rotation.at<double>(1,0), in_rotation.at<double>(0,0));
    }
    else
    {
      x = atan2(-in_rotation.at<double>(1,2), in_rotation.at<double>(1,1));
      y = atan2(-in_rotation.at<double>(2,0), sy);
      z = 0;
    }
    return cv::Point3f(RAD2DEG(x), RAD2DEG(y), RAD2DEG(z));
  }

  void SaveCalibrationFile_All(std::vector<cv::Point3f> euler_angle_vec, std::vector<cv::Point3f> translation_vec, int method)
  {
    const char *homedir;

    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer,sizeof(buffer),"%Y%m%d_%H%M%S",timeinfo);
    std::string datetime_str(buffer);

    if ((homedir = getenv("HOME")) == NULL) {
      homedir = getpwuid(getuid())->pw_dir;
    }

    if (method==1)//1 means normal
    {
          // std::string path_calibration_filename_str,path_camera_filename_str,path_lidar_filename_str;
        path_calibration_filename_str = std::string(homedir) + "/"+datetime_str+"_results_normal_file.csv";

        // std::ofstream image_points_file,lidar_points_file,results_file;

        results_normal_file.open(path_calibration_filename_str,std::ios::out);

        for (int i = 0; i < euler_angle_vec.size(); ++i)
        {
          results_normal_file<<euler_angle_vec[i].x <<","<< euler_angle_vec[i].y <<" ,"<< euler_angle_vec[i].z <<std::endl;
          results_normal_file<<translation_vec[i].x <<","<< translation_vec[i].y <<" ,"<< translation_vec[i].z <<std::endl;
        }
    }else if (method==2)//2 means ransac
    {
        // std::string path_calibration_filename_str,path_camera_filename_str,path_lidar_filename_str;
        path_calibration_filename_str = std::string(homedir) + "/"+datetime_str+"_results_ransac_file.csv";

        // std::ofstream image_points_file,lidar_points_file,results_file;

        results_ransac_file.open(path_calibration_filename_str,std::ios::out);

        for (int i = 0; i < euler_angle_vec.size(); ++i)
        {
          results_ransac_file<<euler_angle_vec[i].x <<","<< euler_angle_vec[i].y <<" ,"<< euler_angle_vec[i].z <<std::endl;
          results_ransac_file<<translation_vec[i].x <<","<< translation_vec[i].y <<" ,"<< translation_vec[i].z <<std::endl;
        }
    }


  }

  void SaveFile_ImageCoordinate()
  {
    const char *homedir;

    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer,sizeof(buffer),"%Y%m%d_%H%M%S",timeinfo);
    std::string datetime_str(buffer);

    if ((homedir = getenv("HOME")) == NULL) {
      homedir = getpwuid(getuid())->pw_dir;
    }

    // std::string path_calibration_filename_str,path_camera_filename_str,path_lidar_filename_str;
    path_camera_filename_str = std::string(homedir) + "/"+datetime_str+"_camera_file.csv";

    // std::ofstream image_points_file,lidar_points_file,results_file;

    image_points_file.open(path_camera_filename_str,std::ios::out);

    for (int i = 0; i < clicked_image_points_.size(); ++i)
    {
      image_points_file<<clicked_image_points_[i].x <<","<< clicked_image_points_[i].y <<std::endl;
    }
  }

  void SaveFile_LiDARCoordinate()
  {
    const char *homedir;

    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer,sizeof(buffer),"%Y%m%d_%H%M%S",timeinfo);
    std::string datetime_str(buffer);

    if ((homedir = getenv("HOME")) == NULL) {
      homedir = getpwuid(getuid())->pw_dir;
    }

    // std::string path_calibration_filename_str,path_camera_filename_str,path_lidar_filename_str;
    path_lidar_filename_str = std::string(homedir) + "/"+datetime_str+"_lidar_file.csv";

    // std::ofstream image_points_file,lidar_points_file,results_file;

    lidar_points_file.open(path_lidar_filename_str,std::ios::out);

    for (int i = 0; i < clicked_velodyne_points_.size(); ++i)
    {
      lidar_points_file<<clicked_velodyne_points_[i].x <<","<< clicked_velodyne_points_[i].y<<","<< clicked_velodyne_points_[i].z <<std::endl;
    }
  }

  void SaveCalibrationFile(cv::Mat in_extrinsic, cv::Mat in_intrinsic, cv::Mat in_dist_coeff, cv::Size in_size, std::string method)
  {
    std::string path_filename_str;
    const char *homedir;

    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer,sizeof(buffer),"%Y%m%d_%H%M%S",timeinfo);
    std::string datetime_str(buffer);

    if ((homedir = getenv("HOME")) == NULL) {
      homedir = getpwuid(getuid())->pw_dir;
    }

    path_filename_str = std::string(homedir) + "/"+datetime_str+method+"_autoware_lidar_camera_calibration.yaml";

    cv::FileStorage fs(path_filename_str.c_str(), cv::FileStorage::WRITE);
    if(!fs.isOpened()){
      fprintf(stderr, "%s : cannot open file\n", path_filename_str.c_str());
      exit(EXIT_FAILURE);
    }

    fs << "CameraExtrinsicMat" << in_extrinsic;
    fs << "CameraMat" << in_intrinsic;
    fs << "DistCoeff" << in_dist_coeff;
    fs << "ImageSize" << in_size;
    fs << "ReprojectionError" << 0;
    fs << "DistModel" << "plumb_bob";

    ROS_INFO("Wrote Autoware Calibration file in: %s", path_filename_str.c_str());
  }

  void CalibrateSensors()
  {
    std::cout << "Number of points: " << clicked_velodyne_points_.size() << ":"  << clicked_image_points_.size() << std::endl;

    if (clicked_velodyne_points_.size() == clicked_image_points_.size()&& clicked_image_points_.size() > 9)
    {
      cv::Mat rotation_vector, translation_vector;

      cv::solvePnP(clicked_velodyne_points_,
                   clicked_image_points_,
                   camera_instrinsics_,
                   distortion_coefficients_,
                   rotation_vector,
                   translation_vector,
                   true,
                   CV_EPNP
      );

      cv::Mat rotation_matrix;
      cv::Rodrigues(rotation_vector, rotation_matrix);

      cv::Mat camera_velodyne_rotation = rotation_matrix.t();
      cv::Point3f camera_velodyne_point(translation_vector);
      cv::Point3f camera_velodyne_translation;

      camera_velodyne_translation.x = -camera_velodyne_point.z;
      camera_velodyne_translation.y = camera_velodyne_point.x;
      camera_velodyne_translation.z = camera_velodyne_point.y;

      std::cout << "Rotation:" << camera_velodyne_rotation << std::endl << std::endl;
      std::cout << "Translation:" << camera_velodyne_translation << std::endl;
      std::cout << "RPY: " << get_rpy_from_matrix(camera_velodyne_rotation) << std::endl << std::endl;

      euler_angle_normal_vec_.push_back(get_rpy_from_matrix(camera_velodyne_rotation));
      translation_normal_vec_.push_back(camera_velodyne_translation);

      cv::Mat extrinsics = cv::Mat::eye(4,4, CV_64F);
      camera_velodyne_rotation.copyTo(extrinsics(cv::Rect_<float>(0,0,3,3)));

      std::cout << extrinsics << std::endl;

      extrinsics.at<double>(0,3) = camera_velodyne_translation.x;
      extrinsics.at<double>(1,3) = camera_velodyne_translation.y;
      extrinsics.at<double>(2,3) = camera_velodyne_translation.z;

      std::cout << extrinsics << std::endl;

      std::string method="solvePnP";

      if (clicked_image_points_.size() > index_save_results)
      {
        SaveCalibrationFile(extrinsics ,camera_instrinsics_, distortion_coefficients_, image_size_,method);
        SaveCalibrationFile_All(euler_angle_normal_vec_, translation_normal_vec_, 1);//1 is normal
        SaveFile_ImageCoordinate();
        SaveFile_LiDARCoordinate();
      }

      
    }
  }

  void CalibrateSensors_ransac()
  {
    std::cout << "Ransac Number of points: " << clicked_velodyne_points_.size() << ":"  << clicked_image_points_.size() << std::endl;

    if (clicked_velodyne_points_.size() == clicked_image_points_.size()
        && clicked_image_points_.size() > 9)
    {
      cv::Mat rotation_vector= cv::Mat::zeros(3, 1, CV_32FC1);
      cv::Mat translation_vector= cv::Mat::zeros(3, 1, CV_32FC1); 

      int iterationsCount = 1000;      // number of Ransac iterations.
      float reprojectionError = 5.991;  // maximum allowed distance to consider it an inlier.
      double confidence = 0.95;        // ransac successful confidence.
      cv::Mat inliers;

      bool b = cv::solvePnPRansac(clicked_velodyne_points_, 
                                  clicked_image_points_, 
                                  camera_instrinsics_, 
                                  distortion_coefficients_, 
                                  rotation_vector, 
                                  translation_vector, 
                                  false, 
                                  iterationsCount, 
                                  reprojectionError, 
                                  confidence, 
                                  inliers, 
                                  CV_ITERATIVE); 

      cv::Mat rotation_matrix;
      cv::Rodrigues(rotation_vector, rotation_matrix);

      cv::Mat camera_velodyne_rotation = rotation_matrix.t();
      cv::Point3f camera_velodyne_point(translation_vector);
      cv::Point3f camera_velodyne_translation;

      camera_velodyne_translation.x = -camera_velodyne_point.z;
      camera_velodyne_translation.y = camera_velodyne_point.x;
      camera_velodyne_translation.z = camera_velodyne_point.y;

      std::cout << "Ransac Rotation:" << camera_velodyne_rotation << std::endl << std::endl;
      std::cout << "Ransac Translation:" << camera_velodyne_translation << std::endl;
      std::cout << "Ransac RPY: " << get_rpy_from_matrix(camera_velodyne_rotation) << std::endl << std::endl;

      euler_angle_ransac_vec_.push_back(get_rpy_from_matrix(camera_velodyne_rotation));
      translation_ransac_vec_.push_back(camera_velodyne_translation);

      cv::Mat extrinsics = cv::Mat::eye(4,4, CV_64F);
      camera_velodyne_rotation.copyTo(extrinsics(cv::Rect_<float>(0,0,3,3)));

      std::cout << extrinsics << std::endl;

      extrinsics.at<double>(0,3) = camera_velodyne_translation.x;
      extrinsics.at<double>(1,3) = camera_velodyne_translation.y;
      extrinsics.at<double>(2,3) = camera_velodyne_translation.z;

      std::cout << extrinsics << std::endl;

      std::string method="solvePnPRansac";

      if (clicked_image_points_.size() > index_save_results)
      {
        SaveCalibrationFile(extrinsics ,camera_instrinsics_, distortion_coefficients_, image_size_,method);
        SaveCalibrationFile_All(euler_angle_ransac_vec_, translation_ransac_vec_, 2);//2 is ransac
      }

    }
  }

  void RvizClickedPointCallback(const geometry_msgs::PointStamped& in_clicked_point)
  {
    clicked_velodyne_points_.push_back(cv::Point3f(in_clicked_point.point.x,
                                                   in_clicked_point.point.y,
                                                   in_clicked_point.point.z));
    std::cout << cv::Point3f(in_clicked_point.point.x,
                             in_clicked_point.point.y,
                             in_clicked_point.point.z) << std::endl << std::endl;
    CalibrateSensors();
    CalibrateSensors_ransac();
  }
  void ImageClickedPointCallback(const geometry_msgs::PointStamped& in_clicked_point)
  {

    clicked_image_points_.push_back(cv::Point2f(in_clicked_point.point.x,
                                                   in_clicked_point.point.y));

    std::cout << cv::Point2f(in_clicked_point.point.x,
                             in_clicked_point.point.y) << std::endl << std::endl;

    CalibrateSensors();
    CalibrateSensors_ransac();
  }

  void ImageCallback(const sensor_msgs::Image& in_image_sensor)
  {

    if (camera_instrinsics_.empty() || distortion_coefficients_.empty())
    {
      ROS_INFO("[%s] No Camera intrinsics loaded. Make sure to publish and set correctly camera_info source topic.", __APP_NAME__);
      return;
    }
  }

  void IntrinsicsCallback(const sensor_msgs::CameraInfo& in_message)
  {
    if (camera_instrinsics_.empty() || distortion_coefficients_.empty())
    {
      image_size_.height = in_message.height;
      image_size_.width = in_message.width;

      camera_instrinsics_ = cv::Mat(3, 3, CV_64F);
      for (int row = 0; row < 3; row++)
      {
        for (int col = 0; col < 3; col++)
        {
          camera_instrinsics_.at<double>(row, col) = in_message.K[row * 3 + col];
        }
      }

      distortion_coefficients_ = cv::Mat(1, 5, CV_64F);
      for (int col = 0; col < 5; col++)
      {
        distortion_coefficients_.at<double>(col) = in_message.D[col];
      }
      ROS_INFO("[%s] Camera Intrinsics Loaded", __APP_NAME__);
    }
  }

public:
  void Run()
  {
    ros::NodeHandle private_node_handle("~");//to receive args

    std::string image_raw_topic_str, points_raw_topic_str, clusters_topic_str, camera_info_topic_str;
    std::string name_space_str = ros::this_node::getNamespace();

    private_node_handle.param<std::string>("image_src", image_raw_topic_str, "/image_rectified");
    private_node_handle.param<std::string>("camera_info_src", camera_info_topic_str, "camera_info");

    if (name_space_str != "/")
    {
      if (name_space_str.substr(0, 2) == "//")
      {
        name_space_str.erase(name_space_str.begin());
      }
      image_raw_topic_str = name_space_str + image_raw_topic_str;
      camera_info_topic_str = name_space_str + camera_info_topic_str;
    }

    ROS_INFO("[%s] image_src: %s",__APP_NAME__, image_raw_topic_str.c_str());
    ROS_INFO("[%s] camera_info_src: %s",__APP_NAME__, camera_info_topic_str.c_str());


    ROS_INFO("[%s] Subscribing to... %s",__APP_NAME__, image_raw_topic_str.c_str());
    subscriber_image_raw_ = node_handle_.subscribe(image_raw_topic_str, 1, &ROSCameraLidarApp::ImageCallback, this);


    ROS_INFO("[%s] Subscribing to... %s",__APP_NAME__, camera_info_topic_str.c_str());
    subscriber_intrinsics_ = node_handle_.subscribe(camera_info_topic_str, 1, &ROSCameraLidarApp::IntrinsicsCallback, this);

    ROS_INFO("[%s] Subscribing to PointCloud ClickedPoint from RVIZ... /clicked_point",__APP_NAME__);
    subscriber_clicked_point_ = node_handle_.subscribe("/clicked_point", 1, &ROSCameraLidarApp::RvizClickedPointCallback, this);

    ROS_INFO("[%s] Subscribing to Image ClickedPoint from JSK ImageView2... %s/screenpoint",__APP_NAME__, image_raw_topic_str.c_str());
    subscriber_image_point_ = node_handle_.subscribe(image_raw_topic_str+"/screenpoint", 1, &ROSCameraLidarApp::ImageClickedPointCallback, this);
    ROS_INFO("[%s] ClickedPoint: %s",__APP_NAME__, (image_raw_topic_str+"/screenpoint").c_str());

    ROS_INFO("[%s] Ready. Waiting for data...",__APP_NAME__);
    ros::spin();
    ROS_INFO("[%s] END",__APP_NAME__);
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, __APP_NAME__);

  ROSCameraLidarApp app;

  app.Run();

  return 0;
}
