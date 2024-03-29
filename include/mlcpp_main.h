#ifndef MLCPP_MAIN_H
#define MLCPP_MAIN_H


#include "utilities.h"

///// C++
#include <algorithm>
#include <signal.h>
#include <string>
#include <sstream>
#include <math.h>
#include <chrono>


///// Eigen, Linear Algebra
#include <Eigen/Eigen> //whole Eigen library

///// OpenCV
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

///// ROS
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

///// PCL
//defaults
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
//conversions
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
//voxel, filters, etc
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
//normal
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>


using namespace std;



//// main class
class mlcpp_class{
  public:
    ///// basic params
    bool m_cam_init=false, m_pcd_load=false, m_pre_process=false;
    bool m_debug_mode=false;
    string m_infile;
    vector<double> m_cam_intrinsic;

    ///// MLCPP params
    double m_max_dist = 15.0;
    double m_max_angle = 60.0;
    double m_view_pt_dist = 10.0; //from points
    double m_view_pt_each_dist = 2.0; //between each viewpoints
    double m_view_overlap = 0.1; //overlap bet two viewpoints
    double m_slice_height = 8.0;
    int m_TSP_trial = 100;
    ///// MLCPP variables
    image_geometry::PinholeCameraModel m_camera_model;
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> m_normal_estimator;
    pcl::PointXYZ m_pcd_center_point;
    pcl::PointCloud<pcl::PointXYZ> m_cloud_map, m_cloud_center, m_cloud_none_viewed;
    pcl::PointCloud<pcl::PointXYZ> m_cloud_initial_view_point, m_optimized_view_point;
    pcl::PointCloud<pcl::PointNormal> m_cloud_normals;
    geometry_msgs::PoseArray m_normal_pose_array;
    nav_msgs::Path m_all_layer_path;

    ///// ROS
    ros::NodeHandle m_nh;
    ros::Subscriber m_path_calc_sub;
    ros::Publisher m_cloud_map_pub, m_cloud_center_pub, m_cloud_none_viewed_pub;
    ros::Publisher m_initial_view_point_pub, m_optimized_view_point_pub;
    ros::Publisher m_cloud_normal_pub, m_all_layer_path_pub;
    ros::Timer m_visualizing_timer;

    ///// Functions    
    //ROS
    void calc_cb(const std_msgs::Empty::ConstPtr& msg);
    void visualizer_timer_func(const ros::TimerEvent& event);
    //init
    void cam_init();
    void load_pcd();
    void preprocess_pcd();
    //others
    bool check_cam_in(Eigen::VectorXd view_point_xyzpy,pcl::PointXYZ point,pcl::Normal normal);
    void flip_normal(pcl::PointXYZ base,pcl::PointXYZ center,float & nx,float & ny, float & nz);
    Eigen::Matrix3d RPYtoR(double roll, double pitch, double yaw);
    void TwoOptSwap(pcl::PointCloud<pcl::PointNormal>::Ptr pclarray, int start, int finish);
    double PclArrayCost(pcl::PointCloud<pcl::PointNormal>::Ptr pclarray, double &distance);
    double TwoOptTSP(pcl::PointCloud<pcl::PointNormal>::Ptr pclarray);
    void OrdreringTSP(pcl::PointCloud<pcl::PointNormal>::Ptr pclarray);

    //constructor
    mlcpp_class(const ros::NodeHandle& n);
    ~mlcpp_class(){};
};









////////////////// can be separated into .cpp file
///// class constructor
mlcpp_class::mlcpp_class(const ros::NodeHandle& n) : m_nh(n){
  // params
  m_nh.param<string>("/infile", m_infile, "resource/1875935000.pcd");
  m_nh.param<bool>("/debug_mode", m_debug_mode, false);
  m_nh.getParam("/cam_intrinsic", m_cam_intrinsic);
  m_nh.param("/slice_height", m_slice_height, 8.0);
  m_nh.param("/max_dist", m_max_dist, 15.0);
  m_nh.param("/max_angle", m_max_angle, 60.0);
  m_nh.param("/view_pt_dist", m_view_pt_dist, 10.0);
  m_nh.param("/view_pt_each_dist", m_view_pt_each_dist, 2.0);
  m_nh.param("/view_overlap", m_view_overlap, 0.1);
  m_nh.param("/TSP_trial", m_TSP_trial, 100);

  //sub
  m_path_calc_sub = m_nh.subscribe<std_msgs::Empty>("/calculate_cpp", 3, &mlcpp_class::calc_cb, this);

  //pub
  m_cloud_map_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/pcl_map", 3);
  m_cloud_center_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/pcl_center", 3);
  m_cloud_none_viewed_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/none_viewed_pcl", 3);
  m_initial_view_point_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/initial_viewpoints", 3);
  m_optimized_view_point_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/optimized_viewpoints", 3);
  m_cloud_normal_pub = m_nh.advertise<geometry_msgs::PoseArray>("/pcl_normals", 3);
  m_all_layer_path_pub = m_nh.advertise<nav_msgs::Path>("/mlcpp_path", 3);

  //timer
  m_visualizing_timer = m_nh.createTimer(ros::Duration(1/5.0), &mlcpp_class::visualizer_timer_func, this);

  //init
  cam_init(); //Set camera parameter
  load_pcd(); //Get Map from pcd
  preprocess_pcd(); //Preprocess pcd: ground removal, normal estimation
}



///// functions
void mlcpp_class::cam_init(){
  std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
  //Set camera parameter
  sensor_msgs::CameraInfo cam_info;
  cam_info.width = (int)m_cam_intrinsic[0];
  cam_info.height = (int)m_cam_intrinsic[1];
  cam_info.distortion_model = "plumb_bob";
  boost::array<double,9> array_K;
  array_K[0] = m_cam_intrinsic[2];array_K[1] = 0.0;array_K[2] = m_cam_intrinsic[4];
  array_K[3] = 0.0;array_K[4] = m_cam_intrinsic[3];array_K[5] = m_cam_intrinsic[5];
  array_K[6] = 0;array_K[7] = 0;array_K[8] = 1.0;
  boost::array<double,12> array_P;
  array_P[0] = m_cam_intrinsic[2];array_P[1] = 0.0;array_P[2] = m_cam_intrinsic[4]; array_P[3] = 0.0;
  array_P[4] = 0.0;array_P[5] = m_cam_intrinsic[3];array_P[6] = m_cam_intrinsic[5]; array_P[7] = 0.0;
  array_P[8] = 0;array_P[9] = 0;array_P[10] = 1.0; array_P[11] = 0.0;
  cam_info.K = array_K;
  cam_info.P = array_P;
  m_camera_model.fromCameraInfo(cam_info);

  std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
  double duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1e3;
  ROS_WARN("Camera info processed in %.3f [ms]", duration);
  m_cam_init = true;
}

void mlcpp_class::load_pcd(){
  std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
  m_cloud_map.clear();
  m_cloud_center.clear();
  m_cloud_none_viewed.clear();
  m_cloud_initial_view_point.clear();
  m_optimized_view_point.clear();

  ROS_INFO("loading %s", m_infile.c_str());
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (m_infile.c_str (), m_cloud_map) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read pcd file \n");
    return;
  }

  std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
  double duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1e3;

  ROS_WARN("Map successfully obtained from PCD in %.3f [ms]", duration);

  m_pcd_load = true;
}

void mlcpp_class::preprocess_pcd(){
  std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
  ////// Ground Eleminate
  ROS_INFO("Ground filtering Start!");
  //#pragma omp parallel for
  m_pcd_center_point.x = 0; m_pcd_center_point.y = 0; m_pcd_center_point.z = 0;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map_nogr(new pcl::PointCloud<pcl::PointXYZ>);
  for(size_t i = 0; i < m_cloud_map.points.size() ; ++i){
    pcl::PointXYZ point;
    point.x = m_cloud_map.points[i].x;
    point.y = m_cloud_map.points[i].y;
    point.z = m_cloud_map.points[i].z;
    if(point.z > 0.3){
      Eigen::Vector4d vec;
      vec<<point.x, point.y, point.z, 1;
      cloud_map_nogr->points.push_back(point);
      m_pcd_center_point.x += point.x;
      m_pcd_center_point.y += point.y;
      m_pcd_center_point.z += point.z;
    }
  }
  m_pcd_center_point.x = m_pcd_center_point.x / cloud_map_nogr->points.size();
  m_pcd_center_point.y = m_pcd_center_point.y / cloud_map_nogr->points.size();
  m_pcd_center_point.z = m_pcd_center_point.z / cloud_map_nogr->points.size();
  m_cloud_center.push_back(m_pcd_center_point);

  m_cloud_map.clear();
  m_cloud_map = *cloud_map_nogr;
  m_cloud_map.width = m_cloud_map.points.size();
  m_cloud_map.height = 1;
  ROS_INFO("Ground filtering Finished!");



  ////// Normal estimation
  ROS_INFO("Normal Estimation Start!");
  m_cloud_normals.clear();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  *cloud_in = m_cloud_map;
  m_normal_estimator.setInputCloud(cloud_in);
  m_normal_estimator.setSearchMethod (tree);
  //m_normal_estimator.setKSearch (20);
  m_normal_estimator.setRadiusSearch (4); //TODO, parameterlize
  m_normal_estimator.compute (*cloud_normals);
  for (size_t i=0; i<cloud_normals->points.size(); ++i)
  {
    flip_normal(cloud_in->points[i], m_pcd_center_point, cloud_normals->points[i].normal[0], cloud_normals->points[i].normal[1], cloud_normals->points[i].normal[2]);
    pcl::PointNormal temp_ptnorm;
    temp_ptnorm.x = cloud_in->points[i].x;
    temp_ptnorm.y = cloud_in->points[i].y;
    temp_ptnorm.z = cloud_in->points[i].z;
    temp_ptnorm.normal[0] = cloud_normals->points[i].normal[0];
    temp_ptnorm.normal[1] = cloud_normals->points[i].normal[1];
    temp_ptnorm.normal[2] = cloud_normals->points[i].normal[2];
    m_cloud_normals.push_back(temp_ptnorm);
  }
  m_cloud_normals.width = m_cloud_normals.points.size();
  m_cloud_normals.height = 1;
  ROS_INFO("Normal Estimation Finish!");
  ROS_INFO("Cloud size : %lu",cloud_in->points.size());
  ROS_INFO("Cloud Normal size : %lu",cloud_normals->points.size());
  
  m_normal_pose_array = pclnormal_to_posearray(m_cloud_normals);
  m_normal_pose_array.header.frame_id = "map";

  std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
  double duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1e3;
  ROS_WARN("PCL preprocessed in %.3f [ms]", duration);

  m_pre_process=true;	
}


void mlcpp_class::calc_cb(const std_msgs::Empty::ConstPtr& msg){
  std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
	if (m_cam_init && m_pcd_load && m_pre_process){
    m_cloud_initial_view_point.clear();
    m_cloud_none_viewed.clear();
    m_optimized_view_point.clear();
    m_all_layer_path.header.stamp = ros::Time::now();
    m_all_layer_path.header.frame_id = "map";
    m_all_layer_path.poses.clear();
    pcl::PointNormal prev_layer_endpoint;
    
    ////// calculate Coverage Path
    ///Make Initial viewpoint
    bool finish = false;
    int current_layer=1;
    pcl::PointNormal minpt;
    pcl::PointNormal maxpt;
    pcl::getMinMax3D(m_cloud_normals, minpt, maxpt);
    float minpt_z = minpt.z;
    ///PCL Slice with Z axis value (INITIAL)
    pcl::PointCloud<pcl::PointNormal>::Ptr Sliced_ptnorm(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_pt_normals(new pcl::PointCloud<pcl::PointNormal>);
    *cloud_pt_normals = m_cloud_normals;
    pcl::PassThrough<pcl::PointNormal> pass;
    pass.setInputCloud (cloud_pt_normals);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (minpt_z,minpt_z+m_slice_height);
    pass.filter (*Sliced_ptnorm);


    while(!finish)
    {
      if (minpt_z+m_slice_height >= maxpt.z){ // Check if last layer
        finish = true;
      }
      ///PCL make viewpoints by points and normals
      pcl::PointCloud<pcl::PointNormal>::Ptr viewpoint_ptnorm(new pcl::PointCloud<pcl::PointNormal>);
      viewpoint_ptnorm->clear();
      for(int i=0;i<Sliced_ptnorm->points.size();i++)
      {
        pcl::PointNormal temp_ptnorm;
        temp_ptnorm.x = Sliced_ptnorm->points[i].x + Sliced_ptnorm->points[i].normal[0] * m_view_pt_dist;
        temp_ptnorm.y = Sliced_ptnorm->points[i].y + Sliced_ptnorm->points[i].normal[1] * m_view_pt_dist;
        temp_ptnorm.z = Sliced_ptnorm->points[i].z + Sliced_ptnorm->points[i].normal[2] * m_view_pt_dist;
        if(temp_ptnorm.z <= 0 ) continue;
        temp_ptnorm.normal[0] = -Sliced_ptnorm->points[i].normal[0];
        temp_ptnorm.normal[1] = -Sliced_ptnorm->points[i].normal[1];
        temp_ptnorm.normal[2] = -Sliced_ptnorm->points[i].normal[2];
        viewpoint_ptnorm->push_back(temp_ptnorm);
      }
      ///PCL downsample viewpoints with VoxelGrid
      pcl::VoxelGrid<pcl::PointNormal> voxgrid;
      pcl::PointCloud<pcl::PointNormal>::Ptr Voxed_Sliced_Viewpt (new pcl::PointCloud<pcl::PointNormal>);
      Voxed_Sliced_Viewpt->clear();
      pcl::PointCloud<pcl::PointNormal>::Ptr Voxed_Sliced_Admitted_Viewpt (new pcl::PointCloud<pcl::PointNormal>);
      Voxed_Sliced_Admitted_Viewpt->clear();
      voxgrid.setInputCloud(viewpoint_ptnorm);
      voxgrid.setLeafSize(m_view_pt_each_dist,m_view_pt_each_dist,m_view_pt_each_dist*0.8);
      voxgrid.filter(*Voxed_Sliced_Viewpt);

      ROS_WARN("current layer %d, not viewed, voxelized initial viewpoints: %d", current_layer, Voxed_Sliced_Viewpt->points.size());
      for (int idx = 0; idx < Voxed_Sliced_Viewpt->points.size(); ++idx)
      {
        pcl::PointXYZ initial_view_pts;
        initial_view_pts.x = Voxed_Sliced_Viewpt->points[idx].x;
        initial_view_pts.y = Voxed_Sliced_Viewpt->points[idx].y;
        initial_view_pts.z = Voxed_Sliced_Viewpt->points[idx].z;
        m_cloud_initial_view_point.push_back(initial_view_pts);
      }

      pcl::PointCloud<pcl::PointNormal>::Ptr Sliced_ptnorm_Unview (new pcl::PointCloud<pcl::PointNormal>);
      Sliced_ptnorm_Unview->clear();
      pcl::copyPointCloud(*Sliced_ptnorm,*Sliced_ptnorm_Unview);
      ///PCL downsample viewpoints by view calculation
      int admitted=0;
      int a[Voxed_Sliced_Viewpt->points.size()];
      for(int i=0;i<Voxed_Sliced_Viewpt->points.size();i++){
        a[i]=i;
      }
      random_shuffle(&a[0], &a[Voxed_Sliced_Viewpt->points.size()]);
      for(int k=0;k<Voxed_Sliced_Viewpt->points.size();k++)
      {
        int i = a[k];
        vector<int> toerase;
        vector<int> view_comp_map;
        Eigen::VectorXd viewpt(5);
        viewpt << Voxed_Sliced_Viewpt->points[i].x,Voxed_Sliced_Viewpt->points[i].y,Voxed_Sliced_Viewpt->points[i].z,
                asin(-Voxed_Sliced_Viewpt->points[i].normal[2])/M_PI*180.0,
                asin(Voxed_Sliced_Viewpt->points[i].normal[1]/cos(-Voxed_Sliced_Viewpt->points[i].normal[2]))/M_PI*180.0;
        #pragma omp parallel for
        for(int j=0;j<Sliced_ptnorm_Unview->points.size();j++)
        {
          pcl::PointXYZ point_toview(Sliced_ptnorm_Unview->points[j].x,Sliced_ptnorm_Unview->points[j].y,
                                     Sliced_ptnorm_Unview->points[j].z);
          pcl::Normal point_normal(Sliced_ptnorm_Unview->points[j].normal[0],
                                   Sliced_ptnorm_Unview->points[j].normal[1],
                                   Sliced_ptnorm_Unview->points[j].normal[2]);
          if(check_cam_in(viewpt,point_toview,point_normal))
          {
            #pragma omp critical
            toerase.push_back(j);
          }
        }
        #pragma omp parallel for
        for (size_t j=0;j<Sliced_ptnorm->points.size();j++)
        {
          pcl::PointXYZ point_toview(Sliced_ptnorm->points[j].x,Sliced_ptnorm->points[j].y,
                                     Sliced_ptnorm->points[j].z);
          pcl::Normal point_normal(Sliced_ptnorm->points[j].normal[0],
                                   Sliced_ptnorm->points[j].normal[1],
                                   Sliced_ptnorm->points[j].normal[2]);
          if(check_cam_in(viewpt,point_toview,point_normal))
          {
            #pragma omp critical
            view_comp_map.push_back(j);
          }
        }
        if(view_comp_map.size()*m_view_overlap < toerase.size())
        {
          sort(toerase.begin(),toerase.end());
          for(int j=toerase.size()-1;j>-1;j--)
          {
            Sliced_ptnorm_Unview->points.erase(Sliced_ptnorm_Unview->points.begin()+toerase[j]);
          }
          if (m_debug_mode){
            ROS_INFO("%d Point Left", Sliced_ptnorm_Unview->points.size());
            ROS_INFO("Viewpoint %d / %d Admitted ", i, Voxed_Sliced_Viewpt->points.size());
          }
          admitted++;
          Voxed_Sliced_Admitted_Viewpt->push_back(Voxed_Sliced_Viewpt->points[i]);
        }
        //else cout<<"Viewpoint "<<i<<"/"<<Voxed_Sliced_Viewpt->points.size()<<" Not Admitted"<<endl;
      }
      if (m_debug_mode){
        ROS_INFO("Admitted Viewpoint: %d", admitted);
      }
      ROS_WARN("current layer %d, still none-viewed points: %d among %d in slice", current_layer, Sliced_ptnorm_Unview->points.size(), Sliced_ptnorm->points.size());
      for (int idx = 0; idx < Sliced_ptnorm_Unview->points.size(); ++idx)
      {
        pcl::PointXYZ none_view_pcl;
        none_view_pcl.x = Sliced_ptnorm_Unview->points[idx].x;
        none_view_pcl.y = Sliced_ptnorm_Unview->points[idx].y;
        none_view_pcl.z = Sliced_ptnorm_Unview->points[idx].z;
        m_cloud_none_viewed.push_back(none_view_pcl);
      }

      
      if (Voxed_Sliced_Admitted_Viewpt->points.size() > 0){
        /// Solve TSP among downsampled viewpoints
        OrdreringTSP(Voxed_Sliced_Admitted_Viewpt);
        double best_distance = TwoOptTSP(Voxed_Sliced_Admitted_Viewpt);

        if (current_layer>1){        
          Eigen::Vector3d current_first(Voxed_Sliced_Admitted_Viewpt->points[0].x, Voxed_Sliced_Admitted_Viewpt->points[0].y, Voxed_Sliced_Admitted_Viewpt->points[0].z);
          Eigen::Vector3d current_end(Voxed_Sliced_Admitted_Viewpt->points[Voxed_Sliced_Admitted_Viewpt->points.size()-1].x, Voxed_Sliced_Admitted_Viewpt->points[Voxed_Sliced_Admitted_Viewpt->points.size()-1].y, Voxed_Sliced_Admitted_Viewpt->points[Voxed_Sliced_Admitted_Viewpt->points.size()-1].z) ;
          Eigen::Vector3d prev_layer_endpoint_points(prev_layer_endpoint.x, prev_layer_endpoint.y, prev_layer_endpoint.z);
          Eigen::Vector3d prev_layer_endpoint_normal(prev_layer_endpoint.normal[0], prev_layer_endpoint.normal[1], prev_layer_endpoint.normal[2]);

          //// forward
          if ( fabs( (current_first-prev_layer_endpoint_points).normalized().dot(prev_layer_endpoint_normal.normalized()) ) < 
            fabs( (current_end-prev_layer_endpoint_points).normalized().dot(prev_layer_endpoint_normal.normalized()) ) )
          {
            for(int idx=0; idx<Voxed_Sliced_Admitted_Viewpt->points.size(); ++idx)
            {
              pcl::PointXYZ optimized_viewpt;
              optimized_viewpt.x = Voxed_Sliced_Admitted_Viewpt->points[idx].x;
              optimized_viewpt.y = Voxed_Sliced_Admitted_Viewpt->points[idx].y;
              optimized_viewpt.z = Voxed_Sliced_Admitted_Viewpt->points[idx].z;
              m_optimized_view_point.push_back(optimized_viewpt);
              m_all_layer_path.poses.push_back(single_pclnormal_to_posestamped(Voxed_Sliced_Admitted_Viewpt->points[idx]));
            }
            prev_layer_endpoint = Voxed_Sliced_Admitted_Viewpt->points[Voxed_Sliced_Admitted_Viewpt->points.size()-1];
          }
          //// reverse
          else
          {
            for(int idx=Voxed_Sliced_Admitted_Viewpt->points.size()-1; idx>=0; --idx)
            {
              pcl::PointXYZ optimized_viewpt;
              optimized_viewpt.x = Voxed_Sliced_Admitted_Viewpt->points[idx].x;
              optimized_viewpt.y = Voxed_Sliced_Admitted_Viewpt->points[idx].y;
              optimized_viewpt.z = Voxed_Sliced_Admitted_Viewpt->points[idx].z;
              m_optimized_view_point.push_back(optimized_viewpt);
              m_all_layer_path.poses.push_back(single_pclnormal_to_posestamped(Voxed_Sliced_Admitted_Viewpt->points[idx]));
            }
            prev_layer_endpoint = Voxed_Sliced_Admitted_Viewpt->points[0];
          }
        }
        else{
          for(int idx=0; idx<Voxed_Sliced_Admitted_Viewpt->points.size(); ++idx)
          {
            pcl::PointXYZ optimized_viewpt;
            optimized_viewpt.x = Voxed_Sliced_Admitted_Viewpt->points[idx].x;
            optimized_viewpt.y = Voxed_Sliced_Admitted_Viewpt->points[idx].y;
            optimized_viewpt.z = Voxed_Sliced_Admitted_Viewpt->points[idx].z;
            m_optimized_view_point.push_back(optimized_viewpt);
            m_all_layer_path.poses.push_back(single_pclnormal_to_posestamped(Voxed_Sliced_Admitted_Viewpt->points[idx]));
          }
          prev_layer_endpoint = Voxed_Sliced_Admitted_Viewpt->points[Voxed_Sliced_Admitted_Viewpt->points.size()-1];
        }
        ROS_WARN("current layer %d, TSP %d points, leng: %.2f m", current_layer, Voxed_Sliced_Admitted_Viewpt->points.size(), best_distance);
      }
      else {
        ROS_WARN("current layer %d, no admitted points, skipping", current_layer);
      }

      ///PCL Slice with Z axis value (untill maxpt.z)
      minpt_z += m_slice_height;
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (minpt_z,minpt_z+m_slice_height);
      pass.filter (*Sliced_ptnorm);
      current_layer++;
    } //while end
	} //if end
	else{
		ROS_WARN("One of cam info / PCD file loading / PCD pre-process has not been done yet");
	}

  std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
  double duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1e3;
  ROS_WARN("MLCPP calculation: %.3f [ms], path len: %.3f", duration, path_length(m_all_layer_path));
}

void mlcpp_class::visualizer_timer_func(const ros::TimerEvent& event){
  std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
	if (m_cam_init && m_pcd_load && m_pre_process){
		m_cloud_map_pub.publish(cloud2msg(m_cloud_map));
    m_cloud_center_pub.publish(cloud2msg(m_cloud_center));
    m_cloud_normal_pub.publish(m_normal_pose_array);
    m_initial_view_point_pub.publish(cloud2msg(m_cloud_initial_view_point));
    m_cloud_none_viewed_pub.publish(cloud2msg(m_cloud_none_viewed));
    m_optimized_view_point_pub.publish(cloud2msg(m_optimized_view_point));
    m_all_layer_path_pub.publish(m_all_layer_path);
	}

  if(m_debug_mode){
    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    double duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1e3;
    ROS_WARN("visualizing in %.2f [ms]", duration);
  }
}






///////// methods
//TODO: flip_normal not from center but from real view point, where recorded PCL for none-convex targets
void mlcpp_class::flip_normal(pcl::PointXYZ base, pcl::PointXYZ center, float & nx, float & ny, float & nz)
{
  float xdif = base.x - center.x;
  float ydif = base.y - center.y;
  if(xdif * nx + ydif * ny <0)
  {
    nx = -nx;
    ny = -ny;
    nz = -nz;
  }
}

bool mlcpp_class::check_cam_in(Eigen::VectorXd view_point_xyzpy,pcl::PointXYZ point,pcl::Normal normal)
{
  Eigen::Vector3d pt_bef_rot(point.x-view_point_xyzpy(0),point.y-view_point_xyzpy(1),point.z-view_point_xyzpy(2));
  Eigen::Vector3d pt_aft_rot = RPYtoR(0,-view_point_xyzpy(3),-view_point_xyzpy(4))*pt_bef_rot;
  Eigen::Vector4d pt_cvv(pt_aft_rot(0),pt_aft_rot(1),pt_aft_rot(2),1);
  Eigen::Matrix4d view_pt;
  view_pt.setIdentity();
  view_pt.block<3,3>(0,0) = RPYtoR(-90,0,-90);
  Eigen::Vector4d new_pt = view_pt.inverse() * pt_cvv;
  cv::Point3d pt_cv(new_pt(0), new_pt(1), new_pt(2));
  cv::Point2d uv;
  uv = m_camera_model.project3dToPixel(pt_cv);
  uv.x = floor(abs(uv.x)) * ((uv.x > 0) - (uv.x < 0));
  uv.y = floor(abs(uv.y)) * ((uv.y > 0) - (uv.y < 0));
  if(uv.x<0 || uv.x>m_cam_intrinsic[0] || uv.y<0 || uv.y>m_cam_intrinsic[1]) return false;
  float dist = sqrt(pow((view_point_xyzpy(0)-point.x),2)+pow((view_point_xyzpy(1)-point.y),2)+
                    pow((view_point_xyzpy(2)-point.z),2));
  if(dist>m_max_dist) return false;
  Eigen::Vector3d normal_pt(normal.normal_x,normal.normal_y,normal.normal_z);
  Eigen::Vector3d Normal_view_pt((view_point_xyzpy(0)-point.x)/dist,(view_point_xyzpy(1)-point.y)/dist,
                                    (view_point_xyzpy(2)-point.z)/dist);
  double inner_product = Normal_view_pt.dot(normal_pt);
  double angle = acos(inner_product)/M_PI*180.0;
  if(abs(angle)>m_max_angle) return false;
  return true;
}

Eigen::Matrix3d mlcpp_class::RPYtoR(double roll,double pitch,double yaw)
{
  Eigen::Matrix3d Rmatrix;
  Eigen::Matrix3d Rmatrix_y;
  Eigen::Matrix3d Rmatrix_p;
  Eigen::Matrix3d Rmatrix_r;
  yaw = yaw*M_PI/180.0;
  roll = roll*M_PI/180.0;
  pitch = pitch*M_PI/180.0;
  Rmatrix_y << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
  Rmatrix_p << cos(pitch), 0, sin(pitch), 0, 1, 0, -sin(pitch), 0, cos(pitch);
  Rmatrix_r << 1, 0, 0, 0, cos(roll), -sin(roll), 0, sin(roll), cos(roll);
  Rmatrix = Rmatrix_y * Rmatrix_p * Rmatrix_r;
  return Rmatrix;
}

//TWO OPT ALGORITHM
void mlcpp_class::TwoOptSwap(pcl::PointCloud<pcl::PointNormal>::Ptr pclarray,int start,int finish)
{
  int size = pclarray->points.size();
  pcl::PointCloud<pcl::PointNormal> temp_Array;
  for(int i=0;i<=start-1;i++) temp_Array.push_back(pclarray->points[i]);
  for(int i=finish;i>=start;i--) temp_Array.push_back(pclarray->points[i]);
  for(int i=finish+1;i<=size-1;i++) temp_Array.push_back(pclarray->points[i]);
  pcl::copyPointCloud(temp_Array,*pclarray);
}

double mlcpp_class::PclArrayCost(pcl::PointCloud<pcl::PointNormal>::Ptr pclarray, double &distance)
{
  double cost = 0;
  double distance_out = 0;
  for(int i=1;i<pclarray->points.size();i++)
  {
    Eigen::Vector3d current(pclarray->points[i-1].x, pclarray->points[i-1].y, pclarray->points[i-1].z);
    Eigen::Vector3d current_normal(pclarray->points[i-1].normal[0], pclarray->points[i-1].normal[1], pclarray->points[i-1].normal[2]);
    Eigen::Vector3d next(pclarray->points[i].x, pclarray->points[i].y, pclarray->points[i].z);
    Eigen::Vector3d direction_vector = next - current;
    double dist = direction_vector.norm();

    distance_out += dist;
    cost = cost + dist + dist*fabs(direction_vector.normalized().dot(current_normal.normalized())); // penalty on penetrating through pcd target
  }
  distance = distance_out;
  return cost;
}

double mlcpp_class::TwoOptTSP(pcl::PointCloud<pcl::PointNormal>::Ptr pclarray)
{
  pcl::PointCloud<pcl::PointNormal> temp_Array;
  pcl::copyPointCloud(*pclarray,temp_Array);
  int size = pclarray->points.size();
  int improve = 0;
  double best_distance = 0.0;
  double best_cost = PclArrayCost(pclarray, best_distance);
  if (m_debug_mode){
    ROS_INFO("Initial distance: %.2f", best_cost);
  }
  while (improve<m_TSP_trial)
  {
    for ( int i = 1; i <size - 2; i++ )
    {
      for ( int k = i + 1; k < size-2; k++)
      {
        TwoOptSwap( pclarray, i,k );
        double new_distance = 0.0;
        double new_cost = PclArrayCost(pclarray, new_distance);
        if ( new_cost < best_cost )
        {
          improve = 0;
          pcl::copyPointCloud(*pclarray,temp_Array);
          best_distance = new_distance;
          best_cost = new_cost;
        }
      }
    }
    improve ++;
  }
  pcl::copyPointCloud(temp_Array,*pclarray);
  if (m_debug_mode){
    ROS_INFO("Final distance: %.2f, cost: %.2f", best_distance, best_cost);
    ROS_INFO("TwoOptTSP Finished");
  }
  return best_distance;
}

void mlcpp_class::OrdreringTSP(pcl::PointCloud<pcl::PointNormal>::Ptr pclarray)
{
  pcl::PointCloud<pcl::PointNormal> temp_Array;
  pcl::PointNormal minpt;
  pcl::PointNormal maxpt;
  pcl::getMinMax3D(*pclarray,minpt,maxpt);
  // cout<<minpt.z<<"<-min,max->"<<maxpt.z<<endl;
  for(int i=0;i<pclarray->points.size();i++)
  {
    if(pclarray->points[i].z == minpt.z)
    {
      temp_Array.push_back(pclarray->points[i]);
      // cout<<pclarray->points[i].z<<endl;
      for(int j=0;j<pclarray->points.size();j++)
      {
        if(pclarray->points[j].z == maxpt.z)
        {
          // cout<<pclarray->points[j].z<<endl;
          for(int k=0;k<pclarray->points.size();k++)
          {
            if(k!=i && k!=j) temp_Array.push_back(pclarray->points[k]);
          }
          temp_Array.push_back(pclarray->points[j]);
          break;
        }
      }
      break;
    }
  }
  pcl::copyPointCloud(temp_Array,*pclarray);
}



#endif
