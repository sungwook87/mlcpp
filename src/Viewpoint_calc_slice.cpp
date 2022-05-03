#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <signal.h>
#include <termios.h>
#include <string>
#include <sstream>
#include <math.h>
#include <3dclass.h>
#include <stdafx.h>
#include <Eigen/Dense>
#include <Eigen/LU>

#define PI 3.141592654
#define max_dist 15
#define max_angle 60
#define viept_dist 10 //from points
#define viewpt_each_dist 2 //between each viewpoints
#define view_overlap 0.1 //overlap bet two viewpoints
#define z_slice 8

using namespace std;
struct termios cooked, raw;
int kfd = 0;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map_nogr(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
pcl::PointCloud<pcl::PointNormal>::Ptr cloud_pt_normals(new pcl::PointCloud<pcl::PointNormal>);
pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_est;
pcl::PointXYZ center_point;
vector<Eigen::VectorXd> Viewpoints;
vector<vector<Eigen::VectorXd> > Viewpoints_seq;


//pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
vector<OctoPoint> OtctoPoints;
Eigen::Matrix4Xd Pts_Matrix(4,1);

image_geometry::PinholeCameraModel CAM;

void fc_mapin(const sensor_msgs::PointCloud2ConstPtr& input);
bool get_map = false;
void imageCb(const sensor_msgs::CameraInfoConstPtr& info_msg);
void flip_normal(pcl::PointXYZ base,pcl::PointXYZ center,float & nx,float & ny, float & nz);
void view_point(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,bool spin);
void view_point(pcl::PointCloud<pcl::PointNormal>::Ptr cloud,bool spin);
Eigen::Matrix3d RPYtoR(double roll,double pitch,double yaw);
pcl::visualization::PCLVisualizer viewer ("MAP");
bool check_cam_in(Eigen::VectorXd point_xyzpy,pcl::PointXYZ point,pcl::Normal normal);
float MAPSCALER = 110; //Max height
float BOTTOMSCALER = 0;
pcl::PointCloud<pcl::PointXYZ>::Ptr Non_view(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr View(new pcl::PointCloud<pcl::PointXYZ>);
sensor_msgs::CameraInfo cam_info;
static Eigen::VectorXd viewpoint(5);

void TwoOptSwap(pcl::PointCloud<pcl::PointNormal>::Ptr pclarray,int start,int finish);
double PclArrayCost(pcl::PointCloud<pcl::PointNormal>::Ptr pclarray);
void TwoOptTSP(pcl::PointCloud<pcl::PointNormal>::Ptr pclarray);
void OrdreringTSP(pcl::PointCloud<pcl::PointNormal>::Ptr pclarray);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Viewpoint_calc");
    ros::NodeHandle nh;
    ros::Publisher pb_mapused = nh.advertise<sensor_msgs::PointCloud2>("/viewpoint/mapused",10);
    ros::Subscriber sb_mapin = nh.subscribe("/viewpoint/mapin",10,fc_mapin);
    /* Camera Parameter Setting */
    ros::Subscriber sb_cam_info =
            nh.subscribe("/front_cam/camera/camera_info", 1, imageCb);
    viewer.setBackgroundColor (0, 0, 0);
    viewer.setCameraPosition (100, 80, 100, -50, -10, 0,0,0,2);
    viewer.setSize (1280, 1024);  // Visualiser window size
    string infile = argv[1];
    char c;
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
    while(ros::ok())
    {
        read(kfd, &c, 1);
        switch(c)
        {
            case 'z': //Set camera parameter
                cout<<"Camera info processed"<<endl;
                cam_info.height = 240;
                cam_info.width = 320;
                cam_info.distortion_model = "plumb_bob";
                boost::array<double,9> array_K;
                array_K[0] = 159.99941228826285;array_K[1] = 0.0;array_K[2] = 160.5;
                array_K[3] = 0.0;array_K[4] = 159.99941228826285;array_K[5] = 120.5;
                array_K[6] = 0;array_K[7] = 0;array_K[8] = 1.0;
                boost::array<double,12> array_P;
                array_P[0] = 159.99941228826285;array_P[1] = 0.0;array_P[2] = 160.5;array_P[3] = 0.0;
                array_P[4] = 0.0;array_P[5] = 159.99941228826285;array_P[6] = 120.5;array_P[7] = 0.0;
                array_P[8] = 0;array_P[9] = 0;array_P[10] = 1.0; array_P[11] = 0.0;
                cam_info.K = array_K;
                cam_info.P = array_P;
                CAM.fromCameraInfo(cam_info);
                break;
            case 'a': //get Map from sensor msgs
                std::cout<<"Get Map from sensor msg!"<<std::endl;
                get_map = true;
                break;
            case 'b': //get Map from pcd
                if (pcl::io::loadPCDFile<pcl::PointXYZ> (infile.c_str (), *cloud_map) == -1) //* load the file
                {
                  PCL_ERROR ("Couldn't read pcd file \n");
                  return (-1);
                }
                printf("Map successfully obtained from PCD!\n");
                for(size_t i = 0; i < cloud_map->points.size() ; ++i){
                    center_point.x += cloud_map->points[i].x;
                    center_point.y += cloud_map->points[i].y;
                    center_point.z += cloud_map->points[i].z;
                    //Pts_Matrix.col(Pts_Matrix.
                }
                center_point.x = center_point.x / cloud_map->points.size();
                center_point.y = center_point.y / cloud_map->points.size();
                center_point.z = center_point.z / cloud_map->points.size();
                break;
            case 'c': //Ground Eleminate
                printf("Ground filtering Start!\n");
//#pragma omp parallel for
                center_point.x = 0; center_point.y = 0; center_point.z = 0;
                for(size_t i = 0; i < cloud_map->points.size() ; ++i){
                  pcl::PointXYZ point;
                  point.x = cloud_map->points[i].x;
                  point.y = cloud_map->points[i].y;
                  point.z = cloud_map->points[i].z;
                  if(point.z > 0.3)
                  {
                      Eigen::Vector4d vec;
                      vec<<point.x, point.y, point.z, 1;
                      cloud_map_nogr->points.push_back(point);
                      Eigen::Matrix4Xd tmp(4,Pts_Matrix.cols()+1);
                      tmp << Pts_Matrix, vec;
                      Pts_Matrix = tmp;
                      center_point.x += point.x;
                      center_point.y += point.y;
                      center_point.z += point.z;
                      //Pts_Matrix.col(Pts_Matrix.cols()-1) = vec;
                      //Pts_Matrix.conservativeResize(Pts_Matrix.rows()+1, Eigen::NoChange);
                  }
                }
                center_point.x = center_point.x / cloud_map_nogr->points.size();
                center_point.y = center_point.y / cloud_map_nogr->points.size();
                center_point.z = center_point.z / cloud_map_nogr->points.size();
                //cout<<Pts_Matrix<<endl;
                cloud_map = cloud_map_nogr;
                cloud_map->width = cloud_map->points.size();
                cloud_map->height = 1;
                pcl::io::savePCDFile("noground.pcd",*cloud_map);
                printf("Ground filtering Finished!\n");
                break;
            case 'd': //Map now
                view_point(cloud_map,true);
                printf("View Finish!\n");
                break;
            case 'e':
                printf("Normal Estimation Start!\n");
                normal_est.setInputCloud(cloud_map);
                static pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
                normal_est.setSearchMethod (tree);
                //normal_est.setKSearch (20);
                normal_est.setRadiusSearch (4);
                normal_est.compute (*cloud_normals);
        //#pragma omp parallel for
                for (size_t i=0; i<cloud_normals->points.size(); ++i)
                {
                    flip_normal(cloud_map->points[i],center_point,cloud_normals->points[i].normal[0],  cloud_normals->points[i].normal[1],cloud_normals->points[i].normal[2]);
                    pcl::PointNormal temp_ptnorm;
                    temp_ptnorm.x = cloud_map->points[i].x;
                    temp_ptnorm.y = cloud_map->points[i].y;
                    temp_ptnorm.z = cloud_map->points[i].z;
                    temp_ptnorm.normal[0] = cloud_normals->points[i].normal[0];
                    temp_ptnorm.normal[1] = cloud_normals->points[i].normal[1];
                    temp_ptnorm.normal[2] = cloud_normals->points[i].normal[2];
                    cloud_pt_normals->push_back(temp_ptnorm);
                    //pcl::flipNormalTowardsViewpoint( cloud_map->points[i], center_point.x,center_point.y,center_point.z,cloud_normals->points[i].normal[0],  cloud_normals->points[i].normal[1],cloud_normals->points[i].normal[2]);
                }
                cloud_pt_normals->width = cloud_pt_normals->points.size();
                cloud_pt_normals->height = 1;
                printf("Normal Estimation Finish!\n");
                printf("Cloud size : %lu\n",cloud_map->points.size());
                printf("Cloud Normal size : %lu\n",cloud_normals->points.size());
                viewer.removeAllPointClouds();
                view_point(cloud_map,false);
                viewer.addSphere (center_point, 1, 0, 0, 1, "sphere");
                viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud_map, cloud_normals, 10, 1, "normals");
                viewer.spin();
                viewer.removeAllShapes();
                break;
            case 'g': //Make Initial viewpoint
                bool finish = false;
                pcl::PointNormal minpt;
                pcl::PointNormal maxpt;
                pcl::getMinMax3D(*cloud_pt_normals,minpt,maxpt);
                float minpt_z = minpt.z;
                //PCL SLICE WITH Z AXIS (INITIAL)
                static pcl::PointCloud<pcl::PointNormal>::Ptr Sliced_ptnorm(new pcl::PointCloud<pcl::PointNormal>);
                pcl::PassThrough<pcl::PointNormal> pass;
                pass.setInputCloud (cloud_pt_normals);
                pass.setFilterFieldName ("z");
                pass.setFilterLimits (minpt_z,minpt_z+z_slice);
                pass.filter (*Sliced_ptnorm);

                while(!finish)
                {
                    //PCL MAKE VIEWPOINTS BY POINT, NORMAL
                    static pcl::PointCloud<pcl::PointNormal>::Ptr viewpoint_ptnorm(new pcl::PointCloud<pcl::PointNormal>);
                    viewpoint_ptnorm->clear();
                    for(int i=0;i<Sliced_ptnorm->points.size();i++)
                    {
                        pcl::PointNormal temp_ptnorm;
                        temp_ptnorm.x = Sliced_ptnorm->points[i].x + Sliced_ptnorm->points[i].normal[0] * viept_dist;
                        temp_ptnorm.y = Sliced_ptnorm->points[i].y + Sliced_ptnorm->points[i].normal[1] * viept_dist;
                        temp_ptnorm.z = Sliced_ptnorm->points[i].z + Sliced_ptnorm->points[i].normal[2] * viept_dist;
                        if(temp_ptnorm.z <= 0 ) continue;
                        temp_ptnorm.normal[0] = -Sliced_ptnorm->points[i].normal[0];
                        temp_ptnorm.normal[1] = -Sliced_ptnorm->points[i].normal[1];
                        temp_ptnorm.normal[2] = -Sliced_ptnorm->points[i].normal[2];
                        viewpoint_ptnorm->push_back(temp_ptnorm);
                    }
                    //PCL DOWNSAMPLE VIEWPOINTS BY VOXELGRID
                    pcl::VoxelGrid<pcl::PointNormal> voxgrid;
                    static pcl::PointCloud<pcl::PointNormal>::Ptr Voxed_Sliced_Viewpt (new pcl::PointCloud<pcl::PointNormal>);
                    Voxed_Sliced_Viewpt->clear();
                    static pcl::PointCloud<pcl::PointNormal>::Ptr Voxed_Sliced_Admitted_Viewpt (new pcl::PointCloud<pcl::PointNormal>);
                    Voxed_Sliced_Admitted_Viewpt->clear();
                    voxgrid.setInputCloud(viewpoint_ptnorm);
                    voxgrid.setLeafSize(viewpt_each_dist,viewpt_each_dist,viewpt_each_dist*0.8);
                    voxgrid.filter(*Voxed_Sliced_Viewpt);
                    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> color(255, 255, 255);
                    viewer.removeAllPointClouds();
                    viewer.removeAllShapes();
                    cout<<Voxed_Sliced_Viewpt->points.size()<<endl;
                    view_point(cloud_map,false);
                    viewer.addPointCloud<pcl::PointNormal> (Voxed_Sliced_Viewpt, color, "sample cloud_Nonview");
                    viewer.spin();
                    static pcl::PointCloud<pcl::PointNormal>::Ptr Sliced_ptnorm_Unview (new pcl::PointCloud<pcl::PointNormal>);
                    Sliced_ptnorm_Unview->clear();
                    pcl::copyPointCloud(*Sliced_ptnorm,*Sliced_ptnorm_Unview);
                    //PCL DOWNSAMPLE VIEWPOINTS BY VIEWS
                    int admitted=0;
                    int a[Voxed_Sliced_Viewpt->points.size()];
                    for(int i=0;i<Voxed_Sliced_Viewpt->points.size();i++){
                        a[i]=i;
                    }
                    random_shuffle(&a[0], &a[Voxed_Sliced_Viewpt->points.size()]);
                    //for(int i=0;i<Voxed_Sliced_Viewpt->points.size();i++)
                    for(int k=0;k<Voxed_Sliced_Viewpt->points.size();k++)
                    {
                        int i = a[k];
                        vector<int> toerase;
                        vector<int> view_comp_map;
                        static Eigen::VectorXd viewpt(5);
                        viewpt = Eigen::VectorXd(5);
                        viewpt << Voxed_Sliced_Viewpt->points[i].x,Voxed_Sliced_Viewpt->points[i].y,Voxed_Sliced_Viewpt->points[i].z,
                                asin(-Voxed_Sliced_Viewpt->points[i].normal[2])/PI*180,
                                asin(Voxed_Sliced_Viewpt->points[i].normal[1]/cos(-Voxed_Sliced_Viewpt->points[i].normal[2]))/PI*180;
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
                        if(view_comp_map.size()*view_overlap < toerase.size())
                        {
                            sort(toerase.begin(),toerase.end());
                            for(int j=toerase.size()-1;j>-1;j--)
                            {
                                Sliced_ptnorm_Unview->points.erase(Sliced_ptnorm_Unview->points.begin()+toerase[j]);
                            }
                            cout<<Sliced_ptnorm_Unview->points.size()<<"Point Left"<<endl;
                            admitted++;
                            std::stringstream ss;
                            ss<<i<<"sphere";
                            cout<<"Viewpoint "<<i<<"/"<<Voxed_Sliced_Viewpt->points.size()<<" Admitted"<<endl;
                            Voxed_Sliced_Admitted_Viewpt->push_back(Voxed_Sliced_Viewpt->points[i]);
                            viewer.addSphere (pcl::PointXYZ(viewpt(0),viewpt(1),viewpt(2)), 0.3, 0, 0, 1, ss.str());
                        }
                        //else cout<<"Viewpoint "<<i<<"/"<<Voxed_Sliced_Viewpt->points.size()<<" Not Admitted"<<endl;
                    }
                    cout<<"Admitted Viewpoint : "<<admitted<<endl;
                    cout<<"None Viewed points : "<<Sliced_ptnorm_Unview->points.size()<<"/"<<Sliced_ptnorm->points.size()<<endl;
                    view_point(Sliced_ptnorm_Unview,false);
                    viewer.spin();
                    OrdreringTSP(Voxed_Sliced_Admitted_Viewpt);
                    viewer.removeAllShapes();
                    viewer.addSphere(Voxed_Sliced_Admitted_Viewpt->points[0],0.2,0,0,1,"Start");
                    for(int lineiter=1;lineiter<Voxed_Sliced_Admitted_Viewpt->points.size();lineiter++)
                    {
                        std::stringstream ss1;
                        ss1<<lineiter<<"sphere";
                        if(lineiter == Voxed_Sliced_Admitted_Viewpt->points.size()-1) viewer.addSphere(Voxed_Sliced_Admitted_Viewpt->points[lineiter],0.2,0,0.5,0.5,"Finish");
                        else viewer.addSphere(Voxed_Sliced_Admitted_Viewpt->points[lineiter],0.2,0,1,0,ss1.str());
                    }
                    viewer.spin();
                    TwoOptTSP(Voxed_Sliced_Admitted_Viewpt);
                    viewer.removeAllShapes();
                    view_point(cloud_map,false);
                    viewer.addSphere(Voxed_Sliced_Admitted_Viewpt->points[0],0.2,0,0,1,"Start");
                    for(int lineiter=1;lineiter<Voxed_Sliced_Admitted_Viewpt->points.size();lineiter++)
                    {
                        std::stringstream ss;
                        ss<<lineiter<<"line";
                        std::stringstream ss1;
                        ss1<<lineiter<<"sphere";
                        viewer.addLine<pcl::PointNormal> (Voxed_Sliced_Admitted_Viewpt->points[lineiter-1],
                                                             Voxed_Sliced_Admitted_Viewpt->points[lineiter], ss.str());
                        if(lineiter == Voxed_Sliced_Admitted_Viewpt->points.size()-1) viewer.addSphere(Voxed_Sliced_Admitted_Viewpt->points[lineiter],0.2,0,0.5,0.5,"Finish");
                        else viewer.addSphere(Voxed_Sliced_Admitted_Viewpt->points[lineiter],0.2,0,1,0,ss1.str());
                    }
                    viewer.spin();
                    /*
                    minpt_z += z_slice;
                    pass.setFilterFieldName ("z");
                    pass.setFilterLimits (minpt_z,minpt_z+z_slice);
                    pass.filter (*Sliced_ptnorm);
                    */
                    finish = true;
                }
                break;
        }
        ros::spinOnce();
    }
}

void fc_mapin(const sensor_msgs::PointCloud2ConstPtr& input)
{
    if(get_map)
    {
        pcl::PCLPointCloud2 pcl_pc;
        pcl_conversions::toPCL(*input, pcl_pc);
        pcl::fromPCLPointCloud2(pcl_pc,*cloud_map);
        printf("Map successfully obtained from MSG!\n");
        get_map = false;
    }
}

void view_point(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,bool spin)
{
    viewer.removeAllPointClouds();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color (new pcl::PointCloud<pcl::PointXYZRGB>);
    for(size_t i = 0; i < cloud->points.size() ; ++i){
      pcl::PointXYZRGB point;
      point.x = cloud->points[i].x;
      point.y = cloud->points[i].y;
      point.z = cloud->points[i].z;
      int zindex = max(0,min(63,(int)((point.z + BOTTOMSCALER) * (60 / MAPSCALER))));
      uint8_t r((int)(hsv[zindex][0]*255)), g((int)(hsv[zindex][1]*255)), b((int)(hsv[zindex][2]*255));
      uint32_t rgb = ( r <<16 | g << 8 | b  );
      point.rgb = *reinterpret_cast<float*>(&rgb);
      cloud_color->points.push_back(point);
    }
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_color);
    viewer.addPointCloud<pcl::PointXYZRGB>(cloud_color,rgb);
    if (spin) viewer.spin();
}
void view_point(pcl::PointCloud<pcl::PointNormal>::Ptr cloud,bool spin)
{
    viewer.removeAllPointClouds();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color (new pcl::PointCloud<pcl::PointXYZRGB>);
    for(size_t i = 0; i < cloud->points.size() ; ++i){
      pcl::PointXYZRGB point;
      point.x = cloud->points[i].x;
      point.y = cloud->points[i].y;
      point.z = cloud->points[i].z;
      int zindex = max(0,min(63,(int)((point.z + BOTTOMSCALER) * (60 / MAPSCALER))));
      uint8_t r((int)(hsv[zindex][0]*255)), g((int)(hsv[zindex][1]*255)), b((int)(hsv[zindex][2]*255));
      uint32_t rgb = ( r <<16 | g << 8 | b  );
      point.rgb = *reinterpret_cast<float*>(&rgb);
      cloud_color->points.push_back(point);
    }
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_color);
    viewer.addPointCloud<pcl::PointXYZRGB>(cloud_color,rgb);
    if (spin) viewer.spin();
}

void flip_normal(pcl::PointXYZ base,pcl::PointXYZ center,float & nx,float & ny, float & nz)
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

bool check_cam_in(Eigen::VectorXd point_xyzpy,pcl::PointXYZ point,pcl::Normal normal)
{
    Eigen::Vector3d pt_bef_rot(point.x-point_xyzpy(0),point.y-point_xyzpy(1),point.z-point_xyzpy(2));
    Eigen::Vector3d pt_aft_rot = RPYtoR(0,-point_xyzpy(3),-point_xyzpy(4))*pt_bef_rot;
    Eigen::Vector4d pt_cvv(pt_aft_rot(0),pt_aft_rot(1),pt_aft_rot(2),1);
    Eigen::Matrix4d view_pt;
    view_pt.setIdentity();
    view_pt.block<3,3>(0,0) = RPYtoR(-90,0,-90);
    Eigen::Vector4d new_pt = view_pt.inverse() * pt_cvv;
    cv::Point3d pt_cv(new_pt(0), new_pt(1), new_pt(2));
    cv::Point2d uv;
    uv = CAM.project3dToPixel(pt_cv);
    uv.x = floor(abs(uv.x)) * ((uv.x > 0) - (uv.x < 0));
    uv.y = floor(abs(uv.y)) * ((uv.y > 0) - (uv.y < 0));
    if(uv.x<0 || uv.x>320 || uv.y<0 || uv.y>240) return false;
    float dist = sqrt(pow((point_xyzpy(0)-point.x),2)+pow((point_xyzpy(1)-point.y),2)+
                      pow((point_xyzpy(2)-point.z),2));
    if(dist>max_dist) return false;
    Eigen::Vector3d normal_pt(normal.normal_x,normal.normal_y,normal.normal_z);
    Eigen::Vector3d Normal_view_pt((point_xyzpy(0)-point.x)/dist,(point_xyzpy(1)-point.y)/dist,
                                      (point_xyzpy(2)-point.z)/dist);
    double inner_product = Normal_view_pt.dot(normal_pt);
    double angle = acos(inner_product)/PI*180;
    if(abs(angle)>max_angle) return false;
    return true;
}

Eigen::Matrix3d RPYtoR(double roll,double pitch,double yaw)
{
    Eigen::Matrix3d Rmatrix;
    Eigen::Matrix3d Rmatrix_y;
    Eigen::Matrix3d Rmatrix_p;
    Eigen::Matrix3d Rmatrix_r;
    yaw = yaw*PI/180;
    roll = roll*PI/180;
    pitch = pitch*PI/180;
    Rmatrix_y << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
    Rmatrix_p << cos(pitch), 0, sin(pitch), 0, 1, 0, -sin(pitch), 0, cos(pitch);
    Rmatrix_r << 1, 0, 0, 0, cos(roll), -sin(roll), 0, sin(roll), cos(roll);
    Rmatrix = Rmatrix_y * Rmatrix_p * Rmatrix_r;
    return Rmatrix;
}
void imageCb(const sensor_msgs::CameraInfoConstPtr& info_msg)
{
    CAM.fromCameraInfo(info_msg);
}

//TWO OPT ALGORITHM
void TwoOptSwap(pcl::PointCloud<pcl::PointNormal>::Ptr pclarray,int start,int finish)
{
    int size = pclarray->points.size();
    pcl::PointCloud<pcl::PointNormal> temp_Array;
    for(int i=0;i<=start-1;i++) temp_Array.push_back(pclarray->points[i]);
    for(int i=finish;i>=start;i--) temp_Array.push_back(pclarray->points[i]);
    for(int i=finish+1;i<=size-1;i++) temp_Array.push_back(pclarray->points[i]);
    pcl::copyPointCloud(temp_Array,*pclarray);
}

double PclArrayCost(pcl::PointCloud<pcl::PointNormal>::Ptr pclarray)
{
    double cost = 0;
    for(int i=1;i<pclarray->points.size();i++)
    {
        double x_dist = pclarray->points[i].x -  pclarray->points[i-1].x;
        double y_dist = pclarray->points[i].y -  pclarray->points[i-1].y;
        double z_dist = pclarray->points[i].z -  pclarray->points[i-1].z;
        cost += sqrt( x_dist*x_dist + y_dist*y_dist + z_dist*z_dist);
    }
    return cost;
}

void TwoOptTSP(pcl::PointCloud<pcl::PointNormal>::Ptr pclarray)
{
    pcl::PointCloud<pcl::PointNormal> temp_Array;
    int size = pclarray->points.size();
    int improve = 0;
    double best_distance = PclArrayCost(pclarray);
    cout<<"Distance : "<<best_distance<<endl;
    while (improve<300)
    {
        for ( int i = 1; i <size - 2; i++ )
        {
            for ( int k = i + 1; k < size-2; k++)
            {
                TwoOptSwap( pclarray, i,k );
                double new_distance = PclArrayCost(pclarray);
                //cout<<"Distance : "<<new_distance;
                if ( new_distance < best_distance )
                {
                    improve = 0;
                    pcl::copyPointCloud(*pclarray,temp_Array);
                    viewer.removeAllShapes();
                    viewer.addSphere(temp_Array.points[0],0.2,0,0,1,"Start");
                    for(int lineiter=1;lineiter<temp_Array.points.size();lineiter++)
                    {
                        std::stringstream ss;
                        ss<<lineiter<<"line";
                        std::stringstream ss1;
                        ss1<<lineiter<<"sphere";
                        viewer.addLine<pcl::PointNormal> (temp_Array.points[lineiter-1],
                                                             temp_Array.points[lineiter], ss.str());
                        if(lineiter == temp_Array.points.size()-1) viewer.addSphere(temp_Array.points[lineiter],0.2,0,0.5,0.5,"Finish");
                        else viewer.addSphere(temp_Array.points[lineiter],0.2,0,1,0,ss1.str());
                    }
                    viewer.spin();
                    best_distance = new_distance;
                    //cout<<"  | New Best Admitted"<<endl;
                    cout<<"Distance : "<<best_distance<<endl;
                }
                //else cout<<"  | Pass"<<endl;
            }
        }
        improve ++;
    }
    pcl::copyPointCloud(temp_Array,*pclarray);
    cout<<"TwoOptTSP Finished"<<endl;
}

void OrdreringTSP(pcl::PointCloud<pcl::PointNormal>::Ptr pclarray)
{
    pcl::PointCloud<pcl::PointNormal> temp_Array;
    pcl::PointNormal minpt;
    pcl::PointNormal maxpt;
    pcl::getMinMax3D(*pclarray,minpt,maxpt);
    cout<<minpt.z<<"<-min,max->"<<maxpt.z<<endl;
    for(int i=0;i<pclarray->points.size();i++)
    {
        if(pclarray->points[i].z == minpt.z)
        {
            temp_Array.push_back(pclarray->points[i]);
            cout<<pclarray->points[i].z<<endl;
            for(int j=0;j<pclarray->points.size();j++)
            {
                if(pclarray->points[j].z == maxpt.z)
                {
                    cout<<pclarray->points[j].z<<endl;
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
