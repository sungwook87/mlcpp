#ifndef THREE_D_CLASS_H
#define THREE_D_CLASS_H


#include <opencv2/opencv.hpp>
#include <vector>
/* Octopoint Center must be over zero.
 *
 *
 *
 */
using namespace std;

class OctoPoint;
class OctoPoints;
class ViewPoint;

class ViewPoint{
    public:
        ViewPoint() {}
        virtual ~ViewPoint() {}
        void add_Octopoint(OctoPoint * octopt)
        {
            View_What.push_back(octopt);
        }
        vector<void *> check_Octopoints()
        {
            return View_What;
        }

    private:
        vector<void *> View_What;
};

class OctoPoint{
    public:
        OctoPoint(cv::Point2d Center_in,cv::Point3d Norm_vector_in)
        {
            Center = Center_in;
            Norm_vector = Norm_vector_in;
        }
        cv::Point2d get_Center()
        {
            return Center;
        }
        cv::Point3d get_Norm()
        {
            return Norm_vector;
        }
        virtual ~OctoPoint() {}
        void clear_ViewPoints()
        {
            What_view.clear();
        }
        void add_ViewPoints(ViewPoint * viewpoint)
        {
            What_view.push_back(viewpoint);
        }
        vector<void *> check_ViewPoints()
        {
            return What_view;
        }
private:
        cv::Point2d Center;
        cv::Point3d Norm_vector;
        vector<void *> What_view;
};

class OctoPoints{
    public:
        OctoPoints(cv::Point2d Map_size_in,double Voxel_size_in,bool is_xy)
        {
            if(is_xy)
            {
                Map_size.x = Map_size_in.x / Voxel_size_in;
                Map_size.y = Map_size_in.y / Voxel_size_in;
            }
            else Map_size = Map_size_in;
            Voxel_size = Voxel_size_in;
            for(int i=0;i<Map_size.x;i++)
            {
                Point_matrix.push_back(vector<void *>());
                for(int j=0;j<Map_size.y;j++)
                {
                   void * temp_pt;
                   Point_matrix[i].push_back(temp_pt);
                }
            }
        }
        void put_Octopoint(OctoPoint * OctoPointPt)
        {
            cv::Point2d center = OctoPointPt->get_Center();
            Point_matrix[center.x / Voxel_size][center.y / Voxel_size] = OctoPointPt;
        }

        virtual ~OctoPoints() {}

    private:
        vector<vector<void *> > Point_matrix;
        cv::Point2d Map_size; // Map_size : X,Y
        double Voxel_size;
};


#endif