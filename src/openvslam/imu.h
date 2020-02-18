#include "iostream"
#include <vector>
#include <mutex>
#include "Matri.h"
//#include <Eigen/Dense>
#include "/usr/include/eigen3/Eigen/Dense"
#include "thread"
//Eigen::MatrixXd F_tran_matrix<double ,6,6,0,6,6>;
namespace openvslam{
using namespace Eigen;
    class imu {
    public:
            double imu_x=0,imu_y=0,imu_z=0,imu_vx=0,imu_vy=0,imu_vz=0;
//        h_mea_matrix[3][6]={{1,0,0,0,0,0},
//            {0,1,0,0,0,0},
//            {0,0,1,0,0,0}};
//        cv::Mat f_tran_matrix = cv::Mat::eye(6,6,CV_32F);
//        double F_tran_matrix[6][6]={{1,0,0,,00,0},
//                                    {0,1,0,0,0,0},
//                                    {0,0,1,0,0,0},   // TODO delta time
//                                    {0,0,0,1,0,0},
//                                    {0,0,0,0,1,0},
//                                    {0,0,0,0,0,1}};
        Matri F_tran_matrix = Matri(6,6,1);
        Matri P_uncer_cov_matrix = Matri(6,6,0);
        Matri H_mea_matrix = Matri(3,6,1);
        Matri R_noise_matrix = Matri(3,3,0);




//        cv::Mat P_uncer_cov_matrix = (cv::Mat_<double>(6,6) << 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0, 1000, 0, 0, 0, 0, 0,0, 1000, 0, 0, 0, 0, 0, 0, 1000);
//        cv::Mat H_meas_matrix = (cv::Mat_<double>(3,6) << 1, 0, 0, 0, 0, 0, 0, 1, 0,0, 0, 0, 0, 0, 1, 0, 0, 0);

        //Eigen::MatrixXd P_uncer_cov_matrix=Eigen::MatrixXd(6,6);
        //P_uncer_cov_matrix(3,3)=1000;

//        double P_uncer_cov_matrix[6][6]={{0,0,0,0,0,0},
//                                         {0,0,0,0,0,0},
//                                         {0,0,0,0,0,0},
//                                         {0,0,0,1000,0,0},
//                                         {0,0,0,0,1000,0},
//                                         {0,0,0,0,0,1000}};
        imu(){
//            H_mea_matrix(3,3)=H_mea_matrix(4,4)=H_mea_matrix(5,5)=0;
            R_noise_matrix.mat[0][0]=.004;
            R_noise_matrix.mat[0][2]=.006;
            R_noise_matrix.mat[1][0]=.001;
            R_noise_matrix.mat[2][1]=.002;
            R_noise_matrix.mat[2][2]=.008;

        }
        Matri update_imu_lvelocity_xyz(Eigen::Vector3d imu_Acc_, double delta_time);
        Matri update_imu_lposition_xyz(Matri imu_lvel_, double delta_time);
        static Eigen::Vector3d acc_xyz_imu(Eigen::Vector3d imu_Acc_, Eigen::Vector3d imu_rpy_);
        Matri filtering_camera_pose_with_imu(Matri imu_pose_cw_, Matri& camera_inputs);

        void reset();
    };

}