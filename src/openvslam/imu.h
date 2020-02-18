#include "iostream"
#include <vector>
#include "MATRIX.h"
//#include <Eigen/Dense>
//#include "/usr/include/eigen3/Eigen/Dense"
//using namespace Eigen;
//MATRIX F_tran_matrix<double ,6,6,0,6,6>;
namespace openvslam{

    class imu{
    private:
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
        MATRIX F_tran_matrix = MATRIX(6,6,1);
        MATRIX P_uncer_cov_matrix = MATRIX(6,6,0);
        MATRIX H_mea_matrix = MATRIX(3,6,1);
        MATRIX R_noise_matrix = MATRIX(3,3,0);




//        cv::Mat P_uncer_cov_matrix = (cv::Mat_<double>(6,6) << 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0, 1000, 0, 0, 0, 0, 0,0, 1000, 0, 0, 0, 0, 0, 0, 1000);
//        cv::Mat H_meas_matrix = (cv::Mat_<double>(3,6) << 1, 0, 0, 0, 0, 0, 0, 1, 0,0, 0, 0, 0, 0, 1, 0, 0, 0);

        //MATRIX P_uncer_cov_matrix=MATRIX(6,6);
//        P_uncer_cov_matrix.mat[3][3]=1000;

//        double P_uncer_cov_matrix[6][6]={{0,0,0,0,0,0},
//                                         {0,0,0,0,0,0},
//                                         {0,0,0,0,0,0},
//                                         {0,0,0,1000,0,0},
//                                         {0,0,0,0,1000,0},
//                                         {0,0,0,0,0,1000}};
        imu(){
            H_mea_matrix.mat[3][3]=H_mea_matrix.mat[4][4]=H_mea_matrix.mat[5][5]=0;
            R_noise_matrix.mat[0][0]=.04;
            R_noise_matrix.mat[0][2]=.06;
            R_noise_matrix.mat[1][0]=.01;
            R_noise_matrix.mat[2][1]=.02;
            R_noise_matrix.mat[2][2]=.08;

        }
    public:
        MATRIX update_imu_lvelocity_xyz(MATRIX imu_Acc_, const double delta_time);
        MATRIX update_imu_lposition_xyz(MATRIX imu_lvel_, const double delta_time);
        MATRIX acc_xyz_imu(MATRIX imu_Acc_, MATRIX imu_rpy_);
        MATRIX filtering_camera_pose_with_imu(MATRIX imu_pose_cw_,MATRIX camera_inputs);

        void reset();
    };

}