#include "iostream"
#include <vector>
//#include <Eigen/Dense>
#include "/usr/include/eigen3/Eigen/Dense"

//Eigen::MatrixXd F_tran_matrix<double ,6,6,0,6,6>;
namespace openvslam{
using namespace Eigen;
    class imu{
    public:
            double imu_x=0,imu_y=0,imu_z=0,imu_vx=0,imu_vy=0,imu_vz=0;
//        h_mea_matrix[3][6]={{1,0,0,0,0,0},
//            {0,1,0,0,0,0},
//            {0,0,1,0,0,0}};
//        cv::Mat f_tran_matrix = cv::Mat::eye(6,6,CV_32F);
//        double F_tran_matrix[6][6]={{1,0,0,0,0,0},
//                                    {0,1,0,0,0,0},
//                                    {0,0,1,0,0,0},   // TODO delta time
//                                    {0,0,0,1,0,0},
//                                    {0,0,0,0,1,0},
//                                    {0,0,0,0,0,1}};
        Eigen::MatrixXd F_tran_matrix = Eigen::MatrixXd::Identity(6,6);
        Eigen::MatrixXd P_uncer_cov_matrix = Eigen::MatrixXd::Zero(6,6);
        Eigen::MatrixXd H_mea_matrix = Eigen::MatrixXd::Identity(3,6);




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

        Eigen::Vector3d update_imu_lvelocity_xyz(Eigen::Vector3d imu_Acc_, const double delta_time);
        Eigen::VectorXd update_imu_lposition_xyz(Eigen::Vector3d imu_lvel_, const double delta_time);
        Eigen::Vector3d acc_xyz_imu(Eigen::Vector3d imu_Acc_, Eigen::Vector3d imu_rpy_);
        Eigen::VectorXd filtering_camera_pose_with_imu(Eigen::VectorXd imu_pose_cw_,Eigen::VectorXd camera_inputs);

    };

}