
#include <cmath>
#include "imu.h"
//#include <utility>
//#include "opencv2/core/eigen.hpp"


namespace openvslam {


    MATRIX imu::update_imu_lvelocity_xyz(MATRIX imu_Acc_, const double delta_time) {
        imu_vx += (imu_Acc_.mat[0][0] * delta_time);    // TODO put delta time here
        imu_vy += (imu_Acc_.mat[1][0] * delta_time);
        imu_vz += (imu_Acc_.mat[2][0] * delta_time);
        std::cout << "start::imu VVVV MATRIX ::" << imu_vx << " " << imu_vy << "  " << imu_vz << std::endl;
        return MATRIX(imu_vx, imu_vy, imu_vz);
    }

    MATRIX imu::update_imu_lposition_xyz(MATRIX imu_lvel_, const double delta_time) {
        MATRIX out = MATRIX(6,1,0);
        imu_x += (imu_lvel_.mat[0][0] * delta_time);    // TODO put delta time here
        imu_y += (imu_lvel_.mat[1][0] * delta_time);
        imu_z += (imu_lvel_.mat[2][0] * delta_time);
        out.mat[0][0] = imu_x;
        out.mat[1][0] = imu_y;
        out.mat[2][0] = imu_z;
        out.mat[3][0] = imu_lvel_.mat[0][0];
        out.mat[4][0] = imu_lvel_.mat[1][0];
        out.mat[5][0] = imu_lvel_.mat[2][0];
        return out;
    }

    MATRIX imu::acc_xyz_imu(MATRIX imu_Acc_, MATRIX imu_rpy_) {
        double g = sqrt(pow(imu_Acc_.mat[0][0],2) + pow(imu_Acc_.mat[1][0], 2) + pow(imu_Acc_.mat[2][0], 2));
        double bias_gx = (-g * sin(imu_rpy_.mat[1][0]));
        double bias_gy = g * sin(imu_rpy_.mat[0][0]) * cos(imu_rpy_.mat[2][0]);
        double bias_gz = g * cos(imu_rpy_.mat[1][0]) * cos(imu_rpy_.mat[2][0]);
        imu_Acc_.mat[0][0] = imu_Acc_.mat[0][0] - bias_gx;
        imu_Acc_.mat[1][0] = imu_Acc_.mat[1][0] - bias_gy;
        imu_Acc_.mat[2][0] = imu_Acc_.mat[2][0] - bias_gz;
//        std::cout << "start::imu  MATRIX ::" << imu_Acc_ << std::endl;
        return imu_Acc_;
    }

    MATRIX imu::filtering_camera_pose_with_imu(MATRIX imu_pose_cw_, MATRIX camera_input) {
        try {
            imu_pose_cw_ = F_tran_matrix * imu_pose_cw_;
//        std::cout<<"start:: "<<(imu_pose_cw_)<<"F MATRIX ::"<< F_tran_matrix <<std::endl;
//        std::cout<<"P MATRIX ::"<< P_uncer_cov_matrix <<std::endl;
            MATRIX temp = (F_tran_matrix * P_uncer_cov_matrix);
            P_uncer_cov_matrix = ( temp * F_tran_matrix^1);
//        std::cout<<"start:: "<<imu_pose_cw_<<"P MATRIX ::"<< P_uncer_cov_matrix <<std::endl;
            temp = P_uncer_cov_matrix * H_mea_matrix^1;
            MATRIX temp1= (H_mea_matrix * imu_pose_cw_);
            MATRIX y = camera_input - temp1;
//            std::cout << "end of y ::" << y << "H MATRIX  ::" << H_mea_matrix << std::endl;
            MATRIX s = (H_mea_matrix * temp) + R_noise_matrix;
//        std::cout<<"end of s -1 ::"<<s.inverse()<<std::endl;
            temp1=s.inverse();
            MATRIX k = (temp*temp1);
//            std::cout << "end of k ::" << k << std::endl;
            temp = k * y;
            imu_pose_cw_ = imu_pose_cw_ + (temp);
            temp1=k * H_mea_matrix;
            P_uncer_cov_matrix = (MATRIX(6, 6,1)- (temp1)) * P_uncer_cov_matrix;
        } catch (std::exception &e) {
            std::cerr << "exception run_camera_slam is : " << e.what() << std::endl;
        }
//        std::cout << "end::" << (imu_pose_cw_) << std::endl;
        return imu_pose_cw_;
    }

    void imu::reset() {
        imu_x = imu_y = imu_z = imu_vx = imu_vy = imu_vz = 0;
    }

}