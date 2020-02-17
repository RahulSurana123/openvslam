
#include "imu.h"

#include <utility>
#include "opencv2/core/eigen.hpp"

namespace openvslam{



    Eigen::Vector3d imu::update_imu_lvelocity_xyz(Eigen::Vector3d imu_Acc_,double delta_time){
        imu_vx+=(imu_Acc_(0)*delta_time);    // TODO put delta time here
        imu_vy+=(imu_Acc_(1)*delta_time);
        imu_vz+=(imu_Acc_(2)*delta_time);
        std::cout<<"start::imu VVVV MATRIX ::"<< imu_vx<<" "<<imu_vy <<"  "<<imu_vz                                                                                                                                                                                                                                                                                                                                                                                                                        <<std::endl;
        return Eigen::Vector3d(imu_vx,imu_vy,imu_vz);
    }
    Eigen::VectorXd imu::update_imu_lposition_xyz(Eigen::Vector3d imu_lvel_,double delta_time){
       Eigen::VectorXd out=Eigen::VectorXd::Zero(6);
        imu_x+=(imu_lvel_(0)*delta_time);    // TODO put delta time here
        imu_y+=(imu_lvel_(1)*delta_time);
        imu_z+=(imu_lvel_(2)*delta_time);
        out(0)=imu_x;
        out(1)=imu_y;
        out(2)=imu_z;
        out(3)=imu_lvel_(0);
        out(4)=imu_lvel_(1);
        out(5)=imu_lvel_(2);
        return out;
    }
    Eigen::Vector3d imu::acc_xyz_imu(Eigen::Vector3d imu_Acc_,Eigen::Vector3d imu_rpy_){
        double g=sqrt(pow(imu_Acc_(0),2)+pow(imu_Acc_(1),2)+pow(imu_Acc_(2),2));
        double bias_gx=(-g*sin(imu_rpy_(1)));
        double bias_gy=g*sin(imu_rpy_(0))*cos(imu_rpy_(2));
        double bias_gz=g*cos(imu_rpy_(1))*cos(imu_rpy_(2));
        imu_Acc_(0)=imu_Acc_(0)-bias_gx;
        imu_Acc_(1)=imu_Acc_(1)-bias_gy;
        imu_Acc_(2)=imu_Acc_(2)-bias_gz;
        std::cout<<"start::imu  MATRIX ::"<< imu_Acc_ <<std::endl;
        return imu_Acc_;
    }
    Eigen::VectorXd imu::filtering_camera_pose_with_imu(Eigen::VectorXd imu_pose_cw_,const Eigen::VectorXd& camera_input){
        try {
            imu_pose_cw_ = F_tran_matrix * imu_pose_cw_;
//        std::cout<<"start:: "<<(imu_pose_cw_)<<"F MATRIX ::"<< F_tran_matrix <<std::endl;
//        std::cout<<"P MATRIX ::"<< P_uncer_cov_matrix <<std::endl;
            P_uncer_cov_matrix = ((F_tran_matrix * P_uncer_cov_matrix) * F_tran_matrix.transpose());
//        std::cout<<"start:: "<<imu_pose_cw_<<"P MATRIX ::"<< P_uncer_cov_matrix <<std::endl;
            Eigen::MatrixXd y = camera_input - (H_mea_matrix * imu_pose_cw_);
        std::cout<<"end of y ::"<<y<<"H MATRIX  ::" <<H_mea_matrix<<std::endl;
            Eigen::MatrixXd s = (H_mea_matrix * P_uncer_cov_matrix * H_mea_matrix.transpose()) + R_noise_matrix;
//        std::cout<<"end of s -1 ::"<<s.inverse()<<std::endl;
            Eigen::MatrixXd k = (P_uncer_cov_matrix * H_mea_matrix.transpose() * s.inverse());
        std::cout<<"end of k ::"<<k<<std::endl;
            imu_pose_cw_ = imu_pose_cw_ + (k * y);
            P_uncer_cov_matrix = (Eigen::MatrixXd::Identity(6, 6) - (k * H_mea_matrix)) * P_uncer_cov_matrix;
        }catch(std::exception& e) {
            std::cerr << "exception run_camera_slam is : " <<e.what()<< std::endl;}
        std::cout<<"end::"<<(imu_pose_cw_)<<std::endl;
        return imu_pose_cw_;
    }
    void imu::reset(){
        imu_x=imu_y=imu_z=imu_vx=imu_vy=imu_vz=0;
    }

}