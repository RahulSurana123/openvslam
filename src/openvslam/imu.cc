
#include "imu.h"

#include <utility>
#include "opencv2/core/eigen.hpp"

namespace openvslam{



    Eigen::Vector3d imu::update_imu_lvelocity_xyz(Eigen::Vector3d imu_Acc_,const double delta_time){
        imu_vx+=(imu_vx+imu_Acc_(0)/1000*delta_time);    // TODO put delta time here
        imu_vy+=(imu_vy+imu_Acc_(1)/1000*delta_time);
        imu_vz+=(imu_vz+imu_Acc_(2)/1000*delta_time);
        return Eigen::Vector3d(imu_vx,imu_vy,imu_vz);
    }
    Eigen::VectorXd imu::update_imu_lposition_xyz(Eigen::Vector3d imu_lvel_, const double delta_time){
       Eigen::VectorXd out=Eigen::VectorXd::Zero(6);
        imu_x+=(imu_x+imu_lvel_(0)*delta_time);    // TODO put delta time here
        imu_y+=(imu_y+imu_lvel_(1)*delta_time);
        imu_z+=(imu_z+imu_lvel_(2)*delta_time);
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
        double bias_gx=-g*sin(imu_rpy_(1));
        double bias_gy=g*sin(imu_rpy_(0))*cos(imu_rpy_(2));
        double bias_gz=g*cos(imu_rpy_(1))*cos(imu_rpy_(2));
        imu_Acc_(0)=imu_Acc_(0)-bias_gx;
        imu_Acc_(1)=imu_Acc_(1)-bias_gy;
        imu_Acc_(2)=imu_Acc_(2)-bias_gz;
        return imu_Acc_;
    }
    Eigen::VectorXd imu::filtering_camera_pose_with_imu(Eigen::VectorXd imu_pose_cw_,Eigen::VectorXd camera_inputs){
        imu_pose_cw_=F_tran_matrix*imu_pose_cw_;
        std::cout<<"start:: "<<(imu_pose_cw_)<<std::endl;
        P_uncer_cov_matrix = F_tran_matrix*P_uncer_cov_matrix*F_tran_matrix.transpose();
        Eigen::VectorXd y =Eigen::VectorXd(std::move(camera_inputs))-H_mea_matrix*imu_pose_cw_;
        std::cout<<"end of y ::"<<y<<std::endl;
        Eigen::MatrixXd s=(H_mea_matrix.inverse()*P_uncer_cov_matrix.inverse()*H_mea_matrix.transpose().inverse());
        Eigen::VectorXd k = ((P_uncer_cov_matrix*H_mea_matrix.transpose())*s);
        imu_pose_cw_=imu_pose_cw_+k*y;
        P_uncer_cov_matrix=(Eigen::MatrixXd::Identity(6,6)-(k*H_mea_matrix))*P_uncer_cov_matrix;

        std::cout<<"end::"<<(imu_pose_cw_)<<std::endl;
        return imu_pose_cw_;
    }


}