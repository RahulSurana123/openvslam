
#include "imu.h"
#include <utility>
#include "opencv2/core/eigen.hpp"

namespace openvslam{



    Matri imu::update_imu_lvelocity_xyz(Eigen::Vector3d imu_Acc_,double delta_time){
        Matri c = Matri(3,1,0);
        c.mat[0][0]+=(imu_Acc_(0)*delta_time);    // TODO put delta time here
        c.mat[1][0]+=(imu_Acc_(1)*delta_time);
        c.mat[2][0]+=(imu_Acc_(2)*delta_time);
     //   std::cout<<"start::imu VVVV MATRIX ::"<< imu_vx<<" "<<imu_vy <<"  "<<imu_vz                                                                                                                                                                                                                                                                                                                                                                                                                        <<std::endl;
        return c;
    }
    Matri imu::update_imu_lposition_xyz(Matri imu_lvel_,double delta_time){
       Matri out=Matri(6,1,0);
        imu_x+=(imu_lvel_.mat[0][0]*delta_time);    // TODO put delta time here
        imu_y+=(imu_lvel_.mat[1][0]*delta_time);
        imu_z+=(imu_lvel_.mat[2][0]*delta_time);
        out.mat[0][0]=imu_x;
        out.mat[1][0]=imu_y;
        out.mat[2][0]=imu_z;
        out.mat[3][0]=imu_lvel_.mat[0][0];
        out.mat[4][0]=imu_lvel_.mat[1][0];
        out.mat[5][0]=imu_lvel_.mat[2][0];
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
        //std::cout<<"start::imu  MATRIX ::"<< imu_Acc_ <<std::endl;
        return imu_Acc_;
    }
    Matri imu::filtering_camera_pose_with_imu(Matri imu_pose_cw_, Matri& camera_input){
        try {

            imu_pose_cw_ = F_tran_matrix*imu_pose_cw_;
//            imu_pose_cw_.print();
        std::cout<<"start:: F MATRIX ::"<<std::endl;
//        std::cout<<"P MATRIX ::"<< P_uncer_cov_matrix <<std::endl;
            P_uncer_cov_matrix = ((F_tran_matrix*P_uncer_cov_matrix)*F_tran_matrix^1);
        std::cout<<"start:: "<<"P MATRIX ::" <<std::endl;
            Matri temp =(H_mea_matrix*imu_pose_cw_);
            Matri temp1 =(H_mea_matrix * P_uncer_cov_matrix);
            Matri y = camera_input-temp;
            temp=H_mea_matrix^1;
        std::cout<<"end of y ::"<<"H MATRIX  ::"<<std::endl;
            Matri s = (temp1 * temp) + R_noise_matrix;
//        std::cout<<"end of s -1 ::"<<s.inverse()<<std::endl;
            temp1=s.inverse();
            temp = temp * temp1;
            Matri k = (P_uncer_cov_matrix*temp);
        std::cout<<"end of k ::"<<std::endl;
            temp =(k*y);
            imu_pose_cw_ = imu_pose_cw_ + temp;
            temp = (k * H_mea_matrix);
            P_uncer_cov_matrix = (Matri(6, 6, 1 ) - temp) * P_uncer_cov_matrix;
            imu_pose_cw_.print();
        }catch(std::exception& e) {
            std::cerr << "exception imu_chu is : " <<e.what()<< std::endl;}
    //    std::cout<<"end::"<<(imu_pose_cw_)<<std::endl;
        std::cout<<"While End : "<<std::endl;
        return imu_pose_cw_;
    }
    void imu::reset(){
        imu_x=imu_y=imu_z=imu_vx=imu_vy=imu_vz=0;
    }

}