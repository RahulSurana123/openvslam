#include "util/tum_rgbd_util.h"

#ifdef USE_PANGOLIN_VIEWER
#include "pangolin_viewer/viewer.h"
#elif USE_SOCKET_PUBLISHER
#include "socket_publisher/publisher.h"
#endif

#include "openvslam/system.h"
#include "openvslam/config.h"

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <numeric>

using std::ifstream;
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <spdlog/spdlog.h>
#include <popl.hpp>
#include <fstream>
//#include <openvslam/MATRIX.h>
#include <openvslam/imu.h>
#include <string>
using std::ofstream;

#ifdef USE_STACK_TRACE_LOGGER
#include <glog/logging.h>
#endif

#ifdef USE_GOOGLE_PERFTOOLS
#include <gperftools/profiler.h>
#endif

void mono_tracking(const std::shared_ptr<openvslam::config>& cfg,
                   const std::string& vocab_file_path, const std::string& sequence_dir_path,
                   const unsigned int frame_skip, const bool no_sleep, const bool auto_term,
                   const bool eval_log, const std::string& map_db_path) {
    tum_rgbd_sequence sequence(sequence_dir_path);
    const auto frames = sequence.get_frames();

    // build a SLAM system
    openvslam::system SLAM(cfg, vocab_file_path);
    // startup the SLAM process
    SLAM.startup();

    // create a viewer object
    // and pass the frame_publisher and the map_publisher
#ifdef USE_PANGOLIN_VIEWER
    pangolin_viewer::viewer viewer(cfg, &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());
#elif USE_SOCKET_PUBLISHER
    socket_publisher::publisher publisher(cfg, &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());
#endif
    ofstream outdata;
    std::vector<double> track_times;
    track_times.reserve(frames.size());
    long long int ttg=0;
    outdata.open("keyframe_mono.txt");
    // run the SLAM in another thread
    std::thread thread([&]() {
        for (unsigned int i = 0; i < frames.size(); ++i) {
            const auto& frame = frames.at(i);
            const auto rgb_img = cv::imread(frame.rgb_img_path_, cv::IMREAD_UNCHANGED);

            const auto tp_1 = std::chrono::steady_clock::now();

            if (!rgb_img.empty() && (i % frame_skip == 0)) {
                // input the current frame and estimate the camera pose
                openvslam::Mat44_t tt = SLAM.feed_monocular_frame(rgb_img, frame.timestamp_);
                if(i>0){
                    SLAM.camera_x+= tt(0,3)*(track_times.at(i-1));
                    SLAM.camera_y+= tt(1,3)*(track_times.at(i-1));
                    SLAM.camera_z+= tt(2,3)*(track_times.at(i-1));
                    outdata<<"frame_no"<<ttg<<"   "<<SLAM.camera_x<< "   "<<SLAM.camera_y<< "   "<<SLAM.camera_z<<"\n";
                    ttg++;}
            }

            const auto tp_2 = std::chrono::steady_clock::now();

            const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
            if (i % frame_skip == 0) {
                track_times.push_back(track_time);
            }

            // wait until the timestamp of the next frame
            if (!no_sleep && i < frames.size() - 1) {
                const auto wait_time = frames.at(i + 1).timestamp_ - (frame.timestamp_ + track_time);
                if (0.0 < wait_time) {
                    std::this_thread::sleep_for(std::chrono::microseconds(static_cast<unsigned int>(wait_time * 1e6)));
                }
            }

            // check if the termination of SLAM system is requested or not
            if (SLAM.terminate_is_requested()) {
                break;
            }
        }

        // wait until the loop BA is finished
        while (SLAM.loop_BA_is_running()) {
            std::this_thread::sleep_for(std::chrono::microseconds(5000));
        }

        // automatically close the viewer
#ifdef USE_PANGOLIN_VIEWER
        if (auto_term) {
            viewer.request_terminate();
        }
#elif USE_SOCKET_PUBLISHER
        if (auto_term) {
            publisher.request_terminate();
        }
#endif
    });

    // run the viewer in the current thread
#ifdef USE_PANGOLIN_VIEWER
    viewer.run();
#elif USE_SOCKET_PUBLISHER
    publisher.run();
#endif

    thread.join();

    // shutdown the SLAM process
    SLAM.shutdown();

    if (eval_log) {
        // output the trajectories for evaluation
        SLAM.save_frame_trajectory("frame_trajectory.txt", "TUM");
        SLAM.save_keyframe_trajectory("keyframe_trajectory.txt", "TUM");
        // output the tracking times for evaluation
        std::ofstream ofs("track_times.txt", std::ios::out);
        if (ofs.is_open()) {
            for (const auto track_time : track_times) {
                ofs << track_time << std::endl;
            }
            ofs.close();
        }
    }

    if (!map_db_path.empty()) {
        // output the map database
        SLAM.save_map_database(map_db_path);
    }

    std::sort(track_times.begin(), track_times.end());
    const auto total_track_time = std::accumulate(track_times.begin(), track_times.end(), 0.0);
    std::cout << "median tracking time: " << track_times.at(track_times.size() / 2) << "[s]" << std::endl;
    std::cout << "mean tracking time: " << total_track_time / track_times.size() << "[s]" << std::endl;
}

void rgbd_tracking(const std::shared_ptr<openvslam::config>& cfg,
                   const std::string& vocab_file_path, const std::string& sequence_dir_path,
                   const unsigned int frame_skip, const bool no_sleep, const bool auto_term,
                   const bool eval_log, const std::string& map_db_path) {
    tum_rgbd_sequence sequence(sequence_dir_path);
    const auto frames = sequence.get_frames();

    // build a SLAM system
    openvslam::system SLAM(cfg, vocab_file_path);
    // startup the SLAM process
    SLAM.startup();

    // create a viewer object
    // and pass the frame_publisher and the map_publisher
#ifdef USE_PANGOLIN_VIEWER
    pangolin_viewer::viewer viewer(cfg, &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());
#elif USE_SOCKET_PUBLISHER
    socket_publisher::publisher publisher(cfg, &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());
#endif

    std::vector<double> track_times;
    track_times.reserve(frames.size());

    // run the SLAM in another thread
    std::thread thread([&]() {
        for (unsigned int i = 0; i < frames.size(); ++i) {
            const auto& frame = frames.at(i);
//            if(i>0) {
//                const auto &frame1 = frames.at(i - 1);
//                double frame11 = frame1.timestamp_;
//            }
            const auto rgb_img = cv::imread(frame.rgb_img_path_, cv::IMREAD_UNCHANGED);
            const auto depth_img = cv::imread(frame.depth_img_path_, cv::IMREAD_UNCHANGED);

            const auto tp_1 = std::chrono::steady_clock::now();

            if (!rgb_img.empty() && !depth_img.empty() && (i % frame_skip == 0)) {
                // input the current frame and estimate the camera pose


                SLAM.feed_RGBD_frame(rgb_img, depth_img, frame.timestamp_);

            }

            const auto tp_2 = std::chrono::steady_clock::now();

            const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
            if (i % frame_skip == 0) {
                track_times.push_back(track_time);
            }

            // wait until the timestamp of the next frame
            if (!no_sleep && i < frames.size() - 1) {
                const auto wait_time = frames.at(i + 1).timestamp_ - (frame.timestamp_ + track_time);
                if (0.0 < wait_time) {
                    std::this_thread::sleep_for(std::chrono::microseconds(static_cast<unsigned int>(wait_time * 1e6)));
                }
            }

            // check if the termination of SLAM system is requested or not
            if (SLAM.terminate_is_requested()) {
                break;
            }
        }

        // wait until the loop BA is finished
        while (SLAM.loop_BA_is_running()) {
            std::this_thread::sleep_for(std::chrono::microseconds(5000));
        }

        // automatically close the viewer
#ifdef USE_PANGOLIN_VIEWER
        if (auto_term) {
            viewer.request_terminate();
        }
#elif USE_SOCKET_PUBLISHER
        if (auto_term) {
            publisher.request_terminate();
        }
#endif
    });

    // run the viewer in the current thread
#ifdef USE_PANGOLIN_VIEWER
    viewer.run();
#elif USE_SOCKET_PUBLISHER
    publisher.run();
#endif

    thread.join();

    // shutdown the SLAM process
    SLAM.shutdown();

    if (eval_log) {
        // output the trajectories for evaluation
        SLAM.save_frame_trajectory("frame_trajectory.txt", "TUM");
        SLAM.save_keyframe_trajectory("keyframe_trajectory.txt", "TUM");
        // output the tracking times for evaluation
        std::ofstream ofs("track_times.txt", std::ios::out);
        if (ofs.is_open()) {
            for (const auto track_time : track_times) {
                ofs << track_time << std::endl;
            }
            ofs.close();
        }
    }

    if (!map_db_path.empty()) {
        // output the map database
        SLAM.save_map_database(map_db_path);
    }

    std::sort(track_times.begin(), track_times.end());
    const auto total_track_time = std::accumulate(track_times.begin(), track_times.end(), 0.0);
    std::cout << "median tracking time: " << track_times.at(track_times.size() / 2) << "[s]" << std::endl;
    std::cout << "mean tracking time: " << total_track_time / track_times.size() << "[s]" << std::endl;
}

void rgbd_imu_tracking(const std::shared_ptr<openvslam::config>& cfg,
                       const std::string& vocab_file_path, const std::string& sequence_dir_path,
                       const unsigned int frame_skip, const bool no_sleep, const bool auto_term,
                       const bool eval_log, const std::string& map_db_path) {
    tum_rgbd_sequence sequence(sequence_dir_path);
    const auto frames = sequence.get_frames();
    ifstream myfile ("accelerometer.txt");
    // build a SLAM system
    openvslam::system SLAM(cfg, vocab_file_path);
    // startup the SLAM process
    SLAM.startup();

    // create a viewer object
    // and pass the frame_publisher and the map_publisher
#ifdef USE_PANGOLIN_VIEWER
    pangolin_viewer::viewer viewer(cfg, &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());
#elif USE_SOCKET_PUBLISHER
    socket_publisher::publisher publisher(cfg, &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());
#endif
    openvslam::imu data2;
    openvslam::MATRIX camera_inputs=openvslam::MATRIX(3,1,0);
    std::vector<double> track_times;
    track_times.reserve(frames.size());
    ifstream outdata,outdata1;
    SLAM.imu_on=0;
    long long int ttg=0;
    outdata.open("accelerometer.txt");
    outdata1.open("gyro.txt");
    // run the SLAM in another thread
    SLAM.camera_x=SLAM.camera_z=SLAM.camera_y=0,SLAM.delta_cam_x=0,SLAM.delta_cam_y=0,SLAM.delta_cam_z=0,SLAM.prev_cam_x=0,SLAM.prev_cam_y=0,SLAM.prev_cam_z=0;
    bool first_ite=true;
    std::thread thread([&]() {
        double x_advance=0,y_advance=0,z_advance=0;
        openvslam::Mat44_t cam_pose_c;
        for (unsigned int i = 0; i < frames.size(); ++i) {

            const auto& frame = frames.at(i);
//            if(i>0) {
//                const auto &frame1 = frames.at(i - 1);
//                double frame11 = frame1.timestamp_;
//            }
            const auto rgb_img = cv::imread(frame.rgb_img_path_, cv::IMREAD_UNCHANGED);
            const auto depth_img = cv::imread(frame.depth_img_path_, cv::IMREAD_UNCHANGED);

            const auto tp_1 = std::chrono::steady_clock::now();
            double delta_try1;
            if (!rgb_img.empty() && !depth_img.empty() && (i % frame_skip == 0)) {
                // input the current frame and estimate the camera pose

                cam_pose_c=SLAM.feed_RGBD_frame(rgb_img, depth_img, frame.timestamp_);
                    if(i>0){
                    double delta_try=track_times.at(i-1);
                //std::cout << "inside slam feedbACK"<< std::endl;
                // input the current frame and estimate the camera pose

                x_advance=(cam_pose_c(0,3)*delta_try);
                y_advance=(cam_pose_c(1,3)*delta_try);
                z_advance=(cam_pose_c(2,3)*delta_try);
                    delta_try1=delta_try;}
                        std::string line;
                        getline (outdata,line) ;
                            std::stringstream   linestream(line);
                            std::string         data;
                            double              garbage;
                            double              accel_x;
                            double              accel_y;
                            double              accel_z;

                            // If you have truly tab delimited data use getline() with third parameter.
                            // If your data is just white space separated data
                            // then the operator >> will do (it reads a space separated word into a string).
                            std::getline(linestream, data, '\t');  // read up-to the first tab (discard tab).

                            // Read the integers using the operator >>
                            linestream >>garbage>>accel_x>>accel_y >> accel_z;

                openvslam::MATRIX gyro_eigen =openvslam::MATRIX(3,1,0);
                openvslam::MATRIX acc_eigen  =openvslam::MATRIX(3,1,0);
                acc_eigen.mat[0][0]=accel_x;
                acc_eigen.mat[1][0]=accel_y;
                acc_eigen.mat[2][0]=accel_z;
                            getline (outdata1,line);

                                std::string         data1;
                                double              garbage1;
                                double              gyro_x;
                                double              gyro_y;
                                double              gyro_z;

                                // If you have truly tab delimited data use getline() with third parameter.
                                // If your data is just white space separated data
                                // then the operator >> will do (it reads a space separated word into a string).
                                std::getline(linestream, data1, '\t');  // read up-to the first tab (discard tab).

                                // Read the integers using the operator >>
                                linestream >>garbage1>>gyro_x>>gyro_y >> gyro_z;
                gyro_eigen.mat[0][0]=gyro_x*delta_try1*delta_try1/2;
                gyro_eigen.mat[1][0]=gyro_y*delta_try1*delta_try1/2;
                gyro_eigen.mat[2][0]=gyro_z*delta_try1*delta_try1/2;
                if(acc_eigen.mat[0][0]<0)SLAM.camera_x-=x_advance;
                else SLAM.camera_x+=x_advance;
                if(acc_eigen.mat[1][0]<0)SLAM.camera_y-=y_advance;
                else SLAM.camera_y+=y_advance;
                if(acc_eigen.mat[2][0]<0)SLAM.camera_z-=z_advance;
                else SLAM.camera_z+=z_advance;
                data2.F_tran_matrix.mat[0][3] = data2.F_tran_matrix.mat[1][4] = data2.F_tran_matrix.mat[2][5] = delta_try1;
                data2.P_uncer_cov_matrix.mat[3][3] = data2.P_uncer_cov_matrix.mat[4][4] = data2.P_uncer_cov_matrix.mat[5][5] = 1000;
                camera_inputs.mat[0][0] = SLAM.camera_x;
                camera_inputs.mat[1][0] = SLAM.camera_y;
                camera_inputs.mat[2][0] = SLAM.camera_z;
//                camera_inputs.print();
                openvslam::MATRIX vv=data2.update_imu_lvelocity_xyz(data2.acc_xyz_imu(acc_eigen, gyro_eigen),delta_try1);
                openvslam::MATRIX xxx=data2.update_imu_lposition_xyz(vv,delta_try1);
                openvslam::MATRIX out= data2.filtering_camera_pose_with_imu(xxx,camera_inputs);
                SLAM.camera_x=out.mat[0][0];
                SLAM.camera_y=out.mat[1][0];
                SLAM.camera_z=out.mat[2][0];

                SLAM.delta_cam_x=SLAM.camera_x-SLAM.prev_cam_x;
                SLAM.delta_cam_y=SLAM.camera_y-SLAM.prev_cam_y;
                SLAM.delta_cam_z=SLAM.camera_z-SLAM.prev_cam_z;
                SLAM.prev_cam_x=SLAM.camera_x;
                SLAM.prev_cam_z=SLAM.camera_z;
                SLAM.prev_cam_y=SLAM.camera_y;
            }

            const auto tp_2 = std::chrono::steady_clock::now();

            const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
            if (i % frame_skip == 0) {
                track_times.push_back(track_time);
            }

            // wait until the timestamp of the next frame
            if (!no_sleep && i < frames.size() - 1) {
                const auto wait_time = frames.at(i + 1).timestamp_ - (frame.timestamp_ + track_time);
                if (0.0 < wait_time) {
                    std::this_thread::sleep_for(std::chrono::microseconds(static_cast<unsigned int>(wait_time * 1e6)));
                }
            }

            // check if the termination of SLAM system is requested or not
            if (SLAM.terminate_is_requested()) {
                break;
            }
        }

        // wait until the loop BA is finished
        while (SLAM.loop_BA_is_running()) {
            std::this_thread::sleep_for(std::chrono::microseconds(5000));
        }

        // automatically close the viewer
#ifdef USE_PANGOLIN_VIEWER
        if (auto_term) {
            viewer.request_terminate();
        }
#elif USE_SOCKET_PUBLISHER
        if (auto_term) {
            publisher.request_terminate();
        }
#endif
    });

    // run the viewer in the current thread
#ifdef USE_PANGOLIN_VIEWER
    viewer.run();
#elif USE_SOCKET_PUBLISHER
    publisher.run();
#endif

    thread.join();

    // shutdown the SLAM process
    SLAM.shutdown();

    if (eval_log) {
        // output the trajectories for evaluation
        SLAM.save_frame_trajectory("frame_trajectory.txt", "TUM");
        SLAM.save_keyframe_trajectory("keyframe_trajectory.txt", "TUM");
        // output the tracking times for evaluation
        std::ofstream ofs("track_times.txt", std::ios::out);
        if (ofs.is_open()) {
            for (const auto track_time : track_times) {
                ofs << track_time << std::endl;
            }
            ofs.close();
        }
    }

    if (!map_db_path.empty()) {
        // output the map database
        SLAM.save_map_database(map_db_path);
        SLAM.save_kf_xyz();
    }

    std::sort(track_times.begin(), track_times.end());
    const auto total_track_time = std::accumulate(track_times.begin(), track_times.end(), 0.0);
    std::cout << "median tracking time: " << track_times.at(track_times.size() / 2) << "[s]" << std::endl;
    std::cout << "mean tracking time: " << total_track_time / track_times.size() << "[s]" << std::endl;
}

int main(int argc, char* argv[]) {
#ifdef USE_STACK_TRACE_LOGGER
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
#endif

    // create options
    popl::OptionParser op("Allowed options");
    auto help = op.add<popl::Switch>("h", "help", "produce help message");
    auto vocab_file_path = op.add<popl::Value<std::string>>("v", "vocab", "vocabulary file path");
    auto data_dir_path = op.add<popl::Value<std::string>>("d", "data-dir", "directory path which contains dataset");
    auto config_file_path = op.add<popl::Value<std::string>>("c", "config", "config file path");
    auto frame_skip = op.add<popl::Value<unsigned int>>("", "frame-skip", "interval of frame skip", 1);
    auto no_sleep = op.add<popl::Switch>("", "no-sleep", "not wait for next frame in real time");
    auto auto_term = op.add<popl::Switch>("", "auto-term", "automatically terminate the viewer");
    auto debug_mode = op.add<popl::Switch>("", "debug", "debug mode");
    auto eval_log = op.add<popl::Switch>("", "eval-log", "store trajectory and tracking times for evaluation");
    auto map_db_path = op.add<popl::Value<std::string>>("p", "map-db", "store a map database at this path after SLAM", "");
    try {
        op.parse(argc, argv);
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // check validness of options
    if (help->is_set()) {
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }
    if (!vocab_file_path->is_set() || !data_dir_path->is_set() || !config_file_path->is_set()) {
        std::cerr << "invalid arguments" << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // setup logger
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] %^[%L] %v%$");
    if (debug_mode->is_set()) {
        spdlog::set_level(spdlog::level::debug);
    }
    else {
        spdlog::set_level(spdlog::level::info);
    }

    // load configuration
    std::shared_ptr<openvslam::config> cfg;
    try {
        cfg = std::make_shared<openvslam::config>(config_file_path->value());
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

#ifdef USE_GOOGLE_PERFTOOLS
    ProfilerStart("slam.prof");
#endif

    // run tracking
    if (cfg->camera_->setup_type_ == openvslam::camera::setup_type_t::Monocular) {
        mono_tracking(cfg, vocab_file_path->value(), data_dir_path->value(),
                      frame_skip->value(), no_sleep->is_set(), auto_term->is_set(),
                      eval_log->is_set(), map_db_path->value());
    }
    else if (cfg->camera_->setup_type_ == openvslam::camera::setup_type_t::RGBD) {
        rgbd_tracking(cfg, vocab_file_path->value(), data_dir_path->value(),
                      frame_skip->value(), no_sleep->is_set(), auto_term->is_set(),
                      eval_log->is_set(), map_db_path->value());
    }
    else {
        throw std::runtime_error("Invalid setup type: " + cfg->camera_->get_setup_type_string());
    }

#ifdef USE_GOOGLE_PERFTOOLS
    ProfilerStop();
#endif

    return EXIT_SUCCESS;
}
