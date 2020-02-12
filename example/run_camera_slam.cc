#ifdef USE_PANGOLIN_VIEWER
#include "pangolin_viewer/viewer.h"
#elif USE_SOCKET_PUBLISHER
#include "socket_publisher/publisher.h"
#endif
#include "openvslam/system.h"
#include "openvslam/config.h"
#include "openvslam/util/stereo_rectifier.h"
#include "openvslam/imu.h"
#include <iostream>
#include <chrono>
#include <numeric>
#include "librealsense2/rs.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <spdlog/spdlog.h>
#include <popl.hpp>


class size;

#ifdef USE_STACK_TRACE_LOGGER
#include <glog/logging.h>
#endif

#ifdef USE_GOOGLE_PERFTOOLS
#include <gperftools/profiler.h>
#endif
#include <iostream>
#include <chrono>

class Timer
{
public:
    Timer() : beg_(clock_::now()) {}
    void reset() { beg_ = clock_::now(); }
    double elapsed() const {
        return std::chrono::duration_cast<second_>
                (clock_::now() - beg_).count(); }

private:
    typedef std::chrono::high_resolution_clock clock_;
    typedef std::chrono::duration<double, std::ratio<1> > second_;
    std::chrono::time_point<clock_> beg_;
};
cv::Mat frame_to_mat(const rs2::frame& f)
{
    using namespace cv;
    using namespace rs2;

    auto vf = f.as<video_frame>();
    const int w = vf.get_width();
    const int h = vf.get_height();

    if (f.get_profile().format() == RS2_FORMAT_BGR8)
    {
        return Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_RGB8)
    {
        auto r = Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
        cvtColor(r, r, COLOR_RGB2BGR);
        return r;
    }
    else if (f.get_profile().format() == RS2_FORMAT_Z16)
    {
        return Mat(Size(w, h), CV_16UC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_Y8)
    {
        return Mat(Size(w, h), CV_8UC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_DISPARITY32)
    {
        return Mat(Size(w, h), CV_32FC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }

    throw std::runtime_error("Frame format is not supported yet!");
}

void mono_tracking(const std::shared_ptr<openvslam::config>& cfg,
                   const std::string& vocab_file_path, const std::string& mask_img_path,
                    const std::string& map_db_path) {
    // load the mask image
    const cv::Mat mask = mask_img_path.empty() ? cv::Mat{} : cv::imread(mask_img_path, cv::IMREAD_GRAYSCALE);

    // build a SLAM system
    openvslam::system SLAM(cfg, vocab_file_path);
    // startup the SLAM process
    SLAM.startup();
    rs2::pipeline p;
    rs2::config c;
    std::cout << "now camera is opened an pineline is started"<< std::endl;
    // Configure and start the pipeline
//    c.disable_all_streams();
    c.enable_stream(RS2_STREAM_COLOR,1280,720,RS2_FORMAT_ANY,0);
//    c.enable_stream(RS2_STREAM_DEPTH,1280,720,RS2_FORMAT_ANY,0);

    p.start(c);
    // create a viewer object
    // and pass the frame_publisher and the map_publisher
#ifdef USE_PANGOLIN_VIEWER
    pangolin_viewer::viewer viewer(cfg, &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());
#elif USE_SOCKET_PUBLISHER
    socket_publisher::publisher publisher(cfg, &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());
#endif

//    auto video = cv::VideoCapture(cam_num);
//    if (!video.isOpened()) {
//        spdlog::critical("cannot open a camera {}", cam_num);
//        SLAM.shutdown();
//        return;
//    }

//    cv::Mat frame;
    double timestamp = 0.0;
    std::vector<double> track_times;

    unsigned int num_frame = 0;
    // run the SLAM in another thread
    std::thread thread([&]() {
        while (true){
            if (SLAM.terminate_is_requested()) {
                break;
            }//std::cout << "waiting for frame"<< std::endl;

            rs2::frameset frames = p.wait_for_frames();

//             is_not_end = video.read(frame);
//            std::cout << "yo frames "<< frames.get_color_frame() << std::endl;

            const auto rgb_img = frame_to_mat(frames.get_color_frame());
         //   cv::Size s=rgb_img.size();
        //    int rows = s.height;
    //        int cols = s.width;
    //        std::cout << "The color size is:"<< rows<<"   " << cols  << std::endl;
   //         std::cout << "rgb taken"<< std::endl;
//            if (scale != 1.0) {
//                cv::resize(frames, frames, cv::Size(), scale, scale, cv::INTER_LINEAR);
//            }

            const auto tp_1 = std::chrono::steady_clock::now();

            // input the current frame and estimate the camera pose
            SLAM.feed_monocular_frame(rgb_img, timestamp, mask);

            const auto tp_2 = std::chrono::steady_clock::now();

            const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
            track_times.push_back(track_time);

            timestamp += 1.0 / cfg->camera_->fps_;
            ++num_frame;
        }

        // wait until the loop BA is finished
        while (SLAM.loop_BA_is_running()) {
            std::this_thread::sleep_for(std::chrono::microseconds(5000));
        }
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

    if (!map_db_path.empty()) {
        // output the map database
        SLAM.save_map_database(map_db_path);
    }

    std::sort(track_times.begin(), track_times.end());
    const auto total_track_time = std::accumulate(track_times.begin(), track_times.end(), 0.0);
    std::cout << "median tracking time: " << track_times.at(track_times.size() / 2) << "[s]" << std::endl;
    std::cout << "mean tracking time: " << total_track_time / track_times.size() << "[s]" << std::endl;
}



//cv::Mat depth_frame_to_meters(const rs2::pipeline& pipe, const rs2::depth_frame& f)
//{
//    using namespace cv;
//    using namespace rs2;
//
//    Mat dm = frame_to_mat(f);
//    dm.convertTo(dm, CV_64F);
//    auto depth_scale = pipe.get_active_profile()
//            .get_device()
//            .first<depth_sensor>()
//            .get_depth_scale();
//    dm = dm * depth_scale;
//    return dm;
//}
void rgbd_tracking(const std::shared_ptr<openvslam::config>& cfg,
                   const std::string& vocab_file_path, const std::string& mask_img_path,
                   const std::string& map_db_path) {
    const cv::Mat mask = mask_img_path.empty() ? cv::Mat{} : cv::imread(mask_img_path, cv::IMREAD_GRAYSCALE);
//    std::cout << "started rgbd track masking"<< std::endl;
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

//    auto video = cv::VideoCapture(cam_num);
//    if (!video.isOpened()) {std::cout << "inside camera is not opened"<< std::endl;
//        spdlog::critical("cannot open a camera {}", cam_num);
//        SLAM.shutdown();
//        return;
//    }



    rs2::config c;
  //  std::cout << "now camera is opened an pineline is started"<< std::endl;
    // Configure and start the pipeline
//    c.disable_all_streams();
    c.enable_stream(RS2_STREAM_COLOR,1280,720,RS2_FORMAT_ANY,0);
    c.enable_stream(RS2_STREAM_DEPTH,1280,720,RS2_FORMAT_ANY,0);
//    c.enable_stream(RS2_STREAM_GYRO);
//    c.enable_stream(RS2_STREAM_ACCEL);
//    c.enable_stream(RS2_STREAM_POSE);
    
    rs2::pipeline p;
    p.start(c);

//    cv::Mat frame;
    double timestamp = 0.0;
    std::vector<double> track_times;
    unsigned int num_frame = 0;

    // run the SLAM in another thread

    std::thread thread([&]() {try{
        while (true) {//std::cout << "camera loop started"<< std::endl;
            if (SLAM.terminate_is_requested()) {
                break;
            }//std::cout << "waiting for frame"<< std::endl;

            rs2::frameset frames = p.wait_for_frames();

//             is_not_end = video.read(frame);
//            std::cout << "yo frames "<< frames.get_color_frame() << std::endl;
            const auto depth_img = frame_to_mat(frames.get_depth_frame());

            const auto rgb_img = frame_to_mat(frames.get_color_frame());

            const auto tp_1 = std::chrono::steady_clock::now();
//            frame=frame_to_mat(frames);
//            i++;
            if (!frames) {//std::cout << "empty frame detected"<< std::endl;
                continue;
            }
//            if (scale != 1.0) {
//                cv::resize(frame, frame, cv::Size(), scale, scale, cv::INTER_LINEAR);
//            }
//            for(int j=0;j<10000000;j++);
            if (frames) {
                        //std::cout << "inside slam feedbACK"<< std::endl;
                        // input the current frame and estimate the camera pose
                        openvslam::Mat44_t camera_pose_check=SLAM.feed_RGBD_frame(rgb_img, depth_img,timestamp);


            }

            const auto tp_2 = std::chrono::steady_clock::now();

            const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();

                track_times.push_back(track_time);

            // wait until the timestamp of the next frame
            timestamp += 1.0 / cfg->camera_->fps_;
            ++num_frame;

            // check if the termination of SLAM system is requested or not

        }

        // wait until the loop BA is finished
        while (SLAM.loop_BA_is_running()) {
            std::this_thread::sleep_for(std::chrono::microseconds(5000));
        }

        // automatically close the viewer
#ifdef USE_PANGOLIN_VIEWER
            viewer.request_terminate();
#elif USE_SOCKET_PUBLISHER

            publisher.request_terminate();
#endif
    }catch(std::exception& e) {
        std::cerr << "exception run_camera_slam is : " <<e.what()<< std::endl;}});

    // run the viewer in the current thread
#ifdef USE_PANGOLIN_VIEWER
    viewer.run();
#elif USE_SOCKET_PUBLISHER
    publisher.run();
#endif

    thread.join();

    // shutdown the SLAM process
    SLAM.shutdown();
//    if (eval_log) {
//        // output the trajectories for evaluation
//        SLAM.save_frame_trajectory("frame_trajectory.txt", "TUM");
//        SLAM.save_keyframe_trajectory("keyframe_trajectory.txt", "TUM");
//        // output the tracking times for evaluation
//        std::ofstream ofs("track_times.txt", std::ios::out);
//        if (ofs.is_open()) {
//            for (const auto track_time : track_times) {
//                ofs << track_time << std::endl;
//            }
//            ofs.close();
//        }
//    }

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
                   const std::string& vocab_file_path, const std::string& mask_img_path,
                   const std::string& map_db_path) {
    const cv::Mat mask = mask_img_path.empty() ? cv::Mat{} : cv::imread(mask_img_path, cv::IMREAD_GRAYSCALE);
//    std::cout << "started rgbd track masking"<< std::endl;
    // build a SLAM system
    openvslam::imu data;
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

//    auto video = cv::VideoCapture(cam_num);
//    if (!video.isOpened()) {std::cout << "inside camera is not opened"<< std::endl;
//        spdlog::critical("cannot open a camera {}", cam_num);
//        SLAM.shutdown();
//        return;
//    }



    rs2::config c;
    //  std::cout << "now camera is opened an pineline is started"<< std::endl;
    // Configure and start the pipeline
//    c.disable_all_streams();
    c.enable_stream(RS2_STREAM_COLOR,1280,720,RS2_FORMAT_ANY,0);
    c.enable_stream(RS2_STREAM_DEPTH,1280,720,RS2_FORMAT_ANY,0);
    c.enable_stream(RS2_STREAM_GYRO);
    c.enable_stream(RS2_STREAM_ACCEL);
//    c.enable_stream(RS2_STREAM_POSE);

    rs2::pipeline p;
    p.start(c);

//    cv::Mat frame;
    double timestamp = 0.0;
    std::vector<double> track_times;
    unsigned int num_frame = 0;

    // run the SLAM in another thread
    std::cout<<"slam dunk  "<<std::endl;
    std::thread thread([&]() {try{
        Timer delta_t;
        while (true) {//std::cout << "camera loop started"<< std::endl;
            openvslam::Mat44_t camera_pose_check=openvslam::Mat44_t::Zero();
            if (SLAM.terminate_is_requested()) {
                break;
            }//std::cout << "waiting for frame"<< std::endl;

            rs2::frameset frames = p.wait_for_frames();

            const auto depth_img = frame_to_mat(frames.get_depth_frame());

            const auto rgb_img = frame_to_mat(frames.get_color_frame());
            const auto tp_1 = std::chrono::steady_clock::now();
            std::cout<<"slam tread dunk "<<std::endl;
            if (!frames) {//std::cout << "empty frame detected"<< std::endl;
                continue;
            }
//            if (scale != 1.0) {
//                cv::resize(frame, frame, cv::Size(), scale, scale, cv::INTER_LINEAR);
//            }
//            for(int j=0;j<10000000;j++);

            if (frames){
                {
//                rs2::motion_frame accel_frame = frames.first_or_default(RS2_STREAM_ACCEL);
//                rs2::motion_frame gyro_frame = frames.first_or_default(RS2_STREAM_GYRO);


                    camera_pose_check = SLAM.feed_RGBD_frame(rgb_img, depth_img, timestamp);

                        //                        std::cout<<"camera time:  "<<out<<std::endl;

//                        std::cout<<"camera pose:  "<<Eigen::Vector3d(camera_pose_check(0,3),camera_pose_check(0,3),camera_pose_check(0,3))<<std::endl;

                    }
//                std::cout<<"camera pose:  "<<camera_x<<"  "<<camera_y<<"  "<<camera_z<<std::endl;
            }
            rs2::motion_frame gyro_frame = frames.first_or_default(RS2_STREAM_GYRO);rs2::motion_frame accel_frame = frames.first_or_default(RS2_STREAM_ACCEL);
            if(gyro_frame && accel_frame){
                std::cout << "gyro ka locha  " << std::endl;

                rs2_vector accel = accel_frame.get_motion_data();
                rs2_vector gyro = gyro_frame.get_motion_data();

                Eigen::Vector3d acc_eigen = Eigen::Vector3d(accel.x, accel.y, accel.z);
                std::cout << "acc" << acc_eigen << std::endl;
                Eigen::Vector3d gyro_eigen = Eigen::Vector3d(gyro.x, gyro.y, gyro.z);
                //std::cout << "inside slam feedbACK"<< std::endl;
                // input the current frame and estimate the camera pose
                std::cout << "timer ka locha  " << std::endl;
                double delta_time = delta_t.elapsed();
                delta_t.reset();
                data.F_tran_matrix(1, 3) = data.F_tran_matrix(2, 4) = data.F_tran_matrix(3, 5) = delta_time;
                data.P_uncer_cov_matrix(3, 3) = data.P_uncer_cov_matrix(4, 4) = data.P_uncer_cov_matrix(5,5) = delta_time;
                Eigen::VectorXd camera_inputs = Eigen::VectorXd::Zero(6);
                camera_inputs(0) = SLAM.camera_x;
                camera_inputs(1) = SLAM.camera_y;
                camera_inputs(2) = SLAM.camera_z;
                camera_inputs(3) = camera_pose_check(0, 3);
                camera_inputs(4) = camera_pose_check(0, 3);
                camera_inputs(5) = camera_pose_check(0, 3);
//                Eigen::VectorXd out= data.filtering_camera_pose_with_imu(data.update_imu_lposition_xyz(data.update_imu_lvelocity_xyz(data.acc_xyz_imu(acc_eigen, gyro_eigen),delta_time),delta_time),camera_inputs);
                std::cout<<"filter ka locha  "<<delta_time<<std::endl;

            }
            const auto tp_2 = std::chrono::steady_clock::now();

            const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();

            track_times.push_back(track_time);

            // wait until the timestamp of the next frame
            timestamp += 1.0 / cfg->camera_->fps_;
            ++num_frame;
            std::cout<<"locha ni khabar nai padti  "<<std::endl;
            // check if the termination of SLAM system is requested or not

        }

        // wait until the loop BA is finished
        while (SLAM.loop_BA_is_running()) {
            std::this_thread::sleep_for(std::chrono::microseconds(5000));
        }

        // automatically close the viewer
#ifdef USE_PANGOLIN_VIEWER
        viewer.request_terminate();
#elif USE_SOCKET_PUBLISHER

        publisher.request_terminate();
#endif
    }catch(std::exception& e) {
        std::cerr << "exception run_camera_slam is : " <<e.what()<< std::endl;}});

    // run the viewer in the current thread
#ifdef USE_PANGOLIN_VIEWER
    viewer.run();
#elif USE_SOCKET_PUBLISHER
    publisher.run();
#endif

    std::cout<<"kai pan thay che  "<<std::endl;
    thread.join();


    // shutdown the SLAM process
    SLAM.shutdown();
//    if (eval_log) {
//        // output the trajectories for evaluation
//        SLAM.save_frame_trajectory("frame_trajectory.txt", "TUM");
//        SLAM.save_keyframe_trajectory("keyframe_trajectory.txt", "TUM");
//        // output the tracking times for evaluation
//        std::ofstream ofs("track_times.txt", std::ios::out);
//        if (ofs.is_open()) {
//            for (const auto track_time : track_times) {
//                ofs << track_time << std::endl;
//            }
//            ofs.close();
//        }
//    }

    if (!map_db_path.empty()) {
        // output the map database
        SLAM.save_map_database(map_db_path);
    }

    std::sort(track_times.begin(), track_times.end());
    const auto total_track_time = std::accumulate(track_times.begin(), track_times.end(), 0.0);
    std::cout << "median tracking time: " << track_times.at(track_times.size() / 2) << "[s]" << std::endl;
    std::cout << "mean tracking time: " << total_track_time / track_times.size() << "[s]" << std::endl;
}
void stereo_tracking(const std::shared_ptr<openvslam::config>& cfg,
                     const std::string& vocab_file_path, const std::string& mask_img_path,
                      const std::string& map_db_path) {
    const cv::Mat mask = mask_img_path.empty() ? cv::Mat{} : cv::imread(mask_img_path, cv::IMREAD_GRAYSCALE);
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

    cv::VideoCapture videos[2];
//    for (int i = 0; i < 2; i++) {
//        videos[i] = cv::VideoCapture(cam_num + i);
//        if (!videos[i].isOpened()) {
//            spdlog::critical("cannot open a camera {}", cam_num + i);
//            SLAM.shutdown();
//            return;
//        }
//    }

    const openvslam::util::stereo_rectifier rectifier(cfg);

    cv::Mat frames[2];
    cv::Mat frames_rectified[2];
    double timestamp = 0.0;
    std::vector<double> track_times;
    unsigned int num_frame = 0;

    bool is_not_end = true;
    // run the SLAM in another thread
    std::thread thread([&]() {
        while (is_not_end) {
            // check if the termination of SLAM system is requested or not
            if (SLAM.terminate_is_requested()) {
                break;
            }

            is_not_end = videos[0].read(frames[0]) && videos[1].read(frames[1]);
            if (frames[0].empty() || frames[1].empty()) {
                continue;
            }
            for (int i = 0; i < 2; i++) {
//                if (scale != 1.0) {
//                    cv::resize(frames[i], frames[i], cv::Size(), scale, scale, cv::INTER_LINEAR);
//                }
            }
            rectifier.rectify(frames[0], frames[1], frames_rectified[0], frames_rectified[1]);

            const auto tp_1 = std::chrono::steady_clock::now();

            // input the current frame and estimate the camera pose
            SLAM.feed_stereo_frame(frames_rectified[0], frames_rectified[1], timestamp, mask);

            const auto tp_2 = std::chrono::steady_clock::now();

            const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
            track_times.push_back(track_time);

            timestamp += 1.0 / cfg->camera_->fps_;
            ++num_frame;
        }

        // wait until the loop BA is finished
        while (SLAM.loop_BA_is_running()) {
            std::this_thread::sleep_for(std::chrono::microseconds(5000));
        }
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

    if (!map_db_path.empty()) {
        // output the map database
        SLAM.save_map_database(map_db_path);
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
try {
    // create options
    popl::OptionParser op("Allowed options");
    auto help = op.add<popl::Switch>("h", "help", "produce help message");
    auto vocab_file_path = op.add<popl::Value<std::string>>("v", "vocab", "vocabulary file path");
//    auto cam_num = op.add<popl::Value<unsigned int>>("n", "number", "camera number");
    auto config_file_path = op.add<popl::Value<std::string>>("c", "config", "config file path");
    auto mask_img_path = op.add<popl::Value<std::string>>("", "mask", "mask image path", "");
//    auto scale = op.add<popl::Value<float>>("s", "scale", "scaling ratio of images", 1.0);
    auto map_db_path = op.add<popl::Value<std::string>>("p", "map-db", "store a map database at this path after SLAM",
                                                        "");
    auto debug_mode = op.add<popl::Switch>("", "debug", "debug mode");
    try {
        op.parse(argc, argv);
    }
    catch (const std::exception &e) {
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
    if (!vocab_file_path->is_set()
        || !config_file_path->is_set()) {
        std::cerr << "invalid arguments" << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // setup logger
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] %^[%L] %v%$");
    if (debug_mode->is_set()) {
        spdlog::set_level(spdlog::level::debug);
    } else {
        spdlog::set_level(spdlog::level::info);
    }

    // load configuration
    std::shared_ptr<openvslam::config> cfg;
    try {
        cfg = std::make_shared<openvslam::config>(config_file_path->value());
    }
    catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

#ifdef USE_GOOGLE_PERFTOOLS
    ProfilerStart("slam.prof");
#endif

    // run tracking
    if (cfg->camera_->setup_type_ == openvslam::camera::setup_type_t::Monocular) {
        mono_tracking(cfg, vocab_file_path->value(), mask_img_path->value(),
                       map_db_path->value());
    } else if (cfg->camera_->setup_type_ == openvslam::camera::setup_type_t::Stereo) {
        stereo_tracking(cfg, vocab_file_path->value(), mask_img_path->value(),
                         map_db_path->value());
    } else if (cfg->camera_->setup_type_ == openvslam::camera::setup_type_t::RGBD) {
        try {
            rgbd_tracking(cfg, vocab_file_path->value(), mask_img_path->value(),
                           map_db_path->value());
        } catch (std::exception &e) {
            std::cerr << "exception first one is : " << e.what() << std::endl;
        }
    }
    else if (cfg->camera_->setup_type_ == openvslam::camera::setup_type_t::RGBDIMU) {
        try {
            rgbd_imu_tracking(cfg, vocab_file_path->value(), mask_img_path->value(),
                          map_db_path->value());
        } catch (std::exception &e) {
            std::cerr << "exception first one is : " << e.what() << std::endl;
        }
    }else {
        throw std::runtime_error("Invalid setup type: " + cfg->camera_->get_setup_type_string());
    }
}catch(std::exception& e) {
    std::cerr << "exception main is : " <<e.what()<< std::endl;}
#ifdef USE_GOOGLE_PERFTOOLS
    ProfilerStop();
#endif

    return EXIT_SUCCESS;
}