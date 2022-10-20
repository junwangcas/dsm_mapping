#ifndef DSM_POSE_GENERATOR_H
#define DSM_POSE_GENERATOR_H
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include "sophus/se3.hpp"

enum DATATYPE {
    VR,
    EUROC
};

class PoseGenerator {
public:
    ~PoseGenerator() {
    }
    PoseGenerator(const PoseGenerator&) = delete;
    PoseGenerator& operator=(const PoseGenerator&) = delete;
    static PoseGenerator& Instance() {
        static PoseGenerator m_pInstance;
        return m_pInstance;
    }

public:
    bool ReadEurocPose(const std::string file_path);
    bool GetPoseByTime(const double time_stamp, Eigen::Matrix4f &pose, double time_threshold = 0.01); // sec,
    void SaveToFile(const double time_stamp, const Eigen::Matrix4f camPose);
    void PrintPose(const Eigen::Matrix4f camPose);
    void SetSavePoseFile(const std::string &file_path);

public:
    bool usePoseGen_ = false;
    bool saveFile_ = false;
    DATATYPE data_set_type_ = DATATYPE::EUROC;


private:
    PoseGenerator() {
    }

    bool ParseLine(const std::string &line, Eigen::Matrix4f &pose, double &time_stamp);

private:
    std::vector<Eigen::Matrix4f> poses_;
    std::vector<double> time_stamps_; // seconds
    std::string pose_save_file_;
};


#endif //DSM_POSE_GENERATOR_H
