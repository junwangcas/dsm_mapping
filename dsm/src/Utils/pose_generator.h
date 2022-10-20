#ifndef DSM_POSE_GENERATOR_H
#define DSM_POSE_GENERATOR_H
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include "sophus/se3.hpp"

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

public:
    bool usePoseGen_ = true;
    bool saveFile_ = false;


private:
    PoseGenerator() {
    }

    bool ParseLine(const std::string &line, Eigen::Matrix4f &pose, double &time_stamp);

private:
    std::vector<Eigen::Matrix4f> poses_;
    std::vector<double> time_stamps_; // seconds
};


#endif //DSM_POSE_GENERATOR_H
