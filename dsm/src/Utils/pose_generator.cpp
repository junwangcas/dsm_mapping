//
// Created by junwangcas on 2022/10/19.
//

#include "pose_generator.h"
#include <sstream>

bool PoseGenerator::ReadEurocPose(const std::string file_path)
{
    this->poses_.clear();
    this->time_stamps_.clear();

    //read timestamps and poses equal to timestamps
    std::ifstream infile;
    infile.open(file_path);
    while (!infile.eof() && infile.good())
    {
        std::string line;
        std::getline(infile, line);

        if (!line.empty())
        {
            Eigen::Matrix4f pose;
            double time_stamp;
            if (ParseLine(line, pose, time_stamp)) {
                this->poses_.push_back(pose);
                this->time_stamps_.push_back(time_stamp);
            }
        }
    }
    infile.close();

    if (this->poses_.size() > 0 && this->time_stamps_.size() == this->poses_.size())
    {
        std::cout << __FUNCTION__ << " get poses from file: " <<this->poses_.size() << "\n";
        return true;
    }
    else
    {
        this->poses_.clear();
        this->time_stamps_.clear();
    }

    return false;
}

bool PoseGenerator::ParseLine(const std::string &line, Eigen::Matrix4f &pose, double &time_stamp)
{
//    {
//        static int count = 0;
//        std::cout << __FUNCTION__ << count++ << line << "\n";
//    }
    if (line.size() < 1U) {
        return false;
    }
    if (line[0] == '#') {
        return false;
    }

    // 1403715274302142976,0.878612,2.142470,0.947262,0.060514,-0.828459,-0.058956,-0.553641,0.009474,-0.014009,-0.002145,-0.002229,0.020700,0.076350,-0.012492,0.547666,0.069073
    Eigen::Quaternionf q;
    Eigen::Vector3f p;
    std::istringstream iss(line);
    char dotc = ',';
    iss >> time_stamp >> dotc >> p.x() >> dotc >> p.y() >> dotc >> p.z() >> dotc >> q.w() >> dotc >> q.x() >> dotc >> q.y() >> dotc >> q.z();
//    std::cout << time_stamp << ", " << p.y() <<  ", " << q.z() << "\n"; // "\n";
    pose = Eigen::Matrix4f::Identity();
    pose.block<3, 3>(0, 0) = q.toRotationMatrix();
    pose.block<3, 1>(0, 3) = p;
    time_stamp = time_stamp / 1e9; // transform to seconds
    return true;
}

bool PoseGenerator::GetPoseByTime(const double time_stamp, Eigen::Matrix4f &pose, double time_threshold)
{
    int idx = std::lower_bound(this->time_stamps_.begin(), this->time_stamps_.end(), time_stamp) - this->time_stamps_.begin();
    if (idx < 0U || idx >= this->time_stamps_.size()) {
        std::cout << __FUNCTION__ << " failed " << idx << "\n";
        return false;
    }

    if (std::fabs(time_stamp - this->time_stamps_[idx]) > time_threshold) {
        std::cout << __FUNCTION__ << " failed " << time_stamp << " , " << this->time_stamps_[idx] << "\n";
        return false;
    }

    pose = Eigen::Matrix4f(this->poses_[idx]);
    std::cout << __FUNCTION__ << " get pose success: " << std::to_string(this->time_stamps_[idx]) << "\n";
    return true;
}