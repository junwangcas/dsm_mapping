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
    if (data_set_type_ == DATATYPE::EUROC) {
        char dotc = ',';
        iss >> time_stamp >> dotc >> p.x() >> dotc >> p.y() >> dotc >> p.z() >> dotc >> q.w() >> dotc >> q.x() >> dotc >> q.y() >> dotc >> q.z();
        time_stamp = time_stamp / 1e9; // transform to seconds
    } else if (data_set_type_ == DATATYPE::VR) {
        std::vector<std::string> strs;
        std::string spacestr;
        while (iss >> spacestr) {
            strs.emplace_back(spacestr);
        }
        time_stamp = std::atof(strs[0].c_str());
        p.x() = std::atof(strs[1].c_str());
        p.y() = std::atof(strs[2].c_str());
        p.z() = std::atof(strs[3].c_str());
        q.x() = std::atof(strs[4].c_str());
        q.y() = std::atof(strs[5].c_str());
        q.z() = std::atof(strs[6].c_str());
        q.w() = std::atof(strs[7].c_str());
//        iss >> time_stamp >> spacestr >> p.x() >> spacestr >> p.y() >> spacestr >> p.z() >> spacestr >> q.x() >> spacestr >> q.y() >> spacestr >> q.z() >> spacestr >> q.w();
    }
//    std::cout << time_stamp << ", " << p.y() <<  ", " << q.z() << "\n"; // "\n";
    pose = Eigen::Matrix4f::Identity();
    pose.block<3, 3>(0, 0) = q.toRotationMatrix();
    pose.block<3, 1>(0, 3) = p;
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
    PrintPose(pose);
    return true;
}

void PoseGenerator::SaveToFile(const double time_stamp, const Eigen::Matrix4f camPose)
{
    if (!saveFile_) {
        return;
    }
    std::string file_name = pose_save_file_; //"/home/junwangcas/Documents/temp/dsm-master/Examples/EurocData/temp/V1_01_easy_short.txt";

    std::string line_str = "";
    line_str = line_str + std::to_string(uint64_t(time_stamp * 1e9)) + ",";

    Eigen::Quaternionf q = Eigen::Quaternionf(camPose.block<3, 3>(0, 0));
    Eigen::Vector3f p = camPose.block<3, 1>(0, 3);

    line_str += std::to_string(p.x()) + "," + std::to_string(p.y()) + "," + std::to_string(p.z()) + ",";

    line_str += std::to_string(q.w()) + "," + std::to_string(q.x()) + "," + std::to_string(q.y()) + "," + std::to_string(q.z()) + "\n";

    std::ofstream fout(file_name, std::ios::app);
    fout << line_str;
    fout.close();
}

void PoseGenerator::PrintPose(const Eigen::Matrix4f camPose)
{
    Eigen::Quaternionf q = Eigen::Quaternionf(camPose.block<3, 3>(0, 0));
    Eigen::Vector3f p = camPose.block<3, 1>(0, 3);

    std::cout << p.transpose() << ", " << q.coeffs().transpose() << "\n";
}

void PoseGenerator::SetSavePoseFile(const std::string &file_path)
{
    pose_save_file_ = file_path;
}