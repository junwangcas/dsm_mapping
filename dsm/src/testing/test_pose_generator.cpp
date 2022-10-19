
#include "Utils/pose_generator.h"

int main(){
    std::string file_name = "/home/junwangcas/dataset/EuRoc/V1_01_easy/mav0/state_groundtruth_estimate0/data.csv";
    PoseGenerator& pose_generator = PoseGenerator::Instance();
    pose_generator.ReadEurocPose(file_name);

    Eigen::Matrix4f pose;
    pose_generator.GetPoseByTime(1403715417597143040.0/1e9, pose);
}