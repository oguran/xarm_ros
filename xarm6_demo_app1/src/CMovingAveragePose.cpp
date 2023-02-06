#include <xarm6_demo_app1/CMovingAveragePose.h>

CMovingAveragePose::CMovingAveragePose (unsigned char num) {
    // TODO use make_unique
    listPose_ = std::unique_ptr<std::list<geometry_msgs::Pose>>(new std::list<geometry_msgs::Pose>());
    denomitator_ = num;
}

void CMovingAveragePose::averagedPose(geometry_msgs::Pose &pose, geometry_msgs::Pose &avePose) {
    if (listPose_->size() < denomitator_) {
        listPose_->push_back(pose);
    } else {
        listPose_->pop_front();
        listPose_->push_back(pose);
    }
    avePose.position.x = 0.0;
    avePose.position.y = 0.0;
    avePose.position.z = 0.0;
    avePose.orientation.x = 0.0;
    avePose.orientation.y = 0.0;
    avePose.orientation.z = 0.0;
    avePose.orientation.w = 0.0;
    for (auto p : *listPose_.get()) {
        avePose.position.x += p.position.x;
        avePose.position.y += p.position.y;
        avePose.position.z += p.position.z;
#if 1
        avePose.orientation.x += p.orientation.x;
        avePose.orientation.y += p.orientation.y;
        avePose.orientation.z += p.orientation.z;
        avePose.orientation.w += p.orientation.w;
#endif
    }
    avePose.position.x = avePose.position.x / (double)listPose_->size();
    avePose.position.y = avePose.position.y / (double)listPose_->size();
    avePose.position.z = avePose.position.z / (double)listPose_->size();
#if 1
    avePose.orientation.x = avePose.orientation.x / (double)listPose_->size();
    avePose.orientation.y = avePose.orientation.y / (double)listPose_->size();
    avePose.orientation.z = avePose.orientation.z / (double)listPose_->size();
    avePose.orientation.w = avePose.orientation.w / (double)listPose_->size();
#else
    avePose.orientation.x = pose.orientation.x;
    avePose.orientation.y = pose.orientation.y;
    avePose.orientation.z = pose.orientation.z;
    avePose.orientation.w = pose.orientation.w;
#endif
}


