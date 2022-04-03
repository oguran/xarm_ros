#include <xarm6_demo_app1/CMovingAveragePose.h>

MovingAveragePose::MovingAveragePose (unsigned char num) {
    // TODO use make_unique
    listPose = std::unique_ptr<std::list<geometry_msgs::Pose>>(new std::list<geometry_msgs::Pose>(num));
}

void MovingAveragePose::averagedPose(geometry_msgs::Pose &pose, geometry_msgs::Pose &avePose) {
    if (listPose->size() < listPose->max_size()) {
        listPose->push_back(pose);
    } else {
        listPose->pop_front();
        listPose->push_back(pose);
    }
    avePose.position.x = 0.0;
    avePose.position.y = 0.0;
    avePose.position.z = 0.0;
    avePose.orientation.x = 0.0;
    avePose.orientation.y = 0.0;
    avePose.orientation.z = 0.0;
    avePose.orientation.w = 0.0;
    for (auto pose : *listPose.get()) {
        avePose.position.x += pose.position.x;
        avePose.position.y += pose.position.y;
        avePose.position.z += pose.position.z;
#if 0
        avePose.orientation.x += pose.orientation.x;
        avePose.orientation.y += pose.orientation.y;
        avePose.orientation.z += pose.orientation.z;
        avePose.orientation.w += pose.orientation.w;
#endif
    }
    avePose.position.x = avePose.position.x / (double)listPose->size();
    avePose.position.y = avePose.position.y / (double)listPose->size();
    avePose.position.z = avePose.position.z / (double)listPose->size();
#if 0
    avePose.orientation.x = avePose.orientation.x / (double)listPose->size();
    avePose.orientation.y = avePose.orientation.y / (double)listPose->size();
    avePose.orientation.z = avePose.orientation.z / (double)listPose->size();
    avePose.orientation.w = avePose.orientation.w / (double)listPose->size();
#else
    avePose.orientation.x = pose.orientation.x;
    avePose.orientation.y = pose.orientation.y;
    avePose.orientation.z = pose.orientation.z;
    avePose.orientation.w = pose.orientation.w;
#endif
}


