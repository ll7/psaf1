#pragma once
#include <ros/ros.h>
#include <rviz/panel.h>
#include <QLineEdit>
#include <QPushButton>
#include <QTimer>
#include <QLabel>
#include <std_msgs/String.h>

namespace psaf_goal {
    class GoalPanel : public rviz::Panel {
        Q_OBJECT
    public:
        GoalPanel(QWidget *parent = 0);

    public Q_SLOTS:
        void sendGoal();
        void spinOnce_Wrapper();

    private:
        void setGoal(float x, float y, float z);
        void statusCallback(const std_msgs::String::ConstPtr& msg);
        QPushButton *bPublish;
        QLineEdit *leX;
        QLineEdit *leY;
        QLineEdit *leZ;
        QLabel *lstatus;

        QTimer *timer;

        ros::Publisher goalPublisher;
        ros::Subscriber statusSubscriber;
        ros::NodeHandle nodeHandle;
        //gps coordinates: current goal
        float x{0.0};
        float z{0.0};
        float y{0.0};
    };
} // end namespace psaf_goal
