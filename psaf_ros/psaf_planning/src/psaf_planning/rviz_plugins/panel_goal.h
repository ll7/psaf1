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
        void setGoal(float latitude, float longitude, float altitude);
        void statusCallback(const std_msgs::String::ConstPtr& msg);
        QPushButton *bPublish;
        QLineEdit *leLongitude;
        QLineEdit *leLatitude;
        QLineEdit *leAltitude;
        QLabel *lstatus;

        QTimer *timer;

        ros::Publisher goalPublisher;
        ros::Subscriber statusSubscriber;
        ros::NodeHandle nodeHandle;
        //gps coordinates: current goal
        double latitude{0.0};
        double longitude{0.0};
        double altitude{0.0};
    };
} // end namespace psaf_goal
