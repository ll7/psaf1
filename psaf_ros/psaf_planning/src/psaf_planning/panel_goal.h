#pragma once
#include <ros/ros.h>
#include <rviz/panel.h>
#include <QLineEdit>
#include <QPushButton>

namespace psaf_goal {
    class GoalPanel : public rviz::Panel {
        Q_OBJECT
    public:
        GoalPanel(QWidget *parent = 0);

    public Q_SLOTS:
        void sendGoal();

    private:
        void setGoal(float latitude, float longitude, float altitude);
        QPushButton *bPublish;
        QLineEdit *leLongitude;
        QLineEdit *leLatitude;
        QLineEdit *leAltitude;
        ros::Publisher goalPublisher;

        ros::NodeHandle nodeHandle;
        //gps coordinates: current goal
        double latitude{0.0};
        double longitude{0.0};
        double altitude{0.0};
    };
} // end namespace psaf_goal
