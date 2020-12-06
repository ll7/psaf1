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
        QPushButton *bPubish;
        QLineEdit *leLongitude;
        QLineEdit *leLatitude;
        QLineEdit *leAltitude;
        ros::Publisher goalPublisher;

        ros::NodeHandle nodeHandle;
        //gps coordinates: current goal
        float latitude{0.0};
        float longitude{0.0};
        float altitude{0.0};
    };
} // end namespace psaf_goal
