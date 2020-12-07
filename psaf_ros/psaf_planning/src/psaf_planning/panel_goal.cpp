#include "panel_goal.h"
#include <QValidator>
#include <QFormLayout>
#include <QIcon>
#include <QPainter>
#include <QPixmap>
#include <QVBoxLayout>
#include <sensor_msgs/NavSatFix.h>

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(psaf_goal::GoalPanel, rviz::Panel)

namespace psaf_goal {
    GoalPanel::GoalPanel(QWidget *parent): rviz::Panel(parent) {
        QVBoxLayout *layout = new QVBoxLayout;
        QHBoxLayout *inputContainer = new QHBoxLayout;

        QFormLayout *gpsLayout = new QFormLayout;
        leLongitude = new QLineEdit("0.0");
        leLongitude->setValidator( new QDoubleValidator(-180, 180, 9, this) ); //longitude ranges from -180 to 180. 9 decimals result in a precision of 0.1mm
        gpsLayout->addRow("Longitude", leLongitude);

        leLatitude = new QLineEdit("0.0");
        leLatitude->setValidator( new QDoubleValidator(-90, 90, 9, this) ); //latitude ranges from -90 to 90. 9 decimals result in a precision of 0.1mm
        gpsLayout->addRow("Latitude", leLatitude);

        leAltitude = new QLineEdit("0.0");
        leAltitude->setValidator( new QDoubleValidator(-500, 500, 9, this) );
        gpsLayout->addRow("Altitude", leAltitude);

        inputContainer->addLayout(gpsLayout);

        layout->addLayout(inputContainer);

        QFormLayout *carlaLayout = new QFormLayout;

        QHBoxLayout *synchronous_layout = new QHBoxLayout;
        QPixmap pixmap(":/icons/play.png");
        QIcon iconPlay(pixmap);
        bPublish = new QPushButton(iconPlay, "");
        synchronous_layout->addWidget(bPublish);
        connect(bPublish, SIGNAL(released()), this, SLOT(sendGoal())); // connect the sendGoal methode with the button

        carlaLayout->addRow("Plan Path:", synchronous_layout);

        layout->addLayout(carlaLayout);

        setLayout(layout);
        goalPublisher = nodeHandle.advertise<sensor_msgs::NavSatFix>("/psaf/goal/set", 1);
        }

    void GoalPanel::setGoal(float latitude, float longitude, float altitude)
    {
        // save gps coordinates
        this->latitude = latitude;
        this->longitude = longitude;
        this->altitude = altitude;
    }

    void GoalPanel::sendGoal()
    {
        if (ros::ok() && goalPublisher) {
            setGoal(leLatitude->text().toDouble(),leLongitude->text().toDouble(),leAltitude->text().toDouble());
            ROS_INFO("Send gps: %lf lat, %lf long, %lf alt",this->latitude, this->longitude, this->altitude);
            sensor_msgs::NavSatFix msg;
            msg.latitude = this->latitude;
            msg.longitude = this->longitude;
            msg.altitude = this->altitude;
            goalPublisher.publish(msg);
        }
    }

} // end namespace psaf_goal
