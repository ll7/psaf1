#include "panel_goal.h"
#include <QValidator>
#include <QFormLayout>
#include <QIcon>
#include <QPainter>
#include <QPixmap>
#include <QVBoxLayout>

#include <sensor_msgs/NavSatFix.h>


namespace psaf_goal {
    GoalPanel::GoalPanel(QWidget *parent): rviz::Panel(parent) {
        QVBoxLayout *layout = new QVBoxLayout;
        QHBoxLayout *inputContainer = new QHBoxLayout;


        QFormLayout *gpsLayout = new QFormLayout;
        leLongitude = new QLineEdit("0.0");
        leLongitude->setValidator( new QDoubleValidator(-500, 500, 100, this) );
        gpsLayout->addRow("Longitude", leLongitude);

        leLatitude = new QLineEdit("0.0");
        leLatitude->setValidator( new QDoubleValidator(-500, 500, 100, this) );
        gpsLayout->addRow("Latitude", leLatitude);

        leAltitude = new QLineEdit("0.0");
        leAltitude->setValidator( new QDoubleValidator(-500, 500, 100, this) );
        gpsLayout->addRow("Altitude", leAltitude);

        inputContainer->addLayout(gpsLayout);

        layout->addLayout(inputContainer);

        QFormLayout *carlaLayout = new QFormLayout;

        QHBoxLayout *synchronous_layout = new QHBoxLayout;
        QPixmap pixmap(":/icons/play.png");
        QIcon iconPlay(pixmap);
        bPubish = new QPushButton(iconPlay, "");
        synchronous_layout->addWidget(bPubish);
        connect(bPubish, SIGNAL(released()), this, SLOT(sendGoal()));

        carlaLayout->addRow("Plan Path:", synchronous_layout);

        layout->addLayout(carlaLayout);

        setLayout(layout);
        goalPublisher = nodeHandle.advertise<sensor_msgs::NavSatFix>("/psaf/goal/set", 1);
         }

    void GoalPanel::setGoal(float latitude, float longitude, float altitude)
    {
        this->latitude = latitude;
        this->longitude = longitude;
        this->altitude = altitude;
    }

    void GoalPanel::sendGoal()
    {
        if (ros::ok() && goalPublisher)
        {
            setGoal(leLatitude->text().toDouble(),leLongitude->text().toDouble(),leAltitude->text().toDouble());
            ROS_INFO("Send values: %lf, %lf, %lf",this->latitude, this->longitude, this->altitude);
            sensor_msgs::NavSatFix msg;
            msg.latitude = this->latitude;
            msg.longitude = this->longitude;
            msg.altitude = this->altitude;
            goalPublisher.publish(msg);
        }
    }

} // end namespace psaf_goal
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(psaf_goal::GoalPanel, rviz::Panel)