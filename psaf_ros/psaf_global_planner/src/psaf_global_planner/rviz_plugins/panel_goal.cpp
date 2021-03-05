#include "panel_goal.h"
#include <QValidator>
#include <QFormLayout>
#include <QIcon>
#include <QPainter>
#include <QPixmap>
#include <QVBoxLayout>
#include <geometry_msgs/Pose.h>

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(psaf_goal::GoalPanel, rviz::Panel)

namespace psaf_goal {
    GoalPanel::GoalPanel(QWidget *parent): rviz::Panel(parent) {
        QVBoxLayout *layout = new QVBoxLayout;
        QHBoxLayout *inputContainer = new QHBoxLayout;

        QFormLayout *gpsLayout = new QFormLayout;
        leX = new QLineEdit("0.0");
        leX->setValidator( new QDoubleValidator(-500, 500, 9, this) );
        gpsLayout->addRow("X", leX);

        leY = new QLineEdit("0.0");
        leY->setValidator( new QDoubleValidator(-500, 500, 9, this) );
        gpsLayout->addRow("Y", leY);

        leZ = new QLineEdit("0.0");
        leZ->setValidator( new QDoubleValidator(-500, 500, 9, this) );
        gpsLayout->addRow("Z", leZ);

        inputContainer->addLayout(gpsLayout);

        layout->addLayout(inputContainer);

        QFormLayout *carlaLayout = new QFormLayout;

        QHBoxLayout *button_layout = new QHBoxLayout;
        QPixmap pixmap(":/icons/play.png");
        QIcon iconPlay(pixmap);
        bPublish = new QPushButton(iconPlay, "");
        button_layout->addWidget(bPublish);
        connect(bPublish, SIGNAL(released()), this, SLOT(sendGoal())); // connect the sendGoal methode with the button
        carlaLayout->addRow("Plan Path:", button_layout);

        QHBoxLayout *status_layout = new QHBoxLayout;
        timer = new QTimer(this);
        connect(timer, SIGNAL(timeout()), this, SLOT(spinOnce_Wrapper()));
        timer->start(500);
        lstatus = new QLabel("");
        lstatus->setWordWrap(true);
        lstatus->setMaximumWidth(this->width()-50);
        lstatus->setMidLineWidth(this->width()-50);
        status_layout->addWidget(lstatus);
        carlaLayout->addRow("Status: ", status_layout);
        layout->addLayout(carlaLayout);

        setLayout(layout);
        goalPublisher = nodeHandle.advertise<geometry_msgs::Pose>("/psaf/goal/set", 1);
        statusSubscriber = nodeHandle.subscribe("/psaf/status", 1000, &GoalPanel::statusCallback, this);

        }

    void GoalPanel::setGoal(float x, float y, float z)
    {
        // save gps coordinates
        this->x = x;
        this->y = y;
        this->z = z;
    }

    void GoalPanel::sendGoal()
    {
        if (ros::ok() && goalPublisher) {
            setGoal(leX->text().toFloat(),leY->text().toFloat(),leZ->text().toFloat());
            ROS_INFO("Send position: %lf lat, %lf long, %lf alt",this->x, this->y, this->z);
            geometry_msgs::Pose msg;
            msg.position.x = this->x;
            msg.position.y = this->y;
            msg.position.z = this->z;
            goalPublisher.publish(msg);
        }
    }
    void GoalPanel::spinOnce_Wrapper() {
        ros::spinOnce();
    }
    void GoalPanel::statusCallback(const std_msgs::String::ConstPtr& msg) {
        lstatus->setText(msg.get()->data.c_str());
    }
} // end namespace psaf_goal
