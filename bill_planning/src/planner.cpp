#include "bill_planning/planner.hpp"

void Planner::setPubs(const ros::Publisher mp, const ros::Publisher fp, const ros::Publisher lp)
{
    _motor_pub = mp;
    _fan_pub = fp;
    _led_pub = lp;
}

void Planner::gridSearch()
{
    // TODO: Implement grid search
}

void Planner::publishStop()
{
    ROS_INFO("Commanding stop");
    _command_msg.command = bill_msgs::MotorCommands::STOP;
    _command_msg.heading = 0;
    _command_msg.speed = 0;
    _motor_pub.publish(_command_msg);
    is_moving = true;
}

void Planner::publishDrive(const int heading, const float speed)
{
    ROS_INFO("Commanding drive, heading: %i and speed: %f", heading, speed);
    _command_msg.command = bill_msgs::MotorCommands::DRIVE;
    _command_msg.heading = heading;
    _command_msg.speed = speed;
    _motor_pub.publish(_command_msg);
    is_moving = false;
}

void Planner::publishTurn(const int heading)
{
    ROS_INFO("Commanding turn, heading: %i", heading);
    _command_msg.command = bill_msgs::MotorCommands::TURN;
    _command_msg.heading = heading;
    _command_msg.speed = 0;  // Speed is hardcoded in the motor driver for turning
    _motor_pub.publish(_command_msg);
    is_moving = false;
}

void Planner::putOutFire()
{
    publishStop();

    // Turn ON/OFF Fan
    std_msgs::Bool cmd;
    cmd.data = true;
    _fan_pub.publish(cmd);  // Turn on fan
    ros::Duration(2).sleep();
    cmd.data = false;
    _fan_pub.publish(cmd);  // Turn off fan
}

void Planner::signalComplete()
{
    publishStop();
    std_msgs::Bool msg;
    msg.data = true;
    _led_pub.publish(msg);
    ros::Duration(2).sleep();
    msg.data = false;
    _led_pub.publish(msg);
}

void Planner::publishDriveToTile(const int currentX, const int currentY, const int x, const int y, const float speed)
{
    // Only publish a set of horizontal/vertical drive commands if we arent currently in the middle of processing a set
    if (drivePoints.empty())
    {
        int heading;

        // We will prioritize driving the shortest leg of the horizontal/vertical drive first
        // If they are the same length, we will drive the horizontal one first
        int xDistSq = (currentX - x)*(currentX - x);
        int yDistSq = (currentY - y)*(currentY - y);

        if (xDistSq <= yDistSq)
        {
            drivePoints.emplace(x,currentY);
            heading = currentX > x ? 180 : 0;
        }
        else
        {
            drivePoints.emplace(currentX, y);
            heading = currentY > y ? 270 : 90;
        }

        drivePoints.emplace(x,y);

        // Publish a drive to the first queued point
        publishDrive(heading, driveSpeed);

        // WRITE TO SENSOR READINGS OUR CURRENT TARGET POINT
    }
    else
    {
        ROS_WARN("Attempting to drive to point while we are already driving to point");
    }
}

void Planner::ProcessNextDrivePoint(const int currentX, const int currentY)
{
    if (!drivePoints.empty())
    {
        // Remove the front point from the queue, we have arrived at it.
        drivePoints.pop();

        if (!drivePoints.empty())
        {
            TilePosition secondLeg = drivePoints.front();
            int heading;

            if (currentX == secondLeg.x)
            {
                // Drive in Y
                heading = currentY > secondLeg.y ? 270 : 90;
            }
            else
            {
                // Drive in X
                heading = currentX > secondLeg.x ? 180 : 0;
            }

            publishDrive(heading, driveSpeed);

            // WRITE TO SENSOR READINGS OUT CURRENT TARGET POINT;
        }
        else
        {
            // WRITE TO SENSOR READINGS AN INVALID TARGET POINT VALUE;
            ROS_INFO("Completed drive to point");
            return;
        }
    }
}