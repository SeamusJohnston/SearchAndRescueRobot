#include "bill_planning/planner.hpp"

Planner::Planner()
{
    drivePoints = std::queue<TilePosition>();
}

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

void Planner::publishDriveToTile(SensorReadings &sensorReadings, const int x, const int y, const float speed)
{
    int heading;
    int currentX = sensorReadings.getCurrentTileX();
    int currentY = sensorReadings.getCurrentTileY();

    // We will prioritize driving the longest leg of the horizontal/vertical drive first
    // If they are the same length, we will drive the vertical one first
    int xDistSq = (currentX - x)*(currentX - x);
    int yDistSq = (currentY - y)*(currentY - y);

    if (xDistSq == 0 && yDistSq == 0)
    {
        ROS_WARN("Attempting to drive to the same tile we are already on, bailing");
        return;
    }
    if (xDistSq <= yDistSq)
    {
        sensorReadings.setTargetPoint(currentX, y);

        drivePoints.emplace(currentX, y);
        heading = currentY > y ? 270 : 90;

        if (xDistSq == 0)
        {
            // We only need one leg to drive to this point
            publishDrive(heading, driveSpeed);
            return;
        }
    }
    else
    {
        sensorReadings.setTargetPoint(x, currentY);

        drivePoints.emplace(x,currentY);
        heading = currentX > x ? 180 : 0;

        if (yDistSq == 0)
        {
            // We only need one leg to drive to this point
            publishDrive(heading, driveSpeed);
            return;
        }
    }

    drivePoints.emplace(x,y);

    // Publish a drive to the first queued point
    publishDrive(heading, driveSpeed);
}

void Planner::ProcessNextDrivePoint(SensorReadings &sensorReadings)
{
    if (!drivePoints.empty())
    {
        int currentX = sensorReadings.getCurrentTileX();
        int currentY = sensorReadings.getCurrentTileY();

        // Make sure we really have arrived at the point we are aiming for
        TilePosition arrivedPosition = drivePoints.front();

        if (arrivedPosition.x != currentX || arrivedPosition.y != currentY)
        {
            ROS_WARN("We are assuming we have arrived at a point when we havent, bailing");
            return;
        }

        // Remove the front point from the queue, we have arrived at it.
        drivePoints.pop();

        if (!drivePoints.empty())
        {
            TilePosition nextLeg = drivePoints.front();
            int heading;

            // One of the two dimensions should always match
            if (currentX == nextLeg.x)
            {
                // Drive in Y
                heading = currentY > nextLeg.y ? 270 : 90;
            }
            else
            {
                // Drive in X
                heading = currentX > nextLeg.x ? 180 : 0;
            }

            sensorReadings.setTargetPoint(nextLeg.x, nextLeg.y);
            publishDrive(heading, driveSpeed);
            return;
        }
    }

    // Write an invalid target value?
    ROS_INFO("Completed drive to point");
    return;
}