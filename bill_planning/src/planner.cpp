#include "bill_planning/planner.hpp"

Planner::Planner()
{
    drivePoints = std::list<TilePosition>();
    graphPath = GraphPath();
}

void Planner::setPubs(const ros::Publisher mp, const ros::Publisher fp, const ros::Publisher lp)
{
    _motor_pub = mp;
    _fan_pub = fp;
    _led_pub = lp;
    while(_motor_pub.getNumSubscribers() == 0 && _fan_pub.getNumSubscribers() == 0 && _led_pub.getNumSubscribers() == 0)
    {}
}

void Planner::publishStop()
{
    ROS_INFO("Commanding stop");
    _command_msg.command = bill_msgs::MotorCommands::STOP;
    _command_msg.heading = 0;
    _command_msg.speed = 0;
    _motor_pub.publish(_command_msg);
    is_moving = false;
}

void Planner::publishDrive(const int heading, const float speed)
{
    ROS_INFO("Commanding drive, heading: %i and speed: %f", heading, speed);
    _command_msg.command = bill_msgs::MotorCommands::DRIVE;
    _command_msg.heading = heading;
    _command_msg.speed =  speed < 0.3 ? speed : 0.3;
    _motor_pub.publish(_command_msg);
    is_moving = true;
}

void Planner::publishTurn(const int heading)
{
    ROS_INFO("Commanding turn, heading: %i", heading);
    _command_msg.command = bill_msgs::MotorCommands::TURN;
    _command_msg.heading = heading;
    _command_msg.speed = 0;  // Speed is hardcoded in the motor driver for turning
    _motor_pub.publish(_command_msg);
    is_moving = true;
}

void Planner::putOutFire()
{
    publishStop();

    // Turn ON/OFF Fan
    std_msgs::Bool cmd;
    cmd.data = true;
    _fan_pub.publish(cmd);  // Turn on fan
    ros::Duration(5).sleep();
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

void Planner::publishDriveToTile(SensorReadings &sensorReadings, const int x, const int y, const float speed, bool scanOnReach)
{
    ROS_INFO("Driving to tile: %i, %i", x, y);
    int heading;
    int currentX = sensorReadings.getCurrentTileX();
    int currentY = sensorReadings.getCurrentTileY();

    if ((currentX == x) && (currentY == y))
    {
        ROS_WARN("Attempting to drive to tile we are currently on, bailing.");
        return;
    }

    ROS_INFO("Using Graph to determine shortest path, starting at %i, %i", currentX, currentY);
    TilePosition start(currentX, currentY);
    TilePosition dest(x, y);
    graphPath.getShortestPath(drivePoints, start, dest, scanOnReach);

    // If we successfully found a path
    // If the list returns empty, that means the path could not be generated.
    if (!drivePoints.empty())
    {
        TilePosition nextLeg = drivePoints.front();

        float vX = (nextLeg.x + 0.5) - (sensorReadings.getCurrentPositionX() / 30.0);
        float vY = (nextLeg.y + 0.5) - (sensorReadings.getCurrentPositionY() / 30.0);

        float oX = 1;
        float oY = 0;

        double dot = oX*vX + oY*vY;
        double det = oX*vY - oY*vX;

        heading = (int)round(atan2(det, dot) * (180.0 / M_PI));

        //ROS_INFO("Targeting new point: %i, %i", nextLeg.x, nextLeg.y);
        sensorReadings.setTargetPoint(nextLeg.x, nextLeg.y);
        sensorReadings.setTargetHeading(heading);
        publishTurn(heading);
    }
    else
    {
        // If for some reason we cant make it to this point, then stop movement and invalidate.
        publishStop();
        sensorReadings.setTargetHeading(-1);
    }
}

void Planner::cancelDriveToTile(SensorReadings &sensorReadings)
{
    // Reset all data structures and targets
    sensorReadings.setTargetHeading(-1);
    sensorReadings.invalidateTargetTile();
    drivePoints.clear();

    // Stop all movement
    publishStop();
}

void Planner::driveAroundObstacle(SensorReadings &sensorReadings) {
    int currentHeading = sensorReadings.getCurrentHeading();
    int obstacleIndex;

    int currentX = sensorReadings.getCurrentTileX();
    int currentY = sensorReadings.getCurrentTileY();

    // Place the obstacle in the graph as necessary based on heading
    // Obstacles are always assumed to be in the next tile forward
    // Remove the edges around the obstacle
    // Facing 0
    if (currentHeading >= 0 && currentHeading < 90) {
        int obstacleX = currentX + 1;
        int obstacleY = currentY;

        obstacleIndex = (obstacleY * 6) + obstacleX;
    }
        // Facing 90
    else if (currentHeading >= 90 && currentHeading < 180) {
        int obstacleX = currentX;
        int obstacleY = currentY + 1;

        obstacleIndex = (obstacleY * 6) + obstacleX;
    }
        // Facing 180
    else if (currentHeading >= 180 && currentHeading < 270) {
        int obstacleX = currentX - 1;
        int obstacleY = currentY;

        obstacleIndex = (obstacleY * 6) + obstacleX;
    }
        // Facing 270
    else {
        int obstacleX = currentX;
        int obstacleY = currentY - 1;

        obstacleIndex = (obstacleY * 6) + obstacleX;
    }

    // Remove the 4 connecting edges to the obstacle node
    graphPath.remove_edge(obstacleIndex, obstacleIndex + 1);
    graphPath.remove_edge(obstacleIndex, obstacleIndex - 1);
    graphPath.remove_edge(obstacleIndex, obstacleIndex + 6);
    graphPath.remove_edge(obstacleIndex, obstacleIndex - 6);

    // Get the original target
    TilePosition originalTarget = drivePoints.back();

    // Lets empty the list
    drivePoints.clear();

    // Find a new path with the obstacle taken into account
    graphPath.getShortestPath(drivePoints, TilePosition(currentX, currentY), originalTarget, originalTarget.scanOnReach);

    if (!drivePoints.empty())
    {
        // Target the new point from resulting path
        TilePosition nextLeg = drivePoints.front();
        int heading;

        float vX = (nextLeg.x + 0.5) - (sensorReadings.getCurrentPositionX() / 30.0);
        float vY = (nextLeg.y + 0.5) - (sensorReadings.getCurrentPositionY() / 30.0);

        float oX = 1;
        float oY = 0;

        double dot = oX*vX + oY*vY;
        double det = oX*vY - oY*vX;

        heading = (int)round(atan2(det, dot) * (180.0 / M_PI));

        //ROS_INFO("Targeting new point: %i, %i", nextLeg.x, nextLeg.y);
        sensorReadings.setTargetPoint(nextLeg.x, nextLeg.y);
        sensorReadings.setTargetHeading(heading);
        publishTurn(heading);
    }
    else
    {
        ROS_WARN("Could not find a new path after adding obstacle");
    }
}

void Planner::scanTimerCallback(const ros::TimerEvent& event)
{
    setIsScanning(false);
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
            ROS_WARN("We are assuming we have arrived at a point when we haven't, stopping movement");
            sensorReadings.setTargetHeading(-1);
            publishStop();
            return;
        }

        bool scanOnReachLastPoint = drivePoints.front().scanOnReach;

        // Remove the front point from the queue, we have arrived at it.
        drivePoints.pop_front();

        if (!drivePoints.empty())
        {
            TilePosition nextLeg = drivePoints.front();
            int heading;

            float vX = (nextLeg.x + 0.5) - (sensorReadings.getCurrentPositionX() / 30.0);
            float vY = (nextLeg.y + 0.5) - (sensorReadings.getCurrentPositionY() / 30.0);

            float oX = 1;
            float oY = 0;

            double dot = oX*vX + oY*vY;
            double det = oX*vY - oY*vX;

            heading = (int)round(atan2(det, dot) * (180.0 / M_PI));

            //ROS_INFO("Targeting new point: %i, %i", nextLeg.x, nextLeg.y);
            sensorReadings.setTargetPoint(nextLeg.x, nextLeg.y);
            sensorReadings.setTargetHeading(heading);
            publishTurn(heading);
            return;
        }
        // Processed the last point in the list
        else
        {
            if (scanOnReachLastPoint)
            {
                setIsScanning(true);

                // Timeout for scan
                ros::NodeHandle nh;
                ros::Timer scanTimeout = nh.createTimer(ros::Duration(2), &Planner::scanTimerCallback, this);

                // Scan at quarter driving speed
                publishDrive(sensorReadings.getCurrentHeading(), 0.16);
		        return;
            }

            // Write an invalid target value
            // Maybe do this for position as well
            ROS_INFO("Completed drive to point");
            sensorReadings.setTargetHeading(-1);
	        publishStop();
            return;
        }
    }
}

bool Planner::isDrivePointsEmpty()
{
    return drivePoints.empty();
}

void Planner::setIsScanning(bool val)
{
    std::lock_guard<std::mutex> guard(_is_scanning_mutex);    
    _is_scanning = val;
}
bool Planner::getIsScanning()
{
    std::lock_guard<std::mutex> guard(_is_scanning_mutex); 
    return _is_scanning;
}
