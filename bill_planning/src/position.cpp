#include "bill_planning/position.hpp"

Position::Position(float xc, float yc)
{
    x = xc;
    y = yc;
}

Position::Position()
{
    x = -500;
    y = -500;
}

TilePosition::TilePosition(int xc, int yc, bool scanOnReachc)
{
    x = xc;
    y = yc;
    scanOnReach = scanOnReachc;
}

TilePosition::TilePosition()
{
    x = -100;
    y = -100;
    scanOnReach = false;
}