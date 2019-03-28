#ifndef POS_HPP
#define POS_HPP

struct Position
{
    public:
        Position(float xc, float yc);
        Position();
        float x;
        float y;
};

struct TilePosition
{
    public:
        TilePosition(int xc, int yc, bool scanOnReachc = false);
        TilePosition();
        int x;
        int y;
        bool scanOnReach;
};


#endif