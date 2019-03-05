#ifndef ENUMS_HPP
#define ENUMS_HPP

enum MajorState
{
    INIT_SEARCH,
    SEARCH_FIRE,
    SEARCH,
    DELIVER,
    RETURN_HOME
};

enum MinorState
{
    RUN,
    FOUND_FIRE,
    FOUND_SURVIVOR,
    FOUND_GROUP,
    FOUND_FOOD,
    VERIFY_FIRE,
    COMPLETE
};

#endif
