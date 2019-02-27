#ifndef ENUMS
#define ENUMS
/*
enum MainStates 
{
    INIT_SCAN = 0,
    FIRE = 0,
}

enum INIT_SCAN
{

}

enum Fire
{
    LOOKFORFIRE = 0,
    VERIFY = 1
}
*/

enum MachineStates{
    STARTINGCOURSE = 0,
    DRIVETOCORNER = 1
    RUNINITIALSEARCH = 2,
    //3
    //4
    LOOKFORFIRE = 5,
    SCANNINGFIRE1 = 6,
    SCANNINGFIRE2 = 7,
    STOPPED = 8,
    COUNT = 9// ALWAYS MAKE COUNT LAST, UGLY SOLUTION TO DETERMINE ITEMS IN ENUM
};

enum SearchingStates{
    DRIVETOPOINT = 0,
    GRIDSEARCH = 1
    //COUNT // ALWAYS MAKE COUNT LAST, UGLY SOLUTION TO DETERMINE ITEMS IN ENUM
};
#endif
