
class Enums
{
 public:
    enum class MachineStates{
        STARTINGCOURSE = 0,
        INITIALSEARCH,
        LOOKFORFIRE,
        SCANNINGFIRE,
        AWAITINGFIRESCAN,
        GRIDSEARCH,
        STOPPED,
        COUNT // ALWAYS MAKE COUNT LAST, UGLY SOLUTION TO DETERMINE ITEMS IN ENUM
    }

    enum class SearchingStates{
        DriveToPoint = 0,
        GridSearch,
        COUNT // ALWAYS MAKE COUNT LAST, UGLY SOLUTION TO DETERMINE ITEMS IN ENUM
    }
};
