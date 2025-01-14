#ifndef RETURNROUTE_H
#define RETURNROUTE_H

#include "MazeMapping.h"
#include "MovementControl.h"

class ReturnRoute {

public:
    ReturnRoute(MazeMapping &mapInfo, MovementControl &movement);
    void ReturnToStart();

private:
        MazeMapping &_mapInfo;
        MovementControl &_movement; 
        void fillInWalls();    
    
};

#endif // RETURNROUTE_H
