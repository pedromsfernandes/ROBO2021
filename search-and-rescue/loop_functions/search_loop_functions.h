#ifndef SEARCH_LOOP_FUNCTIONS_H
#define SEARCH_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <fstream>
#include <string>
#include <map>
#include <vector>
#include <chrono>
#include <ctime>

using namespace argos;
using namespace std;

class CSearchLoopFunctions : public CLoopFunctions
{
public:
    typedef std::map<CFootBotEntity *, std::vector<CVector3>> TWaypointMap;
    TWaypointMap m_tWaypoints;

public:
    CSearchLoopFunctions();
    virtual ~CSearchLoopFunctions()
    {
        file.close();
    }
    virtual void Init(TConfigurationNode &t_tree);
    virtual void Reset();

    virtual void PostStep();

    void trajectoryPositions();

    inline const TWaypointMap &GetWaypoints() const
    {
        return m_tWaypoints;
    }

private:
    int steps;
    int maxSteps;
    CVector2 targetPosition;
    ofstream file;
    string algorithm;
};

#endif