// Loop through every footbot
// If every footbot is in state "READY", then put them on state "READINGS" to start the next iteration of the algorithm

#include "search_loop_functions.h"
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <controllers/searcher.h>
#include <fstream>

/*
 * To reduce the number of waypoints stored in memory,
 * consider two robot positions distinct if they are
 * at least MIN_DISTANCE away from each other
 * This constant is expressed in meters
 */
static const Real MIN_DISTANCE = 0.05f;
/* Convenience constant to avoid calculating the square root in PostStep() */
static const Real MIN_DISTANCE_SQUARED = MIN_DISTANCE * MIN_DISTANCE;

const std::string currentDateTime()
{
    time_t now = time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

    return buf;
}

CSearchLoopFunctions::CSearchLoopFunctions() : steps(0)
{
}

void CSearchLoopFunctions::Init(TConfigurationNode &t_node)
{
    try
    {
        GetNodeAttribute(t_node, "maxSteps", maxSteps);
        GetNodeAttribute(t_node, "algorithm", algorithm);

        file.open("stats/" + algorithm + "-" + currentDateTime() + ".txt");
        file << "iteration, ticks" << endl;
    }
    catch (CARGoSException &ex)
    {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
    }

    /*
    * Go through all the robots in the environment
    * and create an entry in the waypoint map for each of them
    */
    /* Get the map of all foot-bots from the space */
    CSpace::TMapPerType &tFBMap = GetSpace().GetEntitiesByType("foot-bot");
    /* Go through them */
    for (CSpace::TMapPerType::iterator it = tFBMap.begin();
         it != tFBMap.end();
         ++it)
    {
        /* Create a pointer to the current foot-bot */
        CFootBotEntity *pcFB = any_cast<CFootBotEntity *>(it->second);
        /* Create a waypoint vector */
        m_tWaypoints[pcFB] = std::vector<CVector3>();
        /* Add the initial position of the foot-bot */
        m_tWaypoints[pcFB].push_back(pcFB->GetEmbodiedEntity().GetOriginAnchor().Position);
    }
}

void CSearchLoopFunctions::Reset()
{
    /*
    * Clear all the waypoint vectors
    */
    /* Get the map of all foot-bots from the space */
    CSpace::TMapPerType &tFBMap = GetSpace().GetEntitiesByType("foot-bot");
    /* Go through them */
    for (CSpace::TMapPerType::iterator it = tFBMap.begin();
         it != tFBMap.end();
         ++it)
    {
        /* Create a pointer to the current foot-bot */
        CFootBotEntity *pcFB = any_cast<CFootBotEntity *>(it->second);
        /* Clear the waypoint vector */
        m_tWaypoints[pcFB].clear();
        /* Add the initial position of the foot-bot */
        m_tWaypoints[pcFB].push_back(pcFB->GetEmbodiedEntity().GetOriginAnchor().Position);
    }
}

void CSearchLoopFunctions::PostStep()
{
    // CSpace::TMapPerType &m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
    // bool ready = true;

    // for (CSpace::TMapPerType::iterator it = m_cFootbots.begin();
    //      it != m_cFootbots.end();
    //      ++it)
    // {
    //     /* Get handle to foot-bot entity and controller */
    //     CFootBotEntity &cFootBot = *any_cast<CFootBotEntity *>(it->second);
    //     CSearcher &cController = dynamic_cast<CSearcher &>(cFootBot.GetControllableEntity().GetController());

    //     if (!cController.getState() == STATE_READY)
    //     {
    //         ready = false;
    //         break;
    //     }
    // }

    // if (ready)
    // {
    //     for (CSpace::TMapPerType::iterator it = m_cFootbots.begin();
    //          it != m_cFootbots.end();
    //          ++it)
    //     {
    //         CFootBotEntity &cFootBot = *any_cast<CFootBotEntity *>(it->second);
    //         CSearcher &cController = dynamic_cast<CSearcher &>(cFootBot.GetControllableEntity().GetController());

    //         cController.setState(STATE_READINGS);
    //     }
    // }
    steps++;

    trajectoryPositions();

    if (steps != maxSteps)
        return;

    steps = 0;

    CSpace::TMapPerType &m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
    int numRobotsNearTarget = 0;

    for (CSpace::TMapPerType::iterator it = m_cFootbots.begin(); it != m_cFootbots.end(); ++it)
    {
        CFootBotEntity &cFootBot = *any_cast<CFootBotEntity *>(it->second);
        CSearcher &cController = dynamic_cast<CSearcher &>(cFootBot.GetControllableEntity().GetController());

        if (cController.isNearTarget())
        {
            numRobotsNearTarget++;
        }
    }

    file << numRobotsNearTarget << ", " << GetSpace().GetSimulationClock() << endl;
}

void CSearchLoopFunctions::trajectoryPositions()
{
    /* Get the map of all foot-bots from the space */
    CSpace::TMapPerType &tFBMap = GetSpace().GetEntitiesByType("foot-bot");
    /* Go through them */
    for (CSpace::TMapPerType::iterator it = tFBMap.begin();
         it != tFBMap.end();
         ++it)
    {
        /* Create a pointer to the current foot-bot */
        CFootBotEntity *pcFB = any_cast<CFootBotEntity *>(it->second);
        /* Add the current position of the foot-bot if it's sufficiently far from the last */
        if (SquareDistance(pcFB->GetEmbodiedEntity().GetOriginAnchor().Position,
                           m_tWaypoints[pcFB].back()) > MIN_DISTANCE_SQUARED)
        {
            m_tWaypoints[pcFB].push_back(pcFB->GetEmbodiedEntity().GetOriginAnchor().Position);
        }
    }
}

REGISTER_LOOP_FUNCTIONS(CSearchLoopFunctions, "search_loop_functions")
