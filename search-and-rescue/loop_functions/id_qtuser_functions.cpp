#include "id_qtuser_functions.h"
#include "search_loop_functions.h"

/****************************************/
/****************************************/

CIDQTUserFunctions::CIDQTUserFunctions() : m_cSearchLF(dynamic_cast<CSearchLoopFunctions&>(CSimulator::GetInstance().GetLoopFunctions())){
   RegisterUserFunction<CIDQTUserFunctions,CFootBotEntity>(&CIDQTUserFunctions::Draw);
}

/****************************************/
/****************************************/

void CIDQTUserFunctions::Draw(CFootBotEntity& c_entity) {
   /* The position of the text is expressed wrt the reference point of the footbot
    * For a foot-bot, the reference point is the center of its base.
    * See also the description in
    * $ argos3 -q foot-bot
    */
   DrawText(CVector3(0.0, 0.0, 0.3),   // position
            c_entity.GetId().c_str()); // text
}

void CIDQTUserFunctions::DrawInWorld() {
   /* Go through all the robot waypoints and draw them */
   for(CSearchLoopFunctions::TWaypointMap::const_iterator it = m_cSearchLF.GetWaypoints().begin();
       it != m_cSearchLF.GetWaypoints().end();
       ++it) {
      DrawWaypoints(it->second);
   }
}

/****************************************/
/****************************************/

void CIDQTUserFunctions::DrawWaypoints(const std::vector<CVector3>& c_waypoints) {
   /* Start drawing segments when you have at least two points */
   if(c_waypoints.size() > 1) {
      size_t unStart = 0;
      size_t unEnd = 1;
      while(unEnd < c_waypoints.size()) {
         DrawRay(CRay3(c_waypoints[unEnd],
                       c_waypoints[unStart]));
         ++unStart;
         ++unEnd;
      }
   }
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CIDQTUserFunctions, "id_qtuser_functions")
