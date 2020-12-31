#ifndef ID_QTUSER_FUNCTIONS_H
#define ID_QTUSER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

using namespace argos;

class CSearchLoopFunctions;

class CIDQTUserFunctions : public CQTOpenGLUserFunctions
{

public:
   CIDQTUserFunctions();

   virtual ~CIDQTUserFunctions() {}

   void Draw(CFootBotEntity &c_entity);

   virtual void DrawInWorld();

private:
   void DrawWaypoints(const std::vector<CVector3> &c_waypoints);

private:
   CSearchLoopFunctions &m_cSearchLF;
};

#endif
