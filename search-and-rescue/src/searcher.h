/*
 * AUTHOR: Carlo Pinciroli <cpinciro@ulb.ac.be>
 *
 * An example diffusion controller for the foot-bot.
 *
 * This controller makes the robots behave as gas particles. The robots
 * go straight until they get close enough to another robot, in which
 * case they turn, loosely simulating an elastic collision. The net effect
 * is that over time the robots diffuse in the environment.
 *
 * The controller uses the proximity sensor to detect obstacles and the
 * wheels to move the robot around.
 *
 * This controller is meant to be used with the XML files:
 *    experiments/diffusion_1.argos
 *    experiments/diffusion_10.argos
 */

#ifndef SEARCHER_H
#define SEARCHER_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
/* Definition of the omnidirectional camera sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>
/* Definition of the positioning sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
/* Definition of the range and bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the range and bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
/* Definitions for random number generation */
#include <argos3/core/utility/math/rng.h>
/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CSearcher : public CCI_Controller
{

public:
   struct SDefaultParams
   {
      /* Maximum tolerance for the angle between
    * the robot heading direction and
    * the closest obstacle detected. */
      CDegrees m_cAlpha;
      /* Maximum tolerance for the proximity reading between
    * the robot and the closest obstacle.
    * The proximity reading is 0 when nothing is detected
    * and grows exponentially to 1 when the obstacle is
    * touching the robot.
    */
      Real m_fDelta;
      /* Wheel speed. */
      Real m_fWheelVelocity;
      /* Angle tolerance range to go straight.
    * It is set to [-alpha,alpha]. */
      CRange<CRadians> m_cGoStraightAngleRange;

      std::string algorithm;

      void Init(TConfigurationNode &t_tree);
   };

   struct SPSOParams
   {
      Real target_power;
      Real noise;
      Real inertia;
      Real pw;
      Real nw;

      void Init(TConfigurationNode &t_tree);
   };

   /* Class constructor. */
   CSearcher();

   /* Class destructor. */
   virtual ~CSearcher() {}

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><footbot_diffusion_controller> section.
    */
   virtual void Init(TConfigurationNode &t_node);

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep();

   /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    * In this example controller there is no need for resetting anything,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Reset() {}

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Destroy() {}

   Real Intensity(Real distance);

   CVector2 newVelocity(CVector2 speed);

   void SendIntensity(Real intensity);

   void Readings();

   void WriteComms();

   void ReadComms();

   void Move();
   void Rotate();

   void SetWheelSpeedsFromVector(const CVector2 &c_heading);

   void obstacleAvoidance();

private:
   /* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator *m_pcWheels;
   /* Pointer to the foot-bot proximity sensor */
   CCI_FootBotProximitySensor *m_pcProximity;
   /* Pointer to the LEDs actuator */
   CCI_LEDsActuator *m_pcLEDs;
   /* Pointer to the omnidirectional camera sensor */
   CCI_ColoredBlobOmnidirectionalCameraSensor *m_pcCamera;
   /* Pointer to the positioning sensor */
   CCI_PositioningSensor *m_pcPosSens;
   /* Pointer to the range and bearing actuator */
   CCI_RangeAndBearingActuator *m_pcRABA;
   /* Pointer to the range and bearing sensor */
   CCI_RangeAndBearingSensor *m_pcRABS;
   /* The random number generator */
   CRandom::CRNG *m_pcRNG;
   /*
    * The following variables are used as parameters for the
    * algorithm. You can set their value in the <parameters> section
    * of the XML configuration file, under the
    * <controllers><footbot_diffusion_controller> section.
    */

   SDefaultParams defaultParams;
   SPSOParams psoParams;

   // Algorithm variables
   CVector2 bestPosition;
   CVector2 currPosition;
   CVector2 targetPosition;
   CVector2 bestNeighbourhoodPosition;
   Real bestIntensity;
   Real currIntensity;
   UInt64 nSteps;
   UInt64 maxSteps;

   enum EState
   {
      STATE_READINGS = 0,
      STATE_WRITE_COMMS,
      STATE_READ_COMMS,
      STATE_MOVE,
      STATE_ROTATE,
   };

   struct SWheelTurningParams
   {
      /*
       * The turning mechanism.
       * The robot can be in three different turning states.
       */
      enum ETurningMechanism
      {
         NO_TURN = 0, // go straight
         SOFT_TURN,   // both wheels are turning forwards, but at different speeds
         HARD_TURN    // wheels are turning with opposite speeds
      } TurningMechanism;
      /*
       * Angular thresholds to change turning state.
       */
      CRadians HardTurnOnAngleThreshold;
      CRadians SoftTurnOnAngleThreshold;
      CRadians NoTurnAngleThreshold;
      /* Maximum wheel speed */
      Real MaxSpeed;

      void Init(TConfigurationNode &t_tree);
   };

   EState m_eState;
   SWheelTurningParams m_sWheelTurningParams;
   CVector2 speed;
   Real maxVelocity;
};

#endif
