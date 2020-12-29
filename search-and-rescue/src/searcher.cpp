/* Include the controller definition */
#include "searcher.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>

#include <map>
#include <cmath>

/****************************************/
/****************************************/

void CSearcher::SDefaultParams::Init(TConfigurationNode &t_node)
{
   /*
    * Parse the configuration file
    *
    * The user defines this part. Here, the algorithm accepts three
    * parameters and it's nice to put them in the config file so we don't
    * have to recompile if we want to try other settings.
    */
   GetNodeAttribute(t_node, "alpha", m_cAlpha);
   m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
   GetNodeAttribute(t_node, "delta", m_fDelta);
   GetNodeAttribute(t_node, "velocity", m_fWheelVelocity);
   GetNodeAttribute(t_node, "algorithm", algorithm);
}

void CSearcher::SWheelTurningParams::Init(TConfigurationNode &t_node)
{
   try
   {
      TurningMechanism = NO_TURN;
      CDegrees cAngle;
      GetNodeAttribute(t_node, "hard_turn_angle_threshold", cAngle);
      HardTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "soft_turn_angle_threshold", cAngle);
      SoftTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "no_turn_angle_threshold", cAngle);
      NoTurnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "max_speed", MaxSpeed);
   }
   catch (CARGoSException &ex)
   {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex);
   }
}

void CSearcher::SPSOParams::Init(TConfigurationNode &t_node)
{
   GetNodeAttribute(t_node, "inertia", inertia);
   GetNodeAttribute(t_node, "noise", noise);
   GetNodeAttribute(t_node, "pw", pw);
   GetNodeAttribute(t_node, "nw", nw);
}

CSearcher::CSearcher() : m_pcWheels(NULL),
                         m_pcProximity(NULL),
                         m_pcLEDs(NULL),
                         m_pcRABS(NULL),
                         m_pcRABA(NULL),
                         m_pcPosSens(NULL),
                         m_pcCamera(NULL)
{
}

/****************************************/
/****************************************/

void CSearcher::Init(TConfigurationNode &t_node)
{
   /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "differential_steering") corresponds to the
    * XML tag of the device whose handle we want to have. For a list of
    * allowed values, type at the command prompt:
    *
    * $ argos3 -q actuators
    *
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of all the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors
    * internally, on the basis of the lists provided the configuration
    * file at the <controllers><footbot_diffusion><actuators> and
    * <controllers><footbot_diffusion><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */
   m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcProximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
   m_pcLEDs = GetActuator<CCI_LEDsActuator>("leds");
   m_pcCamera = GetSensor<CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
   m_pcPosSens = GetSensor<CCI_PositioningSensor>("positioning");
   m_pcRABA = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");
   m_pcRABS = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");

   m_pcRNG = CRandom::CreateRNG("argos");

   defaultParams.Init(GetNode(t_node, "default"));
   psoParams.Init(GetNode(t_node, "pso"));
   m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));

   /*
    * Other init stuff
    */
   /* Enable camera filtering */
   m_pcCamera->Enable();
   /* Set beacon color to all red to be visible for other robots */
   m_pcLEDs->SetSingleColor(12, CColor::BLUE);

   m_eState = STATE_READINGS;

   speed = CVector2(0, 0);
   bestPosition = CVector2(0, 0);
   bestNeighbourhoodPosition = CVector2(0, 0);
   maxSteps = 100;
   nSteps = 0;
   bestIntensity = 0;
   currIntensity = 0;
   maxVelocity = 20.0f;

   const CVector3 position = m_pcPosSens->GetReading().Position;
   const CVector2 position2d = CVector2(position.GetX(), position.GetY());
   bestPosition = position2d;
   bestNeighbourhoodPosition = position2d;
}

/****************************************/
/****************************************/

void CSearcher::Readings()
{
   /* Get readings from proximity sensor */
   const CCI_FootBotProximitySensor::TReadings &tProxReads = m_pcProximity->GetReadings();
   /* Get the camera readings */
   const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings &sReadings = m_pcCamera->GetReadings();
   const CVector3 position = m_pcPosSens->GetReading().Position;
   const CVector2 position2d = CVector2(position.GetX(), position.GetY());
   currPosition = position2d;
   currIntensity = 0;

   if (!sReadings.BlobList.empty())
   {
      for (size_t i = 0; i < sReadings.BlobList.size(); ++i)
      {
         // Robot sees a red led, which means he sees the target robot
         if (sReadings.BlobList[i]->Color == CColor::RED)
         {
            if (sReadings.BlobList[i]->Distance < 30)
               m_pcLEDs->SetSingleColor(12, CColor::YELLOW);
            Real intensity = Intensity(sReadings.BlobList[i]->Distance);
            currIntensity = intensity;

            if (intensity > bestIntensity)
            {
               bestPosition = position2d;
               bestIntensity = intensity;
            }
            break;
         }
      }
   }

   // if (this->GetId() == "fb3")
   // {
   //    LOG << "CURR INTENSITY: " << currIntensity << " BEST INTENSITY: " << bestIntensity << std::endl;
   //    LOG << "CURR POSITION: " << position2d << " BEST POSITION: " << bestPosition << std::endl;
   // }
   // else
   // {
   //    CRange<Real> range(-10, 10);
   //    bestPosition = CVector2(m_pcRNG->Uniform(range), m_pcRNG->Uniform(range));
   //    bestIntensity = 0;
   // }

   m_eState = STATE_WRITE_COMMS;
}

void CSearcher::WriteComms()
{
   SendIntensity(currIntensity);
   m_eState = STATE_READ_COMMS;
}

void CSearcher::ReadComms()
{

   const CCI_RangeAndBearingSensor::TReadings &tPackets = m_pcRABS->GetReadings();
   UInt64 bestNeightbourIntensity = 0;

   if (!tPackets.empty())
   {
      for (size_t i = 0; i < tPackets.size(); ++i)
      {
         Real length = tPackets[i].Range;

         CVector2 me2Neighbour = CVector2(length, tPackets[i].HorizontalBearing);

         // if(this->GetId() == "fb3")
         //    LOG << "HORIZONTAL BEARING: " << tPackets[i].HorizontalBearing;

         CVector2 pos = currPosition + me2Neighbour;

         UInt64 intensity = *reinterpret_cast<const UInt64 *>((&tPackets[i])->Data.ToCArray());

         // if (this->GetId() == "fb3")
         //    LOG << "READ_INTENSITY: " << this->GetId() << " - " << intensity << std::endl;

         if (this->GetId() == "fb3")
            LOG << "READ_POS: " << me2Neighbour << " : " << pos << std::endl;

         // if (this->GetId() == "fb3")
         //    LOG << "READ_POS: "  << me2Neighbour << " INTENSITY: " << intensity << std::endl;
         if (intensity > bestNeightbourIntensity)
         {
            bestNeightbourIntensity = intensity;
            bestNeighbourhoodPosition = pos;
         }
      }
   }
   else
   {
      bestNeightbourIntensity = 0;
      bestNeighbourhoodPosition = CVector2(0, 0);
   }

   speed = newVelocity(speed);
   // if (this->GetId() == "fb3")
   //    LOG << "POSITIONS: " << bestPosition << " : " << currPosition << std::endl;
   targetPosition = currPosition + speed;

   if (this->GetId() == "fb3")
      argos::LOG << "TARGET POS: " << targetPosition << " SPEED:" << speed << std::endl;

   m_eState = STATE_MOVE;
}

void CSearcher::Move()
{
   const CVector3 position = m_pcPosSens->GetReading().Position;
   const CVector2 position2d = CVector2(position.GetX(), position.GetY());
   const CQuaternion orientation = m_pcPosSens->GetReading().Orientation;
   CRadians cZAngle, cYAngle, cXAngle;
   orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);

   CVector2 me2Target(targetPosition - position2d);
   me2Target.Rotate(-cZAngle);

   // if (this->GetId() == "fb3")
   //    LOG << "ME2TARGET: " << me2Target << " " << me2Target.Length() << std::endl;

   const CCI_FootBotProximitySensor::TReadings &tProxReads = m_pcProximity->GetReadings();
   /* Sum them together */
   CVector2 cAccumulator;
   for (size_t i = 0; i < tProxReads.size(); ++i)
   {
      cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
   }
   cAccumulator /= tProxReads.size();
   /* If the angle of the vector is small enough and the closest obstacle
    * is far enough, continue going straight, otherwise curve a little
    */
   CRadians cAngle = cAccumulator.Angle();
   if (defaultParams.m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
       cAccumulator.Length() < defaultParams.m_fDelta)
   {
      /* Go straight */
      SetWheelSpeedsFromVector(me2Target);
   }
   else
   {
      /* Turn, depending on the sign of the angle */
      if (cAngle.GetValue() > 0.0f)
      {
         m_pcWheels->SetLinearVelocity(defaultParams.m_fWheelVelocity, 0.0f);
      }
      else
      {
         m_pcWheels->SetLinearVelocity(0.0f, defaultParams.m_fWheelVelocity);
      }
   }
}

void CSearcher::SetWheelSpeedsFromVector(const CVector2 &c_heading)
{
   /* Get the heading angle */
   CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
   /* Get the length of the heading vector */
   Real fHeadingLength = c_heading.Length();
   /* Clamp the speed so that it's not greater than MaxSpeed */
   Real fBaseAngularWheelSpeed = m_sWheelTurningParams.MaxSpeed; // Min<Real>(fHeadingLength, m_sWheelTurningParams.MaxSpeed);
   /* State transition logic */
   if (m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::HARD_TURN)
   {
      if (Abs(cHeadingAngle) <= m_sWheelTurningParams.SoftTurnOnAngleThreshold)
      {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   if (m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::SOFT_TURN)
   {
      if (Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold)
      {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if (Abs(cHeadingAngle) <= m_sWheelTurningParams.NoTurnAngleThreshold)
      {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::NO_TURN;
      }
   }
   if (m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::NO_TURN)
   {
      if (Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold)
      {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if (Abs(cHeadingAngle) > m_sWheelTurningParams.NoTurnAngleThreshold)
      {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   /* Wheel speeds based on current turning state */
   Real fSpeed1, fSpeed2;
   switch (m_sWheelTurningParams.TurningMechanism)
   {
   case SWheelTurningParams::NO_TURN:
   {
      /* Just go straight */
      fSpeed1 = fBaseAngularWheelSpeed;
      fSpeed2 = fBaseAngularWheelSpeed;
      break;
   }
   case SWheelTurningParams::SOFT_TURN:
   {
      /* Both wheels go straight, but one is faster than the other */
      Real fSpeedFactor = (m_sWheelTurningParams.HardTurnOnAngleThreshold - Abs(cHeadingAngle)) / m_sWheelTurningParams.HardTurnOnAngleThreshold;
      fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
      fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
      break;
   }
   case SWheelTurningParams::HARD_TURN:
   {
      /* Opposite wheel speeds */
      fSpeed1 = -m_sWheelTurningParams.MaxSpeed;
      fSpeed2 = m_sWheelTurningParams.MaxSpeed;
      break;
   }
   }
   /* Apply the calculated speeds to the appropriate wheels */
   Real fLeftWheelSpeed, fRightWheelSpeed;
   if (cHeadingAngle > CRadians::ZERO)
   {
      /* Turn Left */
      fLeftWheelSpeed = fSpeed1;
      fRightWheelSpeed = fSpeed2;
   }
   else
   {
      /* Turn Right */
      fLeftWheelSpeed = fSpeed2;
      fRightWheelSpeed = fSpeed1;
   }
   /* Finally, set the wheel speeds */
   m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
}

void CSearcher::ControlStep()
{
   if (nSteps == maxSteps)
   {
      nSteps = 0;
      m_eState = STATE_READINGS;
   }

   switch (m_eState)
   {
   case STATE_READINGS:
      Readings();
      break;
   case STATE_WRITE_COMMS:
      WriteComms();
      break;
   case STATE_READ_COMMS:
      ReadComms();
      break;
   case STATE_MOVE:
      Move();
      break;
   default:
      break;
   }

   nSteps++;
}

Real CSearcher::Intensity(Real distance)
{
   return psoParams.pw / pow(distance / 100, 2) /* + m_pcRNG->Gaussian(psoParams.noise) */;
}

CVector2 CSearcher::newVelocity(CVector2 speed)
{
   CRange<Real> range(0, 1);
   Real rand1 = m_pcRNG->Uniform(range);
   Real rand2 = m_pcRNG->Uniform(range);
   CVector2 vec1 = psoParams.inertia * speed;
   CVector2 vec2 = psoParams.pw * (bestPosition - currPosition);
   CVector2 vec3 = psoParams.nw * (bestNeighbourhoodPosition - currPosition);

   // if (this->GetId() == "fb3")
   //    LOG << " BEST_POS: " << bestPosition << " CURR_POS: " << currPosition << " BEST_NEIGHBOUR_POS: " << bestNeighbourhoodPosition << std::endl;

   return vec1 + vec2 + vec3;
}

void CSearcher::SendIntensity(Real intensity)
{
   CByteArray cBuf(10);
   UInt64 uIntensity = (UInt64)argos::Round(intensity);
   // if (this->GetId() == "fb9")
   //    LOG << "SEND INTENSITY: " << this->GetId() << " - " << intensity << " - " << currPosition << std::endl;
   cBuf[0] = uIntensity & 0xff;
   cBuf[1] = uIntensity >> 8 & 0xff;
   cBuf[2] = uIntensity >> 16 & 0xff;
   cBuf[3] = uIntensity >> 24 & 0xff;
   m_pcRABA->SetData(cBuf);
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CSearcher, "search_and_rescue_searcher_controller")
