#include "RVRDAO.h"

namespace argos
{

    RVRDAO::~RVRDAO() {}

    /****************************************/
    /****************************************/

    void RVRDAO::SetWheelsVelocity(const Real &un_left_velocity, const Real &un_right_velocity)
    {
        m_fLeftWheelVelocity = un_left_velocity;
        m_fRightWheelVelocity = un_right_velocity;
    }

    /****************************************/
    /****************************************/

    void RVRDAO::SetWheelsVelocity(const CVector2 &c_velocity_vector)
    {
        m_fLeftWheelVelocity = c_velocity_vector.GetX();
        m_fRightWheelVelocity = c_velocity_vector.GetY();
    }

    /****************************************/
    /****************************************/

    const Real &RVRDAO::GetRightWheelVelocity() const
    {
        return m_fRightWheelVelocity;
    }

    /****************************************/
    /****************************************/

    const Real &RVRDAO::GetLeftWheelVelocity() const
    {
        return m_fLeftWheelVelocity;
    }

    /****************************************/
    /****************************************/

    void RVRDAO::SetRobotIdentifier(const UInt32 &un_robot_id)
    {
        m_unRobotIdentifier = un_robot_id;
    }

    /****************************************/
    /****************************************/

    const UInt32 &RVRDAO::GetRobotIdentifier() const
    {
        return m_unRobotIdentifier;
    }

    /****************************************/
    /****************************************/

    CRandom::CRNG *RVRDAO::GetRandomNumberGenerator() const
    {
        return m_pcRng;
    }

    /****************************************/
    /****************************************/

    const Real &RVRDAO::GetMaxVelocity() const
    {
        return m_fMaxVelocity;
    }

    /****************************************/
    /****************************************/

    void RVRDAO::SetMaxVelocity(const Real &un_max_velocity)
    {
        m_fMaxVelocity = un_max_velocity;
    }
}