#include "ReferenceModel1Dot1.h"
#include <algorithm>
#include <numeric>

/****************************************/
/****************************************/

ReferenceModel1Dot1::ReferenceModel1Dot1()
{
    m_pcRng = CRandom::CreateRNG("argos");
    m_fMaxVelocity = 12; // 12 cm/s (real max speed is 155 cm/s but it is used as is by automode)
    m_fLeftWheelVelocity = 0;
    m_fRightWheelVelocity = 0;
}

/****************************************/
/****************************************/

ReferenceModel1Dot1::~ReferenceModel1Dot1() {}

/****************************************/
/****************************************/

void ReferenceModel1Dot1::Reset()
{
    m_fLeftWheelVelocity = 0;
    m_fRightWheelVelocity = 0;
}

/****************************************/
/****************************************/

CCI_RVRProximitySensor::SReading ReferenceModel1Dot1::GetProximityReading()
{
    CCI_RVRProximitySensor::SReading cOutputReading;
    CVector2 cSumProxi(0, CRadians::ZERO);
    for (UInt8 i = 0; i < m_sProximityInput.size(); i++)
    {
        cSumProxi += CVector2(m_sProximityInput[i].Value, m_sProximityInput[i].Angle.SignedNormalize());
    }

    cOutputReading.Value = (cSumProxi.Length() > 1) ? 1 : cSumProxi.Length();
    cOutputReading.Angle = cSumProxi.Angle().SignedNormalize();

    return cOutputReading;
}

CCI_RVRProximitySensor::TReadings ReferenceModel1Dot1::GetProximityInput() const
{
    return m_sProximityInput;
}

/****************************************/
/****************************************/

void ReferenceModel1Dot1::SetProximityInput(CCI_RVRProximitySensor::TReadings s_prox_input)
{
    m_sProximityInput = s_prox_input;
}

/****************************************/
/****************************************/

CCI_RVRLightSensor::SReading ReferenceModel1Dot1::GetLightInput() const
{
    return m_sLightInput;
}

/****************************************/
/****************************************/

CCI_RVRLightSensor::SReading ReferenceModel1Dot1::GetLightReading()
{
    return m_sLightInput;
}

/****************************************/
/****************************************/

void ReferenceModel1Dot1::SetLightInput(CCI_RVRLightSensor::SReading s_light_input)
{
    m_sLightInput = s_light_input;
}

/****************************************/
/****************************************/

CCI_RVRGroundColorSensor::SReading ReferenceModel1Dot1::GetGroundInput() const
{
    return m_sGroundInput;
}

CColor ReferenceModel1Dot1::GetGroundReading() const
{
    return m_sGroundInput.Color;
}

/****************************************/
/****************************************/

void ReferenceModel1Dot1::SetGroundInput(CCI_RVRGroundColorSensor::SReading s_ground_input)
{
    m_sGroundInput = s_ground_input;
}

/****************************************/
/****************************************/

const UInt8 ReferenceModel1Dot1::GetNumberNeighbors()
{
    return LidarToRobotPositions(m_sLidarInput).size();
}

/****************************************/
/****************************************/

void ReferenceModel1Dot1::SetNumberNeighbors(const UInt8 &un_number_neighbors)
{
    m_unNumberNeighbors = un_number_neighbors;
}

/****************************************/
/****************************************/

CCI_RVRLidarSensor::TReadings ReferenceModel1Dot1::GetLidarInput() const
{
    return m_sLidarInput;
}

/****************************************/
/****************************************/

void ReferenceModel1Dot1::SetLidarInput(CCI_RVRLidarSensor::TReadings s_lidar_input)
{
    m_sLidarInput = s_lidar_input;
}

/****************************************/
/****************************************/

std::vector<CCI_RVRLidarSensor::SReading> ReferenceModel1Dot1::LidarToRobotPositions(CCI_RVRLidarSensor::TReadings s_lidar_input) const
{
    // identify groups of points which are the robots
    UInt8 n_neigh = -1;
    // array of neighbours id which identifies uniquely a given robot
    // vector of same length as lidar input and with flag -1 to mean "no robot at this angle"
    std::vector<int> neighbourId(s_lidar_input.size(), -1);
    // stores the previous index identified as a robot
    UInt16 latestRobotIndex;
    for (std::size_t i = 0; i != s_lidar_input.size(); ++i)
    {
        if (s_lidar_input[i].Value > 0.75 || s_lidar_input[i].Value < 0.10)
        {
            // we consider it is not a robot beyond 75cm or if the reading is too close to the sensor
            continue;
        }
        // the point belongs to a robot

        // first robot point belongs to the first robot
        if (n_neigh == -1)
        {
            // belongs to neighbour 0
            neighbourId[i] = ++n_neigh;
            latestRobotIndex = i;
            continue;
        }

        // ulterior point
        // if the 2 points have a difference of
        // less than 10 cm in distance to current robot and
        // less than 10 degrees (0.175) in angle then they belong to the same robot
        if (Abs(s_lidar_input[i].Value - s_lidar_input[latestRobotIndex].Value) < 0.1 && Abs(s_lidar_input[i].Angle - s_lidar_input[latestRobotIndex].Angle) < CRadians(0.175))
        {
            neighbourId[i] = n_neigh;
            latestRobotIndex = i;
            continue;
        }

        // new group of points
        neighbourId[i] = ++n_neigh;
        latestRobotIndex = i;
    }

    // now we have the value of the group/robot each point belongs to,
    // -1 being no group = not a robot

    // each group will be represented by the average (Value, Angle) over the group
    std::vector<CCI_RVRLidarSensor::SReading> neighbourPositions;
    // group ids go from 0 to n_neigh => n_neigh +1 groups
    neighbourPositions.resize(n_neigh + 1);
    for (int i = 0; i <= n_neigh; ++i)
    {
        // all the positions for this particular group i
        std::vector<CCI_RVRLidarSensor::SReading> groupPositions;
        for (size_t j = 0; j < s_lidar_input.size(); j++)
        {
            if (neighbourId[j] == i)
            {
                groupPositions.push_back(s_lidar_input[j]);
            }
        }
        // compute mean
        std::vector<Real> groupSum(2, 0.0f);
        for (const auto &groupMember : groupPositions)
        {
            groupSum[0] += groupMember.Value;
            groupSum[1] += groupMember.Angle.GetValue();
        }
        neighbourPositions[i] = CCI_RVRLidarSensor::SReading(groupSum[0] / groupPositions.size(), CRadians(groupSum[1] / groupPositions.size()));
    }
    return neighbourPositions;
}

CCI_RVRLidarSensor::SReading ReferenceModel1Dot1::GetAttractionVectorToNeighbors(Real f_alpha_parameter)
{
    auto neighbourPositions = LidarToRobotPositions(m_sLidarInput);
    // we now have the position of each neighbour
    // we can compute the attraction vector
    CVector2 lidarVectorSum(0, CRadians::ZERO);
    for (auto &robotPosition : neighbourPositions)
    {
        lidarVectorSum += CVector2(f_alpha_parameter / (1 + robotPosition.Value), robotPosition.Angle.SignedNormalize());
    }
    CCI_RVRLidarSensor::SReading cLidarReading;
    cLidarReading.Value = lidarVectorSum.Length();
    cLidarReading.Angle = lidarVectorSum.Angle().SignedNormalize();

    return cLidarReading;
}

/****************************************/
/****************************************/

CCI_RVRLidarSensor::SReading ReferenceModel1Dot1::GetNeighborsCenterOfMass()
{
    auto neighbourPositions = LidarToRobotPositions(m_sLidarInput);
    // we now have the position of each neighbour
    // we can compute the attraction vector
    CVector2 lidarVectorSum(0, CRadians::ZERO);
    for (auto &robotPosition : neighbourPositions)
    {
        lidarVectorSum += CVector2(robotPosition.Value, robotPosition.Angle.SignedNormalize());
    }
    // divide by the number of robots
    lidarVectorSum /= neighbourPositions.size();
    CCI_RVRLidarSensor::SReading cLidarReading;
    cLidarReading.Value = lidarVectorSum.Length();
    cLidarReading.Angle = lidarVectorSum.Angle().SignedNormalize();

    return cLidarReading;
}