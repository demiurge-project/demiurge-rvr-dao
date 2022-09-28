#include "ReferenceModel1Dot2.h"
#include <algorithm>
#include <numeric>

/****************************************/
/****************************************/

ReferenceModel1Dot2::ReferenceModel1Dot2()
{
    m_pcRng = CRandom::CreateRNG("argos");
    m_fMaxVelocity = 0; // 12 cm/s (real max speed is 155 cm/s but it is used as is by automode)
    m_fLeftWheelVelocity = 0;
    m_fRightWheelVelocity = 0;
}

/****************************************/
/****************************************/

ReferenceModel1Dot2::~ReferenceModel1Dot2() {}

/****************************************/
/****************************************/

void ReferenceModel1Dot2::Reset()
{
    m_fLeftWheelVelocity = 0;
    m_fRightWheelVelocity = 0;
}

/****************************************/
/****************************************/

CCI_RVRProximitySensor::SReading ReferenceModel1Dot2::GetProximityReading()
{
    CCI_RVRProximitySensor::SReading cOutputReading;
    CVector2 cSumProxi(0, CRadians::ZERO);
    Real fNormalizeDistance = 0.0;
    for (UInt8 i = 0; i < m_sProximityInput.size(); i++)
    {
        //if (m_sProximityInput[i].Value == 0.0f)
        //{
        //    m_sProximityInput[i].Value = -std::numeric_limits<Real>::max();
        //}

        // Added to normalize values with respect to a range of 0.4 set in the proximity sensors, values out of the if are out of range
        // TODO: set this with respect to the parameter in the instantiation of the sensor in C
        if(m_sProximityInput[i].Value < 0.05 || m_sProximityInput[i].Value > 0.4){

            fNormalizeDistance = 0.0;                     
        }
        else {
            // Added division by 17.5 to normalize values with respect to a range of 0.4 set in the proximity sensors
            // TODO: set this with respect to the parameter in the instantiation of the sensor in C
            fNormalizeDistance = ((1 / m_sProximityInput[i].Value) - 2.5) / 17.5;
        }
        
        cSumProxi += CVector2(fNormalizeDistance, m_sProximityInput[i].Angle.SignedNormalize());

    }
    // avoid neighbours
    FindNeighbours();
    for (UInt8 i = 0; i < m_vecNeighbors.size(); i++)
    {
        if (m_vecNeighbors.at(i).Distance <= 0.55) // 0.4 + robot radius
        {
            Real sDistToObstacle = m_vecNeighbors.at(i).Distance - 0.275; // - 2 radii of the robot
            sDistToObstacle = sDistToObstacle <= 0.0f ? 0.001f : sDistToObstacle;
            
            fNormalizeDistance = ((1 / m_sProximityInput[i].Value) - 3.6364) / 996.3636;

            cSumProxi += CVector2(fNormalizeDistance, m_vecNeighbors.at(i).Angle.SignedNormalize());
        }
    }
    
    cOutputReading.Value = (cSumProxi.Length() > 1) ? 1 : cSumProxi.Length();
    cOutputReading.Angle = cSumProxi.Angle().SignedNormalize(); 
    return cOutputReading;
}

CCI_RVRProximitySensor::TReadings ReferenceModel1Dot2::GetProximityInput() const
{
    return m_sProximityInput;
}

/****************************************/
/****************************************/

void ReferenceModel1Dot2::SetProximityInput(CCI_RVRProximitySensor::TReadings s_prox_input)
{
    m_sProximityInput = s_prox_input;
}

/****************************************/
/****************************************/

CCI_RVRLightSensor::SReading ReferenceModel1Dot2::GetLightInput() const
{
    return m_sLightInput;
}

/****************************************/
/****************************************/

CCI_RVRLightSensor::SReading ReferenceModel1Dot2::GetLightReading()
{
    return m_sLightInput;
}

/****************************************/
/****************************************/

void ReferenceModel1Dot2::SetLightInput(CCI_RVRLightSensor::SReading s_light_input)
{
    m_sLightInput = s_light_input;
}

/****************************************/
/****************************************/

CCI_RVRGroundColorSensor::SReading ReferenceModel1Dot2::GetGroundInput() const
{
    return m_sGroundInput;
}

CColor ReferenceModel1Dot2::GetGroundReading() const
{
    return m_sGroundInput.Color;
}

/****************************************/
/****************************************/

void ReferenceModel1Dot2::SetGroundInput(CCI_RVRGroundColorSensor::SReading s_ground_input)
{
    m_sGroundInput = s_ground_input;
}

/****************************************/
/****************************************/

void ReferenceModel1Dot2::SetOmnidirectionalCameraInput(CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings s_omni_input)
{
    m_sOmnidirectionalCameraInput = s_omni_input;
}

/****************************************/
/****************************************/

CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings ReferenceModel1Dot2::GetOmnidirectionalCameraInput() const
{
    return m_sOmnidirectionalCameraInput;
}

/****************************************/
/****************************************/

void ReferenceModel1Dot2::FindNeighbours()
{
    if (true) // always simulation for this branch
    {
        // the camera readings already contains the neighbours
        m_vecNeighbors.clear();
        // set every neighbour as the distance and angle from the bloblist
        for (size_t i = 0; i < m_sOmnidirectionalCameraInput.BlobList.size(); i++)
        {
            if (m_sOmnidirectionalCameraInput.BlobList[i]->Distance < 75 && m_sOmnidirectionalCameraInput.BlobList[i]->Color == CColor::RED)
            {
                m_vecNeighbors.push_back(Neighbour(m_sOmnidirectionalCameraInput.BlobList[i]->Distance,
                                                   m_sOmnidirectionalCameraInput.BlobList[i]->Angle));
            }
        }
        SetNumberNeighbors(m_vecNeighbors.size());
        return;
    }
    // identify groups of points which are the robots
    int16_t n_neigh = -1;
    // array of neighbours id which identifies uniquely a given robot
    // vector of same length as lidar input and with flag -1 to mean "no robot at this angle"
    std::vector<int> neighbourId(m_sLidarInput.size(), -1);
    // stores the previous index identified as a robot
    UInt16 latestRobotIndex;
    for (std::size_t i = 0; i < m_sLidarInput.size(); ++i)
    {
        if (m_sLidarInput[i].Value > 0.75 || m_sLidarInput[i].Value < 0.10)
        {
            // we consider it is not a robot beyond 75cm or if the reading is too close to the sensor
            continue;
        }
        // first robot point belongs to the first robot
        if (n_neigh == -1)
        {
            // belongs to neighbour 0
            neighbourId.at(i) = ++n_neigh;
            latestRobotIndex = i;
            continue;
        }

        // ulterior point
        // if the 2 points have a difference of
        // less than 2 cm in distance to current robot and
        // less than 10 degrees (0.175) in angle then they belong to the same robot
        if (Abs(m_sLidarInput.at(i).Value - m_sLidarInput.at(latestRobotIndex).Value) < 0.02 && Abs(m_sLidarInput.at(i).Angle - m_sLidarInput.at(latestRobotIndex).Angle) < CRadians(0.175))
        {
            neighbourId.at(i) = n_neigh;
            latestRobotIndex = i;
            continue;
        }

        // new group of points
        neighbourId.at(i) = ++n_neigh;
        latestRobotIndex = i;
    }

    // now we have the value of the group/robot each point belongs to,
    // -1 being no group = not a robot
    if (n_neigh == -1)
    {
        // no robot was found
        // std::cout << "No neighbour found" << std::endl;
        m_vecNeighbors.clear();
        return;
    }

    // each group will be represented by the average (Value, Angle) over the group
    std::vector<CCI_RVRLidarSensor::SReading> neighbourPositions;
    // group ids go from 0 to n_neigh => n_neigh +1 groups
    // neighbourPositions.resize(n_neigh + 1);
    for (int i = 0; i <= n_neigh; ++i)
    {
        // all the positions for this particular group i
        std::vector<CCI_RVRLidarSensor::SReading> groupPositions;
        for (size_t j = 0; j < m_sLidarInput.size(); j++)
        {
            if (neighbourId.at(j) == i)
            {
                groupPositions.push_back(m_sLidarInput.at(j));
            }
        }
        if (groupPositions.size() < 5)
        {
            // probably noise, ignore it
            // neighbourPositions.at(i) = CCI_RVRLidarSensor::SReading(0, CRadians::ZERO);
            continue;
        }
        // compute mean
        std::vector<Real> groupSum(2, 0.0f);
        for (const auto &groupMember : groupPositions)
        {
            groupSum[0] += groupMember.Value;
            groupSum[1] += groupMember.Angle.GetValue();
        }
        neighbourPositions.push_back(CCI_RVRLidarSensor::SReading(groupSum[0] / groupPositions.size(), CRadians(groupSum[1] / groupPositions.size())));
    }
    m_vecNeighbors.resize(neighbourPositions.size());
    for (int i = 0; i < neighbourPositions.size(); i++)
    {
        m_vecNeighbors.at(i).Angle = neighbourPositions.at(i).Angle;
        m_vecNeighbors.at(i).Distance = neighbourPositions.at(i).Value;
    }
    SetNumberNeighbors(m_vecNeighbors.size());
}

void ReferenceModel1Dot2::FindBeacons()
{
    if (true) // always simulation for this branch
    {
        // the camera readings already contains the neighbours
        m_vecBeacons.clear();
        // set every neighbour as the distance and angle from the bloblist
        for (size_t i = 0; i < m_sOmnidirectionalCameraInput.BlobList.size(); i++)
        {   
            // No distance limit for beacons
            if (m_sOmnidirectionalCameraInput.BlobList[i]->Color == CColor::YELLOW)
            {
                m_vecBeacons.push_back(Neighbour(m_sOmnidirectionalCameraInput.BlobList[i]->Distance,
                                                   m_sOmnidirectionalCameraInput.BlobList[i]->Angle));
            }
        }
        SetNumberBeacons(m_vecBeacons.size());
        return;
    }
    // identify groups of points which are the robots
    int16_t n_neigh = -1;
    // array of neighbours id which identifies uniquely a given robot
    // vector of same length as lidar input and with flag -1 to mean "no robot at this angle"
    std::vector<int> neighbourId(m_sLidarInput.size(), -1);
    // stores the previous index identified as a robot
    UInt16 latestRobotIndex;
    for (std::size_t i = 0; i < m_sLidarInput.size(); ++i)
    {
        if (m_sLidarInput[i].Value > 0.75 || m_sLidarInput[i].Value < 0.10)
        {
            // we consider it is not a robot beyond 75cm or if the reading is too close to the sensor
            continue;
        }
        // first robot point belongs to the first robot
        if (n_neigh == -1)
        {
            // belongs to neighbour 0
            neighbourId.at(i) = ++n_neigh;
            latestRobotIndex = i;
            continue;
        }

        // ulterior point
        // if the 2 points have a difference of
        // less than 2 cm in distance to current robot and
        // less than 10 degrees (0.175) in angle then they belong to the same robot
        if (Abs(m_sLidarInput.at(i).Value - m_sLidarInput.at(latestRobotIndex).Value) < 0.02 && Abs(m_sLidarInput.at(i).Angle - m_sLidarInput.at(latestRobotIndex).Angle) < CRadians(0.175))
        {
            neighbourId.at(i) = n_neigh;
            latestRobotIndex = i;
            continue;
        }

        // new group of points
        neighbourId.at(i) = ++n_neigh;
        latestRobotIndex = i;
    }

    // now we have the value of the group/robot each point belongs to,
    // -1 being no group = not a robot
    if (n_neigh == -1)
    {
        // no robot was found
        // std::cout << "No neighbour found" << std::endl;
        m_vecBeacons.clear();
        return;
    }

    // each group will be represented by the average (Value, Angle) over the group
    std::vector<CCI_RVRLidarSensor::SReading> neighbourPositions;
    // group ids go from 0 to n_neigh => n_neigh +1 groups
    // neighbourPositions.resize(n_neigh + 1);
    for (int i = 0; i <= n_neigh; ++i)
    {
        // all the positions for this particular group i
        std::vector<CCI_RVRLidarSensor::SReading> groupPositions;
        for (size_t j = 0; j < m_sLidarInput.size(); j++)
        {
            if (neighbourId.at(j) == i)
            {
                groupPositions.push_back(m_sLidarInput.at(j));
            }
        }
        if (groupPositions.size() < 5)
        {
            // probably noise, ignore it
            // neighbourPositions.at(i) = CCI_RVRLidarSensor::SReading(0, CRadians::ZERO);
            continue;
        }
        // compute mean
        std::vector<Real> groupSum(2, 0.0f);
        for (const auto &groupMember : groupPositions)
        {
            groupSum[0] += groupMember.Value;
            groupSum[1] += groupMember.Angle.GetValue();
        }
        neighbourPositions.push_back(CCI_RVRLidarSensor::SReading(groupSum[0] / groupPositions.size(), CRadians(groupSum[1] / groupPositions.size())));
    }
    m_vecBeacons.resize(neighbourPositions.size());
    for (int i = 0; i < neighbourPositions.size(); i++)
    {
        m_vecBeacons.at(i).Angle = neighbourPositions.at(i).Angle;
        m_vecBeacons.at(i).Distance = neighbourPositions.at(i).Value;
    }
    SetNumberBeacons(m_vecBeacons.size());
}

/****************************************/
/****************************************/

const UInt8 ReferenceModel1Dot2::GetNumberNeighbors()
{
    FindNeighbours();
    return m_unNumberNeighbors;
}

/****************************************/
/****************************************/

void ReferenceModel1Dot2::SetNumberNeighbors(const UInt8 &un_number_neighbors)
{
    m_unNumberNeighbors = un_number_neighbors;
}

/****************************************/
/****************************************/

const UInt8 ReferenceModel1Dot2::GetNumberBeacons()
{
    FindBeacons();
    return m_unNumberBeacons;
}

/****************************************/
/****************************************/

void ReferenceModel1Dot2::SetNumberBeacons(const UInt8 &un_number_beacons)
{
    m_unNumberBeacons = un_number_beacons;
}

/****************************************/
/****************************************/

CCI_RVRLidarSensor::TReadings ReferenceModel1Dot2::GetLidarInput() const
{
    return m_sLidarInput;
}

/****************************************/
/****************************************/

void ReferenceModel1Dot2::SetLidarInput(CCI_RVRLidarSensor::TReadings s_lidar_input)
{
    m_sLidarInput = s_lidar_input;
}

/****************************************/
/****************************************/

CCI_RVRLidarSensor::SReading ReferenceModel1Dot2::GetAttractionVectorToNeighbors(Real f_alpha_parameter)
{
    FindNeighbours();
    // for (UInt8 i = 0; i < m_sOmnidirectionalCameraInput.BlobList.size(); i++)
    // {
    //     neighbourPositions[i] = CCI_RVRLidarSensor::SReading(m_sOmnidirectionalCameraInput.BlobList[i]->Distance, m_sOmnidirectionalCameraInput.BlobList[i]->Angle);
    // }
    // we now have the position of each neighbour
    // we can compute the attraction vector
    // In the epuck, the vector sums possibly many messages from the same robot, artificially increasing the attraction vector.
    CVector2 lidarVectorSum(0, CRadians::ZERO);
    for (auto &robotPosition : m_vecNeighbors)
    {
        lidarVectorSum += CVector2(f_alpha_parameter / (1 + (robotPosition.Distance)), robotPosition.Angle.SignedNormalize());
    }
    CCI_RVRLidarSensor::SReading cLidarReading;
    cLidarReading.Value = lidarVectorSum.Length();
    cLidarReading.Angle = lidarVectorSum.Angle().SignedNormalize();

    return cLidarReading;
}

CCI_RVRLidarSensor::SReading ReferenceModel1Dot2::GetAttractionVectorToBeacons()
{
    FindBeacons();
    // for (UInt8 i = 0; i < m_sOmnidirectionalCameraInput.BlobList.size(); i++)
    // {
    //     neighbourPositions[i] = CCI_RVRLidarSensor::SReading(m_sOmnidirectionalCameraInput.BlobList[i]->Distance, m_sOmnidirectionalCameraInput.BlobList[i]->Angle);
    // }
    // we now have the position of each neighbour
    // we can compute the attraction vector
    CVector2 lidarVectorSum(0, CRadians::ZERO);
    for (auto &robotPosition : m_vecBeacons)
    {
        lidarVectorSum += CVector2(robotPosition.Distance, robotPosition.Angle.SignedNormalize());
    }
    CCI_RVRLidarSensor::SReading cLidarReading;
    // Set 1.0 as it was origially set for the e-puck
    if (lidarVectorSum.Length()!=0){
        cLidarReading.Value = 1.0;
    }
    else{
        cLidarReading.Value = 0.0;
    }

    // cLidarReading.Value = lidarVectorSum.Length();
    cLidarReading.Angle = lidarVectorSum.Angle().SignedNormalize();

    return cLidarReading;
}

/****************************************/
/****************************************/

CCI_RVRLidarSensor::SReading ReferenceModel1Dot2::GetNeighborsCenterOfMass()
{
    CCI_RVRLidarSensor::TReadings neighbourPositions(m_sOmnidirectionalCameraInput.BlobList.size());
    for (UInt8 i = 0; i < m_sOmnidirectionalCameraInput.BlobList.size(); i++)
    {
        neighbourPositions[i] = CCI_RVRLidarSensor::SReading(m_sOmnidirectionalCameraInput.BlobList[i]->Distance, m_sOmnidirectionalCameraInput.BlobList[i]->Angle);
    }
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