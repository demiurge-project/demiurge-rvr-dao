#include "ReferenceModel1Dot2.h"
#include <algorithm>
#include <numeric>

/****************************************/
/****************************************/

ReferenceModel1Dot2::ReferenceModel1Dot2()
{
    m_pcRng = CRandom::CreateRNG("argos");
    m_fMaxVelocity = 155.5; // 1.555 m/s
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
    for (UInt8 i = 0; i < m_sProximityInput.size(); i++)
    {
        cSumProxi += CVector2(m_sProximityInput[i].Value, m_sProximityInput[i].Angle.SignedNormalize());
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

CCI_RVRGroundColorSensor::SReading ReferenceModel1Dot2::GetGroundInput()
{
    return m_sGroundInput;
}

CColor ReferenceModel1Dot2::GetGroundReading()
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

const UInt8 ReferenceModel1Dot2::GetNumberNeighbors() const
{
    return m_sOmnidirectionalCameraInput.BlobList.size();
}

/****************************************/
/****************************************/

void ReferenceModel1Dot2::SetNumberNeighbors(const UInt8 &un_number_neighbors)
{
    m_unNumberNeighbors = un_number_neighbors;
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
        lidarVectorSum += CVector2(f_alpha_parameter / (1 + robotPosition.Value), robotPosition.Angle.SignedNormalize());
    }
    CCI_RVRLidarSensor::SReading cLidarReading;
    cLidarReading.Value = lidarVectorSum.Length();
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