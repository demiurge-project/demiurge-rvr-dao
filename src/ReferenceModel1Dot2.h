#ifndef REFERENCE_MODEL_1_1_H
#define REFERENCE_MODEL_1_1_H

#include "RVRDAO.h"

using namespace argos;

class ReferenceModel1Dot2 : public RVRDAO
{
public:
    /*
     *  Class constructor.
     */
    ReferenceModel1Dot2();

    /*
     * Class destructor.
     */
    virtual ~ReferenceModel1Dot2();

    /*
     * Reset function.
     */
    virtual void Reset();

    /*
     * Getter for the proximity input.
     */
    CCI_RVRProximitySensor::TReadings GetProximityInput() const;

    /*
     * Getter for the proximity reading.
     */
    CCI_RVRProximitySensor::SReading GetProximityReading();

    /*
     * Setter for the proximity input.
     */
    void SetProximityInput(CCI_RVRProximitySensor::TReadings s_prox_input);

    /*
     * Getter for the lidar input
     */
    CCI_RVRLidarSensor::TReadings GetLidarInput() const;

    /*
     * Setter for the lidar input.
     */
    void SetLidarInput(CCI_RVRLidarSensor::TReadings s_lidar_input);

    /*
     * Getter for the light input.
     */
    CCI_RVRLightSensor::SReading GetLightInput() const;

    /*
     * Getter for the light reading.
     */
    CCI_RVRLightSensor::SReading GetLightReading();

    /*
     * Setter for the light input.
     */
    void SetLightInput(CCI_RVRLightSensor::SReading s_light_input);

    /*
     * Getter for the ground input.
     */
    CCI_RVRGroundColorSensor::SReading GetGroundInput();

    /*
     * Getter for the ground reading
     */
    CColor GetGroundReading();

    /*
     * Setter for the ground input.
     */
    void SetGroundInput(CCI_RVRGroundColorSensor::SReading s_ground_input);

    /*
     * Getter for the omnidirectional camera input.
     */
    CCI_ColoredBlobOmnidirectionalCameraSensor::TBlobList GetOmnidirectionalCameraInput();

    /*
     * Setter for the omnidirectional camera input.
     */
    void SetOmnidirectionalCameraInput(CCI_ColoredBlobOmnidirectionalCameraSensor::TBlobList s_omnidirectional_camera_input);

    /*
     * Getter for the number of surrounding robots.
     */
    const UInt8 GetNumberNeighbors() const;

    /*
     * Setter for the number of surrounding robots.
     */
    virtual void SetNumberNeighbors(const UInt8 &un_number_neighbors);

    /*
     * Getter for the vector representing the attraction force to the neighbors computed with lidar information
     */
    CCI_RVRLidarSensor::SReading GetAttractionVectorToNeighbors(Real f_alpha_parameter);

    /*
     * Getter for the center of mass of neighbors computed with RaB messages
     */
    CCI_RVRLidarSensor::SReading GetNeighborsCenterOfMass();

private:
    /*
     * The proximity sensors input.
     */
    CCI_RVRProximitySensor::TReadings m_sProximityInput;

    /*
     * The light sensors input.
     */
    CCI_RVRLightSensor::SReading m_sLightInput;

    /*
     * The ground sensors input.
     */
    CCI_RVRGroundColorSensor::SReading m_sGroundInput;

    /*
     * The lidar input.
     */
    CCI_RVRLidarSensor::TReadings m_sLidarInput;

    /*
     * The number of surrounding robots.
     */
    UInt8 m_unNumberNeighbors;

    /* Utility function to turn the lidar readings into robot positions */
    std::vector<CCI_RVRLidarSensor::SReading> LidarToRobotPositions(CCI_RVRLidarSensor::TReadings s_lidar_input) const;
};

#endif