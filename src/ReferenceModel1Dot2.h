#ifndef REFERENCE_MODEL_1_2_H
#define REFERENCE_MODEL_1_2_H

#include "RVRDAO.h"

using namespace argos;

class ReferenceModel1Dot2 : public RVRDAO
{
public:
    struct Neighbour
    {
        Real Distance;
        CRadians Angle;

        Neighbour()
        {
            Distance = 0.0;
            Angle = CRadians::ZERO;
        }

        Neighbour(Real f_distance, CRadians f_angle) : Distance(f_distance),
                                                       Angle(f_angle)
        {
        }
    };

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
    CCI_RVRGroundColorSensor::SReading GetGroundInput() const;

    /*
     * Getter for the ground reading
     */
    CColor GetGroundReading() const;

    /*
     * Setter for the ground input.
     */
    void SetGroundInput(CCI_RVRGroundColorSensor::SReading s_ground_input);

    /*
     * Getter for the omnidirectional camera input.
     */
    CCI_RVRColoredBlobOmnidirectionalCameraSensor::SReadings GetOmnidirectionalCameraInput() const;

    /*
     * Setter for the omnidirectional camera input.
     */
    void SetOmnidirectionalCameraInput(CCI_RVRColoredBlobOmnidirectionalCameraSensor::SReadings s_omnidirectional_camera_input);

    /*
     * Getter for the number of surrounding robots.
     */
    const UInt8 GetNumberNeighbors();

    /*
     * Setter for the number of surrounding robots.
     */
    virtual void SetNumberNeighbors(const UInt8 &un_number_neighbors);

    /*
     * Getter for the number of surrounding beacons.
     */
    const UInt8 GetNumberBeacons();

    /*
     * Setter for the number of surrounding beacons.
     */
    virtual void SetNumberBeacons(const UInt8 &un_number_beacons);

    /*
     * Getter for the vector representing the attraction force to the neighbors computed with lidar information
     */
    CCI_RVRLidarSensor::SReading GetAttractionVectorToNeighbors(Real f_alpha_parameter);

    /*
     * Getter for the vector representing the attraction force to the beacons computed with lidar information
     */
    CCI_RVRLidarSensor::SReading GetAttractionVectorToBeacons();


    /*
     * Getter for the center of mass of neighbors computed with RaB messages
     */
    CCI_RVRLidarSensor::SReading GetNeighborsCenterOfMass();

    /*
     * Clusters the lidar readings to neighbours, or uses the virtual camera in
     * simulation
     */
    virtual void FindNeighbours();

    /*
     * Clusters the lidar readings to beacons, or uses the virtual camera in
     * simulation
     */
    virtual void FindBeacons();

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
     * The omnidirectional camera input.
     */
    CCI_RVRColoredBlobOmnidirectionalCameraSensor::SReadings m_sOmnidirectionalCameraInput;

    /*
     * The number of surrounding robots.
     */
    UInt8 m_unNumberNeighbors;

    /** The list of neighbours */
    std::vector<Neighbour> m_vecNeighbors;

    /*
     * The number of surrounding beacons.
     */
    UInt8 m_unNumberBeacons;

    /** The list of beacons */
    std::vector<Neighbour> m_vecBeacons;
};

#endif
