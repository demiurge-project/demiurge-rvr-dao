#ifndef REFERENCE_MODEL_1_2_H
#define REFERENCE_MODEL_1_2_H

#include "RVRDAO.h"
#include "ros/ros.h"

#include <tf/transform_broadcaster.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <rvr_reference_model/Leds.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Illuminance.h>

// terabee
#include "teraranger_array/RangeArray.h"
#include "sensor_msgs/Range.h"

// lidar
#include "sensor_msgs/LaserScan.h"

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
     * Checks if the reference model has had real robot connection.
     */
    const bool HasRealRobotConnection() const;

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
    CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings GetOmnidirectionalCameraInput() const;

    /*
     * Setter for the omnidirectional camera input.
     */
    void SetOmnidirectionalCameraInput(CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings s_omnidirectional_camera_input);

    /*
     * Getter for the number of surrounding robots.
     */
    const UInt8 GetNumberNeighbors();

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

    /*
     * Setter for the wheels velocity.
     */
    void SetWheelsVelocity(const Real &un_left_velocity, const Real &un_right_velocity);

    /*
     * Setter for the wheels velocity.
     */
    void SetWheelsVelocity(const CVector2 &c_velocity_vector);

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
     * This contains the position of each neighbour, either directly
     * from the virtual camera in simulation, or from the lidar readings
     */
    CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings m_sOmnidirectionalCameraInput;

    /*
     * The number of surrounding robots.
     */
    UInt8 m_unNumberNeighbors;

public:
    /*
    Initializes ROS.
    */
    void InitROS();

    /* Handler for the ground color sensor data from RVR driver */
    virtual void ColorHandler(const std_msgs::ColorRGBA &msg);

    /* Handler for the IMU message from the driver, which contains :
     * - orientation (from the IMU)
     * - angular velocity (from the gyroscope)
     * - linear acceleration (from the accelerometer)
     */
    // virtual void ImuHandler(const sensor_msgs::Imu &msg);

    /* Handler for the ambient light */
    virtual void LightHandler(const sensor_msgs::Illuminance &msg);

    /* Handler for odometry */
    // virtual void OdometryHandler(const nav_msgs::Odometry &msg);

    /* Handler for proximity sensor data */
    virtual void TerarangerHandler(const teraranger_array::RangeArray &msg);

    /* Handler for lidar Laserscan data */
    virtual void LidarHandler(const sensor_msgs::LaserScan &msg);

    virtual void PublishVelocity();

    /*
     * Clusters the lidar readings to neighbours, or uses the virtual camera in
     * simulation
     */
    virtual void FindNeighbours();

private:
    /* Sensors subscribers */
    ros::Subscriber color_sensor_sub;
    ros::Subscriber imu_subscriber;
    ros::Subscriber light_subscriber;
    ros::Subscriber odom_subscriber;

    /* Proximity sensors subscriber */
    ros::Subscriber prox_sub;

    /* Lidar subscriber */
    ros::Subscriber lidar_sub;

    /* Actuators publishers */

    /* Wheel speed publisher */
    ros::Publisher vel_pub;
    std_msgs::Float32MultiArray vel_msg;

    /* LED publisher */
    ros::Publisher led_pub;
    rvr_reference_model::Leds led_msg;
};

#endif