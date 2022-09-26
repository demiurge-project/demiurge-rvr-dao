
/*
 * @file <src/core/RVRDAO.h>
 *
 * @author Raffaele Todesco - <raffaele.todesco@ulb.be>
 *
 * @package ARGoS3-AutoMoDe
 *
 * @license MIT License
 *
 * @brief This class represents the status of the robot.
 * 				It contains the input variables (the sensor inputs) and
 * 				the output variables (the values for the wheel actuators),
 * 				as well as the setters and getters to access them.
 *
 * 				Only one object of this class should be instanciated, and
 * 				is to be used as a brigde between the AutoMoDeController and
 * 				the AutoMoDeFiniteStateMachine classes. In AutoMoDeController,
 * 				the variables of the object shall be updated at each time step.
 * 				The different modules of the  AutoMoDeFiniteStateMachine will
 * 				then use the input variables and update the output variables
 * 				accordingly.
 */

#ifndef RVR_DAO
#define RVR_DAO

#include <vector>
#include <deque>

#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/plugins/robots/rvr/control_interface/ci_rvr_wheels_actuator.h>
#include <argos3/plugins/robots/rvr/control_interface/ci_rvr_rgb_leds_actuator.h>
#include <argos3/plugins/robots/rvr/control_interface/ci_rvr_proximity_sensor.h>
#include <argos3/plugins/robots/rvr/control_interface/ci_rvr_light_sensor.h>
#include <argos3/plugins/robots/rvr/control_interface/ci_rvr_ground_color_sensor.h>
#include <argos3/plugins/robots/rvr/control_interface/ci_rvr_lidar_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>

namespace argos
{
    class RVRDAO
    {
    public:
        virtual ~RVRDAO();

        /*
         * Reset function.
         */
        virtual void Reset() = 0;

        /*
         * Setter for the wheels velocity.
         */
        void SetWheelsVelocity(const Real &un_left_velocity, const Real &un_right_velocity);

        /*
         * Setter for the wheels velocity.
         */
        void SetWheelsVelocity(const CVector2 &c_velocity_vector);

        /*
         * Getter for the right wheel velocity.
         */
        const Real &GetRightWheelVelocity() const;

        /*
         * Getter for the left wheel velocity.
         */
        const Real &GetLeftWheelVelocity() const;

        /*
         * Setter for the robot idientifier.
         */
        void SetRobotIdentifier(const UInt32 &un_robot_id);

        /*
         * Getter for the robot identifier.
         */
        const UInt32 &GetRobotIdentifier() const;

        /*
         * Getter for the maximal wheels velocity.
         */
        const Real &GetMaxVelocity() const;

        /*
         * Setter for the maximal wheels velocity.
         */
        void SetMaxVelocity(const Real &un_max_velocity);

        /*
         * Getter for the random number generetor.
         */
        CRandom::CRNG *GetRandomNumberGenerator() const;

        /*******************/
        /* Virtual classes */
        /*******************/

        /*
         * Getter for the proximity input.
         */
        virtual CCI_RVRProximitySensor::TReadings GetProximityInput() const
        { // RM 1.1
            CCI_RVRProximitySensor::TReadings emptyReadings;
            return emptyReadings;
        };

        virtual CCI_RVRProximitySensor::SReading GetProximityReading()
        { // RM 1.2
            return CCI_RVRProximitySensor::SReading();
        };

        /*
         * Setter for the proximity input.
         */
        virtual void SetProximityInput(CCI_RVRProximitySensor::TReadings s_prox_input){};

        /*
         * Setter for the lidar input.
         */
        virtual void SetLidarInput(CCI_RVRLidarSensor::TReadings s_lidar_input){};

        /*
         * Getter for the lidar input
         */

        virtual CCI_RVRLidarSensor::TReadings GetLidarInput() const
        {
            CCI_RVRLidarSensor::TReadings lidarReadings;
            return lidarReadings;
        }

        virtual CCI_RVRLightSensor::SReading GetLightReading()
        { // RM 1.2
            return CCI_RVRLightSensor::SReading();
        };

        /*
         * Setter for the light input.
         */
        virtual void SetLightInput(CCI_RVRLightSensor::SReading s_light_input){};

        /*
         * Getter for the ground input.
         */
        virtual CCI_RVRGroundColorSensor::SReading GetGroundInput() const
        { // RM 1.1
            return CCI_RVRGroundColorSensor::SReading();
        };

        virtual CColor GetGroundReading() const
        { // RM 1.2
            return CColor();
        };

        /*
         * Setter for the ground input.
         */
        virtual void SetGroundInput(CCI_RVRGroundColorSensor::SReading s_ground_input){};

        /**
         * Getter for the omnidirectional camera input.
         */
        virtual CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings GetOmnidirectionalCameraInput() const
        {
            CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings emptyList;
            return emptyList;
        }

        /**
         * Setter for the omnidirectional camera input.
         */
        virtual void SetOmnidirectionalCameraInput(CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings s_omni_camera_input){};

        /*
         * Getter for the number of surrounding robots.
         */
        virtual const UInt8 GetNumberNeighbors()
        {
            UInt8 unEmptyVariable = 0;
            return unEmptyVariable;
        };

        /*
         * Getter for attraction force to the neighbors computed with lidar information
         */
        virtual CCI_RVRLidarSensor::SReading GetAttractionVectorToNeighbors(Real f_alpha_parameter)
        { // RM 1.2
            return CCI_RVRLidarSensor::SReading();
        };

        /*
         * Getter for the vector representing the attraction force to the neighbors that are sending a message computed with lidar information
         */
        virtual CCI_RVRLidarSensor::SReading GetAttractionVectorToDetectedNeighbors(Real f_alpha_parameter, UInt8 un_message)
        { // RM2.0
            return CCI_RVRLidarSensor::SReading();
        };

        /*
         * Getter for the center of mass of neighbors computed with lidar information
         */
        virtual CCI_RVRLidarSensor::SReading GetNeighborsCenterOfMass()
        {
            return CCI_RVRLidarSensor::SReading();
        };

        /*
         * Getter for the center of mass of detected neighbors computed with lidar information
         */
        virtual CCI_RVRLidarSensor::SReading GetDetectedNeighborsCenterOfMass(UInt8 un_message)
        {
            return CCI_RVRLidarSensor::SReading();
        };

        /*
         * Setter for the number of surrounding robots.
         */
        virtual void SetNumberNeighbors(const UInt8 &un_number_neighbors){};

        /*
         * Setter for the message to send with range and bearing
         */
        // virtual void SetRangeAndBearingMessageToSend(UInt8 un_message){};

        /*
         * Getter for attraction force to the beacon (light source) computed with lidar information
         */
        virtual CCI_RVRLidarSensor::SReading GetAttractionVectorToBeacons()
        { // RM 1.2
            return CCI_RVRLidarSensor::SReading();
        };

        /*
         * Setter for the number of surrounding robots.
         */
        virtual void SetNumberBeacons(const UInt8 &un_number_beacons){};

        /*
         * Getter for the number of surrounding robots.
         */
        virtual const UInt8 GetNumberBeacons()
        {
            UInt8 unEmptyVariable = 0;
            return unEmptyVariable;
        };


    protected:
        /*
         * The left wheel velocity (output variable).
         */
        Real m_fLeftWheelVelocity;

        /*
         * The right wheel velocity (output variable).
         */
        Real m_fRightWheelVelocity;

        /*
         * The maximal wheels velocity.
         */
        Real m_fMaxVelocity;

        /*
         * The robot identifier.
         */
        UInt32 m_unRobotIdentifier;

        /*
         * Pointer to the random number generator.
         */
        CRandom::CRNG *m_pcRng;
    };
}

#endif