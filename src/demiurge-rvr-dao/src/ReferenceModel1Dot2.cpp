#include "ReferenceModel1Dot2.h"
#include <algorithm>
#include <numeric>

/****************************************/
/****************************************/

ReferenceModel1Dot2::ReferenceModel1Dot2()
{
    m_pcRng = CRandom::CreateRNG("argos");
    m_fMaxVelocity = 50; // 12 cm/s (real max speed is 155 cm/s but it is used as is by automode)
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

void ReferenceModel1Dot2::SetWheelsVelocity(const Real &un_left_velocity, const Real &un_right_velocity)
{
    m_fLeftWheelVelocity = un_left_velocity;
    m_fRightWheelVelocity = un_right_velocity;
    PublishVelocity();
}

void ReferenceModel1Dot2::SetWheelsVelocity(const CVector2 &c_velocity_vector)
{
    m_fLeftWheelVelocity = c_velocity_vector.GetX();
    m_fRightWheelVelocity = c_velocity_vector.GetY();
    PublishVelocity();
}

/****************************************/
/****************************************/

void ReferenceModel1Dot2::InitROS()
{
    std::stringstream name;
    name.str("");
    name << "rvr" << m_unRobotIdentifier;

    if (!ros::isInitialized())
    {
        char **argv = NULL;
        int argc = 0;
        ros::init(argc, argv, name.str());
    }
    std::stringstream ss;
    ros::NodeHandle rosNode;

    // setup color sensor subscriber
    color_sensor_sub = rosNode.subscribe("/rvr/ground_color", 10, &ReferenceModel1Dot2::ColorHandler, this);
    // setup IMU subscriber
    // imu_subscriber = rosNode.subscribe("/rvr/imu", 10, &ReferenceModel1Dot2::ImuHandler, this);
    // setup light sensor subscriber
    light_subscriber = rosNode.subscribe("/rvr/ambient_light", 10, &ReferenceModel1Dot2::LightHandler, this);
    // setup odometry subscriber
    // odom_subscriber = rosNode.subscribe("/rvr/odom", 10, &ReferenceModel1Dot2::OdometryHandler, this);

    // setup teraranger subscriber
    prox_sub = rosNode.subscribe("ranges", 10, &ReferenceModel1Dot2::TerarangerHandler, this);

    // setup lidar subscriber
    lidar_sub = rosNode.subscribe("scan", 10, &ReferenceModel1Dot2::LidarHandler, this);

    // setup velocity publisher
    vel_pub = rosNode.advertise<std_msgs::Float32MultiArray>("/rvr/wheels_speed", 10, true);
    // setup velocity messages
    vel_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    vel_msg.layout.dim[0].size = 2;
    vel_msg.layout.dim[0].stride = 1;
    vel_msg.layout.dim[0].label = "wheel_vel";
    vel_msg.data.clear();

    // setup LED publisher
    // ss.str("");
    // ss << "/rvr/rgb_leds";
    // led_pub = rosNode.advertise<rvr_reference_model::Leds>(ss.str(), 10, true);
}

/****************************************/
/****************************************/

void ReferenceModel1Dot2::ColorHandler(const std_msgs::ColorRGBA &msg)
{
    m_sGroundInput.Color.SetRed((argos::UInt8)msg.r);
    m_sGroundInput.Color.SetGreen((argos::UInt8)msg.g);
    m_sGroundInput.Color.SetBlue((argos::UInt8)msg.b);
    m_sGroundInput.Color.SetAlpha((argos::UInt8)msg.a);
    printf("%s\n", m_sGroundInput.Color);
}

/****************************************/
/****************************************/

void ReferenceModel1Dot2::LightHandler(const sensor_msgs::Illuminance &msg)
{
    m_sLightInput.Value = msg.illuminance;
}

/****************************************/
/****************************************/

void ReferenceModel1Dot2::TerarangerHandler(const teraranger_array::RangeArray &msg)
{
    for (short int i = 0; i < 8; ++i)
    {
        if (msg.ranges[i].range <= 0.4)
            m_sProximityInput[i].Value = Exp(-msg.ranges[i].range);
        else
            m_sProximityInput[i].Value = 0.0f;
        CRange<Real>(0.0f, 1.0f).TruncValue(m_sProximityInput[i].Value);
    }
}

void ReferenceModel1Dot2::LidarHandler(const sensor_msgs::LaserScan &msg)
{
    for (short int i = 0; i < 719; ++i)
        m_sLidarInput[i].Value = msg.ranges[i];
}

void ReferenceModel1Dot2::PublishVelocity()
{
    vel_msg.data.clear();
    vel_msg.data.push_back((m_fLeftWheelVelocity / 100.0f)); // convert cm/s to m/s
    vel_msg.data.push_back((m_fLeftWheelVelocity / 100.0f));
    vel_pub.publish(vel_msg);
}