#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <veranda_core/api/filter.h>
#include <veranda_core/api/world_object_component.h>
#include <veranda_perception/dllapi.h>

#include <Box2D/Box2D.h>

class VERANDA_PERCEPTION_DLL SimpleObstacleMap : public WorldObjectComponent
{

Q_OBJECT

//! Id of the WorldObject the gps is part of
object_id objectId = 0;

//! ROS channel to publish on
QString _outputChannel;

//! ROS node to publish messages with
std::shared_ptr<rclcpp::Node> _rosNode;

//! ROS channel to publish Occupancy Grid messages on
rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr _sendChannel;

//! Property: ROS channel to publish on
Property output_channel = Property(PropertyInfo(false, true, false, PropertyInfo::STRING,
                                                "Output channel for obstacle map messages"), "");

//! Property: ROS channel to publish on
Property map_frame_id = Property(PropertyInfo(false, true, false, PropertyInfo::STRING,
                                                "Frame id"), "map");

//! Property: Rate of publishing messages
Property pub_rate = Property(PropertyInfo(false, true, false, PropertyInfo::DOUBLE, "map rate (hz)"),
                             QVariant(10), &Property::abs_double_validator);

//! Property: map resolution (meter/cell)
Property map_resolution = Property(PropertyInfo(false, true, false, PropertyInfo::DOUBLE,
                                        "map resolution (meter/cell)"), QVariant(0.5),
                           &Property::abs_double_validator);

//! Property: map width (cells)
Property map_width = Property(PropertyInfo(false, true, false, PropertyInfo::INT,
                                             "map width (cells)"), QVariant(10),
                                [](QVariant _old, QVariant _new)
                                {
                                    bool valid;
                                    int newVal = _new.toInt(&valid);
                                    if(valid && newVal >= 2)
                                        return _new;
                                    return _old;
                                });

//! Property: map height (cells)
Property map_height = Property(PropertyInfo(false, true, false, PropertyInfo::INT,
                                           "map height (cells)"), QVariant(10),
                              [](QVariant _old, QVariant _new)
                              {
                                  bool valid;
                                  int newVal = _new.toInt(&valid);
                                  if(valid && newVal >= 2)
                                      return _new;
                                  return _old;
                              });

//! Property: minimum perception width (meter)
Property min_perception_width = Property(PropertyInfo(false, true, false, PropertyInfo::DOUBLE,
                                                "minimum perception width (meter)"), QVariant(0.5),
                                   &Property::abs_double_validator);

//! Property: minimum perception height (meter)
Property min_perception_height = Property(PropertyInfo(false, true, false, PropertyInfo::DOUBLE,
                                                "minimum perception height (meter)"), QVariant(0.5),
                                   &Property::abs_double_validator);



#define pview(a) QSharedPointer<PropertyView>(new PropertyView(a))
//! Mapping of lidar propertys by their identifiers
QMap<QString, QSharedPointer<PropertyView>> _properties{
        {"channels/occupancy_map", pview(&output_channel)},
        {"map_frame_id", pview(&map_frame_id)},
        {"map_rate", pview(&pub_rate)},
        {"map_resolution", pview(&map_resolution)},
        {"map_width", pview(&map_width)},
        {"map_height", pview(&map_height)},
        {"min_perception_width", pview(&min_perception_width)},
        {"min_perception_height", pview(&min_perception_height)}
};
#undef pview

std::shared_ptr<nav_msgs::msg::OccupancyGrid> data;
//! Model for the body of the gps
Model* sensor_model = nullptr;

//! Fixture of the main part of the lidar
b2Fixture* sensorFix = nullptr;

//! Joint connecting the main lidar body to the anchor body
b2Joint* weldJoint = nullptr;

//! Box2D world in use
b2World* _world = nullptr;

//! IID of plugin that generated this object
const QString _pluginIID;


public:
/*!
 * \brief Construct a new lidar component
 * \param[in] parent QObject parent
 */
SimpleObstacleMap(const QString& pluginIID, QObject* parent=nullptr);
virtual ~SimpleObstacleMap() override {}

/*!
 * \brief Creates a new Lidar Component
 * \param[in] newParent QObject parent of copy
 * \return A newly constructed Lidar component
 */
WorldObjectComponent *_clone(QObject *newParent) override;

/*!
 * \brief Getter for the lidar-specific properties
 * \return Mapping of lidar properties by identifiers
 */
QMap<QString, QSharedPointer<PropertyView>> _getProperties() override{
return _properties;
}

/*!
 * \brief Check if component uses ROS channels
 * \return true
 */
bool usesChannels() override{
return true;
}

/*!
 * \brief Loads the component into the box2d world
 * \param[in] world Box2D world to add lidar sensor to
 * \param[in] oId object_id of the object the lidar is part of
 * \param[in] anchor Anchor body to joint self to
 */
void _generateBodies(b2World *world, object_id oId, b2Body *anchor) override;

//! Clears all bodies in the Box2d world
void _clearBodies() override;

/*!
 * \brief Sets the ROS node to publish messages with
 * \param[in] node The ROS node to use
 */
void _setROSNode(std::shared_ptr<rclcpp::Node> node) override;

/*!
 * \brief Returns the plugin IID
 * \return The IID of the plugin providing this component
 */
QString getPluginName() override { return _pluginIID; }

//subclass b2QueryCallback
class MyQueryCallback : public b2QueryCallback {
public:
    //std::vector<b2Body*> foundBodies;
    bool point_has_object_collision_;

    bool ReportFixture(b2Fixture* fixture) override{
        //foundBodies.push_back( fixture->GetBody() );
        //std::cerr << "point has collision with object of fixture type:" << fixture->GetType() << " and body type: " << fixture->GetBody()->GetType() << std::endl;
        point_has_object_collision_ = true;
        return false;// stop once collision detected//keep going to find all fixtures in the query area
    }

    bool hasCollision(const b2World* world, b2AABB area_of_collision)
    {
        point_has_object_collision_ = false;

        world->QueryAABB(this, area_of_collision);

        return point_has_object_collision_;
    }
};

private:

    float cell_size_; //in meters
    rclcpp::Clock perception_clock_;

    MyQueryCallback querry_callback_;

    //! Body of the main part of the lidar
    b2Body* sensorBody = nullptr;

    void setMapHeader();

private slots:
//! Refreshes ROS channel
void _channelChanged();

public slots:
//! Connects to all ROS topics
virtual void _connectChannels() override;

//! Disconnects all ROS topics
virtual void _disconnectChannels() override;

//! Updates the time since last message, and if it is time to publish a new message, the lidar ranges are recomputed
virtual void _worldTicked(const double dt) override;

//! Creates the box2d fixture for the lidar
void _attachSensorFixture();

//! Builds the shapes drawn to represent the lidar
void _buildModels();
};