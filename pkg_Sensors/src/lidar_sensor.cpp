#include "lidar_sensor.h"

#include <QDebug>

#include <cmath>
#include <limits>

Lidar_Sensor::Lidar_Sensor(QObject *parent) : WorldObjectComponent_If(parent)
{
    //Update channel out
    connect(&output_channel, &Property::valueSet, this, &Lidar_Sensor::_channelChanged);

    //Update output message dimensions
    connect(&scan_points, &Property::valueSet, this, &Lidar_Sensor::_updateDataMessageDimensions);
    connect(&angle_range, &Property::valueSet, this, &Lidar_Sensor::_updateDataMessageDimensions);
    connect(&pub_rate, &Property::valueSet, this, &Lidar_Sensor::_updateDataMessageDimensions);
    connect(&radius, &Property::valueSet, this, &Lidar_Sensor::_updateDataMessageDimensions);

    sensor_model = new Model({}, {}, this);
    scan_model = new Model({}, {}, this);

    data = std::make_shared<sensor_msgs::msg::LaserScan>();

    //Instantaneous scan
    data->time_increment = 0;

    _updateDataMessageDimensions();
}

void Lidar_Sensor::_updateDataMessageDimensions()
{
    double points = scan_points.get().toInt();
    double range = angle_range.get().toDouble();

    data->ranges.resize(points);

    data->angle_min = -range/2.0 * DEG2RAD;
    data->angle_max = range/2.0 * DEG2RAD;
    data->angle_increment = range/(points-1) * DEG2RAD;

    double rate = pub_rate.get().toDouble();
    if(rate > 0)
    {
        data->scan_time = 1.0/rate;
    }
    else
    {
        data->scan_time = std::numeric_limits<float32>::infinity();
    }

    _buildModels();
}

WorldObjectComponent_If *Lidar_Sensor::clone(QObject *newParent)
{
    Lidar_Sensor* out = new Lidar_Sensor(newParent);

    for(QString s : _properties.keys())
    {
        out->_properties[s].set(_properties[s].get(), true);
    }

    return out;
}

void Lidar_Sensor::setROSNode(std::shared_ptr<rclcpp::Node> node)
{
    _sendChannel.reset();
    _rosNode = node;
}

void Lidar_Sensor::_channelChanged(QVariant)
{
    if(_connected)
    {
        disconnectChannels();
        connectChannels();
    }
}

void Lidar_Sensor::connectChannels()
{
    if(_connected)
        disconnectChannels();

    if(_rosNode)
    {
        _outputChannel = output_channel.get().toString();

        if(_outputChannel.size())
        {
            rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
            custom_qos_profile.depth = 7;

            _sendChannel = _rosNode->create_publisher<sensor_msgs::msg::LaserScan>(_outputChannel.toStdString(), custom_qos_profile);
        }
    }
    _connected = true;
}

void Lidar_Sensor::disconnectChannels()
{
    _sendChannel.reset();
    _connected = false;
}

void Lidar_Sensor::clearBodies(b2World *world)
{
    if(nullptr != sensorBody)
    {
        world->DestroyJoint(weldJoint);
        world->DestroyBody(sensorBody);

        weldJoint = nullptr;
        sensorBody = nullptr;
        sensorFix = nullptr;

        massChanged(this, 0);
    }
}

void Lidar_Sensor::generateBodies(b2World *world, object_id oId, b2Body *anchor)
{
    clearBodies(world);

    b2BodyDef bDef;
    bDef.angle = theta_local.get().toDouble()*DEG2RAD;
    bDef.type = b2_dynamicBody;
    sensorBody = world->CreateBody(&bDef);

    moveBodyToLocalSpaceOfOtherBody(sensorBody, anchor, x_local.get().toDouble(), y_local.get().toDouble());

    b2WeldJointDef weldDef;
    auto anchorPt = anchor->GetWorldCenter();
    weldDef.bodyA = anchor;
    weldDef.bodyB = sensorBody;
    weldDef.localAnchorA = anchor->GetLocalPoint(anchorPt);
    weldDef.localAnchorB = sensorBody->GetLocalPoint(anchorPt);
    weldDef.referenceAngle = sensorBody->GetAngle() - anchor->GetAngle();
    weldJoint = world->CreateJoint(&weldDef);

    objectId = oId;

    _attachSensorFixture();
    _buildModels();
}

void Lidar_Sensor::_attachSensorFixture()
{
    //Can only add fixture if body defined
    if(sensorBody)
    {
        //If old fixture, remove it
        if(sensorFix)
        {
            sensorBody->DestroyFixture(sensorFix);
            sensorFix = nullptr;
        }

        //Create circle at ring radius
        b2FixtureDef fixDef;
        b2CircleShape sensorCircle;

        sensorCircle.m_radius = SHAPE_RADIUS;
        sensorCircle.m_p = b2Vec2(0, 0);

        fixDef.isSensor = true;
        fixDef.shape = &sensorCircle;
        fixDef.filter.groupIndex = -objectId;
        fixDef.density = 1;

        sensorFix = sensorBody->CreateFixture(&fixDef);

        massChanged(this, sensorBody->GetMass());
    }
}

void Lidar_Sensor::_buildModels()
{
    //Clear always-showing model
    QVector<b2Shape*> oldModel = sensor_model->shapes();
    sensor_model->removeShapes(oldModel);
    qDeleteAll(oldModel);

    //Build always-showing model...
    b2CircleShape* ring = new b2CircleShape;
    ring->m_radius = SHAPE_RADIUS;
    ring->m_p = b2Vec2(0,0);

    b2EdgeShape* line = new b2EdgeShape;
    double x = sqrt(3.0)/2.0 * SHAPE_RADIUS;
    line->m_vertex1.Set(-x, 0.5*SHAPE_RADIUS);
    line->m_vertex2.Set(x, 0.5*SHAPE_RADIUS);
    sensor_model->addShapes({ring, line});

    //Clear sensor-hits shapes
    scan_model->removeShapes(scan_model->shapes());
    qDeleteAll(scan_image);

    //Build new sensor-hits shapes
    scan_image.clear();
    scan_image.resize(data->ranges.size());

    double curr_angle = data->angle_min;
    double scan_radius = radius.get().toDouble();

    for(int i=0; i<scan_image.size(); i++, curr_angle += data->angle_increment)
    {
        b2EdgeShape* hit = new b2EdgeShape;
        hit->m_vertex1.Set(0, 0);
        hit->m_vertex2 = _getRayPoint(curr_angle, scan_radius);

        scan_image[i] = hit;
    }
    scan_model->addShapes(scan_image);
}

b2Vec2 Lidar_Sensor::_getRayPoint(double angle_rad, double dist)
{
    return b2Vec2(sin(angle_rad) * dist, cos(angle_rad) * dist);
}

void Lidar_Sensor::worldTicked(const b2World* world, const double dt)
{
    if(sensorBody)
    {
        _timeSinceScan += dt;
        double pr = pub_rate.get().toDouble();

        double x = sensorBody->GetWorldCenter().x;
        double y = sensorBody->GetWorldCenter().y;
        double t = sensorBody->GetAngle();
        sensor_model->setTransform(x, y, t*RAD2DEG);
        scan_model->setTransform(x, y, t*RAD2DEG);

        if(pr > 0 && _timeSinceScan > 1.0/pr)
        {
            double curr_angle = data->angle_min;
            double scan_radius = radius.get().toDouble();

            data->range_min = scan_radius;
            data->range_max = 0;

            b2Vec2 worldOrigin = sensorBody->GetPosition();
            for(int i=0; i<scan_image.size(); i++, curr_angle += data->angle_increment)
            {
                b2Vec2 localEndpoint = _getRayPoint(curr_angle, scan_radius);
                b2Vec2 worldEndpoint = sensorBody->GetWorldPoint(localEndpoint);

                QPair<b2Vec2, double> rayCastResult = _rayCaster.rayCast(world, worldOrigin, worldEndpoint);

                b2EdgeShape* thisLine = dynamic_cast<b2EdgeShape*>(scan_image[i]);
                thisLine->m_vertex2 = sensorBody->GetLocalPoint(rayCastResult.first);

                data->ranges[i] = rayCastResult.second;

                if(data->ranges[i] >= 0)
                    data->range_min = std::min(data->ranges[i], data->range_min);

                if(data->ranges[i] <= scan_radius)
                    data->range_max = std::max(data->ranges[i], data->range_max);
            }

            if(_sendChannel)
            {
                _sendChannel->publish(data);
            }

            //Force lines to be redrawn
            scan_model->forceDraw();

            _timeSinceScan = 0;
        }
    }
}