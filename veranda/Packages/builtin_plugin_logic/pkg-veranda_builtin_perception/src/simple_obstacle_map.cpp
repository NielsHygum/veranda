#include "simple_obstacle_map.h"
#include <tf2/LinearMath/Quaternion.h>
#include <rclcpp/time.hpp>

SimpleObstacleMap::SimpleObstacleMap(const QString& pluginIID, QObject *parent)
        : WorldObjectComponent("Simple Obstacle Map", "Perception", parent)
        , _pluginIID(pluginIID)
{
    //Update channel out
    connect(&output_channel, &Property::valueSet, this, &SimpleObstacleMap::_channelChanged);

    //Update output message dimensions
    //connect(&pub_rate, &Property::valueSet, this, &Lidar_Sensor::_updateDataMessageDimensions);

    data = std::make_shared<nav_msgs::msg::OccupancyGrid>();

    //Instantaneous scan
    //data->time_increment = 0;

    //_updateDataMessageDimensions();

    sensor_model = new Model({}, {}, this);
    registerModel(sensor_model);
    _buildModels();

    std::cerr << "simple obstacle map constructor:)" << std::endl;


}



void SimpleObstacleMap::_setROSNode(std::shared_ptr<rclcpp::Node> node)
{

    _sendChannel.reset();
    _rosNode = node;
}

void SimpleObstacleMap::_channelChanged()
{

    if(_sendChannel)
    {
        connectChannels();
    }
}

void SimpleObstacleMap::_connectChannels()
{

    disconnectChannels();

    if(_rosNode)
    {
        _outputChannel = output_channel.get().toString();

        if(_outputChannel.size())
        {
            _sendChannel = _rosNode->create_publisher<nav_msgs::msg::OccupancyGrid>(_outputChannel.toStdString(), 7);
        }
    }
}

void SimpleObstacleMap::_disconnectChannels()
{
    _sendChannel.reset();
}

WorldObjectComponent *SimpleObstacleMap::_clone(QObject *newParent)
{

    SimpleObstacleMap* out = new SimpleObstacleMap(_pluginIID, newParent);

    return out;
}

void SimpleObstacleMap::setMapHeader()
{


    data->header.frame_id = map_frame_id.get().toString().toStdString();
    data->header.stamp = perception_clock_.now();

    cell_size_ = map_resolution.get().toDouble();
    uint width = map_width.get().toInt();
    uint height = map_height.get().toInt();

    data->info.resolution = cell_size_;
    data->info.width = width;
    data->info.height = height;

}

void SimpleObstacleMap::_worldTicked(const double dt)
{

    if(sensorBody)
    {
        static double time_since_last_pub = 0.0;
        time_since_last_pub += dt;
        double pr = pub_rate.get().toDouble();

        if(pr > 0.0 and time_since_last_pub > 1.0/pr)
        {
            setMapHeader();


            float min_perception_range_x = min_perception_width.get().toDouble();
            float min_perception_range_y = min_perception_height.get().toDouble();
            size_t map_cols = map_width.get().toInt();
            size_t map_rows = map_height.get().toInt();
            float map_width = cell_size_ * map_cols; // in meter
            float map_height = cell_size_ * map_rows; // in meter


            // map origo is lower left
            float map_origo_body_frame_x = -map_width/2.0f;
            float map_origo_body_frame_y = -map_height/2.0f;

            float body_position_veranda_frame_x = sensorBody->GetPosition().x;
            float body_position_veranda_frame_y = sensorBody->GetPosition().y;

            float body_yaw = sensorBody->GetAngle();

            float map_origo_veranda_frame_x = body_position_veranda_frame_x + map_origo_body_frame_x * cos(body_yaw) - map_origo_body_frame_y * sin(body_yaw);
            float map_origo_veranda_frame_y = body_position_veranda_frame_y + map_origo_body_frame_x * sin(body_yaw) + map_origo_body_frame_y * cos(body_yaw);

            if(data->data.size() != map_cols*map_rows)
            {
                data->data.resize(map_cols*map_rows);
            }

            for(size_t row = 0; row < map_rows; row++)
                for(size_t col = 0; col < map_cols; col++)
                {
                    float cell_center_body_frame_x = map_origo_body_frame_x + (row+0.5f)*cell_size_;
                    float cell_center_body_frame_y = map_origo_body_frame_y + (col+0.5f)*cell_size_;

                    if((fabs(cell_center_body_frame_x) <= min_perception_range_x) and (fabs(cell_center_body_frame_y) <= min_perception_range_y))
                    {

                        data->data[col*map_cols+row] = 0;

                    } else {

                        float cell_center_veranda_frame_x = body_position_veranda_frame_x + cell_center_body_frame_x * cos(body_yaw) - cell_center_body_frame_y * sin(body_yaw);
                        float cell_center_veranda_frame_y = body_position_veranda_frame_y + cell_center_body_frame_x * sin(body_yaw) + cell_center_body_frame_y * cos(body_yaw);

                        b2Vec2 cell_center(cell_center_veranda_frame_x, cell_center_veranda_frame_y);

                        b2AABB area_of_collision;
                        area_of_collision.lowerBound = cell_center;
                        area_of_collision.upperBound = cell_center;

                        bool cell_collision = querry_callback_.hasCollision(_world, area_of_collision);

                        data->data[col*map_cols+row] = cell_collision? 100 : 0;
                    }



                }

            tf2::Quaternion body_orientation;
            body_orientation.setRPY( 0.0f, 0.0, sensorBody->GetAngle() );

            data->info.origin.position.x = map_origo_veranda_frame_x;
            data->info.origin.position.y = map_origo_veranda_frame_y;
            data->info.origin.position.z = 0.0;



            data->info.origin.orientation.x = body_orientation.x();
            data->info.origin.orientation.y = body_orientation.y();
            data->info.origin.orientation.z = body_orientation.z();
            data->info.origin.orientation.w = body_orientation.w();

            if(_sendChannel)
            {
                _sendChannel->publish(*data);
                time_since_last_pub = 0.0;
            }

        }
    }



    /*
    if(sensorBody)
    {
        _timeSinceScan += dt;
        double pr = pub_rate.get().toDouble();

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

                QPair<b2Vec2, double> rayCastResult = _rayCaster.rayCast(_world, worldOrigin, worldEndpoint, -objectId);

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
                _sendChannel->publish(*data);
            }

            //Force lines to be redrawn
            scan_model->forceDraw();

            _timeSinceScan = 0;
        }
    }*/
}


void SimpleObstacleMap::_clearBodies()
{

    if(_world)
    {
        _world->DestroyJoint(weldJoint);
        _world->DestroyBody(sensorBody);
        unregisterBody(sensorBody);
        weldJoint = nullptr;
        sensorBody = nullptr;
        sensorFix = nullptr;
    }
    _world = nullptr;
}

void SimpleObstacleMap::_generateBodies(b2World *world, object_id oId, b2Body *anchor)
{
    clearBodies();
    _world = world;

    b2BodyDef bDef;
    bDef.type = b2_dynamicBody;
    sensorBody = world->CreateBody(&bDef);
    registerBody(sensorBody, {sensor_model}, true);

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


void SimpleObstacleMap::_attachSensorFixture()
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

        sensorCircle.m_radius = 1;//SHAPE_RADIUS;
        sensorCircle.m_p = b2Vec2(0, 0);

        fixDef.isSensor = true;
        fixDef.shape = &sensorCircle;
        fixDef.filter.groupIndex = -objectId;
        fixDef.density = 1;

        sensorFix = sensorBody->CreateFixture(&fixDef);
    }
}

void SimpleObstacleMap::_buildModels()
{
    //Clear always-showing model
    QVector<b2Shape*> oldModel = sensor_model->shapes();
    sensor_model->removeShapes(oldModel);
    qDeleteAll(oldModel);

    //Build always-showing model...
    b2CircleShape* ring = new b2CircleShape;
    ring->m_radius = 1;//SHAPE_RADIUS;
    ring->m_p = b2Vec2(0,0);

    double x = sqrt(2.0)/2.0 * 1;//SHAPE_RADIUS;

    b2EdgeShape* line = new b2EdgeShape;
    line->m_vertex1.Set(x, x);
    line->m_vertex2.Set(-x, -x);
    sensor_model->addShapes({ring, line});

    line = new b2EdgeShape;
    line->m_vertex1.Set(-x, x);
    line->m_vertex2.Set(x, -x);
    sensor_model->addShapes({line});

    line = new b2EdgeShape;
    line->m_vertex1.Set(0, 1);
    line->m_vertex2.Set(0, -1);
    sensor_model->addShapes({line});
}
