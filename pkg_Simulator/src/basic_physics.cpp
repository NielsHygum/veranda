#include "basic_physics.h"
#include <QDebug>

#include <string>
#include <stdexcept>

#include <QElapsedTimer>

BasicPhysics::BasicPhysics(QObject *parent) : Simulator_Physics_If(parent)
{
    //No gravity for top down simulation
    b2Vec2 gravity(0.0f, 0.0f);
    //Create world
    world = new b2World(gravity);

    tickRate = 1;
    stepTime = 1;

    tick = new QTimer(this);
    connect(tick, &QTimer::timeout, this, &BasicPhysics::step);
}

void BasicPhysics::start()
{
    tick->setInterval(1000.0f / tickRate);
    tick->setTimerType(Qt::PreciseTimer);

    tick->start();

    emit physicsStarted();
}

void BasicPhysics::stop()
{
    tick->stop();

    emit physicsStopped();
}

void BasicPhysics::clear()
{
    delete world;
    b2Vec2 gravity(0.0f, 0.0f);
    world = new b2World(gravity);
}

void BasicPhysics::setTick(double rate_hz, double duration_s)
{
    tickRate = rate_hz;
    stepTime = duration_s;

    //TODO :: Update QTimer with new rate settings

    emit physicsTickSet(tickRate, stepTime);
}

void BasicPhysics::addWorldObjects(QVector<QPair<WorldObjectPhysics*, object_id>> objs)
{
    for(auto& p : objs)
    {
        object_id& oId = p.second;
        WorldObjectPhysics* obj = p.first;

        if(objects.contains(oId)) throw std::logic_error("object with id " + std::to_string(oId) + " already exists");

        objectWorldData& worldDat = objects[oId];
        worldDat.obj = obj;
        connect(this, &BasicPhysics::worldTick, obj, &WorldObjectPhysics::worldTicked);

        obj->generateBodies(world, oId);

        obj->worldTicked(world, 0);
    }
}

void BasicPhysics::removeWorldObjects(QVector<object_id> oIds)
{
    for(object_id oId : oIds)
    {
        if(objects.contains(oId))
        {
            objectWorldData& dat = objects[oId];

            for(b2Joint* j : dat.joints)
                world->DestroyJoint(j);

            for(b2Body* s : dat.staticBodies)
                world->DestroyBody(s);

            for(b2Body* d : dat.dynamicBodies)
                world->DestroyBody(d);

           objects.remove(oId);
        }
    }
}

void BasicPhysics::step()
{
    world->Step(stepTime, 8, 3); //suggested values for velocity and position iterations

    emit worldTick(world, stepTime);
}
