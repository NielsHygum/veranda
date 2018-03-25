#ifndef POLYGON_PLUGIN_H
#define POLYGON_PLUGIN_H

#include <QObject>

//This should not be fully defined relative; it should be just
//  <sdsmt_simulator/world_object_component_plugin.h>
//however, a bug? in MSVC prevents the Qt MOC from resolving interfaces
//if the path isn't relative like this and it doesn't seem to work to
//do conditional compilation so this would be used only on windows
#include "../../../install/include/sdsmt_simulator/world_object_component_plugin.h"

class Polygon_Plugin : public QObject, public WorldObjectComponent_Plugin_If
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "org.sdsmt.sim.2d.worldObjectComponent.defaults.polygon")
    Q_INTERFACES(WorldObjectComponent_Plugin_If)

public:
    Polygon_Plugin();
    WorldObjectComponent *createComponent();

};

#endif // POLYGON_PLUGIN_H