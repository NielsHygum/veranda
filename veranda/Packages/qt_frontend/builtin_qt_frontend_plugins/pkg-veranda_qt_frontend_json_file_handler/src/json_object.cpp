#include "json_object.h"
#include <QDebug>

WorldObject* JsonObjectLoader::loadFile(QString filePath, QMap<QString, WorldObjectComponent_Factory_If *> plugins)
{
    QFile loadFile(filePath);
    loadFile.open(QIODevice::ReadOnly);

    QByteArray saveData = loadFile.readAll();

    QJsonDocument loadDoc(QJsonDocument::fromJson(saveData));

    return jsonObjectToWorldObject(loadDoc.object(), plugins);
}

void JsonObjectSaver::saveFile(QString filePath, WorldObject* object)
{
    QFile saveFile(filePath);
    saveFile.open(QIODevice::WriteOnly | QIODevice::Truncate);

    QJsonDocument saveDoc(worldObjectToJsonObject(object));
    saveFile.write(saveDoc.toJson());
}
