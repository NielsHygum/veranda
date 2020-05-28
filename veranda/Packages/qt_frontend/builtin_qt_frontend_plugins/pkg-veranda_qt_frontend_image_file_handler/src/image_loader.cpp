//! \file

#include "image_loader.h"
#include "imageparser.h"
#include "optiondialog.h"

#include <QDir>
#include <QDebug>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonDocument>
#include <QVariantMap>

#include <stdexcept>
#include <limits>
#include <ament_index_cpp/get_package_share_directory.hpp>

/*!
 * \brief Converts a QPolygonF to a QVariantList
 * Each element of the list is another QVariantList with 2 elements, x and y
 * \param[in] poly The sequence of points to convert
 * \return The polygon as a QVariantList
 */
QVariant toVariantList(const QPolygonF& poly)
{
    QVariantList list;
    for(const QPointF& p : poly)
        list.append(p);

    return list;
}

/*!
 * After the sequence of Shape types is acquired, each one is normalized before
 * being loaded into a WorldObject. This is done by finding the center of the bounding
 * box of the polygon and subtracting it from all the points in the shape. After the
 * points are all offset this way, the final WorldObject can be translated to that
 * center point to retain the shape's location.
 */
QVector<WorldObject *> ImageLoader::loadFile(QString filePath, QMap<QString, WorldObjectComponent_Factory_If *> plugins)
{
    if(filePath.contains(".json", Qt::CaseInsensitive))
    {
        ImageParameters imageParameters = getImageParametersFromJsonFile(filePath);

        if(imageParameters.parameters_ok)
        {
            return loadWorldUsingJsonFile(filePath, plugins);
        }
    }

    return loadWorldFromImageFile(filePath, plugins);
}

QVector<WorldObject*> ImageLoader::loadWorldFromImageFile(QString filePath, QMap<QString, WorldObjectComponent_Factory_If *> plugins)
{
    ImageParameters image_parameters;

    image_parameters.colorThreshold = lastOptions->getBlackWhiteThreshold();
    image_parameters.crossThreshold = lastOptions->getCrossProductThreshold();
    image_parameters.scaleY = lastOptions->getPxPerHeight();
    image_parameters.scaleX = lastOptions->getPxPerWidth();
    image_parameters.drawColor = lastOptions->getDrawColor();
    image_parameters.imageFilePath = filePath;
    image_parameters.parameters_ok = true;

    return loadWorldFromImageFile( image_parameters, plugins);

    /*
    auto iter = plugins.find("org.roboscience.veranda.worldObjectComponent.defaults.polygon");
    if(iter == plugins.end()) throw std::exception();
    try
    {
        uint64_t colorThreshold = lastOptions->getBlackWhiteThreshold();
        double crossThreshold = lastOptions->getCrossProductThreshold();
        double scaleY = lastOptions->getPxPerHeight();
        double scaleX = lastOptions->getPxPerWidth();
        QColor drawColor = lastOptions->getDrawColor();

        qDebug() << "Loading...";
        QVector<ImageParser::Shape> shapes = getShapesFromFile(filePath, colorThreshold);
        QVector<WorldObject*> objects;

        uint64_t objNum = 0;
        for(ImageParser::Shape& sh : shapes)
        {
            if(!sh.outer.size()) continue;

            //qDebug() << "Normalize and build World Objects...";
            QPointF max = sh.outer[0], min = max;

            //Normalize each object so it's
            //parts have a reasonable local origin
            for(const QPointF& p : sh.outer)
            {
                min.setX(std::min(min.x(), p.x()));
                max.setX(std::max(max.x(), p.x()));

                min.setY(std::min(min.y(), p.y()));
                max.setY(std::max(max.y(), p.y()));
            }

            for(const QPolygonF& poly : sh.inner)
            {
                for(const QPointF& p : poly)
                {
                    min.setX(std::min(min.x(), p.x()));
                    max.setX(std::max(max.x(), p.x()));

                    min.setY(std::min(min.y(), p.y()));
                    max.setY(std::max(max.y(), p.y()));
                }
            }

            QPointF avg = (min + max) / 2.0;
            for(QPolygonF& poly : sh.inner)
                for(QPointF& p : poly)
                    p = p - avg;

            for(QPointF& p : sh.outer)
                p = p - avg;

            QVariant outer = toVariantList(sh.outer);
            QVariantList inner;
            for(const QPolygonF& poly : sh.inner)
                if(poly.size()) inner.append(toVariantList(poly));

            WorldObjectComponent* comp = iter.value()->createComponent();
            auto props = comp->getProperties();

            props["straightness"]->set(QVariant::fromValue(crossThreshold), true);
            props["scale/horiz"]->set(QVariant::fromValue(scaleX), true);
            props["scale/vert"]->set(QVariant::fromValue(scaleY), true);
            props["outer_shape"]->set(outer, true);
            props["inner_shapes"]->set(inner, true);
            props["color/red"]->set(drawColor.red(), true);
            props["color/green"]->set(drawColor.green(), true);
            props["color/blue"]->set(drawColor.blue(), true);

            WorldObject* obj(new WorldObject({comp}, "Image Chunk #" + QString::number(objNum++)));
            obj->translate(avg.x()/(scaleX/2), avg.y()/(scaleY/2));
            objects.push_back(obj);
        }
        return objects;
    }catch(std::exception ex){
        qDebug() << "Unable to load image file: " << QDir(filePath).absolutePath();
    }

    return {};*/
}

QVector<WorldObject*> ImageLoader::loadWorldFromImageFile(ImageParameters image_parameters, QMap<QString, WorldObjectComponent_Factory_If *> plugins)
{
    QString filePath = image_parameters.imageFilePath;
    uint64_t colorThreshold = image_parameters.colorThreshold;
    double crossThreshold = image_parameters.crossThreshold;
    double scaleY = image_parameters.scaleY;
    double scaleX = image_parameters.scaleX;
    QColor drawColor = image_parameters.drawColor;

    if(not image_parameters.parameters_ok)
    {
        qDebug() << "Image parameters not ok!";
        return {};
    }


    auto iter = plugins.find("org.roboscience.veranda.worldObjectComponent.defaults.polygon");
    if(iter == plugins.end()) throw std::exception();

    try {
        qDebug() << "Loading...";
        QVector <ImageParser::Shape> shapes = getShapesFromFile(filePath, colorThreshold);
        QVector < WorldObject * > objects;

        uint64_t objNum = 0;
        for (ImageParser::Shape &sh : shapes) {
            if (!sh.outer.size()) continue;

            //qDebug() << "Normalize and build World Objects...";
            QPointF max = sh.outer[0], min = max;

            //Normalize each object so it's
            //parts have a reasonable local origin
            for (const QPointF &p : sh.outer) {
                min.setX(std::min(min.x(), p.x()));
                max.setX(std::max(max.x(), p.x()));

                min.setY(std::min(min.y(), p.y()));
                max.setY(std::max(max.y(), p.y()));
            }

            for (const QPolygonF &poly : sh.inner) {
                for (const QPointF &p : poly) {
                    min.setX(std::min(min.x(), p.x()));
                    max.setX(std::max(max.x(), p.x()));

                    min.setY(std::min(min.y(), p.y()));
                    max.setY(std::max(max.y(), p.y()));
                }
            }

            QPointF avg = (min + max) / 2.0;
            for (QPolygonF &poly : sh.inner)
                for (QPointF &p : poly)
                    p = p - avg;

            for (QPointF &p : sh.outer)
                p = p - avg;

            QVariant outer = toVariantList(sh.outer);
            QVariantList inner;
            for (const QPolygonF &poly : sh.inner)
                if (poly.size()) inner.append(toVariantList(poly));

            WorldObjectComponent *comp = iter.value()->createComponent();
            auto props = comp->getProperties();

            props["straightness"]->set(QVariant::fromValue(crossThreshold), true);
            props["scale/horiz"]->set(QVariant::fromValue(scaleX), true);
            props["scale/vert"]->set(QVariant::fromValue(scaleY), true);
            props["outer_shape"]->set(outer, true);
            props["inner_shapes"]->set(inner, true);
            props["color/red"]->set(drawColor.red(), true);
            props["color/green"]->set(drawColor.green(), true);
            props["color/blue"]->set(drawColor.blue(), true);

            WorldObject *obj(new WorldObject({comp}, "Image Chunk #" + QString::number(objNum++)));
            obj->translate(avg.x() / (scaleX / 2), avg.y() / (scaleY / 2));
            objects.push_back(obj);
        }
        return objects;
    } catch (std::exception ex) {
        qDebug() << "Unable to load image file: " << QDir(filePath).absolutePath();
    }

    return {};
}

ImageParameters ImageLoader::getImageParametersFromJsonFile(QString filePath)
{
    std::cerr << "loading world from image with json file: " << filePath.toStdString() << std::endl;

    ImageParameters image_parameters;
    image_parameters.parameters_ok = false;

    QFile loadFile(filePath);
    loadFile.open(QIODevice::ReadOnly);

    QByteArray saveData = loadFile.readAll();

    loadFile.close();

    QJsonDocument loadDoc(QJsonDocument::fromJson(saveData));

    QJsonArray jsonArray = loadDoc.array();

    QJsonObject root_obj = loadDoc.object();

    bool image_path_ok = false;
    bool image_resolution_x_ok = false;
    bool image_resolution_y_ok = false;
    bool intensity_threshold_ok = false;
    bool straightness_ok = false;
    double meter_per_px_x;
    double meter_per_px_y;
    int intensity_threshold;
    double straightness_threshold;
    QString pathToImageFile;

    if(root_obj.contains("components"))
    {
        if(root_obj["components"].isArray())
        {
            QJsonArray jsonArray = root_obj["components"].toArray();

            for(QJsonValue component_value : jsonArray)
            {
                if(component_value.isObject())
                {
                    QJsonObject plugin_object = component_value.toObject();

                    if(plugin_object.contains("pluginName") and plugin_object.contains("properties"))
                    {
                        QJsonValue image_loader_value_name = plugin_object["pluginName"]; //

                        if(image_loader_value_name.toString() == "org.roboscience.veranda.fileHandlers.imageLoader")
                        {
                            QJsonValue image_loader_value_properties = plugin_object["properties"];

                            if(image_loader_value_properties.isArray())
                            {
                                QJsonArray properties_array = image_loader_value_properties.toArray();

                                for(QJsonValue property_value : properties_array)
                                {

                                   // qDebug() << property_value;

                                    if(property_value.isObject())
                                    {
                                        QJsonObject property_object = property_value.toObject();

                                        if(property_object.contains("key"))
                                        {
                                            if(property_object["key"].toString() == "Image_path")
                                            {
                                                image_path_ok = true;
                                                pathToImageFile = property_object["value"].toString();
                                                qDebug() << "image path: " << pathToImageFile;
                                            } else if(property_object["key"].toString() == "meter_per_px_x") {
                                                image_resolution_x_ok = true;
                                                meter_per_px_x = property_object["value"].toDouble();
                                                qDebug() << "meter per px x: " << meter_per_px_x;
                                            } else if(property_object["key"].toString() == "meter_per_px_y") {
                                                image_resolution_y_ok = true;
                                                meter_per_px_y = property_object["value"].toDouble();
                                                qDebug() << "meter per px y: " << meter_per_px_y;
                                            } else if(property_object["key"].toString() == "intensity_threshold") {
                                                intensity_threshold_ok = true;
                                                intensity_threshold = property_object["value"].toInt();
                                                qDebug() << "intensity threshold: " << intensity_threshold;
                                            } else if(property_object["key"].toString() == "straightness_threshold") {
                                                straightness_ok = true;
                                                straightness_threshold = property_object["value"].toDouble();
                                                qDebug() << "straightness threshold: " << straightness_threshold;
                                            }
                                        }

                                    }
                                }
                            }

                        }
                    }
                }
            }

        } else  {
            qDebug() << "component has not value that is json array";
        }

    }

    if(image_path_ok and image_resolution_x_ok and image_resolution_y_ok and intensity_threshold_ok and straightness_ok)
    {
        QColor drawColor(0,0,0);
        image_parameters.colorThreshold = intensity_threshold;
        image_parameters.crossThreshold = straightness_threshold;
        image_parameters.scaleY = 1.0/meter_per_px_y;
        image_parameters.scaleX = 1.0/meter_per_px_x;
        image_parameters.drawColor = drawColor;

        if(not pathToImageFile.startsWith("/"))
        {
            //use relative path from veranda package path
            QString veranda_path(ament_index_cpp::get_package_share_directory("veranda_qt_frontend").c_str());
            image_parameters.imageFilePath = veranda_path + "/" + pathToImageFile;
        } else {
            image_parameters.imageFilePath = pathToImageFile;
        }




        image_parameters.parameters_ok = true;
    }

    return image_parameters;
}

QVector<WorldObject*> ImageLoader::loadWorldUsingJsonFile(QString filePath, QMap<QString, WorldObjectComponent_Factory_If *> plugins)
{
    ImageParameters image_parameters = getImageParametersFromJsonFile(filePath);

    if(canLoadFile(image_parameters.imageFilePath, plugins))
    {
        return loadWorldFromImageFile(image_parameters, plugins);
    }

    return {};
}

/*!
 * There are two requirements to be able to load images
 * * The file must be an image
 * * The polygon plugin included with this project must be available
 */
bool ImageLoader::canLoadFile(QString filePath, QMap<QString, WorldObjectComponent_Factory_If *> plugins)
{
    std::cerr << "checking if image loader can load file" << std::endl;

    if(filePath.contains(".json", Qt::CaseInsensitive))
    {
        //todo: check that is valid json file
    } else {

        QImage img(filePath);
        if(img.isNull())
        {
            qDebug() << "image not found";
            return false;
        }
    }


    if(!plugins.contains("org.roboscience.veranda.worldObjectComponent.defaults.polygon"))
    {
        qDebug() << "Cannot load image files without polygons plugin";
        return false;
    }

    return true;
}

void ImageLoader::getUserOptions(QString filePath, QMap<QString, WorldObjectComponent_Factory_If *> plugins)
{
    QImage img(filePath);
    if(img.isNull()) throw std::exception();
    uint64_t fileWidth = img.width(), fileHeight = img.height();

    lastOptions.reset(new ImageOptions(fileWidth, fileHeight));
    lastOptions->exec();
}

/*!
 * Each shape consists of an outer loop and 0 or more inner loops
 * representing holes. This information can be copied almost directly
 * to the polygon shape plugin.
 */
QVector<ImageParser::Shape> ImageLoader::getShapesFromFile(QString filePath, uint64_t colorThreshold)
{
    QImage img(filePath);
    if(img.isNull()) throw std::exception();

    qDebug() << "Parse image...";
    QVector<ImageParser::Shape> shapes = ImageParser::parseImage(img, colorThreshold);

    return shapes;
}
