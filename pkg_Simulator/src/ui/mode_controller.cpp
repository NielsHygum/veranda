#include "include/ui/mode_controller.h"

Mode_Controller::Mode_Controller(visualizerFactory factory, QListWidget* pActive, QTableView* pProperties, QTabWidget* pTabs, QWidget *parent)
{
    makeWidget = factory;
    visual = factory();
    active = pActive;
    properties = pProperties;
    tabs = pTabs;
    tabs->clear();
    setWorldBounds(-100, 100, -100, 100);

    connect(active, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(robotItemClicked(QListWidgetItem*)));
    connect(this, SIGNAL(objectIsSelected(object_id)), visual, SLOT(objectSelected(object_id)));
    connect(this, SIGNAL(nothingIsSelected()), visual, SLOT(nothingSelected()));

    connect(visual, SIGNAL(userDragMoveObject(object_id,double,double)), this, SLOT(simObjectMoveDragged(object_id,double,double)));
    connect(visual, SIGNAL(userDragRotateObject(object_id,double)), this, SLOT(simObjectRotateDragged(object_id,double)));

    connect(visual, SIGNAL(userSelectedObject(object_id)), this, SLOT(objectSelected(object_id)));

    setPropertiesTableView();
}

void Mode_Controller::setPropertiesTableView()
{
    propertiesModel = new QStandardItemModel(0,2,this); //12 Rows and 2 Columns
    propertiesModel->setHorizontalHeaderItem(0, new QStandardItem(QString("Property")));
    propertiesModel->setHorizontalHeaderItem(1, new QStandardItem(QString("Value")));
    properties->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    properties->setModel(propertiesModel);

    nothingSelected();
}

//make this mode visible
void Mode_Controller::open()
{
    visual->setVisible(true);
    tabs->setVisible(true);
    active->setVisible(true);
    properties->setVisible(true);

    nothingSelected();
}

//make this mode visible
void Mode_Controller::close()
{
    visual->setVisible(false);
    tabs->setVisible(false);
    active->setVisible(false);
    properties->setVisible(false);

    nothingSelected();
}

//Add robot to the simulation world view
void Mode_Controller::worldObjectsAddedToSimulation(QVector<QPair<WorldObjectProperties *, object_id>> objs)
{
    for(auto& p : objs)
    {
        object_id& oId = p.second;
        WorldObjectProperties* object = p.first;

        if(worldObjects.contains(oId)) throw std::logic_error("world object " + std::to_string(oId) + " already exists in ui");

        worldObjects[oId] = object;

        visual->objectAddedToScreen(object->getModels(), oId);

        listItems[oId] = new QListWidgetItem();
        listItems[oId]->setData(Qt::DisplayRole, QString::number(oId));
        active->addItem(listItems[oId]);
    }
    if(objs.size())
        objectSelected(objs.last().second);
}

void Mode_Controller::worldObjectsRemovedFromSimulation(QVector<object_id> oIds)
{
    for(object_id oId : oIds)
    {
        visual->objectRemovedFromScreen(oId);
        worldObjects.remove(oId);

        active->removeItemWidget(listItems[oId]);
        delete listItems[oId];
        listItems.remove(oId);

        if(selected == oId)
            nothingSelected();
    }
}

void Mode_Controller::setWorldBounds(double xMin, double xMax, double yMin, double yMax)
{
    if(xMin > xMax) std::swap(xMin, xMax);
    if(yMin > yMax) std::swap(yMin, yMax);

    visual->setWorldBounds(xMin, xMax, yMin, yMax);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Interaction slots for button clicks in UI                                                                                 //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Mode_Controller::cloneSelectedTool()
{
    qDebug() << "cloning ";

    QList<QListWidgetItem*> l = toolTabs[tabs->currentWidget()->accessibleName()]->selectedItems();
    if ( ! l.empty()) {
            selectedTool = new WorldObjectProperties(dynamic_cast<Designer_Widget*>(l.at(0))->component->clone(), this);
            return true;
    }
    return false;
}

int Mode_Controller::getNextId()
{
    while(worldObjects[idIncrementer] != nullptr)
        idIncrementer++;
    return idIncrementer;
}

void Mode_Controller::addObjectToView()
{
    if(cloneSelectedTool())
    {
        WorldObjectProperties* object = selectedTool;

        qDebug() << "here";
        //ad object to active list
        object_id oId = getNextId();
        worldObjects[oId] = object;
        listItems[oId] = new QListWidgetItem();
        listItems[oId]->setData(Qt::DisplayRole, QString::number(oId));
        active->addItem(listItems[oId]);

        //add object to view
        visual->objectAddedToScreen(object->getModels(), oId);
    }
}

void Mode_Controller::deleteObjectFromView()
{
    object_id oId = selected;
    visual->objectRemovedFromScreen(oId);
    worldObjects.remove(oId);

    active->removeItemWidget(listItems[oId]);
    delete listItems[oId];
    listItems.remove(oId);

    nothingSelected();
}

void Mode_Controller::addObjectToTools(WorldObjectComponent* component)
{
    WorldObjectProperties* properties = new WorldObjectProperties(component, this);

    //if tab does not exist, create it then add new designer widget
    if(toolTabs[properties->getType()] == nullptr)
    {
        toolTabs[properties->getType()] = new QListWidget();
        tabs->addTab(toolTabs[properties->getType()], properties->getType());
        toolTabs[properties->getType()]->setViewMode(QListWidget::IconMode);
        toolTabs[properties->getType()]->setIconSize(QSize(150,150));
        toolTabs[properties->getType()]->setResizeMode(QListWidget::Adjust);
    }

    //add new designer widget to a tab
    Designer_Widget* tile = new Designer_Widget(component, properties, makeWidget, toolTabs[properties->getType()]);
    toolTabs[properties->getType()]->addItem(tile);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//World view signals and slots                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Mode_Controller::objectSelected(object_id id)
{
    if(worldObjects.contains(id))
    {
        nothingSelected();

        selected = id;
        WorldObjectProperties* obj = worldObjects[id];
        QStandardItemModel* model = propertiesModel;

        selectedProps = obj->getProperties();
        QStringList propKeys = selectedProps.keys();
        model->setRowCount(selectedProps.size());

        int i = 0;
        for(QString k : propKeys)
        {
           QModelIndex ind;

           //Set key
           ind = model->index(i, 0);
           model->setData(ind, k);

           connect(selectedProps[k].data(), &PropertyView::valueSet, this, &Mode_Controller::updatePropertyInformation);

           displayed_properties[i] = k;
           i++;
        }

        connect(model, &QStandardItemModel::dataChanged, [this, obj, model](QModelIndex tl, QModelIndex br)
        {
           for(int i = tl.row(); i <= br.row(); i++)
               selectedProps[displayed_properties[i]]->set(model->data(model->index(i, 1)));
        });

        updatePropertyInformation();

        active->setCurrentItem(listItems[id]);

        objectIsSelected(id);
    }
}

void Mode_Controller::nothingSelected()
{
    propertiesModel->setRowCount(0);
    disconnect(propertiesModel, &QStandardItemModel::dataChanged, 0, 0);

    if(worldObjects.contains(selected))
    {
        for(auto iter : selectedProps)
            disconnect(iter.data(), 0, this, 0);
    }
    selectedProps.clear();

    nothingIsSelected();
}

void Mode_Controller::robotItemClicked(QListWidgetItem* item)
{
    objectSelected(item->data(Qt::DisplayRole).toInt());
}

void Mode_Controller::updatePropertyInformation()
{
    if(worldObjects.contains(selected))
    {
        QStandardItemModel* model = propertiesModel;

        properties->setUpdatesEnabled(false);
        for(int i=0; i<selectedProps.size(); i++)
        {
            QString key = model->data(model->index(i, 0)).toString();
            model->setData(model->index(i, 1), selectedProps[key]->get().toString(), Qt::DisplayRole);
        }
        properties->setUpdatesEnabled(true);
    }
}

void Mode_Controller::simObjectMoveDragged(object_id id, double dx, double dy)
{
    auto obj = worldObjects.find(id);
    if(obj != worldObjects.end())
    {
        if(simulator)
            obj.value()->translate(dx, dy);
        else
            obj.value()->translateView(dx, dy, 0);
    }
}

void Mode_Controller::simObjectRotateDragged(object_id id, double dt)
{
    qDebug() << "Yo yo " << dt;
    auto obj = worldObjects.find(id);
    if(obj != worldObjects.end())
    {
        if(simulator)
            obj.value()->rotate(dt);
        else
            obj.value()->translateView(0, 0, dt);
    }
}
