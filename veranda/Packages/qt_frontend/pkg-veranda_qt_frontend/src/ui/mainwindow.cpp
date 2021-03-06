#include "ui/mainwindow.h"
#include "ui_mainwindow.h"

#include <QMessageBox>
#include <QFileDialog>
#include <QDebug>
#include <QDate>
#include <QStandardItemModel>
#include <QThread>
#include <QListWidgetItem>
#include <QProgressBar>
#include <QtConcurrent/QtConcurrent>
#include <QWindow>
#include <QInputDialog>

#include <stdexcept>
#include <string>
#include <limits>

MainWindow::MainWindow(visualizerFactory factory, QMap<QString, WorldObjectComponent_Factory_If *> components,
                       QVector<WorldObjectLoader_If*> oloaders, QVector<WorldObjectSaver_If*> osavers,
                       QVector<WorldLoader_If *> wloaders, QVector<WorldSaver_If *> wsavers, WorldLoader_If* defaultLoader_, QWidget *parent) :
    Simulator_Ui_If(parent),
    makeWidget(factory), componentFactories(components),
    ui(new Ui::MainWindow)
{
    qRegisterMetaType<QVector<object_id>>("QVector<object_id>");
    qRegisterMetaType<QVector<WorldObject*>>("QVector<WorldObject*>");

    defaultLoader = defaultLoader_;

    ui->setupUi(this);

    ui->centralWidget->setLayout(ui->mainLayout);

    for(WorldObjectLoader_If* l : oloaders)
        for(QString e : l->fileExts())
            objectLoaders[e] += l;

    for(WorldObjectSaver_If* s : osavers)
        for(QString e : s->fileExts())
            objectSavers[e] += s;

    for(WorldLoader_If* l : wloaders)
        for(QString e : l->fileExts())
            worldLoaders[e] += l;

    for(WorldSaver_If* s : wsavers)
        for(QString e : s->fileExts())
            worldSavers[e] += s;

    //If no world loader plugins exist,
    //don't present button to user
    if(wloaders.size() == 0)
        ui->importMapButton->setVisible(false);

    //Initiate a world view for Simulator and another for Designer
    simulator = new Mode_Controller(factory, ui->simulatorButton, ui->simulatorMenuWidget, ui->simulatorToolsMenu, ui->simulatorActiveWidget,
                                    ui->propertiesTableView, ui->simulatorToolsList, this);
    designer = new Mode_Controller(factory, ui->designerButton, ui->designerMenuWidget, ui->designerToolsMenu, ui->designerActiveWidget,
                                   ui->designerPropertiesTableView, ui->designerToolsList, this);

    //Initialize designer/simulator start states
    simulator->simulator = true;
    ui->worldViewLayout->addWidget(simulator->visual);
    ui->worldViewLayout->addWidget(designer->visual);
    simulator->open();
    designer->close();
    ui->addToolButton->setEnabled(false);
    ui->loadToolsButton->setVisible(false);

    //Main menu button signals and slots
    connect(ui->showBuildObjectsButton, SIGNAL (released()), this, SLOT (showBuildObjectsButtonClick()));
    connect(ui->showMenuButton, SIGNAL (released()), this, SLOT (showMenuButtonClick()));
    connect(ui->simulatorButton, SIGNAL (released()), this, SLOT (simulatorButtonClick()));
    connect(ui->designerButton, SIGNAL (released()), this, SLOT (designerButtonClick()));

    //Simulation mode button signals and slots
    connect(ui->playSimButton, SIGNAL (released()), this, SLOT (playSimButtonClick()));
    connect(ui->speedSimButton, SIGNAL (released()), this, SLOT (speedSimButtonClick()));
    connect(ui->importMapButton, SIGNAL (released()), this, SLOT (loadSimButtonClick()));
    connect(ui->screenshotSimButton, SIGNAL (released()), this, SLOT (screenshotSimButtonClick()));
    connect(ui->joystickButton, SIGNAL (released()), this, SLOT (joystickButtonClick()));
    connect(ui->saveSimButton, SIGNAL (released()), this, SLOT (saveSimButtonClick()));
    connect(ui->quickSaveButton, SIGNAL (released()), this, SLOT (quickSaveButtonClick()));
    connect(ui->quickLoadButton, SIGNAL (released()), this, SLOT (quickLoadButtonClick()));
    connect(ui->newSimButton, SIGNAL (released()), this, SLOT (newSimButtonClick()));

    //Designer mode button signals and slots
    connect(ui->newObjectButton, SIGNAL (released()), this, SLOT (newObjectButtonClick()));
    connect(ui->loadObjectButton, SIGNAL (released()), this, SLOT (loadObjectButtonClick()));
    connect(ui->saveObjectButton, SIGNAL (released()), this, SLOT (saveObjectButtonClick()));

    //Simulation mode tool button signals and slots
    connect(simulator, SIGNAL (requestAddWorldObjects(QVector<WorldObject*>,bool)), this, SIGNAL (userAddWorldObjectsToSimulation(QVector<WorldObject*>, bool)));
    connect(simulator, SIGNAL (requestRemoveWorldObjects(QVector<object_id>)), this, SIGNAL (userRemoveWorldObjectsFromSimulation(QVector<object_id>)));
    connect(ui->addObjectButton, SIGNAL (released()), simulator, SLOT (addObjectToView()));
    connect(ui->deleteObjectButton, SIGNAL (released()), simulator, SLOT (deleteObjectFromView()));
    connect(ui->loadObjectsButton, SIGNAL (released()), this, SLOT (loadObjectsForSimButtonClick()));

    //Designer mode tool button signals and slots
    connect(ui->addToolButton, SIGNAL (released()), designer, SLOT (addObjectToView()));
    connect(ui->deleteToolButton, SIGNAL (released()), designer, SLOT (deleteObjectFromView()));
    connect(ui->exportObjectButton, SIGNAL (released()), this, SLOT (exportObjectButtonClick()));
    //connect(ui->loadToolsButton, SIGNAL (released()), this, SLOT (loadToolsButtonClick()));

    //load all designer tools available
    loadToolsButtonClick();

    connect(this, SIGNAL (objectsAddedToSimulation(QVector<QPair<WorldObjectProperties*, object_id>>)), simulator, SLOT (worldObjectsAddedToSimulation(QVector<QPair<WorldObjectProperties*, object_id>>)));
    connect(this, SIGNAL (objectsRemovedFromSimulation(QVector<object_id>)), simulator, SLOT (worldObjectsRemovedFromSimulation(QVector<object_id>)));

    connect(this, SIGNAL(error(QString)), this, SLOT(errorMessage(QString)));
    //connect(this, SIGNAL (addObjectToSimulation(QVector<QSharedPointer<WorldObject>>)), simulator, SLOT (getItemAsVector(QVector<QSharedPointer<WorldObject>>));

}

MainWindow::~MainWindow()
{
    delete ui;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Public signals and slots                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::physicsStarted()
{
    play = true;
    ui->playSimButton->setToolTip("Stop Simulation");
    ui->playSimButton->setIcon(QIcon(":/sim/Stop"));
    ui->playSimButton->setIconSize(QSize(32,32));

    //disable options while simulation is running
    ui->saveSimButton->setEnabled(false);
    ui->designerButton->setEnabled(false);
    ui->simulatorToolsList->setEnabled(false);
    ui->simulatorToolsMenu->setEnabled(false);
    ui->secondSimulatorMenuWidget->setEnabled(false);

    simulator->visual->setToolsEnabled(false);
}

void MainWindow::physicsStopped()
{
    play = false;
    ui->playSimButton->setToolTip("Play Simulation");
    ui->playSimButton->setIcon(QIcon(":/sim/Play"));
    ui->playSimButton->setIconSize(QSize(32,32));

    //enable options while simulation is running
    ui->saveSimButton->setEnabled(true);
    ui->designerButton->setEnabled(true);
    ui->simulatorToolsList->setEnabled(true);
    ui->simulatorToolsMenu->setEnabled(true);
    ui->secondSimulatorMenuWidget->setEnabled(true);

    simulator->visual->setToolsEnabled(true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main menu button signals and slots                                                                                        //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MainWindow::showBuildObjectsButtonClick()
{
    if (ui->buildToolsWidget->isHidden())
        ui->buildToolsWidget->setVisible(true);
    else
        ui->buildToolsWidget->setVisible((false));
}

void MainWindow::showMenuButtonClick()
{
    if (ui->menuWidget->isHidden())
        ui->menuWidget->setVisible(true);
    else
        ui->menuWidget->setVisible((false));
}

void MainWindow::simulatorButtonClick()
{
    designer->close();
    simulator->open();

    ui->modeLabel->setText("Simulator");
    ui->buildToolsLabel->setText("Simulator Tools");
}

void MainWindow::designerButtonClick()
{
    designer->open();
    simulator->close();

    ui->modeLabel->setText("Designer");
    ui->buildToolsLabel->setText("Designer Tools");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Simulation mode button signals and slots                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MainWindow::playSimButtonClick()
{
    //Reset Simulation Now
    if (play)
    {
        emit userStopPhysics();
    }
    //Play Simulation Now
    else
    {
        emit userStartPhysics();
    }
}

void MainWindow::speedSimButtonClick()
{
    uint64_t next = (speed + 1) % SPEEDBUTTONS.size();
    userSetPhysicsTickMultiplier(SPEEDBUTTONS[next].first);
}

void MainWindow::physicsTickMultiplierChanged(double mult)
{
    double closest = std::numeric_limits<double>::max();
    for(int i=0; i<SPEEDBUTTONS.size(); i++)
    {
        double diff = std::abs(SPEEDBUTTONS[i].first - mult);
        if(diff < closest)
        {
            closest = diff;
            speed = i;
        }
    }

    ui->speedSimButton->setToolTip(SPEEDBUTTONS[speed].second.first);
    ui->speedSimButton->setIcon(QIcon(SPEEDBUTTONS[speed].second.second));
    ui->speedSimButton->setIconSize(QSize(32,32));
}

void MainWindow::screenshotSimButtonClick()
{
    QPixmap pixmap(simulator->visual->size());
    simulator->visual->render(&pixmap);
    qint64 current = QDateTime::currentMSecsSinceEpoch();
    QString fileName = QString::number(current) + ".png";

    QFile file(fileName);
    file.open(QIODevice::WriteOnly);
    pixmap.save(&file, "PNG");
}

void MainWindow::loadSimButtonClick()
{
    QMessageBox msgBox;
    msgBox.setText("WARNING: All objects will be removed from the simulation.");
    msgBox.setInformativeText("Continue?");
    msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
    msgBox.setDefaultButton(QMessageBox::No);
    int ret = msgBox.exec();

    switch (ret) {
      case QMessageBox::Yes:
    {
          QString types;
          qDebug() << "Able to load files:" << worldLoaders.keys();
          for(QString k : worldLoaders.keys())
              if(k.size())
                types += k + ";;";

          types = types.left(types.size()-2);

          // Save was clicked
          QString path = QFileDialog::getOpenFileName(this, tr("Open File"), "/home", types);

          if(path.length())
          {
              QString ext = QFileInfo(path).suffix().toLower();

              bool done = false;
              for(auto it = worldLoaders.begin(); it != worldLoaders.end() && !done; it++)
              {
                  if(it.key().toLower().contains(ext))
                      for(WorldLoader_If* wl : it.value())

                          //Find the first loader that can load this file
                          //and try to load. If it fails, we can't load it
                          if(!done && wl->canLoadFile(path, componentFactories))
                          {
                              //Get user options in main thread
                              wl->getUserOptions(path, componentFactories);

                              //Spin up side thread to actually load it
                              QtConcurrent::run([this, path, wl](){
                                  QVector<WorldObject*> loadedObjs;
                                  try
                                  {
                                      //Load file in separate thread
                                      qDebug() << "Load file";
                                      loadedObjs=wl->loadFile(path, componentFactories);
                                  }catch(std::exception&){}

                                  if(loadedObjs.size())
                                  {
                                      qDebug() << "Clear world";
                                      userRemoveWorldObjectsFromSimulation(simulator->worldObjects.keys().toVector());

                                      qDebug() << "Build new world";
                                      userAddWorldObjectsToSimulation(loadedObjs, false);


                                      //Add default robots
                                      if(defaultLoader && defaultLoader->canLoadFile(path, componentFactories))
                                      {
                                          loadedObjs.clear();
                                          try
                                          {
                                              loadedObjs=defaultLoader->loadFile(path, componentFactories);
                                          }catch(std::exception&){}

                                          userAddWorldObjectsToSimulation(loadedObjs, false);
                                      }
                                  }
                                  else
                                  {
                                    emit error("Unable to open \'" + path + "\' as a world file");
                                  }
                              });

                              //Stop looking for a file handler
                              //for this file
                              done = true;
                          }
              }

              if(!done) emit error("No plugin to open \'" + path + "\' as a world file");

          }

          break;
    }
        break;

      case QMessageBox::No:
          // Don't Save was clicked
          break;
      default:
          // should never be reached
          break;
    }
}

void MainWindow::joystickButtonClick()
{
    //create window and joystick, link them
    QWindow *jWindow = new QWindow();
    JoystickPrototype *joystick = new JoystickPrototype(jWindow);

    //Joystick button signals and slots
    connect(joystick, SIGNAL(joystickMoved(double, double, double, QString)), this, SIGNAL(joystickMoved(double, double, double, QString)));
    connect(joystick, SIGNAL(joystickButtonPress(int, QString)), this, SIGNAL(joystickButtonPress(int, QString)));
    connect(joystick, SIGNAL(joystickButtonRelease(int, QString)), this, SIGNAL(joystickButtonRelease(int, QString)));

    connect(this, &MainWindow::windowClosed, joystick, &JoystickPrototype::deleteLater);
    connect(joystick, &JoystickPrototype::joystickClosed, joystick, &JoystickPrototype::deleteLater);
    connect(joystick, &JoystickPrototype::destroyed, jWindow, &QWindow::deleteLater);

    //show this joystick
    joystick->show();
}

void MainWindow::saveSimButtonClick()
{
    QString types;
    qDebug() << "Able to save files:" << worldSavers.keys();
    for(QString k : worldSavers.keys())
      if(k.size())
        types += k + ";;";

    types = types.left(types.size()-2);

    // Save was clicked
    QString path = QFileDialog::getSaveFileName(this, tr("Save File"), "/home", types);

    if(path.length())
    {
        QString ext = QFileInfo(path).suffix().toLower();

        for(auto it = worldSavers.begin(); it != worldSavers.end(); it++)
        {
            if(it.key().toLower().contains(ext))
            {
                QVector<WorldObject*> worldObjs;
                for(WorldObjectProperties* prop : objects)
                {
                    WorldObject* cast = prop->getObject();
                    if(cast)
                        worldObjs.push_back(cast);
                }

                try
                {
                    it.value()[0]->saveFile(path, worldObjs);
                }catch(std::exception& ex){
                    error("Unable to save file '" + path + "': " + QString::fromStdString(ex.what()));
                }

                return;
            }
        }
        error("No plugin to save file '" + path + "'");
    }
}

void MainWindow::newSimButtonClick()
{
    //prompt for save

    QMessageBox msgBox;
    msgBox.setText("WARNING: You will be prompted to save and then all objects will be removed from the simulation.");
    msgBox.setInformativeText("Continue?");
    msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
    msgBox.setDefaultButton(QMessageBox::No);
    int ret = msgBox.exec();

    switch (ret) {
        case QMessageBox::Yes:
            saveSimButtonClick();
            simulator->clear();
            break;

        case QMessageBox::No:
            break;

        default:
            // should never be reached
            break;
    }
}

void MainWindow::quickSaveButtonClick()
{
    // Save was clicked
    QString path = "quick.json";

    QString ext = "json";

    for(auto it = worldSavers.begin(); it != worldSavers.end(); it++)
    {
        if(it.key().toLower().contains(ext))
        {
            QVector<WorldObject*> worldObjs;
            for(WorldObjectProperties* prop : objects)
            {
                WorldObject* cast = prop->getObject();
                if(cast)
                    worldObjs.push_back(cast);
            }

            try
            {
                it.value()[0]->saveFile(path, worldObjs);
            }catch(std::exception& ex){
                error("Unable to save file '" + path + "': " + QString::fromStdString(ex.what()));
            }

            return;
        }
    }
    error("No plugin to save file '" + path + "'");
}

void MainWindow::quickLoadButtonClick()
{
    QMessageBox msgBox;
    msgBox.setText("WARNING: Changing the map will delete all world objects from this simulation.");
    msgBox.setInformativeText("Would you like to proceed?");
    msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
    msgBox.setDefaultButton(QMessageBox::No);
    int ret = msgBox.exec();

    switch (ret) {
      case QMessageBox::Yes:
    {
          QString path = "quick.json";

          QString ext = "json";

          bool done = false;
          for(auto it = worldLoaders.begin(); it != worldLoaders.end() && !done; it++)
          {
              if(it.key().toLower().contains(ext))
                  for(WorldLoader_If* wl : it.value())

                      //Find the json world loader
                      if(!done && wl->canLoadFile(path, componentFactories))
                      {
                          //Get user options in main thread
                          wl->getUserOptions(path, componentFactories);

                          //Spin up side thread to actually load it
                          QtConcurrent::run([this, path, wl](){
                              QVector<WorldObject*> loadedObjs;
                              try
                              {
                                  //Load file in separate thread
                                  qDebug() << "Load file";
                                  loadedObjs=wl->loadFile(path, componentFactories);
                              }catch(std::exception&){}

                              if(loadedObjs.size())
                              {
                                  qDebug() << "Clear world";
                                  userRemoveWorldObjectsFromSimulation(simulator->worldObjects.keys().toVector());

                                  qDebug() << "Build new world";
                                  userAddWorldObjectsToSimulation(loadedObjs, false);


                                  //Add default robots
                                  if(defaultLoader && defaultLoader->canLoadFile(path, componentFactories))
                                  {
                                      loadedObjs.clear();
                                      try
                                      {
                                          loadedObjs=defaultLoader->loadFile(path, componentFactories);
                                      }catch(std::exception&){}

                                      userAddWorldObjectsToSimulation(loadedObjs, false);
                                  }
                              }
                              else
                              {
                                emit error("Unable to open \'" + path + "\' as a world file");
                              }
                          });

                          //Stop looking for a file handler
                          //for this file
                          done = true;
                      }
          }

          if(!done) emit error("Json plugin not found.");

          break;
    }
        break;

      case QMessageBox::No:
          // Don't Save was clicked
          break;
      default:
          // should never be reached
          break;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Designer mode button signals and slots                                                                                    //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::newObjectButtonClick()
{
    //prompt for save

    QMessageBox msgBox;
    msgBox.setText("WARNING: You will be prompted to save and then all components will be removed from the view.");
    msgBox.setInformativeText("Continue?");
    msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
    msgBox.setDefaultButton(QMessageBox::No);
    int ret = msgBox.exec();

    switch (ret) {
        case QMessageBox::Yes:
            saveObjectButtonClick();
            designer->clear();
            break;

        case QMessageBox::No:
            // Don't Save was clicked
            break;

        default:
            // should never be reached
            break;
    }
}

void MainWindow::loadObjectButtonClick()
{
    QString types;
    qDebug() << "Able to load files:" << objectLoaders.keys();
    for(QString k : objectLoaders.keys())
        if(k.size())
          types += k + ";;";

    types = types.left(types.size()-2);

    QString objFile = QFileDialog::getOpenFileName(nullptr, "", "", types, nullptr);
    if(objFile.isNull()) return;

    QString suff = QFileInfo(objFile).suffix().toLower();

    bool done = false;
    for(QString k : objectLoaders.keys())
    {
        if(k.toLower().contains(suff))
        {
            for(WorldObjectLoader_If* l : objectLoaders[k])
            {
                if(l->canLoadFile(objFile, componentFactories))
                {
                    l->getUserOptions(objFile, componentFactories);

                    try
                    {
                        WorldObject* wobj = l->loadFile(objFile, componentFactories);
                        designer->clear();
                        object_id i = 1;
                        for(WorldObjectComponent* c : wobj->getComponents())
                            designer->worldObjectsAddedToSimulation({{new WorldObjectProperties(c), i++}});

                        return;
                    }catch(std::exception&)
                    {
                        error("Unable to load world object file '" + objFile + "'");
                    }
                    done = true;
                }
                if(done) break;
            }
        }
        if(done) break;
    }
    if(!done) error("No plugin to load world object file '" + objFile + "'");
}

void MainWindow::saveObjectButtonClick()
{
    QVector<WorldObjectComponent*> comps = designer->getComponents();
    WorldObject wo(comps);

    QString types;
    qDebug() << "Able to save files:" << objectSavers.keys();
    for(QString k : objectSavers.keys())
        if(k.size())
          types += k + ";;";

    types = types.left(types.size()-2);

    QString objFile = QFileDialog::getSaveFileName(nullptr, "", "", types, nullptr);
    QString suff = QFileInfo(objFile).suffix().toLower();

    bool done = false;
    for(QString k : objectSavers.keys())
    {
        if(k.toLower().contains(suff))
        {
            for(WorldObjectSaver_If* s : objectSavers[k])
            {
                try
                {
                    s->saveFile(objFile, &wo);

                    return;
                }catch(std::exception&)
                {
                    error("Unable to save world object file '" + objFile + "'");
                }
                done = true;
                break;
            }
        }
        if(done) break;
    }
    if(!done) error("No plugin to save world object file '" + objFile + "'");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// RIGHT HAND MENU - Tool button signals and slots                                                                          //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MainWindow::loadObjectsForSimButtonClick()
{

    QString types;
    qDebug() << "Able to load files:" << objectLoaders.keys();
    for(QString k : objectLoaders.keys())
        if(k.size())
          types += k + ";;";

    types = types.left(types.size()-2);

    QStringList objFiles = QFileDialog::getOpenFileNames(nullptr, "", "", types, nullptr);

    for(const QString& objFile : objFiles)
    {
        QString suff = QFileInfo(objFile).suffix().toLower();

        bool done = false;
        for(QString k : objectLoaders.keys())
        {
            if(k.toLower().contains(suff))
            {
                for(WorldObjectLoader_If* l : objectLoaders[k])
                {
                    if(l->canLoadFile(objFile, componentFactories))
                    {
                        l->getUserOptions(objFile, componentFactories);

                        try
                        {
                            WorldObject* wobj = l->loadFile(objFile, componentFactories);
                            QString name = QFileInfo(objFile).baseName();
                            wobj->getProperties()["Name"]->set(name, true);
                            simulator->addObjectToTools(wobj);
                            done = true;
                            break;
                        }catch(std::exception&)
                        {
                            error("Unable to load world object file '" + objFile + "'");
                        }
                        done = true;
                    }
                    if(done) break;
                }
            }
            if(done) continue;
        }
        if(!done) error("No plugin to load world object file '" + objFile + "'");
    }
}

void MainWindow::exportObjectButtonClick()
{
    bool ok;

    //prompt for name
    QString name = QInputDialog::getText(0, "Name This Robot",
    "Name:", QLineEdit::Normal,"", &ok);

    //if they didn't click cancel, prompt for type
    if(ok)
    {
        QString type = QInputDialog::getText(0, "Give This Robot a Type",
        "Type:", QLineEdit::Normal,"", &ok);

        //if they still didn't click cancel, add robot as tool in the simulator
        if(ok)
        {

            QVector<WorldObjectComponent*> newComponents;
            for(WorldObjectComponent* c : designer->getComponents())
                newComponents.push_back(c->clone());

            WorldObject *object = new WorldObject(newComponents, name, type, this);
            simulator->addObjectToTools(object);
        }
    }
}

void MainWindow::loadToolsButtonClick()
{
    //tools can only be loaded from initial component plugins
    for( auto it = componentFactories.begin(); it != componentFactories.end(); it++)
    {
        WorldObjectComponent* component = it.value()->createComponent();
        designer->addObjectToTools(component);
        ui->addToolButton->setEnabled(true);
    }
}

void MainWindow::errorMessage(QString error)
{
    QMessageBox err;
    err.setText(error);
    err.exec();
}
