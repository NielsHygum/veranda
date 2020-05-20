QT       += core

TARGET = dummy_target
TEMPLATE = app # Technically a plugin, but Intellisense doesn't like that being set

INCLUDEPATH += include
include(../../include_paths.pri)

SOURCES += \
    src/simple_obstacle_map.cpp

HEADERS += \
    include/veranda_perception/simple_obstacle_map.h \
    include/veranda_perception/dllapi.h

DISTFILES += \
    CMakeLists.txt \
    package.xml
