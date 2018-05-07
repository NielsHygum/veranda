QT       += core

TARGET = dummy_target
TEMPLATE = app # Technically a plugin, but Intellisense doesn't like that being set

INCLUDEPATH += include ../pkg_SimulatorAPI/include ../pkg_Box2D/include

SOURCES += \
    src/circle.cpp \
    src/circle_plugin.cpp

HEADERS += \
    include/circle.h \
    include/circle_plugin.h \
    include/defines.h

DISTFILES += \
    CMakeLists.txt \
    package.xml
