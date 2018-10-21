QT       += core

TARGET = dummy_target
TEMPLATE = app # Technically a plugin, but Intellisense doesn't like that being set

INCLUDEPATH += include ../pkg_SimulatorAPI/include ../pkg_Box2D/include /opt/ros/ardent/include

SOURCES += \
    src/swedish_wheel.cpp \
    src/swedish_wheel_plugin.cpp \
    src/encoder.cpp

HEADERS += \
    include/veranda_wheels/swedish_wheel.h \
    include/veranda_wheels/swedish_wheel_plugin.h\
    include/veranda_wheels/encoder.h \
    include/veranda_wheels/dllapi.h

DISTFILES += \
    CMakeLists.txt \
    package.xml