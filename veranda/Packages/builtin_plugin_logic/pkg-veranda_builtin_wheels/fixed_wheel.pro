QT       += core

TARGET = dummy_target
TEMPLATE = app # Technically a plugin, but Intellisense doesn't like that being set

INCLUDEPATH += include
include(../../include_paths.pri)

SOURCES += \
    src/fixed_wheel.cpp \
    src/encoder.cpp

HEADERS += \
    include/veranda_wheels/fixed_wheel.h \
    include/basic_wheel.h \
    include/veranda_wheels/encoder.h \
    include/veranda_wheels/dllapi.h

DISTFILES += \
    CMakeLists.txt \
    package.xml
