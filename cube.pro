QT       += core gui widgets

TARGET = cube
TEMPLATE = app

SOURCES += main.cpp

SOURCES += \
    mainwidget.cpp

HEADERS += \
    mainwidget.h

RESOURCES += \
    shaders.qrc

# install
target.path = $$[QT_INSTALL_EXAMPLES]/opengl/cube
INSTALLS += target
