# QT       += core gui

# greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11 console

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    main.cpp

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

LIBS += -LC:/Users/tgrut/Downloads/opencv/build/x64/vc15/lib/ -lopencv_world450d
LIBS += -LC:/Users/tgrut/Downloads/opencv/build/x64/vc15/lib/ -lopencv_world450

INCLUDEPATH += C:/Users/tgrut/Downloads/opencv/build/include
