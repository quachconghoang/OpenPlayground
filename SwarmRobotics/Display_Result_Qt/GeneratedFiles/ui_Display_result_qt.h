/********************************************************************************
** Form generated from reading UI file 'Display_Result_Qt.ui'
**
** Created by: Qt User Interface Compiler version 5.6.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_DISPLAY_RESULT_QT_H
#define UI_DISPLAY_RESULT_QT_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_Display_Result_QtClass
{
public:
    QAction *actionOpen_PCD;
    QAction *actionShow_Mesh;
    QAction *actionSegmentation;
    QAction *actionShowOrigin;
    QAction *actionChangeColor;
    QAction *actionGenCapturePoints;
    QAction *actionSaveCapturePoints;
    QAction *actionShowGridCloud;
    QAction *actionShowObjectAsCube;
    QWidget *centralWidget;
    QHBoxLayout *horizontalLayout;
    QVTKWidget *qvtkWidget;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QMenu *menuHelp;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;
    QToolBar *processingToolBar;

    void setupUi(QMainWindow *Display_Result_QtClass)
    {
        if (Display_Result_QtClass->objectName().isEmpty())
            Display_Result_QtClass->setObjectName(QStringLiteral("Display_Result_QtClass"));
        Display_Result_QtClass->resize(640, 480);
        actionOpen_PCD = new QAction(Display_Result_QtClass);
        actionOpen_PCD->setObjectName(QStringLiteral("actionOpen_PCD"));
        QIcon icon;
        icon.addFile(QStringLiteral(":/Display_Result_Qt/Resources/open.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionOpen_PCD->setIcon(icon);
        actionShow_Mesh = new QAction(Display_Result_QtClass);
        actionShow_Mesh->setObjectName(QStringLiteral("actionShow_Mesh"));
        actionShow_Mesh->setCheckable(true);
        actionShow_Mesh->setEnabled(false);
        QIcon icon1;
        icon1.addFile(QStringLiteral(":/Display_Result_Qt/Resources/backlines.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionShow_Mesh->setIcon(icon1);
        actionSegmentation = new QAction(Display_Result_QtClass);
        actionSegmentation->setObjectName(QStringLiteral("actionSegmentation"));
        actionSegmentation->setEnabled(false);
        QIcon icon2;
        icon2.addFile(QStringLiteral(":/Display_Result_Qt/Resources/segment.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSegmentation->setIcon(icon2);
        actionShowOrigin = new QAction(Display_Result_QtClass);
        actionShowOrigin->setObjectName(QStringLiteral("actionShowOrigin"));
        actionShowOrigin->setCheckable(true);
        actionShowOrigin->setEnabled(false);
        QIcon icon3;
        icon3.addFile(QStringLiteral(":/Display_Result_Qt/Resources/show_origin_icon.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionShowOrigin->setIcon(icon3);
        actionChangeColor = new QAction(Display_Result_QtClass);
        actionChangeColor->setObjectName(QStringLiteral("actionChangeColor"));
        actionChangeColor->setEnabled(false);
        QIcon icon4;
        icon4.addFile(QStringLiteral(":/Display_Result_Qt/Resources/rgbt.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionChangeColor->setIcon(icon4);
        actionGenCapturePoints = new QAction(Display_Result_QtClass);
        actionGenCapturePoints->setObjectName(QStringLiteral("actionGenCapturePoints"));
        actionGenCapturePoints->setEnabled(false);
        QIcon icon5;
        icon5.addFile(QStringLiteral(":/Display_Result_Qt/Resources/waypoint_icon.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionGenCapturePoints->setIcon(icon5);
        actionSaveCapturePoints = new QAction(Display_Result_QtClass);
        actionSaveCapturePoints->setObjectName(QStringLiteral("actionSaveCapturePoints"));
        actionSaveCapturePoints->setEnabled(false);
        QIcon icon6;
        icon6.addFile(QStringLiteral(":/Display_Result_Qt/Resources/waypoint_save_icon.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSaveCapturePoints->setIcon(icon6);
        actionShowGridCloud = new QAction(Display_Result_QtClass);
        actionShowGridCloud->setObjectName(QStringLiteral("actionShowGridCloud"));
        actionShowGridCloud->setCheckable(true);
        actionShowGridCloud->setEnabled(false);
        QIcon icon7;
        icon7.addFile(QStringLiteral(":/Display_Result_Qt/Resources/GenMesh.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionShowGridCloud->setIcon(icon7);
        actionShowObjectAsCube = new QAction(Display_Result_QtClass);
        actionShowObjectAsCube->setObjectName(QStringLiteral("actionShowObjectAsCube"));
        actionShowObjectAsCube->setCheckable(true);
        actionShowObjectAsCube->setEnabled(false);
        QIcon icon8;
        icon8.addFile(QStringLiteral(":/Display_Result_Qt/Resources/bbox.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionShowObjectAsCube->setIcon(icon8);
        centralWidget = new QWidget(Display_Result_QtClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        horizontalLayout = new QHBoxLayout(centralWidget);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        qvtkWidget = new QVTKWidget(centralWidget);
        qvtkWidget->setObjectName(QStringLiteral("qvtkWidget"));

        horizontalLayout->addWidget(qvtkWidget);

        Display_Result_QtClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(Display_Result_QtClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 640, 26));
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QStringLiteral("menuFile"));
        menuHelp = new QMenu(menuBar);
        menuHelp->setObjectName(QStringLiteral("menuHelp"));
        Display_Result_QtClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(Display_Result_QtClass);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        mainToolBar->setEnabled(true);
        Display_Result_QtClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(Display_Result_QtClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        Display_Result_QtClass->setStatusBar(statusBar);
        processingToolBar = new QToolBar(Display_Result_QtClass);
        processingToolBar->setObjectName(QStringLiteral("processingToolBar"));
        Display_Result_QtClass->addToolBar(Qt::LeftToolBarArea, processingToolBar);

        menuBar->addAction(menuFile->menuAction());
        menuBar->addAction(menuHelp->menuAction());
        mainToolBar->addAction(actionOpen_PCD);
        mainToolBar->addAction(actionSaveCapturePoints);
        mainToolBar->addSeparator();
        mainToolBar->addAction(actionShowOrigin);
        mainToolBar->addAction(actionShowGridCloud);
        mainToolBar->addAction(actionShow_Mesh);
        mainToolBar->addAction(actionShowObjectAsCube);
        processingToolBar->addAction(actionSegmentation);
        processingToolBar->addAction(actionGenCapturePoints);
        processingToolBar->addSeparator();
        processingToolBar->addAction(actionChangeColor);

        retranslateUi(Display_Result_QtClass);

        QMetaObject::connectSlotsByName(Display_Result_QtClass);
    } // setupUi

    void retranslateUi(QMainWindow *Display_Result_QtClass)
    {
        Display_Result_QtClass->setWindowTitle(QApplication::translate("Display_Result_QtClass", "Display_Result_Qt", 0));
        actionOpen_PCD->setText(QApplication::translate("Display_Result_QtClass", "Open PCD", 0));
        actionShow_Mesh->setText(QApplication::translate("Display_Result_QtClass", "Show Mesh", 0));
        actionSegmentation->setText(QApplication::translate("Display_Result_QtClass", "Segmentation", 0));
        actionShowOrigin->setText(QApplication::translate("Display_Result_QtClass", "ShowOrigin", 0));
        actionChangeColor->setText(QApplication::translate("Display_Result_QtClass", "ChangeColor", 0));
        actionGenCapturePoints->setText(QApplication::translate("Display_Result_QtClass", "GenCapturePoints", 0));
        actionSaveCapturePoints->setText(QApplication::translate("Display_Result_QtClass", "SaveCapturePoints", 0));
        actionShowGridCloud->setText(QApplication::translate("Display_Result_QtClass", "ShowGridCloud", 0));
        actionShowObjectAsCube->setText(QApplication::translate("Display_Result_QtClass", "ShowObjectAsCube", 0));
        menuFile->setTitle(QApplication::translate("Display_Result_QtClass", "File", 0));
        menuHelp->setTitle(QApplication::translate("Display_Result_QtClass", "Help", 0));
        processingToolBar->setWindowTitle(QApplication::translate("Display_Result_QtClass", "toolBar", 0));
    } // retranslateUi

};

namespace Ui {
    class Display_Result_QtClass: public Ui_Display_Result_QtClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_DISPLAY_RESULT_QT_H
