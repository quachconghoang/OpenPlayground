/********************************************************************************
** Form generated from reading UI file 'Display_result_qt.ui'
**
** Created by: Qt User Interface Compiler version 5.6.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_DISPLAY_RESULT_QT_H
#define UI_DISPLAY_RESULT_QT_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
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
    QWidget *centralWidget;
    QVTKWidget *qvtkWidget;
    QMenuBar *menuBar;
    QMenu *menuFiles;
    QMenu *menuHelps;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;
    QToolBar *toolBar;

    void setupUi(QMainWindow *Display_Result_QtClass)
    {
        if (Display_Result_QtClass->objectName().isEmpty())
            Display_Result_QtClass->setObjectName(QStringLiteral("Display_Result_QtClass"));
        Display_Result_QtClass->resize(640, 533);
        centralWidget = new QWidget(Display_Result_QtClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        qvtkWidget = new QVTKWidget(centralWidget);
        qvtkWidget->setObjectName(QStringLiteral("qvtkWidget"));
        qvtkWidget->setGeometry(QRect(0, 0, 640, 480));
        Display_Result_QtClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(Display_Result_QtClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 640, 21));
        menuFiles = new QMenu(menuBar);
        menuFiles->setObjectName(QStringLiteral("menuFiles"));
        menuHelps = new QMenu(menuBar);
        menuHelps->setObjectName(QStringLiteral("menuHelps"));
        Display_Result_QtClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(Display_Result_QtClass);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        Display_Result_QtClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(Display_Result_QtClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        Display_Result_QtClass->setStatusBar(statusBar);
        toolBar = new QToolBar(Display_Result_QtClass);
        toolBar->setObjectName(QStringLiteral("toolBar"));
        Display_Result_QtClass->addToolBar(Qt::TopToolBarArea, toolBar);

        menuBar->addAction(menuFiles->menuAction());
        menuBar->addAction(menuHelps->menuAction());

        retranslateUi(Display_Result_QtClass);

        QMetaObject::connectSlotsByName(Display_Result_QtClass);
    } // setupUi

    void retranslateUi(QMainWindow *Display_Result_QtClass)
    {
        Display_Result_QtClass->setWindowTitle(QApplication::translate("Display_Result_QtClass", "Display_Result_Qt", 0));
        menuFiles->setTitle(QApplication::translate("Display_Result_QtClass", "Files", 0));
        menuHelps->setTitle(QApplication::translate("Display_Result_QtClass", "Help", 0));
        toolBar->setWindowTitle(QApplication::translate("Display_Result_QtClass", "toolBar", 0));
    } // retranslateUi

};

namespace Ui {
    class Display_Result_QtClass: public Ui_Display_Result_QtClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_DISPLAY_RESULT_QT_H
