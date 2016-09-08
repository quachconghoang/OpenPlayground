/********************************************************************************
** Form generated from reading UI file 'Display_Result_Qt.ui'
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
    QWidget *centralWidget;
    QHBoxLayout *horizontalLayout;
    QVTKWidget *qvtkWidget;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QMenu *menuHelp;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *Display_Result_QtClass)
    {
        if (Display_Result_QtClass->objectName().isEmpty())
            Display_Result_QtClass->setObjectName(QStringLiteral("Display_Result_QtClass"));
        Display_Result_QtClass->resize(640, 480);
        actionOpen_PCD = new QAction(Display_Result_QtClass);
        actionOpen_PCD->setObjectName(QStringLiteral("actionOpen_PCD"));
        actionShow_Mesh = new QAction(Display_Result_QtClass);
        actionShow_Mesh->setObjectName(QStringLiteral("actionShow_Mesh"));
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
        menuBar->setGeometry(QRect(0, 0, 640, 21));
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QStringLiteral("menuFile"));
        menuHelp = new QMenu(menuBar);
        menuHelp->setObjectName(QStringLiteral("menuHelp"));
        Display_Result_QtClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(Display_Result_QtClass);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        Display_Result_QtClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(Display_Result_QtClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        Display_Result_QtClass->setStatusBar(statusBar);

        menuBar->addAction(menuFile->menuAction());
        menuBar->addAction(menuHelp->menuAction());
        mainToolBar->addAction(actionOpen_PCD);
        mainToolBar->addAction(actionShow_Mesh);

        retranslateUi(Display_Result_QtClass);

        QMetaObject::connectSlotsByName(Display_Result_QtClass);
    } // setupUi

    void retranslateUi(QMainWindow *Display_Result_QtClass)
    {
        Display_Result_QtClass->setWindowTitle(QApplication::translate("Display_Result_QtClass", "Display_Result_Qt", 0));
        actionOpen_PCD->setText(QApplication::translate("Display_Result_QtClass", "Open PCD", 0));
        actionShow_Mesh->setText(QApplication::translate("Display_Result_QtClass", "Show Mesh", 0));
        menuFile->setTitle(QApplication::translate("Display_Result_QtClass", "File", 0));
        menuHelp->setTitle(QApplication::translate("Display_Result_QtClass", "Help", 0));
    } // retranslateUi

};

namespace Ui {
    class Display_Result_QtClass: public Ui_Display_Result_QtClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_DISPLAY_RESULT_QT_H
