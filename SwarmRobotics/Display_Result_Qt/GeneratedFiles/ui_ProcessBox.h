/********************************************************************************
** Form generated from reading UI file 'ProcessBox.ui'
**
** Created by: Qt User Interface Compiler version 5.6.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PROCESSBOX_H
#define UI_PROCESSBOX_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>

QT_BEGIN_NAMESPACE

class Ui_ProcessBox
{
public:
    QDialogButtonBox *buttonBox;
    QDoubleSpinBox *param1SpinBox;
    QLabel *param1Label;
    QDoubleSpinBox *param2SpinBox;
    QLabel *param2Label;
    QLabel *param3Label;
    QDoubleSpinBox *param3SpinBox;
    QLabel *param1Unit;
    QLabel *param2Unit;
    QLabel *param3Unit;
    QDoubleSpinBox *param4SpinBox;
    QLabel *param4Unit;
    QLabel *param4Label;

    void setupUi(QDialog *ProcessBox)
    {
        if (ProcessBox->objectName().isEmpty())
            ProcessBox->setObjectName(QStringLiteral("ProcessBox"));
        ProcessBox->resize(322, 230);
        buttonBox = new QDialogButtonBox(ProcessBox);
        buttonBox->setObjectName(QStringLiteral("buttonBox"));
        buttonBox->setGeometry(QRect(110, 180, 190, 40));
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);
        param1SpinBox = new QDoubleSpinBox(ProcessBox);
        param1SpinBox->setObjectName(QStringLiteral("param1SpinBox"));
        param1SpinBox->setGeometry(QRect(150, 20, 81, 31));
        param1SpinBox->setMinimum(2);
        param1SpinBox->setMaximum(10);
        param1SpinBox->setSingleStep(0.1);
        param1SpinBox->setValue(2);
        param1Label = new QLabel(ProcessBox);
        param1Label->setObjectName(QStringLiteral("param1Label"));
        param1Label->setGeometry(QRect(20, 20, 101, 31));
        QFont font;
        font.setPointSize(10);
        param1Label->setFont(font);
        param2SpinBox = new QDoubleSpinBox(ProcessBox);
        param2SpinBox->setObjectName(QStringLiteral("param2SpinBox"));
        param2SpinBox->setGeometry(QRect(150, 60, 81, 31));
        param2SpinBox->setMinimum(300);
        param2SpinBox->setMaximum(5000);
        param2SpinBox->setValue(600);
        param2Label = new QLabel(ProcessBox);
        param2Label->setObjectName(QStringLiteral("param2Label"));
        param2Label->setGeometry(QRect(20, 60, 121, 31));
        param2Label->setFont(font);
        param3Label = new QLabel(ProcessBox);
        param3Label->setObjectName(QStringLiteral("param3Label"));
        param3Label->setGeometry(QRect(20, 100, 101, 31));
        param3Label->setFont(font);
        param3SpinBox = new QDoubleSpinBox(ProcessBox);
        param3SpinBox->setObjectName(QStringLiteral("param3SpinBox"));
        param3SpinBox->setGeometry(QRect(150, 100, 81, 31));
        param3SpinBox->setMinimum(300);
        param3SpinBox->setMaximum(5000);
        param3SpinBox->setValue(400);
        param1Unit = new QLabel(ProcessBox);
        param1Unit->setObjectName(QStringLiteral("param1Unit"));
        param1Unit->setGeometry(QRect(240, 20, 61, 31));
        param1Unit->setFont(font);
        param2Unit = new QLabel(ProcessBox);
        param2Unit->setObjectName(QStringLiteral("param2Unit"));
        param2Unit->setGeometry(QRect(240, 60, 61, 31));
        param2Unit->setFont(font);
        param3Unit = new QLabel(ProcessBox);
        param3Unit->setObjectName(QStringLiteral("param3Unit"));
        param3Unit->setGeometry(QRect(240, 100, 61, 31));
        param3Unit->setFont(font);
        param4SpinBox = new QDoubleSpinBox(ProcessBox);
        param4SpinBox->setObjectName(QStringLiteral("param4SpinBox"));
        param4SpinBox->setGeometry(QRect(150, 140, 81, 31));
        param4SpinBox->setMinimum(300);
        param4SpinBox->setMaximum(5000);
        param4SpinBox->setValue(400);
        param4Unit = new QLabel(ProcessBox);
        param4Unit->setObjectName(QStringLiteral("param4Unit"));
        param4Unit->setGeometry(QRect(240, 140, 61, 31));
        param4Unit->setFont(font);
        param4Label = new QLabel(ProcessBox);
        param4Label->setObjectName(QStringLiteral("param4Label"));
        param4Label->setGeometry(QRect(20, 140, 101, 31));
        param4Label->setFont(font);

        retranslateUi(ProcessBox);

        QMetaObject::connectSlotsByName(ProcessBox);
    } // setupUi

    void retranslateUi(QDialog *ProcessBox)
    {
        ProcessBox->setWindowTitle(QApplication::translate("ProcessBox", "ProcessBox", 0));
        param1Label->setText(QApplication::translate("ProcessBox", "Min plane area:", 0));
        param2Label->setText(QApplication::translate("ProcessBox", "Distance threshold:", 0));
        param3Label->setText(QApplication::translate("ProcessBox", "Max interations", 0));
        param1Unit->setText(QApplication::translate("ProcessBox", "m2", 0));
        param2Unit->setText(QApplication::translate("ProcessBox", "millimeter", 0));
        param3Unit->setText(QApplication::translate("ProcessBox", "millimeter", 0));
        param4Unit->setText(QApplication::translate("ProcessBox", "millimeter", 0));
        param4Label->setText(QApplication::translate("ProcessBox", "Cluster threshold", 0));
    } // retranslateUi

};

namespace Ui {
    class ProcessBox: public Ui_ProcessBox {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PROCESSBOX_H
