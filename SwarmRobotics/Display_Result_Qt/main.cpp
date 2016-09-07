#include "Display_result_qt.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	Display_Result_Qt w;
	w.show();
	return a.exec();
}
