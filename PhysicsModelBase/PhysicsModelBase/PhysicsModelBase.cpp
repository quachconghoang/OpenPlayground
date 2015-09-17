// PhysicsModelBase.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "conio.h"
#include "iostream"

#include "GL/glew.h"
#include "GLFW/glfw3.h"

using namespace std;

int _tmain(int argc, _TCHAR* argv[])
{
	GLFWwindow* window;
	if (!glfwInit())
		return -1;
	glewInit();

	window = glfwCreateWindow(640, 480, "Hello World", NULL, NULL);
	if (!window){
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);
	

	glfwSwapInterval(1);
	int i = 0;
	/* Loop until the user closes the window */
	while (!glfwWindowShouldClose(window))
	{
		i++;
		/* Render here */
		
		/* Swap front and back buffers */
		glfwSwapBuffers(window);
		if (i==30)
		{
			i = 0;
			cout << "flip-flop" << endl;
		}
		
		/* Poll for and process events */
		glfwPollEvents();
	}

	glfwTerminate();
	return 0;
}