#include <iostream>
#include <Windows.h>

//#define GLEW_STATIC
#include <GL/glew.h>

#include <GLFW/glfw3.h>

#include <glm\glm.hpp>
#include <glm\gtc\matrix_transform.hpp>
#include <glm\gtc\type_ptr.hpp>

#include "Shader.h"

//#include <SOIL2\SOIL2.h>

#include <Box2D/Box2D.h>

//#include "hullfinder2.h"

#include "Level.h"

const GLuint HEIGHT = 1200, WIDTH = 1600;


int main() {
	
	glfwInit();

	//Create a window
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
	glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

	GLFWwindow* window = glfwCreateWindow(WIDTH, HEIGHT, "Heidu", nullptr, nullptr);

	int screenwidth, screenheight;
	glfwGetFramebufferSize(window, &screenwidth, &screenheight);

	if (window == nullptr) {
		std::cout << "Failed to create GLFW Window." << std::endl;
		glfwTerminate();

		return EXIT_FAILURE;
	}

	glfwMakeContextCurrent(window);

	glewExperimental = GL_TRUE;

	if (glewInit() != GLEW_OK) {
		std::cout << "Failed to initialize GLEW." << std::endl;
		return EXIT_FAILURE;
	}

	// Set viewport
	glViewport(0, 0, screenwidth, screenheight);

	glEnable(GL_DEPTH_TEST);

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	Level* level = new Level();

	glm::mat4 projection;
	projection = glm::perspective(45.0f, (GLfloat)screenwidth / (GLfloat)screenheight, 0.1f, 1000.0f);

	//Game loop
	const int PHYSICS_FRAME_RATE = 60;
	const float32 PHYSICS_TICK = 1.0f / PHYSICS_FRAME_RATE;
	const int FRAME_SKIP_MAX = 10;
	std::cout.precision(0);

	double nextGameTick = glfwGetTime();

	int loops;

	while (!glfwWindowShouldClose(window)) {
		//1. GAME LOGIC
		loops = 0;

		while (glfwGetTime() > nextGameTick && loops < FRAME_SKIP_MAX) {
			glfwPollEvents();

			//physics
			double t = glfwGetTime();
			level->Step(PHYSICS_TICK);
			std::cout << "Physics time: " << (glfwGetTime() - t) * 1000 << " ms ";
			
			nextGameTick += PHYSICS_TICK;
			loops++;
		}
		std::cout << loops << std::endl;
		
		//2. DRAWING
		//Clear buffers
		glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		level->blob->draw(projection);
		//level->blob->findHull();

		glfwSwapBuffers(window);
		//Sleep(3000);
	}

	delete level;
	glfwTerminate();

	return EXIT_SUCCESS;
}