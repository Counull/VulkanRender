#pragma once

#include "VulkanRender.h"
// Tell SDL not to mess with main()
#define SDL_MAIN_HANDLED

#include <glm/glm.hpp>
#include <SDL2/SDL.h>
#include <SDL2/SDL_syswm.h>
#include <SDL2/SDL_vulkan.h>
#include <iostream>
#include <vector>


class Application
{
public:
	int run();

	void cleanup();
private:
	int initForm();
	int initRender();

	int  mainLoop();
	void handleEvent();
	std::vector<const char*> extensions;
	VulkanRender render;
	SDL_Window* window;
	bool stillRunning = true;
	
	bool windowMinimized = false;


	
};

