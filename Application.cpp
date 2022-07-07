#include "Application.h"
#include  "Config.h"
int Application::run()
{

	int ret = 0;
	ret = initForm();
	ret = initRender();
	ret = mainLoop();

	return ret;
}

int Application::initForm()
{
	// Create an SDL window that supports Vulkan rendering.
	if (SDL_Init(SDL_INIT_VIDEO) != 0) {
		std::cout << "Could not initialize SDL." << std::endl;
		return 1;
	}
	window = SDL_CreateWindow("Vulkan Window", SDL_WINDOWPOS_CENTERED,
		SDL_WINDOWPOS_CENTERED, static_cast<int>(Config::WIDTH), static_cast<int>(Config::HEIGHT), SDL_WINDOW_VULKAN | SDL_WINDOW_RESIZABLE);
	if (window == NULL) {
		std::cout << "Could not create SDL window." << std::endl;
		return 1;
	}

	

	// Get WSI extensions from SDL (we can add more if we like - we just can't remove these)
	unsigned extension_count;
	if (!SDL_Vulkan_GetInstanceExtensions(window, &extension_count, NULL)) {
		std::cout << "Could not get the number of required instance extensions from SDL." << std::endl;
		return 1;
	}
	extensions = std::vector<const char*>(extension_count);
	if (!SDL_Vulkan_GetInstanceExtensions(window, &extension_count, extensions.data())) {
		std::cout << "Could not get the names of required instance extensions from SDL." << std::endl;
		return 1;
	}


	return 0;
}

int Application::initRender()
{
	render.createInstance(extensions);


	VkSurfaceKHR c_surface;
	if (!SDL_Vulkan_CreateSurface(window, static_cast<VkInstance>(render.instance), &c_surface)) {
		std::cout << "Could not create a Vulkan surface." << std::endl;
		return 1;
	}

	render.init(vk::SurfaceKHR(c_surface));

}

int Application::mainLoop()
{

	while (stillRunning) {

		handleEvent();
		if (windowMinimized)
		{
			continue;
		}
		render.drawFrame();

	}

	render.waitDraw();
	return 0;
}

void Application::handleEvent()
{


	SDL_Event event;
	while (SDL_PollEvent(&event)) {

		switch (event.type) {

		case SDL_QUIT:
			stillRunning = false;
			break;
		case SDL_WINDOWEVENT:
		
			if (event.window.event == SDL_WINDOWEVENT_SIZE_CHANGED  )
			{
				Config::WIDTH = event.window.data1;
				Config::HEIGHT = event.window.data2;
				std::cout << "windows size change : " << "\twidth:" << Config::WIDTH << "\theight:" << Config::HEIGHT << std::endl;
				render.onWindowResize(Config::WIDTH, Config::HEIGHT);
			}
			else if (event.window.event == SDL_WINDOWEVENT_MINIMIZED)
			{
				windowMinimized = true;

			}
			else if( event.window.event == SDL_WINDOWEVENT_RESTORED)
			{
				windowMinimized = false;
			}
			break;

		default:
			// Do nothing.
			break;
		}
	}
}

void Application::cleanup()
{
	render.destroy();
	SDL_DestroyWindow(window);
	SDL_Quit();

}
