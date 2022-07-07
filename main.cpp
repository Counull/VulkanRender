


#include "Application.h"
int main()
{
	
	Application app;
	int ret = app.run();
	app.cleanup();
	return ret;
}
