


#include "Application.h"


std::string cmdPopen(const std::string& cmdLine) {
	char buffer[1024] = { '\0' };
	FILE* pf = NULL;
	pf = _popen(cmdLine.c_str(), "r");
	if (NULL == pf) {
		printf("open pipe failed\n");
		return std::string("");
	}
	std::string ret;
	while (fgets(buffer, sizeof(buffer), pf)) {
		ret += buffer;
	}
	_pclose(pf);
	return ret;
}



int main()
{

	Application app;
	int ret = app.run();
	app.cleanup();
	return ret;
}
