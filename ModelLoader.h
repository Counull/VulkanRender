#pragma once

#include  "VkSupport.h"
class ModelLoader
{
public:
	void loadModel(const std::string& modelPath, std::vector<Vertex>& vertices, std::vector<uint32_t>& indices);
};

