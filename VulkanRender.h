#pragma once

#include "VkSupport.h"

#include "ModelLoader.h"



class VulkanRender
{

public:

	vk::Instance instance;

	VulkanRender();
	void createInstance(const std::vector<const char*>& requiredExtension);
	void init(const vk::SurfaceKHR& surface);
	void drawFrame();
	void waitDraw();
	void destroy();
	//log应当放在单独一类里   如果没有必要的话接下来应该不会浪费时间写log
	void logExtensionSupport(std::vector<vk::ExtensionProperties> logExtensions);
	void logLayerSupport();
	void logVulkanApiVersion();
	void logPhysicalDeviceProperties(vk::PhysicalDevice physicalDevice);
	void logSwapChainSupport();


	void onWindowResize(uint32_t width, uint32_t height);

private:


	bool isMsaaEnable = VK_TRUE;

	std::vector<Vertex> vertices;

	std::vector<uint32_t> indices;


	const std::string MODEL_PATH = "Model\\spot\\spot_triangulated_good.obj";

	const std::string TEXTURE_PATH = "Model\\spot\\spot_texture.png";

	vk::SampleCountFlagBits msaaSamples = vk::SampleCountFlagBits::e1;


	//const std::string MODEL_PATH = "D:\\3D\\viking_room\\viking_room.obj";

	//const std::string TEXTURE_PATH = "D:\\3D\\viking_room\\viking_room.png";

	const int MAX_FRAMES_IN_FLIGHT = 2;
	bool framebufferResized = false;
	size_t currentFrame = 0;

	uint32_t mipLevels;


	SwapChainSupportDetails swapChainSupportDetails;
	vk::PhysicalDevice physicalDevice;
	vk::Device device;
	vk::Queue graphicQueue;
	vk::Queue presentQueue;
	vk::SurfaceKHR surface;
	vk::SwapchainKHR swapChain;
	vk::Format swapChainImageFormat;
	vk::Extent2D swapChainExtent;
	vk::DescriptorSetLayout descriptorSetLayout;
	vk::DescriptorPool descriptorPool;
	vk::PipelineLayout pipelineLayout;
	vk::Pipeline graphicsPipeline;
	vk::RenderPass renderPass;
	vk::CommandPool commandPool;
	vk::Buffer vertexBuffer;
	vk::DeviceMemory vertexBufferMemory;
	vk::Buffer indexBuffer;
	vk::DeviceMemory indexBufferMemory;

	vk::Image textureImage;
	vk::DeviceMemory textureImageMemory;
	vk::ImageView	textureImageView;
	vk::Sampler textureSampler;

	vk::Image depthImage;
	vk::DeviceMemory depthImageMemory;
	vk::ImageView	depthImageView;

	vk::Image colorImage;
	vk::DeviceMemory colorImageMemory;
	vk::ImageView	colorImageView;

	std::vector < vk::Semaphore > imageAvailableSemaphores;
	std::vector < vk::Semaphore >  renderFinishedSemaphores;
	std::vector<vk::Fence> inFlightFences;
	std::vector<const char*> enableLayers;
	std::vector<const char*> enableExtensions;
	std::vector<const char*> deviceExtensions;
	std::vector<uint32_t> availableQueueFamilyIndex;
	std::vector<vk::QueueFamilyProperties> queueFamilies;
	std::vector<vk::ImageView> swapChainImageViews;
	std::vector<vk::Framebuffer> swapChainFramebuffers;
	std::vector<vk::CommandBuffer> presentCommandBuffers;
	std::vector<vk::ExtensionProperties> availableDeviceExtensions;
	std::vector<vk::ExtensionProperties> extensions;
	std::vector<vk::LayerProperties> layerSupport;
	std::vector<vk::Image> swapChainImages;
	std::vector<vk::Buffer> uniformBuffers;
	std::vector<vk::DeviceMemory> uniformBuffersMemory;
	std::vector<vk::DescriptorSet> descriptorSets;


	std::vector<uint32_t> findQueueFamilies(vk::QueueFlagBits flag);
	vk::ShaderModule createShaderModule(const std::vector<char>& code);

	void initPhysicalDevice();
	void createLogicalDevice();
	void createSwapChain();
	void createImageViews();
	void createGraphicsPipeline();
	void createRenderPass();
	void createFrameBuffers();
	void createCommandPool();
	void createPresentCommandBuffer();
	void createTextureImage();
	void createTextureImageView();
	void createTextureSampler();
	void createDepthResources();
	void createVertexBuffer();
	void createIndexBuffer();
	void createColorResources();
	void createUniformBuffers();
	void createDescriptorPool();
	void createDescriptorSets();
	void createDescriptorSetLayout();
	void createSyncObjects();

	void loadModel();


	void createBuffer(vk::DeviceSize size, vk::BufferUsageFlags usage, vk::MemoryPropertyFlags properties, vk::Buffer& buffer, vk::DeviceMemory& bufferMemory);

	void createImage(uint32_t width, uint32_t height, uint32_t mipLevel,
		vk::SampleCountFlagBits numSamples, vk::Format format, vk::ImageTiling tiling,
		vk::ImageUsageFlags usage, vk::MemoryPropertyFlags properties, vk::Image& image, vk::DeviceMemory& imageMemory);

	vk::ImageView createImageView(vk::Image image, vk::Format format, vk::ImageAspectFlags aspectFlags, uint32_t mipLevel);

	vk::CommandBuffer beginSingleTimeCommands();
	void endSingleTimeCommands(vk::CommandBuffer commandBuffer);


	void copyBuffer(const vk::Buffer& srcBuffer, const vk::Buffer& dstBuffer, vk::DeviceSize size);
	void transitionImageLayout(vk::Image image, vk::Format format, vk::ImageLayout oldLayout, vk::ImageLayout newLayout, uint32_t mipLevel);
	void copyBufferToImage(vk::Buffer buffer, vk::Image image, uint32_t width, uint32_t height);
	void generateMipmaps(vk::Image image, vk::Format imageFormat, int32_t texWidth, int32_t texHeight, uint32_t mipLevels);

	void recordCommandBuffer(const vk::CommandBuffer& commandBuffer, uint32_t imageIndex);

	void recreateSwapChain();
	void updateUniformBuffer(uint32_t currentImage);

	void cleanupSwapChain();
	void checkValidationExtensionSupport();
	void checkValidationLayerSupport();
	bool checkPresentSupport(int i);
	void checkDeviceExtension();
	void checkSwapChainSupport();
	uint32_t findMemoryType(uint32_t typeFilter, vk::MemoryPropertyFlags properties);

	vk::SurfaceFormatKHR chooseSwapSurfaceFormat();
	vk::PresentModeKHR chooseSwapPresentMode();
	vk::Extent2D chooseSwapExtent();

	vk::Format findSupportFormant(const std::vector<vk::Format>& candidates, vk::ImageTiling tiling, vk::FormatFeatureFlags feature);
	vk::Format findDepthFormat();
	bool hasStencilComponent(vk::Format format);

	vk::SampleCountFlagBits getMaxUsableSampleCount();

	VkDebugUtilsMessengerEXT debugCallback;
	VkResult CreateDebugUtilsMessengerEXT(
		const VkDebugUtilsMessengerCreateInfoEXT* pCreateInfo,
		const VkAllocationCallbacks* pAllocator,
		VkDebugUtilsMessengerEXT* pCallback);
	void DestroyDebugUtilsMessengerEXT(VkInstance instance,
		VkDebugUtilsMessengerEXT callback,
		const VkAllocationCallbacks* pAllocator);
};

















