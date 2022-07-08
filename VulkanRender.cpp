#include "VulkanRender.h"
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>
#include<limits>
#include  "Utils.h"
#include "Config.h"



static VKAPI_ATTR VkBool32 VKAPI_CALL DebugCallback(
	VkDebugUtilsMessageSeverityFlagBitsEXT messageServerity,
	VkDebugUtilsMessageTypeFlagBitsEXT messageType,
	const VkDebugUtilsMessengerCallbackDataEXT* pCallbackData,
	void* pUserData)
{
	std::cerr << "validation layer :" << pCallbackData->pMessage << std::endl;
	std::cout << "***********************" << std::endl;
	return VK_FALSE;
}


void VulkanRender::init(const vk::SurfaceKHR& surface)
{
	this->surface = surface;
	initPhysicalDevice();
	createLogicalDevice();
	checkDeviceExtension();
	checkSwapChainSupport();
	createSwapChain();
	createImageViews();
	createRenderPass();
	createDescriptorSetLayout();
	createGraphicsPipeline();


	createCommandPool();

	createColorResources();
	createDepthResources();
	createFrameBuffers();
	createTextureImage();
	createTextureImageView();
	createTextureSampler();
	loadModel();
	createVertexBuffer();
	createIndexBuffer();
	createUniformBuffers();
	createDescriptorPool();
	createDescriptorSets();
	createPresentCommandBuffer();
	createSyncObjects();

#if defined(_DEBUG)
	logExtensionSupport(availableDeviceExtensions);
	logSwapChainSupport();
#endif
}

void VulkanRender::drawFrame()
{
	device.waitForFences(1, &inFlightFences[currentFrame], VK_TRUE, (std::numeric_limits<uint64_t >::max)());


	uint32_t imageIndex;
	auto ret = device.acquireNextImageKHR(swapChain, (std::numeric_limits<uint64_t>::max)(), imageAvailableSemaphores[currentFrame], VK_NULL_HANDLE, &imageIndex);
	if (ret == vk::Result::eErrorOutOfDateKHR)
	{
		recreateSwapChain();

		return;
	}
	else if (ret != vk::Result::eSuccess && ret != vk::Result::eSuboptimalKHR)
	{
		throw std::runtime_error("failed to acquire swap chain image");
	}

	device.resetFences(1, &inFlightFences[currentFrame]);


	updateUniformBuffer(currentFrame);


	presentCommandBuffers[currentFrame].reset(vk::CommandBufferResetFlagBits(0));
	recordCommandBuffer(presentCommandBuffers[currentFrame], imageIndex);


	vk::SubmitInfo submitInfo;
	vk::Semaphore waitSemaphores[] = { imageAvailableSemaphores[currentFrame] };
	vk::Semaphore signalSemaphores[] = { renderFinishedSemaphores[currentFrame] };
	vk::SwapchainKHR swapChains[] = { swapChain };
	vk::PipelineStageFlags  waitStages[] = { vk::PipelineStageFlags(VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT) };
	submitInfo.setWaitSemaphoreCount(1)
		.setPWaitSemaphores(waitSemaphores)
		.setPWaitDstStageMask(waitStages)
		.setSignalSemaphoreCount(1)
		.setPSignalSemaphores(signalSemaphores)
		.setCommandBufferCount(1)
		.setPCommandBuffers(&presentCommandBuffers[currentFrame]);


	if (graphicQueue.submit(1, &submitInfo, inFlightFences[currentFrame]) != vk::Result::eSuccess)
	{
		throw std::runtime_error("failed to submit draw command buffer");
	}

	vk::PresentInfoKHR presentInfo;
	presentInfo.setWaitSemaphoreCount(1)
		.setPWaitSemaphores(signalSemaphores)
		.setSwapchainCount(1)
		.setPSwapchains(swapChains)
		.setPImageIndices(&imageIndex)
		.setPResults(nullptr);// Optional

	ret = presentQueue.presentKHR(&presentInfo);

	if (ret == vk::Result::eErrorOutOfDateKHR
		|| ret == vk::Result::eSuboptimalKHR
		|| framebufferResized)
	{

		framebufferResized = false;
		recreateSwapChain();

	}
	else if (ret != vk::Result::eSuccess)
	{
		throw std::runtime_error("failed to present swap chain image");
	}

	currentFrame = (currentFrame + 1) %
		MAX_FRAMES_IN_FLIGHT;





}

void VulkanRender::waitDraw()
{
	device.waitIdle();
}

void VulkanRender::createSwapChain()
{
	auto surfaceFormat = chooseSwapSurfaceFormat();
	auto presentMode = chooseSwapPresentMode();
	auto extent = chooseSwapExtent();
	auto& capabilities = swapChainSupportDetails.capabilities;
	uint32_t imageCount = capabilities.minImageCount + 1;
	if (capabilities.maxImageCount > 0 && imageCount > capabilities.maxImageCount)
	{
		imageCount = capabilities.maxImageCount;
	}

	vk::SwapchainCreateInfoKHR create_info;
	create_info.setSurface(surface)
		.setMinImageCount(imageCount)
		.setImageFormat(surfaceFormat.format)
		.setImageColorSpace(surfaceFormat.colorSpace)
		.setImageExtent(extent)
		.setImageArrayLayers(1)
		.setImageUsage(vk::ImageUsageFlagBits::eColorAttachment)
		.setImageSharingMode(vk::SharingMode::eExclusive)
		.setQueueFamilyIndexCount(1) // Optional
		.setPQueueFamilyIndices(availableQueueFamilyIndex.data()) // Optional
		.setPreTransform(capabilities.currentTransform)
		.setPresentMode(presentMode)
		.setClipped(VK_TRUE)
		.setOldSwapchain(VK_NULL_HANDLE);
	if (device.createSwapchainKHR(&create_info, nullptr, &swapChain) != vk::Result::eSuccess)
	{
		throw std::runtime_error("failed to create swap chain");

	}

	device.getSwapchainImagesKHR(swapChain, &imageCount, nullptr);
	swapChainImages.resize(imageCount);
	device.getSwapchainImagesKHR(swapChain, &imageCount, swapChainImages.data());

	swapChainImageFormat = surfaceFormat.format;
	swapChainExtent = extent;


}

void VulkanRender::createImageViews()
{
	swapChainImageViews.resize(swapChainImages.size());

	for (size_t i = 0; i < swapChainImages.size(); i++)
	{
		swapChainImageViews[i] = createImageView(swapChainImages[i], swapChainImageFormat, vk::ImageAspectFlagBits::eColor, 1);

	}
}

void VulkanRender::createGraphicsPipeline()
{
	auto vertShaderCode = Utils::readFile("Shader\\vert.spv");
	auto fragShaderCode = Utils::readFile("Shader\\frag.spv");
	auto vertShaderModule = createShaderModule(vertShaderCode);
	auto fragShaderModule = createShaderModule(fragShaderCode);

	vk::PipelineShaderStageCreateInfo vertShaderStageInfo;
	vk::PipelineShaderStageCreateInfo fragShaderStageInfo;

	vertShaderStageInfo.setStage(vk::ShaderStageFlagBits::eVertex)
		.setModule(vertShaderModule)
		.setPName("main");


	fragShaderStageInfo.setStage(vk::ShaderStageFlagBits::eFragment)
		.setModule(fragShaderModule)
		.setPName("main");

	vk::PipelineShaderStageCreateInfo shaderStages[]{ vertShaderStageInfo ,fragShaderStageInfo };


	auto bindingDescription = Vertex::getBindingDescription();
	auto attributeDescriptions = Vertex::getAttributeDescriptions();

	vk::PipelineVertexInputStateCreateInfo  vertexInputInfo;
	vertexInputInfo.setVertexBindingDescriptionCount(1)
		.setPVertexBindingDescriptions(&bindingDescription) //Optional
		.setVertexAttributeDescriptionCount(static_cast<uint32_t>(attributeDescriptions.size()))
		.setPVertexAttributeDescriptions(attributeDescriptions.data());//Optional

	vk::PipelineInputAssemblyStateCreateInfo inputAssemblyInfo;
	inputAssemblyInfo.setTopology(vk::PrimitiveTopology::eTriangleList);
	inputAssemblyInfo.setPrimitiveRestartEnable(VK_FALSE);

	vk::Viewport viewport;
	viewport.setX(0.0f)
		.setY(0.0f)
		.setWidth(swapChainExtent.width)
		.setHeight(swapChainExtent.height)
		.setMinDepth(0.0f)
		.setMaxDepth(1.0f);

	vk::Rect2D scissor;
	scissor.setOffset(vk::Offset2D(0, 0))
		.setExtent(swapChainExtent);



	vk::PipelineViewportStateCreateInfo viewportState;
	viewportState.setViewportCount(1)
		.setPViewports(&viewport)
		.setScissorCount(1)
		.setPScissors(&scissor);

	vk::PipelineRasterizationStateCreateInfo rasterizer;
	rasterizer.setDepthClampEnable(VK_FALSE)
		.setRasterizerDiscardEnable(VK_FALSE)
		.setPolygonMode(vk::PolygonMode::eFill)
		.setCullMode(vk::CullModeFlagBits::eBack)
		.setFrontFace(vk::FrontFace::eCounterClockwise)
		.setDepthBiasEnable(VK_FALSE)
		.setDepthBiasConstantFactor(0.0f)// Optional
		.setDepthBiasClamp(0.0f)// Optional
		.setDepthBiasSlopeFactor(0.0f)// Optional
		.setLineWidth(1.0f);

	//多重采样
	vk::PipelineMultisampleStateCreateInfo multisample;
	multisample.setSampleShadingEnable(VK_TRUE)
		.setRasterizationSamples(msaaSamples)
		.setMinSampleShading(0.1f)
		.setPSampleMask(nullptr)
		.setAlphaToCoverageEnable(VK_FALSE)
		.setAlphaToOneEnable(VK_FALSE)
		;



	vk::PipelineColorBlendAttachmentState colorBlendAttachment;
	colorBlendAttachment.setColorWriteMask(vk::ColorComponentFlags(
		vk::ColorComponentFlagBits::eA | vk::ColorComponentFlagBits::eR
		| vk::ColorComponentFlagBits::eG | vk::ColorComponentFlagBits::eB))
		.setBlendEnable(VK_TRUE)
		.setSrcColorBlendFactor(vk::BlendFactor::eSrcAlpha)
		.setDstColorBlendFactor(vk::BlendFactor::eOneMinusSrcAlpha)
		.setColorBlendOp(vk::BlendOp::eAdd)
		.setSrcAlphaBlendFactor(vk::BlendFactor::eOne)
		.setDstAlphaBlendFactor(vk::BlendFactor::eZero)
		.setAlphaBlendOp(vk::BlendOp::eAdd);

	vk::PipelineColorBlendStateCreateInfo colorBlendCreateInfo;
	colorBlendCreateInfo.setLogicOpEnable(VK_FALSE)
		.setLogicOp(vk::LogicOp::eCopy)
		.setAttachmentCount(1)
		.setPAttachments(&colorBlendAttachment)
		.setBlendConstants(std::array<float, 4> { 0.0f, 0.0f, 0.0f, 0.0f });// Optional

	vk::PipelineLayoutCreateInfo pipelineLayoutInfo;
	pipelineLayoutInfo.setSetLayoutCount(1)
		.setPSetLayouts(&descriptorSetLayout)
		.setPushConstantRangeCount(0)// Optional
		.setPPushConstantRanges(nullptr);// Optional

	if (device.createPipelineLayout(&pipelineLayoutInfo, nullptr, &pipelineLayout) != vk::Result::eSuccess)
	{
		throw std::runtime_error("failed to create pipeline layout");

	}


	vk::PipelineDepthStencilStateCreateInfo depthStencil;
	depthStencil.setDepthTestEnable(VK_TRUE)
		.setDepthWriteEnable(VK_TRUE)
		.setDepthCompareOp(vk::CompareOp::eLess)
		.setDepthBoundsTestEnable(VK_FALSE)
		.setStencilTestEnable(VK_FALSE)
		.setFront({})// Optional
		.setBack({})// Optional
		.setMinDepthBounds(0.0f)// Optional
		.setMaxDepthBounds(1.0f);// Optional


	vk::GraphicsPipelineCreateInfo pipelineCreateInfo;
	pipelineCreateInfo.setStageCount(2)
		.setPStages(shaderStages)
		.setPVertexInputState(&vertexInputInfo)
		.setPInputAssemblyState(&inputAssemblyInfo)
		.setPViewportState(&viewportState)
		.setPRasterizationState(&rasterizer)
		.setPMultisampleState(&multisample)
		.setPColorBlendState(&colorBlendCreateInfo)
		.setLayout(pipelineLayout)
		.setRenderPass(renderPass)
		.setSubpass(0)
		.setPDepthStencilState(&depthStencil)
		.setBasePipelineIndex(-1)// Optional
		.setBasePipelineHandle(VK_NULL_HANDLE)// Optional
		.setPDynamicState(nullptr);// Optional






	if (device.createGraphicsPipelines(nullptr, 1,
		&pipelineCreateInfo, nullptr, &graphicsPipeline)
		!= vk::Result::eSuccess)
	{
		throw std::runtime_error("failed to create graphics pipeline");
	}

	device.destroyShaderModule(vertShaderModule, nullptr);
	device.destroyShaderModule(fragShaderModule, nullptr);
}




vk::ShaderModule VulkanRender::createShaderModule(const std::vector<char>& code)
{
	vk::ShaderModuleCreateInfo create_info;
	create_info.setCodeSize(code.size())
		.setPCode(reinterpret_cast <const uint32_t*>(code.data()));
	vk::ShaderModule shaderModule;
	if (device.createShaderModule(&create_info, nullptr, &shaderModule) != vk::Result::eSuccess)
	{
		throw std::runtime_error("failed to create shader module");
	}
	return  shaderModule;
}

void VulkanRender::createRenderPass()
{

	vk::AttachmentDescription colorAttachment;
	colorAttachment.setFormat(swapChainImageFormat)
		.setSamples(msaaSamples)
		.setLoadOp(vk::AttachmentLoadOp::eClear)
		.setStoreOp(vk::AttachmentStoreOp::eStore)
		.setStencilLoadOp(vk::AttachmentLoadOp::eDontCare)
		.setStencilStoreOp(vk::AttachmentStoreOp::eDontCare)
		.setInitialLayout(vk::ImageLayout::eColorAttachmentOptimal)
		.setFinalLayout(vk::ImageLayout::eColorAttachmentOptimal);





	vk::AttachmentDescription depthAttachment;
	depthAttachment.setFormat(findDepthFormat())
		.setSamples(msaaSamples)
		.setLoadOp(vk::AttachmentLoadOp::eClear)
		.setStoreOp(vk::AttachmentStoreOp::eDontCare)
		.setStencilLoadOp(vk::AttachmentLoadOp::eDontCare)
		.setStencilStoreOp(vk::AttachmentStoreOp::eDontCare)
		.setInitialLayout(vk::ImageLayout::eUndefined)
		.setFinalLayout(vk::ImageLayout::eDepthStencilAttachmentOptimal);



	vk::AttachmentDescription colorAttachmentResolve;
	colorAttachmentResolve.setFormat(swapChainImageFormat)
		.setSamples(vk::SampleCountFlagBits::e1)
		.setLoadOp(vk::AttachmentLoadOp::eDontCare)
		.setStoreOp(vk::AttachmentStoreOp::eStore)
		.setStencilLoadOp(vk::AttachmentLoadOp::eDontCare)
		.setStencilStoreOp(vk::AttachmentStoreOp::eDontCare)
		.setInitialLayout(vk::ImageLayout::eUndefined)
		.setFinalLayout(vk::ImageLayout::ePresentSrcKHR);


	vk::AttachmentReference colorAttachmentRef;
	colorAttachmentRef.setAttachment(0)
		.setLayout(vk::ImageLayout::eColorAttachmentOptimal);

	vk::AttachmentReference depthAttachmentRef;
	depthAttachmentRef.setAttachment(1)
		.setLayout(vk::ImageLayout::eDepthStencilAttachmentOptimal);


	vk::AttachmentReference colorAttachmentResolveRef;
	colorAttachmentRef.setAttachment(2)
		.setLayout(vk::ImageLayout::eColorAttachmentOptimal);


	vk::SubpassDescription subpass;
	//这里设置的颜色附着在数组中的索引会被片段着色器使用，对应我们在片段着色器中使用的 layout(location = 0) out vec4 outColor 语句。
	subpass.setPipelineBindPoint(vk::PipelineBindPoint::eGraphics)
		.setColorAttachmentCount(1)
		.setPColorAttachments(&colorAttachmentRef)
		.setPDepthStencilAttachment(&depthAttachmentRef)
		.setPResolveAttachments(&colorAttachmentResolveRef);

	vk::SubpassDependency dependency;
	dependency.setSrcSubpass(VK_SUBPASS_EXTERNAL).setDstSubpass(0)
		.setSrcStageMask(vk::PipelineStageFlagBits::eColorAttachmentOutput | vk::PipelineStageFlagBits::eEarlyFragmentTests)
		.setDstStageMask(vk::PipelineStageFlagBits::eColorAttachmentOutput | vk::PipelineStageFlagBits::eEarlyFragmentTests)
		.setSrcAccessMask(vk::AccessFlagBits::eNoneKHR)
		.setDstAccessMask(vk::AccessFlagBits::eColorAttachmentRead | vk::AccessFlagBits::eColorAttachmentWrite)
		;


	std::array<vk::AttachmentDescription, 3> attachments = { colorAttachmentResolve,depthAttachment,colorAttachment };

	vk::RenderPassCreateInfo renderPassInfo;
	renderPassInfo.setAttachmentCount(static_cast<uint32_t> (attachments.size()))
		.setPAttachments(attachments.data())
		.setSubpassCount(1)
		.setPSubpasses(&subpass)
		.setDependencyCount(1)
		.setDependencies(dependency)
		;


	if (device.createRenderPass(&renderPassInfo, nullptr, &renderPass) != vk::Result::eSuccess)
	{
		throw std::runtime_error("failed to create render pass");
	}

}

void VulkanRender::createFrameBuffers()
{
	swapChainFramebuffers.resize(swapChainImageViews.size());
	for (size_t i = 0; i < swapChainImageViews.size(); i++)
	{
		std::array<	vk::ImageView, 3> attachments = { swapChainImageViews[i],depthImageView, colorImageView };
		vk::FramebufferCreateInfo framebufferInfo;
		framebufferInfo.setRenderPass(renderPass)
			.setAttachmentCount(attachments.size())
			.setPAttachments(attachments.data())
			.setWidth(swapChainExtent.width)
			.setHeight(swapChainExtent.height)
			.setLayers(1);

		if (device.createFramebuffer(&framebufferInfo, nullptr, &swapChainFramebuffers[i]) != vk::Result::eSuccess)
		{
			throw std::runtime_error("failed to create frame buffer");
		}
	}
}

void VulkanRender::createCommandPool()
{
	vk::CommandPoolCreateInfo poolInfo;
	poolInfo.setQueueFamilyIndex(availableQueueFamilyIndex[0]);
	poolInfo.setFlags(vk::CommandPoolCreateFlagBits::eResetCommandBuffer);
	if (device.createCommandPool(&poolInfo, nullptr, &commandPool) != vk::Result::eSuccess)
	{
		throw std::runtime_error("failed to create command pool");
	}
}

void VulkanRender::createPresentCommandBuffer()
{
	presentCommandBuffers.resize(MAX_FRAMES_IN_FLIGHT);

	vk::CommandBufferAllocateInfo allocateInfo;
	allocateInfo.setCommandPool(commandPool)
		.setLevel(vk::CommandBufferLevel::ePrimary)
		.setCommandBufferCount(presentCommandBuffers.size());

	if (device.allocateCommandBuffers(&allocateInfo, presentCommandBuffers.data()) != vk::Result::eSuccess)
	{
		throw std::runtime_error("failed to allocate command buffers");
	}


}

void VulkanRender::createTextureImage()
{
	int texWidth, texHeight, texChannels;

	stbi_uc* pixels = stbi_load(TEXTURE_PATH.c_str(), &texWidth, &texHeight, &texChannels, STBI_rgb_alpha);

	if (!pixels) {
		throw std::runtime_error("failed to load texture image!");
	}

	mipLevels = static_cast<uint32_t>(std::floor(std::log2(std::max(texWidth, texHeight)))) + 1;

	vk::DeviceSize imageSize = texWidth * texHeight * 4;
	vk::Buffer stagingBuffer;
	vk::DeviceMemory stagingBufferMemory;
	createBuffer(imageSize, vk::BufferUsageFlagBits::eTransferSrc,
		vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent,
		stagingBuffer, stagingBufferMemory);

	void* data;
	device.mapMemory(stagingBufferMemory, 0, imageSize,
		vk::MemoryMapFlagBits(0), &data);
	memcpy(data, pixels, static_cast<size_t>(imageSize));
	device.unmapMemory(stagingBufferMemory);
	stbi_image_free(pixels);

	createImage(texWidth, texHeight, mipLevels, vk::SampleCountFlagBits::e1,
		vk::Format::eR8G8B8A8Srgb,
		vk::ImageTiling::eOptimal,
		vk::ImageUsageFlagBits::eTransferDst | vk::ImageUsageFlagBits::eTransferSrc
		| vk::ImageUsageFlagBits::eSampled,
		vk::MemoryPropertyFlagBits::eDeviceLocal, textureImage, textureImageMemory);

	transitionImageLayout(textureImage, vk::Format::eR8G8B8A8Srgb,
		vk::ImageLayout::eUndefined,
		vk::ImageLayout::eTransferDstOptimal, mipLevels);
	copyBufferToImage(stagingBuffer, textureImage, static_cast<uint32_t>(texWidth),
		static_cast<uint32_t>(texHeight));

	/*transitionImageLayout(textureImage, vk::Format::eR8G8B8A8Srgb,
		vk::ImageLayout::eTransferDstOptimal,
		vk::ImageLayout::eShaderReadOnlyOptimal, mipLevels);*/

	generateMipmaps(textureImage, vk::Format::eR8G8B8A8Srgb, texWidth, texHeight, mipLevels);

	device.destroyBuffer(stagingBuffer);
	device.freeMemory(stagingBufferMemory);


}

void VulkanRender::createVertexBuffer()
{
	const	vk::DeviceSize bufferSize(sizeof(vertices[0]) * vertices.size());



	vk::Buffer stagingBuffer;
	vk::DeviceMemory stagingBufferMemory;
	createBuffer(bufferSize, vk::BufferUsageFlagBits::eTransferSrc,
		vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent,
		stagingBuffer, stagingBufferMemory);

	void* data;
	device.mapMemory(stagingBufferMemory, 0, bufferSize,
		vk::MemoryMapFlagBits(0), &data);
	memcpy(data, vertices.data(), (size_t)bufferSize);
	device.unmapMemory(stagingBufferMemory);



	createBuffer(bufferSize,
		vk::BufferUsageFlagBits::eVertexBuffer | vk::BufferUsageFlagBits::eTransferDst,
		vk::MemoryPropertyFlagBits::eDeviceLocal,
		vertexBuffer, vertexBufferMemory);

	copyBuffer(stagingBuffer, vertexBuffer, bufferSize);
	device.destroyBuffer(stagingBuffer);
	device.freeMemory(stagingBufferMemory);


}

void VulkanRender::createIndexBuffer()
{
	const	vk::DeviceSize bufferSize(sizeof(indices[0]) * indices.size());

	vk::Buffer stagingBuffer;
	vk::DeviceMemory stagingBufferMemory;
	createBuffer(bufferSize, vk::BufferUsageFlagBits::eTransferSrc,
		vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent,
		stagingBuffer, stagingBufferMemory);

	void* data;
	device.mapMemory(stagingBufferMemory, 0, bufferSize,
		vk::MemoryMapFlagBits(0), &data);
	memcpy(data, indices.data(), (size_t)bufferSize);
	device.unmapMemory(stagingBufferMemory);



	createBuffer(bufferSize,
		vk::BufferUsageFlagBits::eIndexBuffer | vk::BufferUsageFlagBits::eTransferDst,
		vk::MemoryPropertyFlagBits::eDeviceLocal,
		indexBuffer, indexBufferMemory);

	copyBuffer(stagingBuffer, indexBuffer, bufferSize);
	device.destroyBuffer(stagingBuffer);
	device.freeMemory(stagingBufferMemory);
}

void VulkanRender::createColorResources()
{
	auto format = swapChainImageFormat;

	createImage(swapChainExtent.width, swapChainExtent.height,
		1, msaaSamples, format
		, vk::ImageTiling::eOptimal
		, vk::ImageUsageFlagBits::eTransientAttachment | vk::ImageUsageFlagBits::eColorAttachment
		, vk::MemoryPropertyFlagBits::eDeviceLocal,
		colorImage, colorImageMemory);

	colorImageView = createImageView(colorImage, format, vk::ImageAspectFlagBits::eColor, 1);

	transitionImageLayout(colorImage, format, vk::ImageLayout::eUndefined, vk::ImageLayout::eColorAttachmentOptimal, 1);
}

void VulkanRender::createUniformBuffers()
{
	vk::DeviceSize size = sizeof(UniformBufferObject);

	uniformBuffers.resize(MAX_FRAMES_IN_FLIGHT);
	uniformBuffersMemory.resize(MAX_FRAMES_IN_FLIGHT);
	for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
		createBuffer(size, vk::BufferUsageFlagBits::eUniformBuffer,
			vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent,
			uniformBuffers[i], uniformBuffersMemory[i]
		);
	}
}

void VulkanRender::createDescriptorPool()
{
	std::array< vk::DescriptorPoolSize, 2> poolSizes;
	poolSizes[0].setType(vk::DescriptorType::eUniformBuffer).setDescriptorCount(swapChainImages.size());
	poolSizes[1].setType(vk::DescriptorType::eCombinedImageSampler).setDescriptorCount(swapChainImages.size());

	vk::DescriptorPoolCreateInfo poolInfo;
	poolInfo.setPoolSizeCount(poolSizes.size())
		.setPPoolSizes(poolSizes.data())
		.setMaxSets(swapChainImages.size());
	vk::resultCheck(device.createDescriptorPool(&poolInfo, nullptr, &descriptorPool),
		"failed to create descriptor pool");

}

void VulkanRender::createDescriptorSets()
{

	std::vector<vk::DescriptorSetLayout> layouts(MAX_FRAMES_IN_FLIGHT, descriptorSetLayout);
	vk::DescriptorSetAllocateInfo allocInfo;

	allocInfo.setDescriptorPool(descriptorPool)
		.setDescriptorSetCount(swapChainImages.size())
		.setPSetLayouts(layouts.data());


	descriptorSets.resize(MAX_FRAMES_IN_FLIGHT);

	vk::resultCheck(device.allocateDescriptorSets(&allocInfo, descriptorSets.data())
		, "failed to allocate descriptor sets");

	for (size_t i = 0; i < swapChainImages.size(); i++) {
		vk::DescriptorBufferInfo bufferInfo;
		bufferInfo.setBuffer(uniformBuffers[i])
			.setOffset(0)
			.setRange(sizeof(UniformBufferObject));

		vk::DescriptorImageInfo imageInfo;
		imageInfo.setImageLayout(vk::ImageLayout::eShaderReadOnlyOptimal)
			.setSampler(textureSampler)
			.setImageView(textureImageView);




		std::array< vk::WriteDescriptorSet, 2> descriptorWrites;
		descriptorWrites[0].setDstSet(descriptorSets[i])
			.setDstBinding(0)
			.setDstArrayElement(0)
			.setDescriptorCount(1)
			.setDescriptorType(vk::DescriptorType::eUniformBuffer)
			.setPBufferInfo(&bufferInfo)
			.setPImageInfo(nullptr)// Optional
			.setPTexelBufferView(nullptr);// Optional


		descriptorWrites[1].setDstSet(descriptorSets[i])
			.setDstBinding(1)
			.setDstArrayElement(0)
			.setDescriptorCount(1)
			.setDescriptorType(vk::DescriptorType::eCombinedImageSampler)
			.setPBufferInfo(nullptr)// Optional
			.setPImageInfo(&imageInfo)
			.setPTexelBufferView(nullptr);// Optional

		device.updateDescriptorSets(descriptorWrites.size(), descriptorWrites.data(), 0, nullptr);

	}


}

void VulkanRender::createSyncObjects()
{
	imageAvailableSemaphores.resize(MAX_FRAMES_IN_FLIGHT);
	renderFinishedSemaphores.resize(MAX_FRAMES_IN_FLIGHT);
	inFlightFences.resize(MAX_FRAMES_IN_FLIGHT);
	vk::SemaphoreCreateInfo  semaphoreInfo;
	vk::FenceCreateInfo fenceInfo;
	fenceInfo.setFlags(vk::FenceCreateFlagBits::eSignaled);
	for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {

		if (device.createSemaphore(&semaphoreInfo, nullptr, &imageAvailableSemaphores[i]) != vk::Result::eSuccess
			|| device.createSemaphore(&semaphoreInfo, nullptr, &renderFinishedSemaphores[i]) != vk::Result::eSuccess
			|| device.createFence(&fenceInfo, nullptr, &inFlightFences[i]) != vk::Result::eSuccess)
		{
			throw std::runtime_error("failed to create semaphore");
		}


	}
}

void VulkanRender::loadModel()
{
	ModelLoader loader;
	loader.loadModel(MODEL_PATH, vertices, indices);


}

void VulkanRender::createDescriptorSetLayout()
{
	vk::DescriptorSetLayoutBinding uboLayoutBinding;
	uboLayoutBinding.setBinding(0)
		.setDescriptorCount(1)
		.setDescriptorType(vk::DescriptorType::eUniformBuffer)
		.setStageFlags(vk::ShaderStageFlagBits::eVertex)
		.setPImmutableSamplers(nullptr);// Optional




	/*VkDescriptorSetLayoutBinding samplerLayoutBinding
	{
		.binding = 1 写的要是C风格就好了
	};*/

	vk::DescriptorSetLayoutBinding samplerLayoutBinding;
	samplerLayoutBinding.setBinding(1)
		.setDescriptorCount(1)
		.setDescriptorType(vk::DescriptorType::eCombinedImageSampler)
		.setPImmutableSamplers(nullptr)
		.setStageFlags(vk::ShaderStageFlagBits::eFragment);

	std::array<vk::DescriptorSetLayoutBinding, 2>bindings = {
		uboLayoutBinding,samplerLayoutBinding
	};

	vk::DescriptorSetLayoutCreateInfo layoutInfo;
	layoutInfo.setBindingCount(bindings.size())
		.setPBindings(bindings.data());


	vk::resultCheck(device.createDescriptorSetLayout(&layoutInfo, nullptr, &descriptorSetLayout), "failed to create descriptor set layout");




}

void VulkanRender::createTextureImageView()
{
	textureImageView = createImageView(textureImage, vk::Format::eR8G8B8A8Srgb, vk::ImageAspectFlagBits::eColor, mipLevels);
}

void VulkanRender::createTextureSampler()
{
	vk::SamplerCreateInfo samplerInfo;
	samplerInfo.setMagFilter(vk::Filter::eLinear)
		.setMinFilter(vk::Filter::eLinear)
		.setAddressModeU(vk::SamplerAddressMode::eRepeat)
		.setAddressModeV(vk::SamplerAddressMode::eRepeat)
		.setAddressModeW(vk::SamplerAddressMode::eRepeat)
		.setAnisotropyEnable(VK_TRUE)//各向异性过滤
		.setMaxAnisotropy(16)
		.setBorderColor(vk::BorderColor::eIntOpaqueBlack)
		.setUnnormalizedCoordinates(VK_FALSE) //设定采样范围是否为width，height false为0-1 
		.setCompareEnable(VK_TRUE)//阴影贴图的时候有用
		.setCompareOp(vk::CompareOp::eAlways)
		.setMipmapMode(vk::SamplerMipmapMode::eLinear)
		.setMipLodBias(0.0f)
		.setMinLod(0.0f)
		.setMaxLod(static_cast<float>(mipLevels));

	vk::resultCheck(device.createSampler(&samplerInfo, nullptr, &textureSampler),
		"failed to create texture sampler");

}

void VulkanRender::createDepthResources()
{
	vk::Format depthFormat = findDepthFormat();
	createImage(swapChainExtent.width, swapChainExtent.height, 1, msaaSamples
		, depthFormat
		, vk::ImageTiling::eOptimal
		, vk::ImageUsageFlagBits::eDepthStencilAttachment, vk::MemoryPropertyFlagBits::eDeviceLocal,
		depthImage, depthImageMemory);

	depthImageView = createImageView(depthImage, depthFormat, vk::ImageAspectFlagBits::eDepth, 1);


	transitionImageLayout(depthImage, depthFormat, vk::ImageLayout::eUndefined,
		vk::ImageLayout::eDepthStencilAttachmentOptimal, 1);
}

vk::ImageView VulkanRender::createImageView(vk::Image image, vk::Format format, vk::ImageAspectFlags aspectFlags, uint32_t mipLevel)
{
	vk::ImageView imageView;

	vk::ImageSubresourceRange subresourceRange(aspectFlags, 0, mipLevel, 0, 1);

	vk::ImageViewCreateInfo viewInfo;
	viewInfo.setImage(image)
		.setViewType(vk::ImageViewType::e2D)
		.setFormat(format)
		.setSubresourceRange(subresourceRange);

	vk::resultCheck(device.createImageView(&viewInfo, nullptr, &imageView),
		"failed to create texture image view");
	return  imageView;
}


void VulkanRender::createBuffer(vk::DeviceSize size, vk::BufferUsageFlags usage, vk::MemoryPropertyFlags properties,
	vk::Buffer& buffer, vk::DeviceMemory& bufferMemory)
{
	vk::BufferCreateInfo bufferInfo;
	bufferInfo.setSize(size)
		.setUsage(usage)
		.setSharingMode(vk::SharingMode::eExclusive);

	vk::resultCheck(device.createBuffer(&bufferInfo, nullptr, &buffer), "failed to create buffer");

	vk::MemoryRequirements memRequirements;
	device.getBufferMemoryRequirements(buffer, &memRequirements);
	vk::MemoryAllocateInfo allocInfo;
	allocInfo.setAllocationSize(memRequirements.size)
		.setMemoryTypeIndex(findMemoryType(memRequirements.memoryTypeBits, properties));

	vk::resultCheck(device.allocateMemory(&allocInfo, nullptr, &bufferMemory), "failed to allocate buffer memory");

	device.bindBufferMemory(buffer, bufferMemory, 0);

}



void VulkanRender::createImage(uint32_t width, uint32_t height, uint32_t mipLevel, vk::SampleCountFlagBits numSamples,
	vk::Format format, vk::ImageTiling tiling, vk::ImageUsageFlags usage, vk::MemoryPropertyFlags properties, vk::Image& image, vk::DeviceMemory& imageMemory)
{
	vk::ImageCreateInfo imageInfo;
	imageInfo.setImageType(vk::ImageType::e2D)
		.setExtent(vk::Extent3D(width, height, 1))
		.setFormat(format)
		.setTiling(tiling)
		.setUsage(usage)
		.setMipLevels(mipLevel)
		.setArrayLayers(1)
		.setInitialLayout(vk::ImageLayout::eUndefined)
		.setSharingMode(vk::SharingMode::eExclusive)
		.setSamples(numSamples)
		.setFlags(vk::ImageCreateFlags(0));

	vk::resultCheck(device.createImage(&imageInfo, nullptr, &image),
		"failed to create image");


	vk::MemoryRequirements memRequirements;
	device.getImageMemoryRequirements(image, &memRequirements);
	vk::MemoryAllocateInfo allocateInfo;
	allocateInfo.setAllocationSize(memRequirements.size)
		.setMemoryTypeIndex(findMemoryType(memRequirements.memoryTypeBits, properties));
	vk::resultCheck(device.allocateMemory(&allocateInfo, nullptr, &imageMemory),
		"failed to allocate image memory");
	device.bindImageMemory(image, imageMemory, 0);

}


/*
//用于将非device local的buffer传输至gpu local的内存
*/

void VulkanRender::copyBuffer(const vk::Buffer& srcBuffer, const vk::Buffer& dstBuffer, vk::DeviceSize size)
{

	auto transferCommandBuffer = beginSingleTimeCommands();

	vk::BufferCopy copyRegion;
	copyRegion.setSize(size)
		.setSrcOffset(0).setDstOffset(0);

	transferCommandBuffer.copyBuffer(srcBuffer, dstBuffer, 1, &copyRegion);

	endSingleTimeCommands(transferCommandBuffer);

}

void VulkanRender::transitionImageLayout(vk::Image image, vk::Format format, vk::ImageLayout oldLayout,
	vk::ImageLayout newLayout, uint32_t mipLevel)
{

	auto commandBuffer = beginSingleTimeCommands();

	vk::ImageSubresourceRange subresource;
	subresource.setBaseMipLevel(0)
		.setBaseArrayLayer(0)
		.setLevelCount(mipLevel)
		.setLayerCount(1);

	if (newLayout == vk::ImageLayout::eDepthStencilAttachmentOptimal)
	{
		subresource.setAspectMask(vk::ImageAspectFlagBits::eDepth);
		if (hasStencilComponent(format))
		{
			subresource.aspectMask |= vk::ImageAspectFlagBits::eStencil;
		}
	}
	else
	{
		subresource.setAspectMask(vk::ImageAspectFlagBits::eColor);
	}

	vk::ImageMemoryBarrier barrier;
	barrier.setOldLayout(oldLayout)
		.setNewLayout(newLayout)
		.setSrcQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED)
		.setDstQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED)
		.setImage(image)
		.setSubresourceRange(subresource);

	vk::PipelineStageFlags sourceStage;
	vk::PipelineStageFlags destinationStage;

	if (oldLayout == vk::ImageLayout::eUndefined && newLayout == vk::ImageLayout::eTransferDstOptimal)
	{
		barrier.setSrcAccessMask(vk::AccessFlags(0)).
			setDstAccessMask(vk::AccessFlagBits::eTransferWrite);

		sourceStage = vk::PipelineStageFlagBits::eTopOfPipe;
		destinationStage = vk::PipelineStageFlagBits::eTransfer;
	}
	else if (oldLayout == vk::ImageLayout::eTransferDstOptimal && newLayout == vk::ImageLayout::eShaderReadOnlyOptimal)
	{
		barrier.setSrcAccessMask(vk::AccessFlagBits::eTransferWrite).
			setDstAccessMask(vk::AccessFlagBits::eShaderRead);

		sourceStage = vk::PipelineStageFlagBits::eTransfer;
		destinationStage = vk::PipelineStageFlagBits::eFragmentShader;
	}
	else if (oldLayout == vk::ImageLayout::eUndefined && newLayout == vk::ImageLayout::eDepthStencilAttachmentOptimal)
	{
		barrier.setSrcAccessMask(vk::AccessFlags(0))
			.setDstAccessMask(vk::AccessFlagBits::eDepthStencilAttachmentRead
				| vk::AccessFlagBits::eDepthStencilAttachmentWrite);
		sourceStage = vk::PipelineStageFlagBits::eTopOfPipe;
		destinationStage = vk::PipelineStageFlagBits::eEarlyFragmentTests;
	}
	else if (oldLayout == vk::ImageLayout::eUndefined && newLayout == vk::ImageLayout::eColorAttachmentOptimal)
	{
		barrier.setSrcAccessMask(vk::AccessFlags(0))
			.setDstAccessMask(
				vk::AccessFlagBits::eColorAttachmentRead
				| vk::AccessFlagBits::eColorAttachmentWrite);
		sourceStage = vk::PipelineStageFlagBits::eTopOfPipe;
		destinationStage = vk::PipelineStageFlagBits::eColorAttachmentOutput;

	}
	else
	{
		throw std::invalid_argument("unsupported layout transition!");
	}



	commandBuffer.pipelineBarrier(sourceStage, destinationStage,
		vk::DependencyFlags(0), 0, nullptr,
		0, nullptr,
		1, &barrier);




	endSingleTimeCommands(commandBuffer);

}

void VulkanRender::copyBufferToImage(vk::Buffer buffer, vk::Image image, uint32_t width, uint32_t height)
{
	auto commandBuffer = beginSingleTimeCommands();

	vk::ImageSubresourceLayers subresourceLayers;
	subresourceLayers.setMipLevel(0)
		.setAspectMask(vk::ImageAspectFlagBits::eColor)
		.setBaseArrayLayer(0)
		.setLayerCount(1);

	vk::BufferImageCopy region;
	region.setBufferOffset(0)
		.setBufferRowLength(0)
		.setBufferImageHeight(0)
		.setImageSubresource(subresourceLayers)
		.setImageOffset({ 0,0,0 })
		.setImageExtent({ width,height,1 });

	commandBuffer.copyBufferToImage(buffer, image, vk::ImageLayout::eTransferDstOptimal, 1, &region);

	endSingleTimeCommands(commandBuffer);

}


void VulkanRender::generateMipmaps(vk::Image image, vk::Format imageFormat, int32_t texWidth, int32_t texHeight, uint32_t mipLevels)
{
	//通常不会再运行时动态生成mipmap 而是预先生成然后存储后读取


	vk::FormatProperties formatProperties;
	physicalDevice.getFormatProperties(imageFormat, &formatProperties);
	if (!(formatProperties.optimalTilingFeatures & vk::FormatFeatureFlagBits::eSampledImageFilterLinear)) {
		throw std::runtime_error("texture image format does not support linear blitting!");
	}

	vk::ImageSubresourceRange range;
	range.setAspectMask(vk::ImageAspectFlagBits::eColor)
		.setBaseArrayLayer(0)
		.setLayerCount(1)
		.setLevelCount(1)
		;

	auto commandBuffer = beginSingleTimeCommands();
	vk::ImageMemoryBarrier barrier{};
	barrier.setImage(image)
		.setSrcQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED)
		.setDstQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED)
		.setSubresourceRange(range);



	int32_t mipWidth = texWidth;
	int32_t mipHeight = texHeight;

	for (uint32_t i = 1; i < mipLevels; i++) {
		barrier.subresourceRange.baseMipLevel = i - 1;

		barrier.setOldLayout(vk::ImageLayout::eTransferDstOptimal)
			.setNewLayout(vk::ImageLayout::eTransferSrcOptimal)
			.setSrcAccessMask(vk::AccessFlagBits::eTransferWrite)
			.setDstAccessMask(vk::AccessFlagBits::eTransferRead);

		commandBuffer.pipelineBarrier(vk::PipelineStageFlagBits::eTransfer
			, vk::PipelineStageFlagBits::eTransfer, vk::DependencyFlags(0),
			0, nullptr, 0,
			nullptr, 1, &barrier);


		vk::ImageBlit blit = {};
		blit.srcOffsets[0] = vk::Offset3D{ 0, 0, 0 };
		blit.srcOffsets[1] = vk::Offset3D{ mipWidth, mipHeight, 1 };
		blit.srcSubresource.aspectMask = vk::ImageAspectFlagBits::eColor;
		blit.srcSubresource.mipLevel = i - 1;
		blit.srcSubresource.baseArrayLayer = 0;
		blit.srcSubresource.layerCount = 1;

		blit.dstOffsets[0] = vk::Offset3D{ 0, 0, 0 };
		blit.dstOffsets[1] = vk::Offset3D{ mipWidth > 1 ? mipWidth / 2 : 1, mipHeight > 1 ? mipHeight / 2 : 1, 1 };
		blit.dstSubresource.aspectMask = vk::ImageAspectFlagBits::eColor;
		blit.dstSubresource.mipLevel = i;
		blit.dstSubresource.baseArrayLayer = 0;
		blit.dstSubresource.layerCount = 1;
		commandBuffer.blitImage(image, vk::ImageLayout::eTransferSrcOptimal,
			image, vk::ImageLayout::eTransferDstOptimal, 1, &blit,
			vk::Filter::eLinear);

		barrier.setOldLayout(vk::ImageLayout::eTransferSrcOptimal)
			.setNewLayout(vk::ImageLayout::eShaderReadOnlyOptimal)
			.setSrcAccessMask(vk::AccessFlagBits::eTransferRead)
			.setDstAccessMask(vk::AccessFlagBits::eShaderRead);

		commandBuffer.pipelineBarrier(vk::PipelineStageFlagBits::eTransfer
			, vk::PipelineStageFlagBits::eFragmentShader, vk::DependencyFlags(0),
			0, nullptr, 0,
			nullptr, 1, &barrier);

		if (mipWidth > 1) mipWidth /= 2;
		if (mipHeight > 1) mipHeight /= 2;
	}

	barrier.subresourceRange.baseMipLevel = mipLevels - 1;
	barrier.setOldLayout(vk::ImageLayout::eTransferDstOptimal)
		.setNewLayout(vk::ImageLayout::eShaderReadOnlyOptimal)
		.setSrcAccessMask(vk::AccessFlagBits::eTransferWrite)
		.setDstAccessMask(vk::AccessFlagBits::eShaderRead);

	commandBuffer.pipelineBarrier(vk::PipelineStageFlagBits::eTransfer
		, vk::PipelineStageFlagBits::eFragmentShader, vk::DependencyFlags(0),
		0, nullptr, 0,
		nullptr, 1, &barrier);


	endSingleTimeCommands(commandBuffer);
}


vk::CommandBuffer VulkanRender::beginSingleTimeCommands()
{
	vk::CommandBufferAllocateInfo allocateInfo;
	allocateInfo.setLevel(vk::CommandBufferLevel::ePrimary)
		.setCommandPool(commandPool)
		.setCommandBufferCount(1);

	vk::CommandBuffer commandBuffer;
	device.allocateCommandBuffers(&allocateInfo, &commandBuffer);
	vk::CommandBufferBeginInfo beginInfo;
	beginInfo.setFlags(vk::CommandBufferUsageFlagBits::eOneTimeSubmit);
	commandBuffer.begin(&beginInfo);
	return  commandBuffer;
}

void VulkanRender::endSingleTimeCommands(vk::CommandBuffer commandBuffer)
{
	commandBuffer.end();
	vk::SubmitInfo submitInfo;
	submitInfo.setCommandBufferCount(1)
		.setPCommandBuffers(&commandBuffer);
	graphicQueue.submit(1, &submitInfo, VK_NULL_HANDLE);
	graphicQueue.waitIdle();
	device.freeCommandBuffers(commandPool, 1, &commandBuffer);

}

void VulkanRender::recordCommandBuffer(const vk::CommandBuffer& commandBuffer, uint32_t imageIndex)
{


	vk::CommandBufferBeginInfo beginInfo;
	beginInfo.setFlags(vk::CommandBufferUsageFlagBits::eSimultaneousUse);

	if (commandBuffer.begin(&beginInfo) != vk::Result::eSuccess)
	{
		throw std::runtime_error("failed to begin recording command buffer");
	}

	std::array<vk::ClearValue, 3>  clearValues = {};
	clearValues[0].color = vk::ClearColorValue(std::array<float, 4>({ 0.0f, 0.0f, 0.0f, 1.0f }));
	clearValues[1].depthStencil = vk::ClearDepthStencilValue(1.0f, 0);
	clearValues[2].color = vk::ClearColorValue(std::array<float, 4>({ 0.0f, 0.0f, 0.0f, 1.0f }));

	vk::RenderPassBeginInfo renderPassInfo;
	renderPassInfo.setRenderPass(renderPass)
		.setFramebuffer(swapChainFramebuffers[imageIndex])
		.setRenderArea(vk::Rect2D(vk::Offset2D(0, 0), swapChainExtent))
		.setClearValueCount(clearValues.size())
		.setPClearValues(clearValues.data());

	vk::Buffer vertexBuffers[] = { vertexBuffer };
	vk::DeviceSize offsets[] = { 0 };


	commandBuffer.beginRenderPass(&renderPassInfo, vk::SubpassContents::eInline);
	commandBuffer.bindPipeline(vk::PipelineBindPoint::eGraphics, graphicsPipeline);
	commandBuffer.bindDescriptorSets(vk::PipelineBindPoint::eGraphics, pipelineLayout, 0, 1, &descriptorSets[currentFrame], 0, nullptr);
	commandBuffer.bindVertexBuffers(0, 1, vertexBuffers, offsets);
	commandBuffer.bindIndexBuffer(indexBuffer, 0, vk::IndexType::eUint32);
	//commandBuffer.draw(static_cast<uint32_t>(vertices.size()), 1, 0, 0);
	commandBuffer.drawIndexed(static_cast<uint32_t>(indices.size()), 1, 0, 0, 0);
	commandBuffer.endRenderPass();
	commandBuffer.end();


}

void VulkanRender::recreateSwapChain()
{
	device.waitIdle();
	cleanupSwapChain();

	createSwapChain();
	createImageViews();
	createRenderPass();
	createGraphicsPipeline();
	createColorResources();
	createDepthResources();
	createFrameBuffers();
	createPresentCommandBuffer();
}

void VulkanRender::updateUniformBuffer(uint32_t currentImage)
{
	static  auto startTime = std::chrono::high_resolution_clock::now();
	auto currentTime = std::chrono::high_resolution_clock::now();
	float time = std::chrono::duration<float, std::chrono::seconds::period>(currentTime - startTime).count();

	UniformBufferObject ubo{};
	glm::mat4 baseRotate = glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
	//glm::mat4 baseRotate{ 1.0 };
	ubo.model = glm::rotate(baseRotate, time * glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f));
	ubo.view = glm::lookAt(glm::vec3(2.0f, 2.0f, 2.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f));
	ubo.proj = glm::perspective(glm::radians(45.0f), swapChainExtent.width / (float)swapChainExtent.height, 0.1f, 10.0f);
	ubo.proj[1][1] *= -1;

	void* data;
	device.mapMemory(uniformBuffersMemory[currentImage], 0, sizeof(ubo),
		vk::MemoryMapFlagBits(0), &data);
	memcpy(data, &ubo, sizeof(ubo));
	device.unmapMemory(uniformBuffersMemory[currentImage]);
}


VulkanRender::VulkanRender()
{
	deviceExtensions.push_back(VK_KHR_SWAPCHAIN_EXTENSION_NAME);
}

void VulkanRender::createInstance(const std::vector<const char*>& requiredExtension)
{
	checkValidationExtensionSupport();
	checkValidationLayerSupport();
	enableExtensions.insert(enableExtensions.end(), requiredExtension.begin(), requiredExtension.end());



#if defined(_DEBUG)
	logVulkanApiVersion();
	logExtensionSupport(extensions);
	logLayerSupport();
	enableLayers.push_back("VK_LAYER_KHRONOS_validation");
	enableExtensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
#endif




	// vk::ApplicationInfo allows the programmer to specifiy some basic information about the
 // program, which can be useful for layers and tools to provide more debug information.
	vk::ApplicationInfo appInfo = vk::ApplicationInfo()
		.setPApplicationName("Vulkan C++ Windowed Program Template")
		.setApplicationVersion(1)
		.setPEngineName("LunarG SDK")
		.setEngineVersion(1)
		.setApiVersion(VK_API_VERSION_1_0);

	// vk::InstanceCreateInfo is where the programmer specifies the layers and/or extensions that
	// are needed.
	vk::InstanceCreateInfo instInfo = vk::InstanceCreateInfo()
		.setFlags(vk::InstanceCreateFlags())
		.setPApplicationInfo(&appInfo)
		.setEnabledExtensionCount(static_cast<uint32_t>(enableExtensions.size()))
		.setPpEnabledExtensionNames(enableExtensions.data())
		.setEnabledLayerCount(static_cast<uint32_t>(enableLayers.size()))
		.setPpEnabledLayerNames(enableLayers.data());

	// Create the Vulkan instance.

	try {
		instance = vk::createInstance(instInfo);
	}
	catch (const std::exception& e) {
		std::cout << "Could not create a Vulkan instance: " << e.what() << std::endl;
	}

#if defined(_DEBUG)
	vk::DebugUtilsMessengerCreateInfoEXT debugUtilsCreateInfo = vk::DebugUtilsMessengerCreateInfoEXT();
	debugUtilsCreateInfo.setMessageSeverity(
		vk::DebugUtilsMessageSeverityFlagsEXT(
			VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT |
			VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT |
			VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT)
	);
	debugUtilsCreateInfo.setMessageType(
		vk::DebugUtilsMessageTypeFlagsEXT(
			VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT |
			VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT |
			VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT
		));
	debugUtilsCreateInfo.setPfnUserCallback(
		PFN_vkDebugUtilsMessengerCallbackEXT(DebugCallback)
	);
	debugUtilsCreateInfo.setPUserData(nullptr);
	if (CreateDebugUtilsMessengerEXT(reinterpret_cast<VkDebugUtilsMessengerCreateInfoEXT*> (&debugUtilsCreateInfo), nullptr, &debugCallback)
		!= VK_SUCCESS) {
		throw std::runtime_error("failed to set up debug callback");

	}



#endif
	// Create a Vulkan surface for rendering


}

void VulkanRender::initPhysicalDevice()
{
	uint32_t deviceCount = 0;
	instance.enumeratePhysicalDevices(&deviceCount, nullptr);
	std::vector<vk::PhysicalDevice> devices;
	devices.resize(deviceCount);
	instance.enumeratePhysicalDevices(&deviceCount, devices.data());


	if (deviceCount == 0) {
		std::cout << "device not found" << std::endl;
		return;
	}

#if defined(_DEBUG)
	for (auto& d : devices)
	{
		logPhysicalDeviceProperties(d);
	}
#endif



	for (auto& d : devices)
	{
		auto properties = d.getProperties();
		if (properties.deviceType == vk::PhysicalDeviceType::eDiscreteGpu)
		{
			physicalDevice = d;
			msaaSamples = getMaxUsableSampleCount();
			return;
		}
	}
	physicalDevice = devices[0];
	msaaSamples = getMaxUsableSampleCount();
}

void VulkanRender::createLogicalDevice()
{
	auto grahicsQueueFamilyIndex = findQueueFamilies(vk::QueueFlagBits::eGraphics)[0];
	//确认物理设备具有VK_QUEUE_GRAPHICS_BIT队列族
	float queuePriority = 1.0;//队列优先级

	vk::DeviceQueueCreateInfo queueCreateInfo = {};
	queueCreateInfo.setQueueFamilyIndex(grahicsQueueFamilyIndex);
	queueCreateInfo.setQueueCount(1);
	queueCreateInfo.setPQueuePriorities(&queuePriority);


	vk::PhysicalDeviceFeatures features;
	features.setSamplerAnisotropy(VK_TRUE)
		.setSampleRateShading(VK_TRUE);

	vk::DeviceCreateInfo createInfo;
	createInfo.setPQueueCreateInfos(&queueCreateInfo);
	createInfo.setQueueCreateInfoCount(1);
	createInfo.setPEnabledFeatures(&features);
	createInfo.setEnabledExtensionCount(static_cast<uint32_t>(deviceExtensions.size()));
	createInfo.setPpEnabledExtensionNames(deviceExtensions.data());

#if defined(_DEBUG)
	createInfo.enabledLayerCount = static_cast<uint32_t>(enableLayers.size());
	createInfo.ppEnabledLayerNames = enableLayers.data();
#else
	createInfo.setEnabledLayerCount(0);
#endif // (_DEBUG)

	if (physicalDevice.createDevice(&createInfo, nullptr, &device) != vk::Result::eSuccess) {
		std::cout << "failed to create logical device" << std::endl;
		return;
	}
	device.getQueue(grahicsQueueFamilyIndex, 0, &graphicQueue);

	//偷懒代码
	bool isPresentSupport = checkPresentSupport(grahicsQueueFamilyIndex);
	if (isPresentSupport) {
		device.getQueue(grahicsQueueFamilyIndex, 0, &presentQueue);
	}
	return;
}




void VulkanRender::cleanupSwapChain()
{
	for (auto& framebuffer : swapChainFramebuffers)
	{
		device.destroyFramebuffer(framebuffer, nullptr);
	}



	device.destroyImageView(depthImageView);
	device.destroyImage(depthImage);
	device.freeMemory(depthImageMemory);

	device.destroyImageView(colorImageView);
	device.destroyImage(colorImage);
	device.freeMemory(colorImageMemory);

	device.freeCommandBuffers(commandPool, static_cast <uint32_t>(presentCommandBuffers.size()), presentCommandBuffers.data());

	device.destroyPipeline(graphicsPipeline, nullptr);
	device.destroyPipelineLayout(pipelineLayout, nullptr);
	device.destroyRenderPass(renderPass, nullptr);

	for (const auto& imageView : swapChainImageViews)
	{
		device.destroyImageView(imageView, nullptr);
	}
	device.destroySwapchainKHR(swapChain);
}

void VulkanRender::destroy()
{
#if defined(_DEBUG)
	DestroyDebugUtilsMessengerEXT(instance, debugCallback, nullptr);
#endif // (_DEBUG)
	for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
		device.destroySemaphore(imageAvailableSemaphores[i]);
		device.destroySemaphore(renderFinishedSemaphores[i]);
		device.destroyFence(inFlightFences[i]);
	}


	cleanupSwapChain();
	device.destroySampler(textureSampler);
	device.destroyImageView(textureImageView);
	device.destroyImage(textureImage);
	device.freeMemory(textureImageMemory);


	for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
		device.destroyBuffer(uniformBuffers[i]);
		device.freeMemory(uniformBuffersMemory[i]);

	}

	device.destroyDescriptorPool(descriptorPool);
	device.destroyDescriptorSetLayout(descriptorSetLayout);

	device.destroyBuffer(vertexBuffer);
	device.destroyBuffer(indexBuffer);
	device.freeMemory(vertexBufferMemory);
	device.freeMemory(indexBufferMemory);

	device.destroyCommandPool(commandPool, nullptr);
	device.destroy();
	instance.destroySurfaceKHR(surface);
	instance.destroy();
}

void VulkanRender::logExtensionSupport(std::vector<vk::ExtensionProperties> logExtensions)
{
	std::cout << "Log Extension Support:" << std::endl;
	for (const auto& extension : logExtensions)
	{
		std::cout << extension.extensionName << std::endl;

	}
	std::cout << "***********************" << std::endl;
}

void VulkanRender::logLayerSupport()
{
	std::cout << "Log Layer Support:" << std::endl;
	for (const auto& layer : layerSupport)
	{
		std::cout << layer.layerName << std::endl;

	}
	std::cout << "***********************" << std::endl;
}

void VulkanRender::logVulkanApiVersion()
{
	uint32_t version = 0;
	vk::enumerateInstanceVersion(&version);



	std::cout << "Vulkan version:"
		<< VK_API_VERSION_VARIANT(version) << "."
		<< VK_API_VERSION_MAJOR(version) << "."
		<< VK_API_VERSION_MINOR(version) << "."
		<< VK_API_VERSION_PATCH(version)
		<< std::endl;
	std::cout << "***********************" << std::endl;
}

void VulkanRender::logPhysicalDeviceProperties(vk::PhysicalDevice device)
{
	std::cout << "***********************" << std::endl;
	std::cout << "Log Physical Device Properties:" << std::endl;
	auto properties = device.getProperties();
	auto version = properties.apiVersion;
	std::cout << "device name:" << properties.deviceName << std::endl;
	std::cout << "device ID:" << properties.deviceID << std::endl;
	std::cout << "device type" << vk::to_string(properties.deviceType) << std::endl;
	std::cout << "GPU API Version:" <<
		VK_API_VERSION_VARIANT(version) << "."
		<< VK_API_VERSION_MAJOR(version) << "."
		<< VK_API_VERSION_MINOR(version) << "."
		<< VK_API_VERSION_PATCH(version)
		<< std::endl; //可以写成util
	std::cout << "***********************" << std::endl;


}

void VulkanRender::logSwapChainSupport()
{

	std::cout << "***********************" << std::endl;
	std::cout << "Log SwapChain Support:" << std::endl;
	std::cout << "formats:" << std::endl;
	for (auto format : swapChainSupportDetails.formats) {
		std::cout << vk::to_string(format.format) << "\t\t" << vk::to_string(format.colorSpace) << std::endl;
	}

	std::cout << std::endl << "presentMode:" << std::endl;
	for (auto presentMode : swapChainSupportDetails.presentModes) {

		std::cout << vk::to_string(presentMode) << std::endl;
	}
	std::cout << "***********************" << std::endl;
}

void VulkanRender::onWindowResize(uint32_t width, uint32_t height)
{
	framebufferResized = true;
}

vk::SurfaceFormatKHR VulkanRender::chooseSwapSurfaceFormat()
{
	auto availableFormats = swapChainSupportDetails.formats;
	if (availableFormats.size() == 1 && availableFormats[0].format == vk::Format::eUndefined) {
		return vk::SurfaceFormatKHR(vk::Format(VK_FORMAT_B8G8R8A8_UNORM), vk::ColorSpaceKHR(VK_COLOR_SPACE_SRGB_NONLINEAR_KHR));
	}
	for (const auto& availableFormat : availableFormats) {
		if (availableFormat.format == vk::Format::eB8G8R8A8Srgb
			&& availableFormat.colorSpace ==
			vk::ColorSpaceKHR::eVkColorspaceSrgbNonlinear) {
			return availableFormat;
		}
	}

	return vk::SurfaceFormatKHR();
}

vk::PresentModeKHR VulkanRender::chooseSwapPresentMode()
{
	vk::PresentModeKHR bestMode = vk::PresentModeKHR::eFifo;
	for (const auto& presentMode : swapChainSupportDetails.presentModes) {
		if (presentMode == vk::PresentModeKHR::eMailbox) {
			bestMode = vk::PresentModeKHR::eMailbox;
			break;
		}
		else if (presentMode == vk::PresentModeKHR::eImmediate) {
			bestMode = vk::PresentModeKHR::eImmediate;
		}
	}

	return bestMode;
}

vk::Extent2D VulkanRender::chooseSwapExtent()
{

	auto capabilities = swapChainSupportDetails.capabilities;
	/*if (capabilities.currentExtent.width != (std::numeric_limits<uint32_t>::max)())
	{
		return capabilities.currentExtent;
	}*/

	vk::Extent2D actualExtent(Config::WIDTH, Config::HEIGHT);
	//actualExtent.width = (std::max)(capabilities.minImageExtent.width,
	//	(std::min)(capabilities.maxImageExtent.width, actualExtent.width));
	//actualExtent.height = (std::max)(capabilities.minImageExtent.height,
	//	(std::min)(capabilities.maxImageExtent.height, actualExtent.height));
	return actualExtent;
}

vk::Format VulkanRender::findSupportFormant(const std::vector<vk::Format>& candidates, vk::ImageTiling tiling,
	vk::FormatFeatureFlags feature)
{
	for (vk::Format format : candidates)
	{
		vk::FormatProperties props;
		physicalDevice.getFormatProperties(format, &props);

		if (tiling == vk::ImageTiling::eLinear
			&& (props.linearTilingFeatures & feature) == feature)
		{
			return format;
		}
		else if (tiling == vk::ImageTiling::eOptimal
			&& (props.optimalTilingFeatures & feature) == feature)
		{
			return format;
		}

	}

	throw std::runtime_error("failed to find support format");

}

vk::Format VulkanRender::findDepthFormat()
{
	return findSupportFormant({ vk::Format::eD32Sfloat ,vk::Format::eD32SfloatS8Uint,vk::Format::eD24UnormS8Uint },
		vk::ImageTiling::eOptimal, vk::FormatFeatureFlagBits::eDepthStencilAttachment);
}

bool VulkanRender::hasStencilComponent(vk::Format format)
{
	return  format == vk::Format::eD32SfloatS8Uint || format == vk::Format::eD24UnormS8Uint;
}

vk::SampleCountFlagBits VulkanRender::getMaxUsableSampleCount()
{
	auto properties = physicalDevice.getProperties();

	vk::SampleCountFlags counts = properties.limits.framebufferColorSampleCounts & properties.limits.framebufferDepthSampleCounts;
	if (counts & vk::SampleCountFlagBits::e64) { return vk::SampleCountFlagBits::e64; }
	if (counts & vk::SampleCountFlagBits::e32) { return vk::SampleCountFlagBits::e32; }
	if (counts & vk::SampleCountFlagBits::e16) { return vk::SampleCountFlagBits::e16; }
	if (counts & vk::SampleCountFlagBits::e8) { return vk::SampleCountFlagBits::e8; }
	if (counts & vk::SampleCountFlagBits::e4) { return vk::SampleCountFlagBits::e4; }
	if (counts & vk::SampleCountFlagBits::e2) { return vk::SampleCountFlagBits::e2; }
	return vk::SampleCountFlagBits::e1;

}

void VulkanRender::checkValidationExtensionSupport()
{

	uint32_t extensionCount;
	vk::enumerateInstanceExtensionProperties(nullptr, &extensionCount, nullptr);
	extensions.resize(extensionCount);
	vk::enumerateInstanceExtensionProperties(nullptr, &extensionCount, extensions.data());


}

void VulkanRender::checkValidationLayerSupport()
{
	uint32_t layerCount;
	vk::enumerateInstanceLayerProperties(&layerCount, nullptr);
	layerSupport.resize(layerCount);
	vk::enumerateInstanceLayerProperties(&layerCount, layerSupport.data());
}

bool VulkanRender::checkPresentSupport(int i)
{
	VkBool32 presentSupport = false;
	physicalDevice.getSurfaceSupportKHR(i, surface, &presentSupport);
	return presentSupport;
}

void VulkanRender::checkDeviceExtension()
{
	uint32_t extensionCount;
	physicalDevice.enumerateDeviceExtensionProperties(nullptr, &extensionCount, nullptr);
	availableDeviceExtensions.resize(extensionCount);
	physicalDevice.enumerateDeviceExtensionProperties(nullptr, &extensionCount, availableDeviceExtensions.data());


}

void VulkanRender::checkSwapChainSupport()
{

	physicalDevice.getSurfaceCapabilitiesKHR(surface, &swapChainSupportDetails.capabilities);
	uint32_t formatCount;
	physicalDevice.getSurfaceFormatsKHR(surface, &formatCount, nullptr);
	if (formatCount != 0) {
		swapChainSupportDetails.formats.resize(formatCount);
		physicalDevice.getSurfaceFormatsKHR(surface, &formatCount,
			swapChainSupportDetails.formats.data());
	}
	uint32_t presentModeCount;
	physicalDevice.getSurfacePresentModesKHR(surface, &presentModeCount, nullptr);
	if (presentModeCount != 0) {
		swapChainSupportDetails.presentModes.resize(presentModeCount);
		physicalDevice.getSurfacePresentModesKHR(surface, &presentModeCount, swapChainSupportDetails.presentModes.data());
	}

}

uint32_t VulkanRender::findMemoryType(uint32_t typeFilter, vk::MemoryPropertyFlags properties)
{
	vk::PhysicalDeviceMemoryProperties memoryProperties;
	physicalDevice.getMemoryProperties(&memoryProperties);

	for (uint32_t i = 0; i < memoryProperties.memoryTypeCount; i++)
	{
		if (typeFilter & (1 << i)
			&& (memoryProperties.memoryTypes[i].propertyFlags & properties) == properties)
		{
			return  i;
		}
	}
	throw std::runtime_error("failed to find suitable memory type");

}

VkResult VulkanRender::CreateDebugUtilsMessengerEXT(const VkDebugUtilsMessengerCreateInfoEXT* pCreateInfo, const VkAllocationCallbacks* pAllocator, VkDebugUtilsMessengerEXT* pCallback)
{

	auto func = (PFN_vkCreateDebugUtilsMessengerEXT)vkGetInstanceProcAddr(instance, "vkCreateDebugUtilsMessengerEXT");
	if (func != nullptr) {
		return func(instance, pCreateInfo, pAllocator, pCallback);
	}
	return VK_ERROR_EXTENSION_NOT_PRESENT;
}




std::vector<uint32_t> VulkanRender::findQueueFamilies(vk::QueueFlagBits flag) {
	if (queueFamilies.size() < 1) {
		uint32_t queueFamilyCount = 0;
		physicalDevice.getQueueFamilyProperties(&queueFamilyCount, nullptr);
		queueFamilies.resize(queueFamilyCount);
		physicalDevice.getQueueFamilyProperties(&queueFamilyCount, queueFamilies.data());
	}

	int i = 0;
	for (const auto& queueFamily : queueFamilies) {
		if (queueFamily.queueFlags & flag) {
			availableQueueFamilyIndex.push_back(i);
		}
		i++;
	}

	return availableQueueFamilyIndex;
}


void VulkanRender::DestroyDebugUtilsMessengerEXT(VkInstance instance, VkDebugUtilsMessengerEXT callback, const VkAllocationCallbacks* pAllocator)
{
	auto func = (PFN_vkDestroyDebugUtilsMessengerEXT)
		vkGetInstanceProcAddr(instance,
			"vkDestroyDebugUtilsMessengerEXT");
	if (func != nullptr) {
		func(instance, callback, pAllocator);
	}
}

//画三角形1000
//深度缓冲+贴图1567
//模型加载1576