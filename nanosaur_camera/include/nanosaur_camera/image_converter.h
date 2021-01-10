#ifndef __ROS_DEEP_LEARNING_IMAGE_CONVERTER_H_
#define __ROS_DEEP_LEARNING_IMAGE_CONVERTER_H_

#include <jetson-utils/cudaUtility.h>
#include <jetson-utils/imageFormat.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <vision_msgs/msg/classification2_d.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/vision_info.hpp>

namespace vision_msgs
{
	typedef msg::Classification2D Classification2D;
	typedef msg::Detection2D		Detection2D;
	typedef msg::Detection2DArray Detection2DArray;
	typedef msg::ObjectHypothesis ObjectHypothesis;
	typedef msg::ObjectHypothesisWithPose ObjectHypothesisWithPose;
	typedef msg::VisionInfo 		VisionInfo;
}

namespace sensor_msgs
{
	typedef msg::Image Image;
	typedef msg::Image::SharedPtr ImagePtr;
	typedef msg::Image::ConstSharedPtr ImageConstPtr;
}

/**
 * GPU image conversion
 */
class imageConverter
{
public:
	/**
	 * Output image pixel type
	 */
	typedef float4 PixelType;

	/**
	 * Image format used for internal CUDA processing
	 */
	static const imageFormat InternalFormat = IMAGE_RGBA32F;

	/**
	 * Image format used for outputting ROS image messages
	 */
	static const imageFormat ROSOutputFormat = IMAGE_BGR8;

	/**
	 * Constructor
	 */
	imageConverter(rclcpp::Node* node);

	/**
	 * Destructor
	 */
	~imageConverter();

	/**
	 * Free the memory
	 */
	void Free();

	/**
	 * Convert to 32-bit RGBA floating point
	 */
	bool Convert( const sensor_msgs::ImageConstPtr& input );

	/**
	 * Convert to ROS sensor_msgs::Image message
	 */
	bool Convert( sensor_msgs::Image& msg_out, imageFormat outputFormat );

	/**
	 * Convert to ROS sensor_msgs::Image message
	 */
	bool Convert( sensor_msgs::Image& msg_out, imageFormat outputFormat, PixelType* imageGPU );

	/**
	 * Resize the memory (if necessary)
	 */
	bool Resize( uint32_t width, uint32_t height, imageFormat inputFormat );

	/**
	 * Retrieve the converted image width
	 */
	inline uint32_t GetWidth() const		{ return mWidth; }

	/**
	 * Retrieve the converted image height
	 */
	inline uint32_t GetHeight() const		{ return mHeight; }

	/**
	 * Retrieve the GPU pointer of the converted image
	 */
	inline PixelType* ImageGPU() const		{ return mOutputGPU; }

private:

	uint32_t mWidth;
	uint32_t mHeight;	
	size_t   mSizeInput;
	size_t   mSizeOutput;

	void* mInputCPU;
	void* mInputGPU;

	PixelType* mOutputCPU;
	PixelType* mOutputGPU;

	rclcpp::Node* node;
};

#endif