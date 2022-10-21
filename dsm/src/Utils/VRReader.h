#pragma once

#include <string>
#include <vector>

#include "opencv2/core.hpp"

#include "FullSystem/DSMLib.h"

namespace dsm
{
	class DSM_EXPORTS_DLL VRReader
	{
	public:

		VRReader(const std::string &imageFolder, const std::string &timestampFile, bool reverse);
		~VRReader();

		// grab image files
		bool open();

		// resets sequence
		void reset();

		// check if successfully opened
		bool isOpened() const;

		// reads next image
		bool read(cv::Mat &img, double &timestamp);

		// sequence frames per second
		double fps() const;

	private:

		bool readImageNames();

	private:

		// input data
		const std::string imagePath;
		const std::string timestampPath;
		const int inc;

		// dataset data
		std::vector<std::string> files;
		std::vector<double> timestamps;
		double fps_;

		// image counter
		int id;	
	};
}