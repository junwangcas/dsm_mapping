#include "VRReader.h"

#include <iostream>
#include <fstream>
#include <unistd.h>
#include <dirent.h>

#include "opencv2/highgui.hpp"

namespace dsm
{
	VRReader::VRReader(const std::string &imageFolder, const std::string &timestampFile, bool reverse) :
		imagePath(imageFolder), timestampPath(timestampFile), id(0), inc(reverse ? -1 : 1)
	{}

	VRReader::~VRReader()
	{}

	bool VRReader::open()
	{
		bool readOK = this->readImageNames();

		if (readOK)
		{
			//sequence length in seconds
			double diff = this->timestamps.back() - this->timestamps.front();

			//fps
			this->fps_ = this->timestamps.size() / diff;

			// reset
			this->reset();

			std::cout << "YVRMav sequence found!" << std::endl;

			return true;
		}

		return false;
	}

	void VRReader::reset()
	{
		if (this->inc > 0) this->id = 0;
		else this->id = (int)this->files.size() - 1;
	}

	bool VRReader::isOpened() const
	{
		return (this->files.size() > 0);
	}

	bool VRReader::read(cv::Mat &img, double &timestamp)
	{
		if (this->id < this->files.size() && this->id >= 0)
		{
			img = cv::imread(this->files[this->id], cv::IMREAD_UNCHANGED);
			timestamp = this->timestamps[this->id];

			this->id += this->inc;

			return true;
		}

		return false;
	}

	double VRReader::fps() const
	{
		return this->fps_;
	}

	bool VRReader::readImageNames()
	{
		//clear all data
		this->timestamps.clear();
		this->files.clear();

		// read timestamps and images with names equal to timestamps
//		std::ifstream infile;
//		infile.open(this->timestampPath);
//		while (!infile.eof() && infile.good())
//		{
//			std::string line;
//			std::getline(infile, line);
//
//			if (!line.empty())
//			{
//				this->files.push_back(imagePath + "/" + line + ".png");
//				this->timestamps.push_back(std::atof(line.c_str()) / 1e9);		// transform to seconds
//			}
//		}

    DIR *dir;
    struct dirent *ptr;
    if ((dir = opendir(imagePath.c_str())) == nullptr) {
      std::cerr << "open image path failed!" << std::endl;
      return false;
    }
    while ((ptr = readdir(dir)) != nullptr) {
      if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0)
        continue;
      auto file_name = std::string(reinterpret_cast<char*>(ptr->d_name));
      this->files.push_back(imagePath + "/" + file_name);
      auto ts = file_name.substr(0, 13);
      this->timestamps.push_back(std::atof(ts.c_str()) / 1e9);
      std::cout << file_name << " **** " << ts << " **** " << std::to_string(std::atof(ts.c_str()) / 1e9) << std::endl;
    }
    closedir(dir);
    std::sort(this->files.begin(), this->files.end());
    std::sort(this->timestamps.begin(), this->timestamps.end());

		if (this->timestamps.size() > 0 && this->timestamps.size() == this->files.size())
		{
			return true;
		}
		else
		{
			this->timestamps.clear();
			this->files.clear();
		}

		return false;
	}
}