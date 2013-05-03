/*
 * Copyright (c) 2012, Frederic Dubouchet
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the CERN nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY Frederic Dubouchet ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Frederic DUBOUCHET BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <string>
#include <sstream>
#include <vector>
#include <algorithm>

#include <opencv.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "acquisition_buffer.h"
#include "bunch_buffer.h"
#include "spectrogram.h"
#include "cv_image.h"

std::vector<short> remove_multiple_occurence(const std::vector<short>& vec) {
	std::vector<short> already_in;
	for (int i = 0; i < vec.size(); ++i) {
		if (std::find(
			already_in.begin(), 
			already_in.end(), 
			vec[i]) == already_in.end())
		{
			already_in.push_back(vec[i]);
		}
	}
	return already_in;
}

void save_to_file(
	const spectrogram& spect,
	const std::string& file_name,
	bool with_scale,
	bool black_white,
   float min_tune,
   float max_tune) 
{
	std::pair<uint32_t, uint32_t> size(spect.pitch(), spect.line_count());
   std::pair<uint32_t, uint32_t> scale(
         (uint32_t)((min_tune * 2.0f) * (float)size.first),
         (uint32_t)((max_tune * 2.0f) * (float)size.first));
	cv::Mat img(size.second, scale.second - scale.first, CV_8UC(3));
	for (uint32_t y = 0; y < size.second; ++y) {
		for (uint32_t x = scale.first; x < scale.second; ++x) {
			if (!black_white) { // color
				uint8_t red = 0x0;
				uint8_t green = 0x0;
				uint8_t blue = 0x0;
				float val = spect.data()[(y * size.first) + x];
				if (val < 1.0f / 3.0f) { // black to blue
					val *= 3.0f;
					blue = (uint32_t)(255.0f * val);
				} else if (val < 2.0f / 3.0f) { // blue to green
					val -= 1.0f / 3.0f;
					val *= 3.0f;
					blue = (uint8_t)(255.0f);
					green = (uint8_t)(255.0f * val);
				} else  { // green to red
					val -= 2.0f / 3.0f;
					val *= 3.0f;
					blue = (uint8_t)(255.0f);
					green = (uint8_t)(255.0f);
					red = (uint8_t)(255.0f * val);
				}
				cv::Vec3b pixel(blue, green, red);
				img.at<cv::Vec3b>(y, x - scale.first) = pixel;
			} else { // black & white
				float val = spect.data()[(y * size.first) + x];
				cv::Vec3b pixel(
					(uint8_t)(val * 255.0f),
					(uint8_t)(val * 255.0f),
					(uint8_t)(val * 255.0f));
				img.at<cv::Vec3b>(y, x - scale.first) = pixel;
			}
		}
	}
	if (with_scale) {
/*		{ // bunch pattern
			std::stringstream ss("");
			if (spect.bunch_mask().count() > 1)
				ss << "bunches : ";
			else
				ss << "bunch : ";
			std::vector<short> bunch_pattern = 
				remove_multiple_occurence(spect.bunch_pattern());
			for (size_t i = 0; i < bunch_pattern.size(); ++i)
				if (spect.bunch_mask()[i])
					ss << bunch_pattern[i] << " ";
			cv::putText(
				img,
				ss.str(),
				cv::Point(30, 50),
				cv::FONT_HERSHEY_COMPLEX_SMALL, 0.9,
				cv::Scalar(255, 255, 255));
		}*/
		for (int y = 0; y < (size.second - 20); y += 100) {
			std::string time_string = "";
			long long time_stamp = spect.time(y);
			{ // convert to time
				std::stringstream ss("");
				boost::posix_time::ptime line_time;
				line_time = boost::posix_time::from_time_t(
					time_stamp / 1000000000L);
//				line_time += boost::posix_time::microseconds(
//					(time_stamp % 1000000000L) / 1000);
				ss << line_time;
				time_string = ss.str();
			}
			cv::line(
				img, 
				cv::Point(0, y), 
				cv::Point(4, y), 
				cv::Scalar(255,255,255));
			cv::putText(
				img, 
				time_string, 
				cv::Point(5, y + ((y) ? 5 : 15)), 
				cv::FONT_HERSHEY_COMPLEX_SMALL, 
            0.7, 
				cv::Scalar(255, 255, 255));
		}
      float tune = min_tune - 0.1;
		for (int i = (size.first / 5); i < size.first; i+= size.first / 5) {
         tune += 0.1f;
         int pos = i - scale.first;
         if (pos < 100) continue;
         if (pos + 30 > scale.second) continue;
			cv::line(
				img,
				cv::Point(pos, 0),
				cv::Point(pos, 4),
				cv::Scalar(255, 255, 255));
			std::stringstream ss("");
			ss << tune;
			cv::putText(
				img, 
				ss.str(),
				cv::Point(pos - 15, 20),
				cv::FONT_HERSHEY_COMPLEX_SMALL,
				0.7,
				cv::Scalar(255, 255, 255));
		}
	}
	cv::imwrite(file_name.c_str(), img);
}
