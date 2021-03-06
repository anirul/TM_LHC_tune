/*
 * Copyright (c) 2012, Frederic DUBOUCHET
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
 * THIS SOFTWARE IS PROVIDED BY Frederic DUBOUCHET ``AS IS'' AND ANY
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

#include <iostream>
#include <vector>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <limits>

#include "glut_win.h"
#include "acquisition_buffer.h"
#include "bunch_buffer.h"
#include "spectrogram.h"
#include "win_data_check.h"
#include "cv_image.h"

using namespace boost::program_options;
using namespace boost::posix_time;

class chk_cmd : public commands {
public :
	chk_cmd() {}
	virtual void operator()(bunch_buffer_f& bb, std::vector<float>& out) const {
		size_t new_size = (size_t)log2(bb.buffer_size());
		new_size = powl(2, (double)new_size);
		std::cout << std::endl;
		time_duration notch_time = bb.notch();
		std::cout << "notch time      : " << notch_time << std::endl;
		bb.resize(new_size);
		time_duration duration = bb.fft_multiple();
		std::cout << "fft time (FFTW) : " << duration << std::endl;
		time_duration amp_time = bb.amplitude();
		std::cout << "amplitude time  : " << amp_time << std::endl;
		time_duration acc_time = bb.accumulate(out);
		std::cout << "accumulate time : " << acc_time << std::endl;
		time_duration norm_time = bb.normalize(out);
		std::cout << "normalize time  : " << norm_time << std::endl;
	}
};

int main(int ac, char** av) {
	unsigned int dx = 1280;
	unsigned int dy = 768;
	unsigned int nb_acc = 10;
	int64_t start_time = 0; // 1st January 1970
	int64_t end_time = std::numeric_limits<int64_t>::max();
	bool enable_fullscreen;
	bool no_label = false;
	bool black_white = false;
   float min_tune = 0.0f;
   float max_tune = 0.5f;
	std::string path = "";
	std::string output_file = "";
	std::string input_file = "";
	std::string output_image = "";
   std::string output_matlab = "";
	std::bitset<16> bunch_mask(std::string("111111"));
	try {
		// parse command line
		options_description desc("Allowed options");
		desc.add_options
		()
		("help,h", "produce help message")
		("path,p", value<std::string>(), "path to the datas (default : \".\")")
		("nb-acc,n", value<unsigned int>(),	"averaging in turn (default : 10)")
		("fullscreen,f", "fullscreen")
		("output-file,o", value<std::string>(), "output file (dump the values)")
		("output-image,b", value<std::string>(), "output an image")
      ("output-matlab", value<std::string>(), "output a matlab file")
		("no-label", "disable label in images")
		("input-file,i", value<std::string>(), "input file (read from dump)")
		("bunch-mask,m", value<std::string>(), "bunch mask (default : 111111)")
		("pre-notch", "in case data was already notched")
		("black-white", "output picture in monochrome")
		("start-time", value<std::string>(), "start time in ns from epoch")
		("end-time", value<std::string>(), "end time in ns from epoch")
      ("min-tune", value<float>(), "minimum tune picture (default : 0.0)")
      ("max-tune", value<float>(), "maximum tune picture (default : 0.5)")
      ;
		variables_map vm;
		store(command_line_parser(ac, av).options(desc).run(), vm);
		if (vm.count("help")) {
			std::cout << desc << std::endl;
			return 1;
		}
      if (vm.count("min-tune")) {
         min_tune = vm["min-tune"].as<float>();
         std::cout << "min-tune        : " << min_tune << std::endl;
      }
      if (vm.count("max-tune")) {
         max_tune = vm["max-tune"].as<float>();
         std::cout << "max-tune        : " << max_tune << std::endl;
      }
		if (vm.count("path")) {
			path = vm["path"].as<std::string>();
			std::cout << "path            : " << path << std::endl;
		}
		if (vm.count("nb-acc")) {
			nb_acc = vm["nb-acc"].as<unsigned int>();
		}
		std::cout << "nb acc          : " << nb_acc << std::endl;
		if (vm.count("fullscreen")) {
			enable_fullscreen = true;
		} else {
			enable_fullscreen = false;
		}
		if (vm.count("output-file")) {
			output_file = vm["output-file"].as<std::string>();
			std::cout << "output file     : " << output_file << std::endl;
		}
		if (vm.count("input-file")) {
			input_file = vm["input-file"].as<std::string>();
			std::cout << "input file      : " << input_file << std::endl;
		}
		if (vm.count("output-image")) {
			output_image = vm["output-image"].as<std::string>();
			std::cout << "output image    : " << output_image << std::endl;
		}
      if (vm.count("output-matlab")) {
         output_matlab = vm["output-matlab"].as<std::string>();
         std::cout << "output matlab   : " << output_matlab << std::endl;
      }
		if (vm.count("no-label")) {
			no_label = true;
			std::cout << "no label        : true" << std::endl;
		}
		if (vm.count("bunch-mask")) {
			bunch_mask = std::bitset<16>(vm["bunch-mask"].as<std::string>());
		}
		std::cout << "bunch mask      : " << bunch_mask << std::endl;
		if (vm.count("black-white")) {
			black_white = true;
			std::cout << "black & white   : true" << std::endl;
		}
		if (vm.count("start-time")) {
			{
				std::string start_time_str = vm["start-time"].as<std::string>();
				start_time = boost::lexical_cast<long long>(start_time_str);
			}
			boost::posix_time::ptime ptime_time;
			{ // convert to time
				ptime_time = boost::posix_time::from_time_t(
						start_time / 1000000000L);
				ptime_time += boost::posix_time::microseconds(
						(start_time % 1000000000L) / 1000);
			}
			std::cout << "start time      : " << start_time << " ["
					<< ptime_time << "]" << std::endl;
		}
		if (vm.count("end-time")) {
			{
				std::string end_time_str = vm["end-time"].as<std::string>();
				end_time = boost::lexical_cast<long long>(end_time_str);
			}
			boost::posix_time::ptime ptime_time;
			{ // convert to time
				ptime_time = boost::posix_time::from_time_t(
						end_time / 1000000000L);
				ptime_time += boost::posix_time::microseconds(
						(end_time % 1000000000L) / 1000);
			}
			std::cout << "end time        : " << end_time << " [" << ptime_time
					<< "]" << std::endl;
		}
		{
			spectrogram spect(nb_acc, bunch_mask);
			if (path.size()) {
				chk_cmd cmd;
				fftwf_fft fft_instance;
				spect.load_files(path, cmd, &fft_instance, start_time, end_time);
			} else if (input_file.size()) {
				spect.load_dump(input_file);
			} else {
				throw std::runtime_error("invalid parameter list (see --help)");
			}
			if (output_file.size()) {
				spect.save_dump(output_file);
				return 0;
			}
			if (output_image.size()) {
				save_to_file(
                  spect, 
                  output_image, 
                  !no_label, 
                  black_white, 
                  min_tune, 
                  max_tune);
				return 0;
			}
         if (output_matlab.size()) {
            spect.save_matlab(output_matlab);
            return 0;
         }
			win_data_check wdc(std::make_pair(dx, dy), spect);
			glut_win* pwin = glut_win::instance(std::string("data check"),
					std::make_pair<unsigned int, unsigned int>(dx, dy), &wdc,
					enable_fullscreen);
			pwin->run();
		}
		// error handling
	} catch (std::exception& ex) {
		std::cerr << "exception (std) : " << ex.what() << std::endl;
		return -1;
	}
	return 0;
}
