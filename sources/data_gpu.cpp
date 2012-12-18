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

#include <iostream>
#include <vector>
#include <bitset>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include "acquisition_buffer.h"
#include "bunch_buffer.h"
#include "spectrogram.h"
#include "cl_fft.h"
#include "cl_util.h"

using namespace boost::program_options;
using namespace boost::posix_time;

class gpu_cmd : public commands {
protected :
public :
	gpu_cmd() {}
	virtual void operator()(bunch_buffer_f& bb, std::vector<float>& out) const {
		out.resize(2048);
		bb.average();
		bb.resize(2048);
		time_duration duration = bb.fft_multiple();
		std::cout << "fft time (GPU)  : " << duration << std::endl;
		time_duration amp_time = bb.amplitude();
		std::cout << "amplitude time  : " << amp_time << std::endl;
		bb.clean(0, bb.buffer_size() / 20);
		bb.buffer(0, out);
		time_duration norm_time = bb.normalize(out);
		std::cout << "normalize time  : " << norm_time << std::endl;
	}
};

int main(int ac, char** av) {
	unsigned int nb_acc = 10;
	int64_t start_time = 0; // 1st January 1970
	int64_t end_time = std::numeric_limits<int64_t>::max();
	std::string path = "";
	std::string output_file = "";
	std::bitset<16> bunch_mask(std::string("111111"));
	try {
		// parse command line
		options_description desc("Allowed options");
		desc.add_options
		()
		("help,h", "produce help message")
		("path,p", value<std::string>(), "path to the datas (default : \".\")")
		("nb-acc,n", value<unsigned int>(),	"averaging in turn (default : 10)")
		("output-file,o", value<std::string>(), "output file (dump the values)")
		("bunch-mask,m", value<std::string>(), "bunch mask (default : 111111)")
		("start-time", value<std::string>(), "start time in ns from epoch")
		("end-time", value<std::string>(), "end time in ns from epoch");
		variables_map vm;
		store(command_line_parser(ac, av).options(desc).run(), vm);
		if (vm.count("help")) {
			std::cout << desc << std::endl;
			return 1;
		}
		if (vm.count("path")) {
			path = vm["path"].as<std::string>();
			std::cout << "path            : " << path << std::endl;
		} else {
			throw std::runtime_error("path needed (see --help)");
		}
		if (vm.count("nb-acc")) {
			nb_acc = vm["nb-acc"].as<unsigned int>();
		}
		std::cout << "nb acc          : " << nb_acc << std::endl;
		if (vm.count("output-file")) {
			output_file = vm["output-file"].as<std::string>();
			std::cout << "output file     : " << output_file << std::endl;
		} else {
			throw std::runtime_error("output file needed (see --help)");
		}
		if (vm.count("bunch-mask")) {
			bunch_mask = std::bitset<16>(vm["bunch-mask"].as<std::string>());
		}
		std::cout << "bunch mask      : " << bunch_mask << std::endl;
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
		{ // do stuff
			spectrogram spect(nb_acc, bunch_mask);
			if (!path.size() || !output_file.size()) {
				throw std::runtime_error("invalid parameter list (see --help)");
			}
			if (path.size()) {
				gpu_cmd cmd;
				cl_fft fft_instance;
				spect.load_files(path, cmd, &fft_instance, start_time, end_time);
			}
			if (output_file.size()) {
				spect.save_dump(output_file);
				return 0;
			}
		}
	} catch (cl::Error& err) {
		std::cerr << "exception (CL)  : (" << err << ") " << err.what() << std::endl;
		return -2;
	} catch (std::exception& ex) {
		std::cerr << "exception (std) : " << ex.what() << std::endl;
		return -1;
	}
	return 0;
}
