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

#include "acquisition_buffer.h"
#include "bunch_buffer.h"
#include "spectrogram.h"
#include "gsl_svd.h"

using namespace boost::program_options;
using namespace boost::posix_time;

int main(int ac, char** av) {
	unsigned int nb_acc = 10;
	bool pre_notch = false;
	std::string path = "";
	std::string output_file = "";
	std::bitset<16> bunch_mask(std::string("111111"));
	try {
		// parse command line
		options_description desc("Allowed options");
		desc.add_options()
		("help,h", "produce help message")
		("path,p", value<std::string>(), "path to the datas (default : \".\")")
		("nb-acc,n", value<unsigned int>(), "averaging in turn (default : 10)")
		("output-file,o", value<std::string>(), "output file (dump the values)")
		("bunch-mask,m", value<std::string>(), "bunch mask (default : 111111)")
		("pre-notch", "in case data was already notched")
		;
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
		if (vm.count("pre-notch")) {
			pre_notch = true;
			std::cout << "pre notch       : true" << std::endl;
		}
		if (vm.count("bunch-mask")) {
			bunch_mask = std::bitset<16>(vm["bunch-mask"].as<std::string>());
		}
		std::cout << "bunch mask      : " << bunch_mask << std::endl;
		{ // do stuff

		}
	} catch (std::exception& ex) {
		std::cerr << "exception (std) : " << ex.what() << std::endl;
		return -1;
	}
	return 0;
}