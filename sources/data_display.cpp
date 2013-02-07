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

#include "acquisition_buffer.h"
#include "bunch_buffer.h"
#include "spectrogram.h"

using namespace boost::program_options;

int main(int ac, char** av) {
	int64_t start_time = 0;
	int64_t end_time = std::numeric_limits<int64_t>::max();
	std::string output_file = "";
	std::string input_file = "";
	unsigned int nb_acc = 10;
	std::bitset<16> bunch_mask(std::string("111111"));
	try {
		// parse command line
		options_description desc("Allowed options");
		desc.add_options
		()
		("help,h", "produce help message")
		("output-file,o", value<std::string>(), "output file (values as CSV)")
		("input-file,i", value<std::string>(), "input file (read from dump)");
		variables_map vm;
		store(command_line_parser(ac, av).options(desc).run(), vm);
		if (vm.count("help")) {
			std::cout << desc << std::endl;
			return 1;
		}
		if (vm.count("output-file")) {
			output_file = vm["output-file"].as<std::string>();
			std::cout << "output file     : " << output_file << std::endl;
		} else {
			throw std::runtime_error("Need an input file!");
		}
		if (vm.count("input-file")) {
			input_file = vm["input-file"].as<std::string>();
			std::cout << "input file      : " << input_file << std::endl;
		} else {
			throw std::runtime_error("Need an output file!");
		}
		spectrogram spect(nb_acc, bunch_mask);
		spect.load_dump(input_file);
		spect.save_csv(output_file);
	} catch (std::exception& ex) {
		std::cerr << "exception (std) : " << ex.what() << std::endl;
		return -1;
	}
	return 0;
}
