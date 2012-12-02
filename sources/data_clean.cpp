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

#include <stdexcept>
#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/program_options.hpp>

#include "bunch_buffer.h"

using namespace boost::program_options;
using namespace boost::posix_time;
namespace fs = boost::filesystem;

int main(int ac, char** av) {
	std::string path = "";
	try {
		// parse command line
		options_description desc("Allowed options");
		desc.add_options()("help,h", "produce help message")("path,p",
				value<std::string>(), "path to the datas (default : \".\")");
		variables_map vm;
		store(command_line_parser(ac, av).options(desc).run(), vm);
		if (vm.count("help")) {
			std::cout << desc << std::endl;
			return 1;
		}
		if (vm.count("path")) {
			path = vm["path"].as<std::string>();
			std::cout << "path            : " << path << std::endl;
		}
		fs::path open_path(path.c_str());
		std::vector<fs::path> list_file;
		if (fs::exists(open_path) && fs::is_directory(open_path)) {
			std::copy(
				fs::directory_iterator(open_path),
				fs::directory_iterator(),
				std::back_inserter(list_file));
			sort(list_file.begin(), list_file.end());
		} else {
			throw std::runtime_error(path + " is not a directory");
		}
		std::vector<fs::path>::iterator ite;
		for (ite = list_file.begin(); ite != list_file.end(); ++ite) {
			if (fs::is_directory(*ite)) continue;
			std::string full_path = fs::canonical(*ite).string();
			try {
				std::cout << "checking file   : " << full_path << std::endl;
				bunch_buffer_f bb(full_path, NULL);
				if (bb.empty()) {
					std::cout << "file empty      : " << full_path << std::endl;
					fs::remove(*ite);
				}
			} catch (std::exception& ex) {
				std::cerr << "exception (std) : " << ex.what() << std::endl;
			}
		}
	} catch (std::exception& ex) {
		std::cerr << "exception (std) : " << ex.what() << std::endl;
		return -1;
	}
	return 0;
}
