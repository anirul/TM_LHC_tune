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

#include "glut_win.h"
#include "acquisition_buffer.h"
#include "bunch_buffer.h"
#include "spectrogram.h"
#include "win_data_check.h"
#include "cv_image.h"

using namespace boost::program_options;
using namespace boost::posix_time;

int main(int ac, char** av) {
   unsigned int dx = 1280;
   unsigned int dy = 768;
   unsigned int nb_acc = 10;
   bool enable_fullscreen;
   bool pre_notch = false;
   bool no_label = false;
   bool black_white = false;
   std::string path = "";
   std::string output_file = "";
   std::string input_file = "";
   std::string output_image = "";
   std::bitset<16> bunch_mask(std::string("111111"));
   try {
      // parse command line
      options_description desc("Allowed options");
      desc.add_options()
         ("help,h", "produce help message")
         ("path,p", value<std::string>(), "path to the datas (default : \".\")")
         ("nb-acc,n", value<unsigned int>(), "averaging in turn (default : 10)")
         ("fullscreen,f", "fullscreen")
         ("output-file,o", value<std::string>(), "output file (dump the values)")
         ("output-image,b", value<std::string>(), "output an image")
         ("no-label", "disable label in images")
         ("input-file,i", value<std::string>(), "input file (read from dump)")
         ("bunch-mask,m", value<std::string>(), "bunch mask (default : 111111)")
         ("pre-notch", "in case data was already notched")
         ("black-white", "output picture in monochrome")
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
      if (vm.count("pre-notch")) {
         pre_notch = true;
         std::cout << "pre notch       : true" << std::endl;
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
      {
         spectrogram spect(nb_acc, bunch_mask);
         if (path.size()) {
            spect.load_files(path, pre_notch);
         } else if (input_file.size()) { 
            spect.load_dump(input_file);
         } else {
            std::cout << desc << std::endl;
            return 1;
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
               black_white);
            return 0;
         }
         win_data_check wdc(std::make_pair(dx, dy), spect);
         glut_win* pwin = glut_win::instance(
            std::string("data check"),
            std::make_pair<unsigned int, unsigned int>(dx, dy),
            &wdc,
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

