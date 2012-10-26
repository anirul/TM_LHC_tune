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

#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/stream.hpp>

#include "bunch_buffer.h"

bunch_buffer_f::bunch_buffer_f(
      const std::vector<short>& data,
      const unsigned long bunch_count)
{
   bunch_count_ = bunch_count;
   for (unsigned long i = 0; i < bunch_count_; ++i) {
      buffers_.push_back(acquisition_buffer_f(data, bunch_count_, i));
   }
}

bunch_buffer_d::bunch_buffer_d(
      const std::vector<short>& data,
      const unsigned long bunch_count)
{
   bunch_count_ = bunch_count;
   for (unsigned long i = 0; i < bunch_count_; ++i) {
      buffers_.push_back(acquisition_buffer_d(data, bunch_count_, i));
   }
}

bunch_buffer_f::~bunch_buffer_f() {}

bunch_buffer_d::~bunch_buffer_d() {}

bunch_buffer_f::bunch_buffer_f(const std::string& file_name) {
   bunch_count_ = 0;
   if (file_name.find(".gz") != std::string::npos) {
      load_gzip(file_name);
      return;
   }
   if (file_name.find(".xml") != std::string::npos) {
      load_txt(file_name);
      return;
   }
   if (file_name.find(".bin") != std::string::npos) {
      load_bin(file_name);
      return;
   }
}

bunch_buffer_d::bunch_buffer_d(const std::string& file_name) {
   bunch_count_ = 0;
   if (file_name.find(".gz") != std::string::npos) {
      load_gzip(file_name);
      return;
   }
   if (file_name.find(".xml") != std::string::npos) {
      load_txt(file_name);
      return;
   }
   if (file_name.find(".bin") != std::string::npos) {
      load_bin(file_name);
      return;
   }
}

std::vector<unsigned long> bunch_buffer_f::peak_detect(
      const unsigned long min,
      const unsigned long max) 
{
   std::vector<unsigned long> ret;
   for (unsigned long i = 0; i < bunch_count_; ++i)
      ret.push_back(buffers_[i].peak_detect(min, max));
   return ret;
}

std::vector<unsigned long> bunch_buffer_d::peak_detect(
      const unsigned long min,
      const unsigned long max)
{
   std::vector<unsigned long> ret;
   for (unsigned long i = 0; i < bunch_count_; ++i)
      ret.push_back(buffers_[i].peak_detect(min, max));
   return ret;
}

void bunch_buffer_f::buffer(
      const unsigned long index,
      std::vector<float>& out) const
{
   if (index >= bunch_count_)
      throw std::runtime_error("bunch_buffer_f::buffer := index out of band");
   buffers_[index].buffer_real(out);
}

void bunch_buffer_d::buffer(
      const unsigned long index,
      std::vector<double>& out) const
{
   if (index >= bunch_count_)
      throw std::runtime_error("bunch_buffer_d::buffer := index out of band");
   buffers_[index].buffer_real(out);
}

size_t bunch_buffer_f::bunch_count() const {
   return bunch_count_;
}

size_t bunch_buffer_d::bunch_count() const {
   return bunch_count_;
}

size_t bunch_buffer_f::buffer_size() const {
   if (buffers_.size())
      return buffers_[0].size();
   return 0;
}

size_t bunch_buffer_d::buffer_size() const {
   if (buffers_.size())
      return buffers_[0].size();
   return 0;
}

void bunch_buffer_f::notch() {
   for (unsigned long i = 0; i < bunch_count_; ++i)
      buffers_[i].notch();
}

void bunch_buffer_d::notch() {
   for (unsigned long i = 0; i < bunch_count_; ++i)
      buffers_[i].notch();
}

float bunch_buffer_f::check_rms() {
   float acc = 0.0f;
   for (unsigned long i = 0; i < bunch_count_; ++i)
      acc += buffers_[i].check_rms(); 
   return acc / (float)bunch_count_;
}

double bunch_buffer_d::check_rms() {
   double acc = 0.0;
   for (unsigned long i = 0; i < bunch_count_; ++i)
      acc += buffers_[i].check_rms();
   return acc / (double)bunch_count_;
}

void bunch_buffer_f::fft() {
   for (unsigned long i = 0; i < bunch_count_; ++i)
      buffers_[i].fft();
}

void bunch_buffer_d::fft() {
   for (unsigned long i = 0; i < bunch_count_; ++i)
      buffers_[i].fft();
}

void bunch_buffer_f::amplitude() {
   for (unsigned long i = 0; i < bunch_count_; ++i)
      buffers_[i].amplitude();
}

void bunch_buffer_d::amplitude() {
   for (unsigned long i = 0; i < bunch_count_; ++i)
      buffers_[i].amplitude();
}

void bunch_buffer_f::phase_deg() {
   for (unsigned long i = 0; i < bunch_count_; ++i)
      buffers_[i].phase_deg();
}

void bunch_buffer_d::phase_deg() {
   for (unsigned long i = 0; i < bunch_count_; ++i)
      buffers_[i].phase_deg();
}

void bunch_buffer_f::log10() {
   for (unsigned long i = 0; i < bunch_count_; ++i)
      buffers_[i].log10();
}

void bunch_buffer_d::log10() {
   for (unsigned long i = 0; i < bunch_count_; ++i)
      buffers_[i].log10();
}

void bunch_buffer_f::save_txt(
   std::ostream& os,
   const std::string& time_stamp,
   const std::string& bunch_pattern)
{
   os 
      << "<bunch_buffer_f time-stamp=\"" << time_stamp 
      << "\" bunch-pattern=\"" << bunch_pattern 
      << "\">" << std::endl;
   for (unsigned long i = 0; i < bunch_count_; ++i) {
      os << "\t<buffer" << i << ">" << std::endl;
      os << "\t\t"; 
      buffers_[i].save_txt(os);
      os << std::endl;
      os << "\t</buffer" << i << ">" << std::endl;
   }
   os
      << "</bunch_buffer_f>" << std::endl;
}

void bunch_buffer_d::save_txt(
   std::ostream& os,
   const std::string& time_stamp,
   const std::string& bunch_pattern)
{
   os 
      << "<bunch_buffer_d time-stamp=\"" << time_stamp 
      << "\" bunch-pattern=\"" << bunch_pattern 
      << "\">" << std::endl;
   for (unsigned long i = 0; i < bunch_count_; ++i) {
      os << "\t<buffer" << i << ">" << std::endl;
      os << "\t\t"; 
      buffers_[i].save_txt(os);
      os << std::endl;
      os << "\t</buffer" << i << ">" << std::endl;
   }
   os
      << "</bunch_buffer_d>" << std::endl;
}

bool bunch_buffer_f::save_txt(
      const std::string& file_name,
      const std::string& time_stamp,
      const std::string& bunch_pattern) 
{
   std::ofstream ofs(file_name.c_str());
   if (!ofs.is_open()) return false;
   save_txt(ofs, time_stamp, bunch_pattern);
   ofs.close();
   return true;
}

bool bunch_buffer_d::save_txt(
      const std::string& file_name, 
      const std::string& time_stamp,
      const std::string& bunch_pattern) 
{
   std::ofstream ofs(file_name.c_str());
   if (!ofs.is_open()) return false;
   save_txt(ofs, time_stamp, bunch_pattern);
   ofs.close();
   return true;
}

bool bunch_buffer_f::load_txt(const std::string& file_name) {
   std::ifstream ifs(file_name.c_str());
   if (!ifs.is_open()) return false;
   load_txt(ifs);
   ifs.close();
   return true;
}

bool bunch_buffer_d::load_txt(const std::string& file_name) {
   std::ifstream ifs(file_name.c_str());
   if (!ifs.is_open()) return false;
   load_txt(ifs);
   ifs.close();
   return true;
}

void bunch_buffer_f::load_txt(std::istream& is) {
   while (!is.eof()) {
      std::string item;
      std::getline(is, item);
      if (item.find("bunch-pattern=") != std::string::npos) {
         size_t pos = item.find("bunch-pattern=");
         std::string sub_item = item.substr(
            item.find_first_of("\"", pos) + 1,
            item.find_last_of("\"") - item.find_first_of("\"", pos) - 2);
         std::stringstream ss(sub_item);
         std::string token = "";
         while (!ss.eof()) {
            ss >> token;
            bunch_count_++;
         }
      }
      if (item.find("<buffer") != std::string::npos) {
         std::string values;
         std::getline(is, values);
         buffers_.push_back(acquisition_buffer_f(values));
      }
   }
}

void bunch_buffer_d::load_txt(std::istream& is) {
   while (!is.eof()) {
      std::string item;
      std::getline(is, item);
      if (item.find("bunch-pattern=") != std::string::npos) {
         size_t pos = item.find("bunch-pattern=");
         std::string sub_item = item.substr(
            item.find_first_of("\"", pos) + 1,
            item.find_last_of("\"") - item.find_first_of("\"", pos) - 2);
         std::stringstream ss(sub_item);
         std::string token = "";
         while (!ss.eof()) {
            ss >> token;
            bunch_count_++;
         }
      }
      if (item.find("<buffer") != std::string::npos) {
         std::string values;
         std::getline(is, values);
         buffers_.push_back(acquisition_buffer_d(values));
      }
   }
}

bool bunch_buffer_f::save_gzip(
      const std::string& file_name,
      const std::string& time_stamp,
      const std::string& bunch_pattern) 
{
   std::ofstream ofs(file_name.c_str(), std::ofstream::binary);
   if (!ofs.is_open()) return false;
   boost::iostreams::filtering_streambuf<boost::iostreams::input> in;
   in.push(boost::iostreams::gzip_compressor());
   std::stringstream data;
   save_txt(data, time_stamp, bunch_pattern);
   in.push(data);
   boost::iostreams::copy(in, ofs);
   ofs.close();
   return true;
}

bool bunch_buffer_d::save_gzip(
      const std::string& file_name,
      const std::string& time_stamp,
      const std::string& bunch_pattern) 
{
   std::ofstream ofs(file_name.c_str(), std::ofstream::binary);
   if (!ofs.is_open()) return false;
   boost::iostreams::filtering_streambuf<boost::iostreams::input> in;
   in.push(boost::iostreams::gzip_compressor());
   std::stringstream data;
   save_txt(data, time_stamp, bunch_pattern);
   in.push(data);
   boost::iostreams::copy(in, ofs);
   ofs.close();
   return true;
}

bool bunch_buffer_f::load_gzip(const std::string& file_name) {
   std::stringstream ss("");
   {
      std::ifstream ifs(file_name.c_str(), std::ios_base::in | std::ios_base::binary);
      if (!ifs.is_open()) return false;
      boost::iostreams::filtering_streambuf<boost::iostreams::input> in;
      in.push(boost::iostreams::gzip_decompressor());
      in.push(ifs);
      boost::iostreams::copy(in, ss);
      ifs.close();
   }
   ss.seekg(0, std::ios::beg);
   load_txt(ss);
   return true;
}

bool bunch_buffer_d::load_gzip(const std::string& file_name) {
   std::stringstream ss("");
   {
      std::ifstream ifs(file_name.c_str(), std::ios_base::in | std::ios_base::binary);
      if (!ifs.is_open()) return false;
      boost::iostreams::filtering_streambuf<boost::iostreams::input> in;
      in.push(boost::iostreams::gzip_decompressor());
      in.push(ifs);
      boost::iostreams::copy(in, ss);
      ifs.close();
   }
   ss.seekg(0, std::ios::beg);
   load_txt(ss);
   return true;
}

bool bunch_buffer_f::save_bin(
      const std::string& file_name, 
      long long time_stamp,
      const std::vector<short>& bunch_pattern) 
{
   FILE* file = fopen(file_name.c_str(), "wb");
   if (!file)
      return false;
   fwrite(&time_stamp, sizeof(long long), 1, file);
   unsigned long bunch_pattern_size = bunch_pattern.size();
   fwrite(&bunch_pattern_size, sizeof(unsigned long), 1, file);
   fwrite(&bunch_pattern[0], sizeof(short), bunch_pattern_size, file);
   for (unsigned long i = 0; i < bunch_count_; ++i)
      buffers_[i].save_bin(file);
   fclose(file);
   return true;
}

bool bunch_buffer_d::save_bin(
      const std::string& file_name, 
      long long time_stamp,
      const std::vector<short>& bunch_pattern) 
{
   FILE* file = fopen(file_name.c_str(), "wb");
   if (!file)
      return false;
   fwrite(&time_stamp, sizeof(long long), 1, file);
   unsigned long bunch_pattern_size = bunch_pattern.size();
   fwrite(&bunch_pattern_size, sizeof(unsigned long), 1, file);
   fwrite(&bunch_pattern[0], sizeof(short), bunch_pattern_size, file);
   for (unsigned long i = 0; i < bunch_count_; ++i)
      buffers_[i].save_bin(file);
   fclose(file);
   return true;
}

bool bunch_buffer_f::load_bin(const std::string& file_name) {
   throw std::runtime_error("Not implemented!");
}

bool bunch_buffer_d::load_bin(const std::string& file_name) {
   throw std::runtime_error("Not implemented!");
}

bool bunch_buffer_f::empty() const {
   for (int i = 0; i < buffers_.size(); ++i)
      if (!buffers_[i].empty())
         return false;
   return true;
}

bool bunch_buffer_d::empty() const {
   for (int i = 0; i < buffers_.size(); ++i)
      if (!buffers_[i].empty())
         return false;
   return true;
}
