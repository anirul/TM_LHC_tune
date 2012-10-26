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
#include <iostream>
#include <vector>
#include <stdexcept>
#include <png.h>
#include <stdint.h>

#include "texture_png.h"

texture_png::texture_png(const std::string& file_name) :
	byte_width_(1),
	bytes_per_pixel_(3),
	file_name_(file_name)
{}

texture_png::~texture_png() {}

void texture_png::set_size(const std::pair<size_t, size_t> & size) {
	size_ = size;
}

void texture_png::save(const std::vector<float>& ptr) {
	FILE* fp = fopen(file_name_.c_str(), "wb");
	if (!fp) 
		throw std::runtime_error("could not open file : " + file_name_);
	png_structp png_ptr = png_create_write_struct(
		PNG_LIBPNG_VER_STRING,
		NULL,
		NULL,
		NULL);
	if (!png_ptr) {
		fclose(fp);
		throw std::runtime_error("error writting PNG struct");
	} 
	png_infop info_ptr = png_create_info_struct(png_ptr);
	if (info_ptr == NULL) {
		png_destroy_write_struct(&png_ptr, NULL);
		fclose(fp);
		throw std::runtime_error("error getting PNG info struct");
	}
	if (setjmp(png_jmpbuf(png_ptr))) {
		png_destroy_write_struct(&png_ptr, &info_ptr);
		fclose(fp);
		throw std::runtime_error("error setting PNG error handling");
	}
	png_set_IHDR(
		png_ptr,
		info_ptr,
		size_.first,
		size_.second,
		8,
		PNG_COLOR_TYPE_RGB,
		PNG_INTERLACE_NONE,
		PNG_COMPRESSION_TYPE_DEFAULT,
		PNG_FILTER_TYPE_DEFAULT);
	png_uint_32 bytes_per_row = size_.first * bytes_per_pixel_;
	png_bytep* row_pointers = (png_bytep*)png_malloc(
		png_ptr, 
		size_.second * sizeof(png_bytep));
	for (int y = 0; y < size_.second; ++y) {
		uint8_t* row = (uint8_t*)png_malloc(
			png_ptr, 
			png_get_rowbytes(png_ptr, info_ptr));
		row_pointers[y] = (png_byte*)row;
		for (int x = 0; x < size_.first; ++x) {
			float pixel = ptr[y * size_.first + x];
			if (pixel < (1.0f / 4.0f)) { // Black -> Blue
				pixel *= 4.0f;
				*row++ = 0x00; // R
				*row++ = 0x00; // G
				*row++ = (uint8_t)(pixel * 255.0f); // B
			} else if (pixel < (2.0f / 4.0f)) { // Blue -> Green
				pixel -= 1.0f / 4.0f;
				pixel *= 4.0f;
				*row++ = 0x00; // R
				*row++ = (uint8_t)(pixel * 255.0f); // G
				*row++ = (uint8_t)((1.0f / pixel) * 255.0f); // B
			} else if (pixel < (3.0f / 4.0f)) { // Green -> Red
				pixel -= 2.0f / 4.0f;
				pixel *= 4.0f;
				*row++ = (uint8_t)(pixel * 255.0f); // R
				*row++ = (uint8_t)((1.0f / pixel) * 255.0f); // G
				*row++ = 0x00;
			} else { // Red -> White
				pixel -= 3.0f / 4.0f;
				pixel *= 4.0f;
				*row++ = 0xff; // R
				*row++ = (uint8_t)(pixel * 255.0f); // G
				*row++ = (uint8_t)(pixel * 255.0f); // B
			}
		}
	}
	// write the image
	png_init_io(png_ptr, fp);
	png_set_rows(png_ptr, info_ptr, row_pointers);
	png_write_png(png_ptr, info_ptr, PNG_TRANSFORM_IDENTITY, NULL);
	// cleanup
	for (int y = 0; y < size_.second; ++y) 
		png_free(png_ptr, row_pointers[y]);
	png_free(png_ptr, row_pointers);
	// finish writting
	png_destroy_write_struct(&png_ptr, &info_ptr);
	fclose(fp);
}
