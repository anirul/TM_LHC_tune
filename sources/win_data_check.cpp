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

#include <stdexcept>
#include <string>
#include <vector>
#ifdef __linux__
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#endif
#ifdef __APPLE__
#include <glut/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#endif
#include <boost/date_time/posix_time/posix_time.hpp>

#include "acquisition_buffer.h"
#include "bunch_buffer.h"
#include "glut_win.h"
#include "spectrogram.h"
#include "win_data_check.h"

using namespace boost::posix_time;

win_data_check::win_data_check(
	const std::pair<unsigned int, unsigned int>& range,
	spectrogram const& spectrogram_ref) : 
		range_(range),
		texture_id_(0), 
		spectrogram_ref_(spectrogram_ref),
		line_(0)
{}

win_data_check::~win_data_check() {}

void win_data_check::init() {
	glClearColor(0, 0, 0, 0);
	gluOrtho2D(-1, 1, -1, 1);
	glGenTextures(1, &texture_id_);
}

void win_data_check::display() {
	glClear(GL_COLOR_BUFFER_BIT);
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, texture_id_);
	glPushMatrix();
		glBegin(GL_QUADS);
			glTexCoord2f(0, 1);
			glVertex2f(-1, 1);
			glTexCoord2f(1, 1);
			glVertex2f(1, 1);
			glTexCoord2f(1, 0);
			glVertex2f(1, -1);
			glTexCoord2f(0, 0);
			glVertex2f(-1, -1);
		glEnd();
	glPopMatrix();
	glDisable(GL_TEXTURE_2D);
	glFlush();
	glutPostRedisplay();
} 

void win_data_check::idle() {
	glFinish();
	glBindTexture(GL_TEXTURE_2D, texture_id_);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexImage2D(
			GL_TEXTURE_2D, 
			0, 
			1, 
			spectrogram_ref_.pitch(), 
			range_.second, 
			0, 
			GL_LUMINANCE, 
			GL_FLOAT, 
			spectrogram_ref_.line(line_, range_.second));
	glFinish();
} 

void win_data_check::reshape(int w, int h) {
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glViewport(0, 0, w, h);
	glMatrixMode(GL_MODELVIEW);
	glFinish();
}

void win_data_check::mouse_event(int button, int state, int x, int y) {
	if ((button == 3) || (button == 4)) // it's a wheel event
	{
		if (button == 3) {
			line_ -= 10; 
			if (line_ < 0) 
				line_ = 0;
		}
		if (button == 4) {
			line_ += 10;
			if (line_ > (spectrogram_ref_.line_count() - range_.second))
				line_ = spectrogram_ref_.line_count() - range_.second;
		}
	}
} 

void win_data_check::mouse_move(int x, int y) {} 

void win_data_check::special(int key, int x, int y) {
	if (key == GLUT_KEY_UP) {
		line_ -= 10; 
		if (line_ < 0) 
			line_ = 0;
	}
	if (key == GLUT_KEY_DOWN) {
		line_ += 10;
		if (line_ > (spectrogram_ref_.line_count() - range_.second))
			line_ = spectrogram_ref_.line_count() - range_.second;
	}
}

void win_data_check::keyboard(unsigned char key, int x, int y) {} 

void win_data_check::finish() {}
