# a simple makefile for data check

UNAME := $(shell uname)

ifeq ($(UNAME), Darwin)
CC = clang
CXX = clang++
CFLAGS = -g -O0 \
	-I/opt/local/include \
	-I/usr/local/include \
	-I/opt/local/include/opencv2
LIBS = -L/opt/local/lib/ \
	-framework GLUT -framework Cocoa -framework OpenGL -framework SFML \
	-lboost_program_options -lboost_system -lboost_iostreams -lboost_filesystem \
	-lfftw3 -lfftw3f \
	-lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_video -lopencv_objdetect
endif

ifeq ($(UNAME), Linux)
CC = clang
CXX = clang++
CFLAGS = -g -O0 \
	-I/usr/local/include \
	-I/usr/include \
	-I/usr/include/opencv2
LIBS = -L/usr/local/lib -L/usr/local/lib64 -L/usr/lib64 \
	-lglut -lGL -lGLU \
	-lboost_program_options -lboost_system -lboost_iostreams -lboost_filesystem \
	-lfftw3f -lfftw3 \
	-lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_video -lopencv_objdetect
endif

ALL=data_check

all: $(ALL)

glut_win.o : glut_win.cpp glut_win.h
	$(CXX) -o glut_win.o -c glut_win.cpp $(CFLAGS)
acquisition_buffer.o : acquisition_buffer.cpp acquisition_buffer.h
	$(CXX) -o acquisition_buffer.o -c acquisition_buffer.cpp $(CFLAGS)
bunch_buffer.o : bunch_buffer.cpp bunch_buffer.h acquisition_buffer.h
	$(CXX) -o bunch_buffer.o -c bunch_buffer.cpp $(CFLAGS)
fftw_fft.o : fftw_fft.cpp fftw_fft.h
	$(CXX) -o fftw_fft.o -c fftw_fft.cpp $(CFLAGS)
win_data_check.o : win_data_check.cpp win_data_check.h glut_win.h
	$(CXX) -o win_data_check.o -c win_data_check.cpp $(CFLAGS)
spectrogram.o : spectrogram.cpp spectrogram.h bunch_buffer.h acquisition_buffer.h
	$(CXX) -o spectrogram.o -c spectrogram.cpp $(CFLAGS)
data_check.o : data_check.cpp fftw_fft.h acquisition_buffer.h bunch_buffer.h glut_win.h win_data_check.h
	$(CXX) -o data_check.o -c data_check.cpp $(CFLAGS)
cv_image.o : cv_image.cpp cv_image.h spectrogram.h acquisition_buffer.h bunch_buffer.h
	$(CXX) -o cv_image.o -c cv_image.cpp $(CFLAGS)

data_check : data_check.o glut_win.o acquisition_buffer.o bunch_buffer.o fftw_fft.o win_data_check.o spectrogram.o cv_image.o
	$(CXX) -o data_check data_check.o glut_win.o acquisition_buffer.o bunch_buffer.o fftw_fft.o win_data_check.o spectrogram.o cv_image.o $(LIBS)	

clean:
	rm -f $(ALL) *.o

