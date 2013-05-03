# a simple makefile for data check generate the executable

UNAME := $(shell uname)

ifeq ($(UNAME), Darwin)
CC = clang
CXX = clang++
CFLAGS = -g -O0 \
	-I.. \
	-I/opt/local/include \
	-I/usr/local/include \
	-I/opt/local/include/opencv2
LIBS = -L/opt/local/lib/ \
	-framework GLUT -framework Cocoa -framework OpenGL -framework SFML -framework OpenCL \
	-lboost_program_options -lboost_system -lboost_iostreams -lboost_filesystem \
	-lfftw3_threads -lfftw3f_threads -lfftw3 -lfftw3f \
	-lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_video -lopencv_objdetect \
	-lgsl -lcblas
endif

ifeq ($(UNAME), Linux)
CC = gcc
CXX = g++
CFLAGS = -g -O0 \
	-I.. \
   -I$(HOME)/include \
	-I/usr/local/include \
	-I/usr/include \
	-I/usr/include/opencv2 \
	-I/usr/lib/x86_64-linux-gnu 
LIBS = \
   -L/usr/lib64/nvidia \
   -L/usr/local/lib \
   -L/usr/local/lib64 \
   -L/usr/lib64 \
	-lglut -lGL -lGLU -lOpenCL \
	-lboost_program_options -lboost_system -lboost_iostreams -lboost_filesystem \
	-lfftw3f -lfftw3 \
	-lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_video -lopencv_objdetect \
	-lgsl -lgslcblas -lm
endif

ALL=data_check data_svd data_clean data_gpu data_display

all: $(ALL)

glut_win.o : glut_win.cpp glut_win.h
	$(CXX) -o glut_win.o -c glut_win.cpp $(CFLAGS)
acquisition_buffer.o : acquisition_buffer.cpp acquisition_buffer.h i_fft.h
	$(CXX) -o acquisition_buffer.o -c acquisition_buffer.cpp $(CFLAGS)
bunch_buffer.o : bunch_buffer.cpp bunch_buffer.h acquisition_buffer.h gsl_svd.h
	$(CXX) -o bunch_buffer.o -c bunch_buffer.cpp $(CFLAGS)
fftw_fft.o : fftw_fft.cpp fftw_fft.h i_fft.h
	$(CXX) -o fftw_fft.o -c fftw_fft.cpp $(CFLAGS)
win_data_check.o : win_data_check.cpp win_data_check.h glut_win.h
	$(CXX) -o win_data_check.o -c win_data_check.cpp $(CFLAGS)
spectrogram.o : spectrogram.cpp spectrogram.h bunch_buffer.h acquisition_buffer.h
	$(CXX) -o spectrogram.o -c spectrogram.cpp $(CFLAGS)
cv_image.o : cv_image.cpp cv_image.h spectrogram.h acquisition_buffer.h bunch_buffer.h
	$(CXX) -o cv_image.o -c cv_image.cpp $(CFLAGS)
gsl_svd.o : gsl_svd.cpp gsl_svd.h
	$(CXX) -o gsl_svd.o -c gsl_svd.cpp $(CFLAGS)
cl_fft.o : cl_fft.h cl_fft.cpp
	$(CXX) -o cl_fft.o -c cl_fft.cpp $(CFLAGS)
cl_util.o : cl_util.h cl_util.cpp
	$(CXX) -o cl_util.o -c cl_util.cpp $(CFLAGS)

data_check.o : data_check.cpp fftw_fft.h acquisition_buffer.h bunch_buffer.h glut_win.h win_data_check.h
	$(CXX) -o data_check.o -c data_check.cpp $(CFLAGS)
data_svd.o : data_svd.cpp spectrogram.h bunch_buffer.h acquisition_buffer.h fftw_fft.h i_fft.h
	$(CXX) -o data_svd.o -c data_svd.cpp $(CFLAGS)
data_clean.o : data_clean.cpp fftw_fft.h acquisition_buffer.h bunch_buffer.h i_fft.h
	$(CXX) -o data_clean.o -c data_clean.cpp $(CFLAGS)
data_gpu.o : data_gpu.cpp cl_fft.h fftw_fft.h spectrogram.h bunch_buffer.h acquisition_buffer.h i_fft.h cl_util.h
	$(CXX) -o data_gpu.o -c data_gpu.cpp $(CFLAGS)
data_display.o : data_display.cpp fftw_fft.h spectrogram.h bunch_buffer.h acquisition_buffer.h i_fft.h
	$(CXX) -o data_display.o -c data_display.cpp $(CFLAGS)

data_check : data_check.o glut_win.o acquisition_buffer.o bunch_buffer.o fftw_fft.o win_data_check.o spectrogram.o cv_image.o gsl_svd.o
	$(CXX) -o data_check data_check.o glut_win.o acquisition_buffer.o bunch_buffer.o fftw_fft.o win_data_check.o spectrogram.o cv_image.o gsl_svd.o $(LIBS)
data_svd : data_svd.o acquisition_buffer.o bunch_buffer.o spectrogram.o gsl_svd.o fftw_fft.o
	$(CXX) -o data_svd data_svd.o acquisition_buffer.o bunch_buffer.o spectrogram.o gsl_svd.o fftw_fft.o $(LIBS)
data_gpu : data_gpu.o acquisition_buffer.o bunch_buffer.o spectrogram.o gsl_svd.o fftw_fft.o cl_fft.o cl_util.o
	$(CXX) -o data_gpu data_gpu.o acquisition_buffer.o bunch_buffer.o spectrogram.o gsl_svd.o fftw_fft.o cl_fft.o cl_util.o $(LIBS)
data_clean : data_clean.o acquisition_buffer.o bunch_buffer.o fftw_fft.o gsl_svd.o
	$(CXX) -o data_clean data_clean.o acquisition_buffer.o bunch_buffer.o fftw_fft.o gsl_svd.o $(LIBS)
data_display : data_display.o acquisition_buffer.o bunch_buffer.o fftw_fft.o spectrogram.o cv_image.o gsl_svd.o
	$(CXX) -o data_display data_display.o glut_win.o acquisition_buffer.o bunch_buffer.o fftw_fft.o cl_fft.o cl_util.o spectrogram.o cv_image.o gsl_svd.o $(LIBS)
	
clean:
	rm -f $(ALL) *.o

