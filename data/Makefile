# a simple makefile to treat the data

UNAME := $(shell uname)

BINARY_DIR := ../binary/
PICTURES_DIR := ../pictures/

ifeq ($(UNAME), Darwin)
	DATA_CHECK = ../sources/data_check
endif

ifeq ($(UNAME), Linux)
	DATA_CHECK = ../sources/data_check
endif

ALL = \
	md-121011-vb1-m1-bunch000001-64acc.png \
	md-121011-vb1-m1-bunch000010-64acc.png \
	md-121011-vb1-m1-bunch000100-64acc.png \
	md-121011-vb1-m1-bunch001000-64acc.png \
	md-121011-vb1-m1-bunch010000-64acc.png \
	md-121011-vb1-m1-bunch100000-64acc.png \
	md-121011-vb2-m1-bunch000001-64acc.png \
	md-121011-vb2-m1-bunch000010-64acc.png \
	md-121011-vb2-m1-bunch000100-64acc.png \
	md-121011-vb2-m1-bunch001000-64acc.png \
	md-121011-vb2-m1-bunch010000-64acc.png \
	md-121011-vb2-m1-bunch100000-64acc.png \
	md-121011-hb1-m1-bunch000001-64acc.png \
	md-121011-hb1-m1-bunch000010-64acc.png \
	md-121011-hb1-m1-bunch000100-64acc.png \
	md-121011-hb1-m1-bunch001000-64acc.png \
	md-121011-hb1-m1-bunch010000-64acc.png \
	md-121011-hb1-m1-bunch100000-64acc.png \
	md-121011-hb2-m1-bunch000001-64acc.png \
	md-121011-hb2-m1-bunch000010-64acc.png \
	md-121011-hb2-m1-bunch000100-64acc.png \
	md-121011-hb2-m1-bunch001000-64acc.png \
	md-121011-hb2-m1-bunch010000-64acc.png \
	md-121011-hb2-m1-bunch100000-64acc.png

all: $(ALL)

# binary VB1

md-121011-vb1-m1-bunch000001-64acc.bin:
	$(DATA_CHECK) -p MD-121011/VB1/M1/ -o md-121011-vb1-m1-bunch000001-64acc.bin -n 64 -m 000001 --pre-notch
md-121011-vb1-m1-bunch000010-64acc.bin:
	$(DATA_CHECK) -p MD-121011/VB1/M1/ -o md-121011-vb1-m1-bunch000010-64acc.bin -n 64 -m 000010 --pre-notch
md-121011-vb1-m1-bunch000100-64acc.bin:
	$(DATA_CHECK) -p MD-121011/VB1/M1/ -o md-121011-vb1-m1-bunch000100-64acc.bin -n 64 -m 000100 --pre-notch
md-121011-vb1-m1-bunch001000-64acc.bin:
	$(DATA_CHECK) -p MD-121011/VB1/M1/ -o md-121011-vb1-m1-bunch001000-64acc.bin -n 64 -m 001000 --pre-notch
md-121011-vb1-m1-bunch010000-64acc.bin:
	$(DATA_CHECK) -p MD-121011/VB1/M1/ -o md-121011-vb1-m1-bunch010000-64acc.bin -n 64 -m 010000 --pre-notch
md-121011-vb1-m1-bunch100000-64acc.bin:
	$(DATA_CHECK) -p MD-121011/VB1/M1/ -o md-121011-vb1-m1-bunch100000-64acc.bin -n 64 -m 100000 --pre-notch

# binary VB2

md-121011-vb2-m1-bunch000001-64acc.bin:
	$(DATA_CHECK) -p MD-121011/VB2/M1/ -o md-121011-vb2-m1-bunch000001-64acc.bin -n 64 -m 000001 --pre-notch
md-121011-vb2-m1-bunch000010-64acc.bin:
	$(DATA_CHECK) -p MD-121011/VB2/M1/ -o md-121011-vb2-m1-bunch000010-64acc.bin -n 64 -m 000010 --pre-notch
md-121011-vb2-m1-bunch000100-64acc.bin:
	$(DATA_CHECK) -p MD-121011/VB2/M1/ -o md-121011-vb2-m1-bunch000100-64acc.bin -n 64 -m 000100 --pre-notch
md-121011-vb2-m1-bunch001000-64acc.bin:
	$(DATA_CHECK) -p MD-121011/VB2/M1/ -o md-121011-vb2-m1-bunch001000-64acc.bin -n 64 -m 001000 --pre-notch
md-121011-vb2-m1-bunch010000-64acc.bin:
	$(DATA_CHECK) -p MD-121011/VB2/M1/ -o md-121011-vb2-m1-bunch010000-64acc.bin -n 64 -m 010000 --pre-notch
md-121011-vb2-m1-bunch100000-64acc.bin:
	$(DATA_CHECK) -p MD-121011/VB2/M1/ -o md-121011-vb2-m1-bunch100000-64acc.bin -n 64 -m 100000 --pre-notch

# binary HB1

md-121011-hb1-m1-bunch000001-64acc.bin:
	$(DATA_CHECK) -p MD-121011/HB1/M1/ -o md-121011-hb1-m1-bunch000001-64acc.bin -n 64 -m 000001 --pre-notch
md-121011-hb1-m1-bunch000010-64acc.bin:
	$(DATA_CHECK) -p MD-121011/HB1/M1/ -o md-121011-hb1-m1-bunch000010-64acc.bin -n 64 -m 000010 --pre-notch
md-121011-hb1-m1-bunch000100-64acc.bin:
	$(DATA_CHECK) -p MD-121011/HB1/M1/ -o md-121011-hb1-m1-bunch000100-64acc.bin -n 64 -m 000100 --pre-notch
md-121011-hb1-m1-bunch001000-64acc.bin:
	$(DATA_CHECK) -p MD-121011/HB1/M1/ -o md-121011-hb1-m1-bunch001000-64acc.bin -n 64 -m 001000 --pre-notch
md-121011-hb1-m1-bunch010000-64acc.bin:
	$(DATA_CHECK) -p MD-121011/HB1/M1/ -o md-121011-hb1-m1-bunch010000-64acc.bin -n 64 -m 010000 --pre-notch
md-121011-hb1-m1-bunch100000-64acc.bin:
	$(DATA_CHECK) -p MD-121011/HB1/M1/ -o md-121011-hb1-m1-bunch100000-64acc.bin -n 64 -m 100000 --pre-notch

# binary HB2

md-121011-hb2-m1-bunch000001-64acc.bin:
	$(DATA_CHECK) -p MD-121011/HB2/M1/ -o md-121011-hb2-m1-bunch000001-64acc.bin -n 64 -m 000001 --pre-notch
md-121011-hb2-m1-bunch000010-64acc.bin:
	$(DATA_CHECK) -p MD-121011/HB2/M1/ -o md-121011-hb2-m1-bunch000010-64acc.bin -n 64 -m 000010 --pre-notch
md-121011-hb2-m1-bunch000100-64acc.bin:
	$(DATA_CHECK) -p MD-121011/HB2/M1/ -o md-121011-hb2-m1-bunch000100-64acc.bin -n 64 -m 000100 --pre-notch
md-121011-hb2-m1-bunch001000-64acc.bin:
	$(DATA_CHECK) -p MD-121011/HB2/M1/ -o md-121011-hb2-m1-bunch001000-64acc.bin -n 64 -m 001000 --pre-notch
md-121011-hb2-m1-bunch010000-64acc.bin:
	$(DATA_CHECK) -p MD-121011/HB2/M1/ -o md-121011-hb2-m1-bunch010000-64acc.bin -n 64 -m 010000 --pre-notch
md-121011-hb2-m1-bunch100000-64acc.bin:
	$(DATA_CHECK) -p MD-121011/HB2/M1/ -o md-121011-hb2-m1-bunch100000-64acc.bin -n 64 -m 100000 --pre-notch

# pictures VB1

md-121011-vb1-m1-bunch000001-64acc.png: md-121011-vb1-m1-bunch000001-64acc.bin
	$(DATA_CHECK) -i md-121011-vb1-m1-bunch000001-64acc.bin -p md-121011-vb1-m1-bunch000001-64acc.png
md-121011-vb1-m1-bunch000010-64acc.png: md-121011-vb1-m1-bunch000010-64acc.bin
	$(DATA_CHECK) -i md-121011-vb1-m1-bunch000010-64acc.bin -p md-121011-vb1-m1-bunch000010-64acc.png
md-121011-vb1-m1-bunch000100-64acc.png: md-121011-vb1-m1-bunch000100-64acc.bin
	$(DATA_CHECK) -i md-121011-vb1-m1-bunch000100-64acc.bin -p md-121011-vb1-m1-bunch000100-64acc.png
md-121011-vb1-m1-bunch001000-64acc.png: md-121011-vb1-m1-bunch001000-64acc.bin
	$(DATA_CHECK) -i md-121011-vb1-m1-bunch001000-64acc.bin -p md-121011-vb1-m1-bunch001000-64acc.png
md-121011-vb1-m1-bunch010000-64acc.png: md-121011-vb1-m1-bunch010000-64acc.bin
	$(DATA_CHECK) -i md-121011-vb1-m1-bunch010000-64acc.bin -p md-121011-vb1-m1-bunch010000-64acc.png
md-121011-vb1-m1-bunch100000-64acc.png: md-121011-vb1-m1-bunch100000-64acc.bin
	$(DATA_CHECK) -i md-121011-vb1-m1-bunch100000-64acc.bin -p md-121011-vb1-m1-bunch100000-64acc.png

# pictures VB2

md-121011-vb2-m1-bunch000001-64acc.png: md-121011-vb2-m1-bunch000001-64acc.bin
	$(DATA_CHECK) -i md-121011-vb2-m1-bunch000001-64acc.bin -p md-121011-vb2-m1-bunch000001-64acc.png
md-121011-vb2-m1-bunch000010-64acc.png: md-121011-vb2-m1-bunch000010-64acc.bin
	$(DATA_CHECK) -i md-121011-vb2-m1-bunch000010-64acc.bin -p md-121011-vb2-m1-bunch000010-64acc.png
md-121011-vb2-m1-bunch000100-64acc.png: md-121011-vb2-m1-bunch000100-64acc.bin
	$(DATA_CHECK) -i md-121011-vb2-m1-bunch000100-64acc.bin -p md-121011-vb2-m1-bunch000100-64acc.png
md-121011-vb2-m1-bunch001000-64acc.png: md-121011-vb2-m1-bunch001000-64acc.bin
	$(DATA_CHECK) -i md-121011-vb2-m1-bunch001000-64acc.bin -p md-121011-vb2-m1-bunch001000-64acc.png
md-121011-vb2-m1-bunch010000-64acc.png: md-121011-vb2-m1-bunch010000-64acc.bin
	$(DATA_CHECK) -i md-121011-vb2-m1-bunch010000-64acc.bin -p md-121011-vb2-m1-bunch010000-64acc.png
md-121011-vb2-m1-bunch100000-64acc.png: md-121011-vb2-m1-bunch100000-64acc.bin
	$(DATA_CHECK) -i md-121011-vb2-m1-bunch100000-64acc.bin -p md-121011-vb2-m1-bunch100000-64acc.png

# pictures HB1

md-121011-hb1-m1-bunch000001-64acc.png: md-121011-hb1-m1-bunch000001-64acc.bin
	$(DATA_CHECK) -i md-121011-hb1-m1-bunch000001-64acc.bin -p md-121011-hb1-m1-bunch000001-64acc.png
md-121011-hb1-m1-bunch000010-64acc.png: md-121011-hb1-m1-bunch000010-64acc.bin
	$(DATA_CHECK) -i md-121011-hb1-m1-bunch000010-64acc.bin -p md-121011-hb1-m1-bunch000010-64acc.png
md-121011-hb1-m1-bunch000100-64acc.png: md-121011-hb1-m1-bunch000100-64acc.bin
	$(DATA_CHECK) -i md-121011-hb1-m1-bunch000100-64acc.bin -p md-121011-hb1-m1-bunch000100-64acc.png
md-121011-hb1-m1-bunch001000-64acc.png: md-121011-hb1-m1-bunch001000-64acc.bin
	$(DATA_CHECK) -i md-121011-hb1-m1-bunch001000-64acc.bin -p md-121011-hb1-m1-bunch001000-64acc.png
md-121011-hb1-m1-bunch010000-64acc.png: md-121011-hb1-m1-bunch010000-64acc.bin
	$(DATA_CHECK) -i md-121011-hb1-m1-bunch010000-64acc.bin -p md-121011-hb1-m1-bunch010000-64acc.png
md-121011-hb1-m1-bunch100000-64acc.png: md-121011-hb1-m1-bunch100000-64acc.bin
	$(DATA_CHECK) -i md-121011-hb1-m1-bunch100000-64acc.bin -p md-121011-hb1-m1-bunch100000-64acc.png

# pictures HB2

md-121011-hb2-m1-bunch000001-64acc.png: md-121011-hb2-m1-bunch000001-64acc.bin
	$(DATA_CHECK) -i md-121011-hb2-m1-bunch000001-64acc.bin -p md-121011-hb2-m1-bunch000001-64acc.png
md-121011-hb2-m1-bunch000010-64acc.png: md-121011-hb2-m1-bunch000010-64acc.bin
	$(DATA_CHECK) -i md-121011-hb2-m1-bunch000010-64acc.bin -p md-121011-hb2-m1-bunch000010-64acc.png
md-121011-hb2-m1-bunch000100-64acc.png: md-121011-hb2-m1-bunch000100-64acc.bin
	$(DATA_CHECK) -i md-121011-hb2-m1-bunch000100-64acc.bin -p md-121011-hb2-m1-bunch000100-64acc.png
md-121011-hb2-m1-bunch001000-64acc.png: md-121011-hb2-m1-bunch001000-64acc.bin
	$(DATA_CHECK) -i md-121011-hb2-m1-bunch001000-64acc.bin -p md-121011-hb2-m1-bunch001000-64acc.png
md-121011-hb2-m1-bunch010000-64acc.png: md-121011-hb2-m1-bunch010000-64acc.bin
	$(DATA_CHECK) -i md-121011-hb2-m1-bunch010000-64acc.bin -p md-121011-hb2-m1-bunch010000-64acc.png
md-121011-hb2-m1-bunch100000-64acc.png: md-121011-hb2-m1-bunch100000-64acc.bin
	$(DATA_CHECK) -i md-121011-hb2-m1-bunch100000-64acc.bin -p md-121011-hb2-m1-bunch100000-64acc.png

clean:
	rm -f $(ALL)
