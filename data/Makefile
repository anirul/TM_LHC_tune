# a simple makefile to treat the data

UNAME := $(shell uname)

BINARY_DIR := ../binary/
PICTURES_DIR := ../pictures/
ACC := 32

ifeq ($(UNAME), Darwin)
	DATA_CHECK = ../sources/data_check
endif

ifeq ($(UNAME), Linux)
	DATA_CHECK = ../sources/data_check
endif

ALL = \
	$(PICTURES_DIR)md-121011-vb1-m1-bunch000001-$(ACC)acc.png \
	$(PICTURES_DIR)md-121011-vb1-m1-bunch000010-$(ACC)acc.png \
	$(PICTURES_DIR)md-121011-vb1-m1-bunch000100-$(ACC)acc.png \
	$(PICTURES_DIR)md-121011-vb1-m1-bunch001000-$(ACC)acc.png \
	$(PICTURES_DIR)md-121011-vb1-m1-bunch010000-$(ACC)acc.png \
	$(PICTURES_DIR)md-121011-vb1-m1-bunch100000-$(ACC)acc.png \
	$(PICTURES_DIR)md-121011-vb2-m1-bunch000001-$(ACC)acc.png \
	$(PICTURES_DIR)md-121011-vb2-m1-bunch000010-$(ACC)acc.png \
	$(PICTURES_DIR)md-121011-vb2-m1-bunch000100-$(ACC)acc.png \
	$(PICTURES_DIR)md-121011-vb2-m1-bunch001000-$(ACC)acc.png \
	$(PICTURES_DIR)md-121011-vb2-m1-bunch010000-$(ACC)acc.png \
	$(PICTURES_DIR)md-121011-vb2-m1-bunch100000-$(ACC)acc.png \
	$(PICTURES_DIR)md-121011-hb1-m1-bunch000001-$(ACC)acc.png \
	$(PICTURES_DIR)md-121011-hb1-m1-bunch000010-$(ACC)acc.png \
	$(PICTURES_DIR)md-121011-hb1-m1-bunch000100-$(ACC)acc.png \
	$(PICTURES_DIR)md-121011-hb1-m1-bunch001000-$(ACC)acc.png \
	$(PICTURES_DIR)md-121011-hb1-m1-bunch010000-$(ACC)acc.png \
	$(PICTURES_DIR)md-121011-hb1-m1-bunch100000-$(ACC)acc.png \
	$(PICTURES_DIR)md-121011-hb2-m1-bunch000001-$(ACC)acc.png \
	$(PICTURES_DIR)md-121011-hb2-m1-bunch000010-$(ACC)acc.png \
	$(PICTURES_DIR)md-121011-hb2-m1-bunch000100-$(ACC)acc.png \
	$(PICTURES_DIR)md-121011-hb2-m1-bunch001000-$(ACC)acc.png \
	$(PICTURES_DIR)md-121011-hb2-m1-bunch010000-$(ACC)acc.png \
	$(PICTURES_DIR)md-121011-hb2-m1-bunch100000-$(ACC)acc.png

all: $(ALL)

# binary VB1

$(BINARY_DIR)md-121011-vb1-m1-bunch000001-$(ACC)acc.bin:
	$(DATA_CHECK) -p MD-121011/VB1/M1/ -o $(BINARY_DIR)md-121011-vb1-m1-bunch000001-$(ACC)acc.bin -n $(ACC) -m 000001 --pre-notch
$(BINARY_DIR)md-121011-vb1-m1-bunch000010-$(ACC)acc.bin:
	$(DATA_CHECK) -p MD-121011/VB1/M1/ -o $(BINARY_DIR)md-121011-vb1-m1-bunch000010-$(ACC)acc.bin -n $(ACC) -m 000010 --pre-notch
$(BINARY_DIR)md-121011-vb1-m1-bunch000100-$(ACC)acc.bin:
	$(DATA_CHECK) -p MD-121011/VB1/M1/ -o $(BINARY_DIR)md-121011-vb1-m1-bunch000100-$(ACC)acc.bin -n $(ACC) -m 000100 --pre-notch
$(BINARY_DIR)md-121011-vb1-m1-bunch001000-$(ACC)acc.bin:
	$(DATA_CHECK) -p MD-121011/VB1/M1/ -o $(BINARY_DIR)md-121011-vb1-m1-bunch001000-$(ACC)acc.bin -n $(ACC) -m 001000 --pre-notch
$(BINARY_DIR)md-121011-vb1-m1-bunch010000-$(ACC)acc.bin:
	$(DATA_CHECK) -p MD-121011/VB1/M1/ -o $(BINARY_DIR)md-121011-vb1-m1-bunch010000-$(ACC)acc.bin -n $(ACC) -m 010000 --pre-notch
$(BINARY_DIR)md-121011-vb1-m1-bunch100000-$(ACC)acc.bin:
	$(DATA_CHECK) -p MD-121011/VB1/M1/ -o $(BINARY_DIR)md-121011-vb1-m1-bunch100000-$(ACC)acc.bin -n $(ACC) -m 100000 --pre-notch

# binary VB2

$(BINARY_DIR)md-121011-vb2-m1-bunch000001-$(ACC)acc.bin:
	$(DATA_CHECK) -p MD-121011/VB2/M1/ -o $(BINARY_DIR)md-121011-vb2-m1-bunch000001-$(ACC)acc.bin -n $(ACC) -m 000001 --pre-notch
$(BINARY_DIR)md-121011-vb2-m1-bunch000010-$(ACC)acc.bin:
	$(DATA_CHECK) -p MD-121011/VB2/M1/ -o $(BINARY_DIR)md-121011-vb2-m1-bunch000010-$(ACC)acc.bin -n $(ACC) -m 000010 --pre-notch
$(BINARY_DIR)md-121011-vb2-m1-bunch000100-$(ACC)acc.bin:
	$(DATA_CHECK) -p MD-121011/VB2/M1/ -o $(BINARY_DIR)md-121011-vb2-m1-bunch000100-$(ACC)acc.bin -n $(ACC) -m 000100 --pre-notch
$(BINARY_DIR)md-121011-vb2-m1-bunch001000-$(ACC)acc.bin:
	$(DATA_CHECK) -p MD-121011/VB2/M1/ -o $(BINARY_DIR)md-121011-vb2-m1-bunch001000-$(ACC)acc.bin -n $(ACC) -m 001000 --pre-notch
$(BINARY_DIR)md-121011-vb2-m1-bunch010000-$(ACC)acc.bin:
	$(DATA_CHECK) -p MD-121011/VB2/M1/ -o $(BINARY_DIR)md-121011-vb2-m1-bunch010000-$(ACC)acc.bin -n $(ACC) -m 010000 --pre-notch
$(BINARY_DIR)md-121011-vb2-m1-bunch100000-$(ACC)acc.bin:
	$(DATA_CHECK) -p MD-121011/VB2/M1/ -o $(BINARY_DIR)md-121011-vb2-m1-bunch100000-$(ACC)acc.bin -n $(ACC) -m 100000 --pre-notch

# binary HB1

$(BINARY_DIR)md-121011-hb1-m1-bunch000001-$(ACC)acc.bin:
	$(DATA_CHECK) -p MD-121011/HB1/M1/ -o $(BINARY_DIR)md-121011-hb1-m1-bunch000001-$(ACC)acc.bin -n $(ACC) -m 000001 --pre-notch
$(BINARY_DIR)md-121011-hb1-m1-bunch000010-$(ACC)acc.bin:
	$(DATA_CHECK) -p MD-121011/HB1/M1/ -o $(BINARY_DIR)md-121011-hb1-m1-bunch000010-$(ACC)acc.bin -n $(ACC) -m 000010 --pre-notch
$(BINARY_DIR)md-121011-hb1-m1-bunch000100-$(ACC)acc.bin:
	$(DATA_CHECK) -p MD-121011/HB1/M1/ -o $(BINARY_DIR)md-121011-hb1-m1-bunch000100-$(ACC)acc.bin -n $(ACC) -m 000100 --pre-notch
$(BINARY_DIR)md-121011-hb1-m1-bunch001000-$(ACC)acc.bin:
	$(DATA_CHECK) -p MD-121011/HB1/M1/ -o $(BINARY_DIR)md-121011-hb1-m1-bunch001000-$(ACC)acc.bin -n $(ACC) -m 001000 --pre-notch
$(BINARY_DIR)md-121011-hb1-m1-bunch010000-$(ACC)acc.bin:
	$(DATA_CHECK) -p MD-121011/HB1/M1/ -o $(BINARY_DIR)md-121011-hb1-m1-bunch010000-$(ACC)acc.bin -n $(ACC) -m 010000 --pre-notch
$(BINARY_DIR)md-121011-hb1-m1-bunch100000-$(ACC)acc.bin:
	$(DATA_CHECK) -p MD-121011/HB1/M1/ -o $(BINARY_DIR)md-121011-hb1-m1-bunch100000-$(ACC)acc.bin -n $(ACC) -m 100000 --pre-notch

# binary HB2

$(BINARY_DIR)md-121011-hb2-m1-bunch000001-$(ACC)acc.bin:
	$(DATA_CHECK) -p MD-121011/HB2/M1/ -o $(BINARY_DIR)md-121011-hb2-m1-bunch000001-$(ACC)acc.bin -n $(ACC) -m 000001 --pre-notch
$(BINARY_DIR)md-121011-hb2-m1-bunch000010-$(ACC)acc.bin:
	$(DATA_CHECK) -p MD-121011/HB2/M1/ -o $(BINARY_DIR)md-121011-hb2-m1-bunch000010-$(ACC)acc.bin -n $(ACC) -m 000010 --pre-notch
$(BINARY_DIR)md-121011-hb2-m1-bunch000100-$(ACC)acc.bin:
	$(DATA_CHECK) -p MD-121011/HB2/M1/ -o $(BINARY_DIR)md-121011-hb2-m1-bunch000100-$(ACC)acc.bin -n $(ACC) -m 000100 --pre-notch
$(BINARY_DIR)md-121011-hb2-m1-bunch001000-$(ACC)acc.bin:
	$(DATA_CHECK) -p MD-121011/HB2/M1/ -o $(BINARY_DIR)md-121011-hb2-m1-bunch001000-$(ACC)acc.bin -n $(ACC) -m 001000 --pre-notch
$(BINARY_DIR)md-121011-hb2-m1-bunch010000-$(ACC)acc.bin:
	$(DATA_CHECK) -p MD-121011/HB2/M1/ -o $(BINARY_DIR)md-121011-hb2-m1-bunch010000-$(ACC)acc.bin -n $(ACC) -m 010000 --pre-notch
$(BINARY_DIR)md-121011-hb2-m1-bunch100000-$(ACC)acc.bin:
	$(DATA_CHECK) -p MD-121011/HB2/M1/ -o $(BINARY_DIR)md-121011-hb2-m1-bunch100000-$(ACC)acc.bin -n $(ACC) -m 100000 --pre-notch

# pictures VB1

$(PICTURES_DIR)md-121011-vb1-m1-bunch000001-$(ACC)acc.png: $(BINARY_DIR)md-121011-vb1-m1-bunch000001-$(ACC)acc.bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-121011-vb1-m1-bunch000001-$(ACC)acc.bin -b $(PICTURES_DIR)md-121011-vb1-m1-bunch000001-$(ACC)acc.png
$(PICTURES_DIR)md-121011-vb1-m1-bunch000010-$(ACC)acc.png: $(BINARY_DIR)md-121011-vb1-m1-bunch000010-$(ACC)acc.bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-121011-vb1-m1-bunch000010-$(ACC)acc.bin -b $(PICTURES_DIR)md-121011-vb1-m1-bunch000010-$(ACC)acc.png
$(PICTURES_DIR)md-121011-vb1-m1-bunch000100-$(ACC)acc.png: $(BINARY_DIR)md-121011-vb1-m1-bunch000100-$(ACC)acc.bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-121011-vb1-m1-bunch000100-$(ACC)acc.bin -b $(PICTURES_DIR)md-121011-vb1-m1-bunch000100-$(ACC)acc.png
$(PICTURES_DIR)md-121011-vb1-m1-bunch001000-$(ACC)acc.png: $(BINARY_DIR)md-121011-vb1-m1-bunch001000-$(ACC)acc.bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-121011-vb1-m1-bunch001000-$(ACC)acc.bin -b $(PICTURES_DIR)md-121011-vb1-m1-bunch001000-$(ACC)acc.png
$(PICTURES_DIR)md-121011-vb1-m1-bunch010000-$(ACC)acc.png: $(BINARY_DIR)md-121011-vb1-m1-bunch010000-$(ACC)acc.bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-121011-vb1-m1-bunch010000-$(ACC)acc.bin -b $(PICTURES_DIR)md-121011-vb1-m1-bunch010000-$(ACC)acc.png
$(PICTURES_DIR)md-121011-vb1-m1-bunch100000-$(ACC)acc.png: $(BINARY_DIR)md-121011-vb1-m1-bunch100000-$(ACC)acc.bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-121011-vb1-m1-bunch100000-$(ACC)acc.bin -b $(PICTURES_DIR)md-121011-vb1-m1-bunch100000-$(ACC)acc.png

# pictures VB2

$(PICTURES_DIR)md-121011-vb2-m1-bunch000001-$(ACC)acc.png: $(BINARY_DIR)md-121011-vb2-m1-bunch000001-$(ACC)acc.bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-121011-vb2-m1-bunch000001-$(ACC)acc.bin -b $(PICTURES_DIR)md-121011-vb2-m1-bunch000001-$(ACC)acc.png
$(PICTURES_DIR)md-121011-vb2-m1-bunch000010-$(ACC)acc.png: $(BINARY_DIR)md-121011-vb2-m1-bunch000010-$(ACC)acc.bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-121011-vb2-m1-bunch000010-$(ACC)acc.bin -b $(PICTURES_DIR)md-121011-vb2-m1-bunch000010-$(ACC)acc.png
$(PICTURES_DIR)md-121011-vb2-m1-bunch000100-$(ACC)acc.png: $(BINARY_DIR)md-121011-vb2-m1-bunch000100-$(ACC)acc.bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-121011-vb2-m1-bunch000100-$(ACC)acc.bin -b $(PICTURES_DIR)md-121011-vb2-m1-bunch000100-$(ACC)acc.png
$(PICTURES_DIR)md-121011-vb2-m1-bunch001000-$(ACC)acc.png: $(BINARY_DIR)md-121011-vb2-m1-bunch001000-$(ACC)acc.bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-121011-vb2-m1-bunch001000-$(ACC)acc.bin -b $(PICTURES_DIR)md-121011-vb2-m1-bunch001000-$(ACC)acc.png
$(PICTURES_DIR)md-121011-vb2-m1-bunch010000-$(ACC)acc.png: $(BINARY_DIR)md-121011-vb2-m1-bunch010000-$(ACC)acc.bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-121011-vb2-m1-bunch010000-$(ACC)acc.bin -b $(PICTURES_DIR)md-121011-vb2-m1-bunch010000-$(ACC)acc.png
$(PICTURES_DIR)md-121011-vb2-m1-bunch100000-$(ACC)acc.png: $(BINARY_DIR)md-121011-vb2-m1-bunch100000-$(ACC)acc.bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-121011-vb2-m1-bunch100000-$(ACC)acc.bin -b $(PICTURES_DIR)md-121011-vb2-m1-bunch100000-$(ACC)acc.png

# pictures HB1

$(PICTURES_DIR)md-121011-hb1-m1-bunch000001-$(ACC)acc.png: $(BINARY_DIR)md-121011-hb1-m1-bunch000001-$(ACC)acc.bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-121011-hb1-m1-bunch000001-$(ACC)acc.bin -b $(PICTURES_DIR)md-121011-hb1-m1-bunch000001-$(ACC)acc.png
$(PICTURES_DIR)md-121011-hb1-m1-bunch000010-$(ACC)acc.png: $(BINARY_DIR)md-121011-hb1-m1-bunch000010-$(ACC)acc.bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-121011-hb1-m1-bunch000010-$(ACC)acc.bin -b $(PICTURES_DIR)md-121011-hb1-m1-bunch000010-$(ACC)acc.png
$(PICTURES_DIR)md-121011-hb1-m1-bunch000100-$(ACC)acc.png: $(BINARY_DIR)md-121011-hb1-m1-bunch000100-$(ACC)acc.bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-121011-hb1-m1-bunch000100-$(ACC)acc.bin -b $(PICTURES_DIR)md-121011-hb1-m1-bunch000100-$(ACC)acc.png
$(PICTURES_DIR)md-121011-hb1-m1-bunch001000-$(ACC)acc.png: $(BINARY_DIR)md-121011-hb1-m1-bunch001000-$(ACC)acc.bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-121011-hb1-m1-bunch001000-$(ACC)acc.bin -b $(PICTURES_DIR)md-121011-hb1-m1-bunch001000-$(ACC)acc.png
$(PICTURES_DIR)md-121011-hb1-m1-bunch010000-$(ACC)acc.png: $(BINARY_DIR)md-121011-hb1-m1-bunch010000-$(ACC)acc.bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-121011-hb1-m1-bunch010000-$(ACC)acc.bin -b $(PICTURES_DIR)md-121011-hb1-m1-bunch010000-$(ACC)acc.png
$(PICTURES_DIR)md-121011-hb1-m1-bunch100000-$(ACC)acc.png: $(BINARY_DIR)md-121011-hb1-m1-bunch100000-$(ACC)acc.bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-121011-hb1-m1-bunch100000-$(ACC)acc.bin -b $(PICTURES_DIR)md-121011-hb1-m1-bunch100000-$(ACC)acc.png

# pictures HB2

$(PICTURES_DIR)md-121011-hb2-m1-bunch000001-$(ACC)acc.png: $(BINARY_DIR)md-121011-hb2-m1-bunch000001-$(ACC)acc.bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-121011-hb2-m1-bunch000001-$(ACC)acc.bin -b $(PICTURES_DIR)md-121011-hb2-m1-bunch000001-$(ACC)acc.png
$(PICTURES_DIR)md-121011-hb2-m1-bunch000010-$(ACC)acc.png: $(BINARY_DIR)md-121011-hb2-m1-bunch000010-$(ACC)acc.bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-121011-hb2-m1-bunch000010-$(ACC)acc.bin -b $(PICTURES_DIR)md-121011-hb2-m1-bunch000010-$(ACC)acc.png
$(PICTURES_DIR)md-121011-hb2-m1-bunch000100-$(ACC)acc.png: $(BINARY_DIR)md-121011-hb2-m1-bunch000100-$(ACC)acc.bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-121011-hb2-m1-bunch000100-$(ACC)acc.bin -b $(PICTURES_DIR)md-121011-hb2-m1-bunch000100-$(ACC)acc.png
$(PICTURES_DIR)md-121011-hb2-m1-bunch001000-$(ACC)acc.png: $(BINARY_DIR)md-121011-hb2-m1-bunch001000-$(ACC)acc.bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-121011-hb2-m1-bunch001000-$(ACC)acc.bin -b $(PICTURES_DIR)md-121011-hb2-m1-bunch001000-$(ACC)acc.png
$(PICTURES_DIR)md-121011-hb2-m1-bunch010000-$(ACC)acc.png: $(BINARY_DIR)md-121011-hb2-m1-bunch010000-$(ACC)acc.bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-121011-hb2-m1-bunch010000-$(ACC)acc.bin -b $(PICTURES_DIR)md-121011-hb2-m1-bunch010000-$(ACC)acc.png
$(PICTURES_DIR)md-121011-hb2-m1-bunch100000-$(ACC)acc.png: $(BINARY_DIR)md-121011-hb2-m1-bunch100000-$(ACC)acc.bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-121011-hb2-m1-bunch100000-$(ACC)acc.bin -b $(PICTURES_DIR)md-121011-hb2-m1-bunch100000-$(ACC)acc.png

clean:
	rm -f $(ALL)
