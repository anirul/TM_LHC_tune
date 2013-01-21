# a simple makefile to treat the data

UNAME := $(shell uname)

BINARY_DIR := ../binary/
PICTURES_DIR := ../pictures/
FLAGS := --start-time 1350394620000000000 --end-time 1350395340000000000
TIME := -1337-1349

ACC := 8
MD_DATE := 121016

ifeq ($(MD_DATE), 121011)
	FLAGS += --pre-notch
endif

ifeq ($(UNAME), Darwin)
	DATA_CHECK = ../sources/data_check
endif

ifeq ($(UNAME), Linux)
	DATA_CHECK = ../sources/data_check
endif

ALL = \
	$(PICTURES_DIR)md-$(MD_DATE)-vb1-m1-bunch000001-$(ACC)acc$(TIME).png \
	$(PICTURES_DIR)md-$(MD_DATE)-vb1-m1-bunch000010-$(ACC)acc$(TIME).png \
	$(PICTURES_DIR)md-$(MD_DATE)-vb1-m1-bunch000100-$(ACC)acc$(TIME).png \
	$(PICTURES_DIR)md-$(MD_DATE)-vb1-m1-bunch001000-$(ACC)acc$(TIME).png \
	$(PICTURES_DIR)md-$(MD_DATE)-vb1-m1-bunch010000-$(ACC)acc$(TIME).png \
	$(PICTURES_DIR)md-$(MD_DATE)-vb1-m1-bunch100000-$(ACC)acc$(TIME).png \
	$(PICTURES_DIR)md-$(MD_DATE)-vb2-m1-bunch000001-$(ACC)acc$(TIME).png \
	$(PICTURES_DIR)md-$(MD_DATE)-vb2-m1-bunch000010-$(ACC)acc$(TIME).png \
	$(PICTURES_DIR)md-$(MD_DATE)-vb2-m1-bunch000100-$(ACC)acc$(TIME).png \
	$(PICTURES_DIR)md-$(MD_DATE)-vb2-m1-bunch001000-$(ACC)acc$(TIME).png \
	$(PICTURES_DIR)md-$(MD_DATE)-vb2-m1-bunch010000-$(ACC)acc$(TIME).png \
	$(PICTURES_DIR)md-$(MD_DATE)-vb2-m1-bunch100000-$(ACC)acc$(TIME).png \
	$(PICTURES_DIR)md-$(MD_DATE)-hb1-m1-bunch000001-$(ACC)acc$(TIME).png \
	$(PICTURES_DIR)md-$(MD_DATE)-hb1-m1-bunch000010-$(ACC)acc$(TIME).png \
	$(PICTURES_DIR)md-$(MD_DATE)-hb1-m1-bunch000100-$(ACC)acc$(TIME).png \
	$(PICTURES_DIR)md-$(MD_DATE)-hb1-m1-bunch001000-$(ACC)acc$(TIME).png \
	$(PICTURES_DIR)md-$(MD_DATE)-hb1-m1-bunch010000-$(ACC)acc$(TIME).png \
	$(PICTURES_DIR)md-$(MD_DATE)-hb1-m1-bunch100000-$(ACC)acc$(TIME).png \
	$(PICTURES_DIR)md-$(MD_DATE)-hb2-m1-bunch000001-$(ACC)acc$(TIME).png \
	$(PICTURES_DIR)md-$(MD_DATE)-hb2-m1-bunch000010-$(ACC)acc$(TIME).png \
	$(PICTURES_DIR)md-$(MD_DATE)-hb2-m1-bunch000100-$(ACC)acc$(TIME).png \
	$(PICTURES_DIR)md-$(MD_DATE)-hb2-m1-bunch001000-$(ACC)acc$(TIME).png \
	$(PICTURES_DIR)md-$(MD_DATE)-hb2-m1-bunch010000-$(ACC)acc$(TIME).png \
	$(PICTURES_DIR)md-$(MD_DATE)-hb2-m1-bunch100000-$(ACC)acc$(TIME).png

all: $(ALL)

# executable

$(DATA_CHECK):
	$(MAKE) -C ../sources

# binary VB1

$(BINARY_DIR)md-$(MD_DATE)-vb1-m1-bunch000001-$(ACC)acc$(TIME).bin: $(DATA_CHECK)
	$(DATA_CHECK) -p MD-$(MD_DATE)/VB1/M1/ -o $(BINARY_DIR)md-$(MD_DATE)-vb1-m1-bunch000001-$(ACC)acc$(TIME).bin -n $(ACC) -m 000001 $(FLAGS)
$(BINARY_DIR)md-$(MD_DATE)-vb1-m1-bunch000010-$(ACC)acc$(TIME).bin: $(DATA_CHECK) 
	$(DATA_CHECK) -p MD-$(MD_DATE)/VB1/M1/ -o $(BINARY_DIR)md-$(MD_DATE)-vb1-m1-bunch000010-$(ACC)acc$(TIME).bin -n $(ACC) -m 000010 $(FLAGS)
$(BINARY_DIR)md-$(MD_DATE)-vb1-m1-bunch000100-$(ACC)acc$(TIME).bin: $(DATA_CHECK)
	$(DATA_CHECK) -p MD-$(MD_DATE)/VB1/M1/ -o $(BINARY_DIR)md-$(MD_DATE)-vb1-m1-bunch000100-$(ACC)acc$(TIME).bin -n $(ACC) -m 000100 $(FLAGS)
$(BINARY_DIR)md-$(MD_DATE)-vb1-m1-bunch001000-$(ACC)acc$(TIME).bin: $(DATA_CHECK)
	$(DATA_CHECK) -p MD-$(MD_DATE)/VB1/M1/ -o $(BINARY_DIR)md-$(MD_DATE)-vb1-m1-bunch001000-$(ACC)acc$(TIME).bin -n $(ACC) -m 001000 $(FLAGS)
$(BINARY_DIR)md-$(MD_DATE)-vb1-m1-bunch010000-$(ACC)acc$(TIME).bin: $(DATA_CHECK)
	$(DATA_CHECK) -p MD-$(MD_DATE)/VB1/M1/ -o $(BINARY_DIR)md-$(MD_DATE)-vb1-m1-bunch010000-$(ACC)acc$(TIME).bin -n $(ACC) -m 010000 $(FLAGS)
$(BINARY_DIR)md-$(MD_DATE)-vb1-m1-bunch100000-$(ACC)acc$(TIME).bin: $(DATA_CHECK)
	$(DATA_CHECK) -p MD-$(MD_DATE)/VB1/M1/ -o $(BINARY_DIR)md-$(MD_DATE)-vb1-m1-bunch100000-$(ACC)acc$(TIME).bin -n $(ACC) -m 100000 $(FLAGS)

# binary VB2

$(BINARY_DIR)md-$(MD_DATE)-vb2-m1-bunch000001-$(ACC)acc$(TIME).bin: $(DATA_CHECK)
	$(DATA_CHECK) -p MD-$(MD_DATE)/VB2/M1/ -o $(BINARY_DIR)md-$(MD_DATE)-vb2-m1-bunch000001-$(ACC)acc$(TIME).bin -n $(ACC) -m 000001 $(FLAGS)
$(BINARY_DIR)md-$(MD_DATE)-vb2-m1-bunch000010-$(ACC)acc$(TIME).bin: $(DATA_CHECK)
	$(DATA_CHECK) -p MD-$(MD_DATE)/VB2/M1/ -o $(BINARY_DIR)md-$(MD_DATE)-vb2-m1-bunch000010-$(ACC)acc$(TIME).bin -n $(ACC) -m 000010 $(FLAGS)
$(BINARY_DIR)md-$(MD_DATE)-vb2-m1-bunch000100-$(ACC)acc$(TIME).bin: $(DATA_CHECK)
	$(DATA_CHECK) -p MD-$(MD_DATE)/VB2/M1/ -o $(BINARY_DIR)md-$(MD_DATE)-vb2-m1-bunch000100-$(ACC)acc$(TIME).bin -n $(ACC) -m 000100 $(FLAGS)
$(BINARY_DIR)md-$(MD_DATE)-vb2-m1-bunch001000-$(ACC)acc$(TIME).bin: $(DATA_CHECK)
	$(DATA_CHECK) -p MD-$(MD_DATE)/VB2/M1/ -o $(BINARY_DIR)md-$(MD_DATE)-vb2-m1-bunch001000-$(ACC)acc$(TIME).bin -n $(ACC) -m 001000 $(FLAGS)
$(BINARY_DIR)md-$(MD_DATE)-vb2-m1-bunch010000-$(ACC)acc$(TIME).bin: $(DATA_CHECK)
	$(DATA_CHECK) -p MD-$(MD_DATE)/VB2/M1/ -o $(BINARY_DIR)md-$(MD_DATE)-vb2-m1-bunch010000-$(ACC)acc$(TIME).bin -n $(ACC) -m 010000 $(FLAGS)
$(BINARY_DIR)md-$(MD_DATE)-vb2-m1-bunch100000-$(ACC)acc$(TIME).bin: $(DATA_CHECK)
	$(DATA_CHECK) -p MD-$(MD_DATE)/VB2/M1/ -o $(BINARY_DIR)md-$(MD_DATE)-vb2-m1-bunch100000-$(ACC)acc$(TIME).bin -n $(ACC) -m 100000 $(FLAGS)

# binary HB1

$(BINARY_DIR)md-$(MD_DATE)-hb1-m1-bunch000001-$(ACC)acc$(TIME).bin: $(DATA_CHECK)
	$(DATA_CHECK) -p MD-$(MD_DATE)/HB1/M1/ -o $(BINARY_DIR)md-$(MD_DATE)-hb1-m1-bunch000001-$(ACC)acc$(TIME).bin -n $(ACC) -m 000001 $(FLAGS)
$(BINARY_DIR)md-$(MD_DATE)-hb1-m1-bunch000010-$(ACC)acc$(TIME).bin: $(DATA_CHECK)
	$(DATA_CHECK) -p MD-$(MD_DATE)/HB1/M1/ -o $(BINARY_DIR)md-$(MD_DATE)-hb1-m1-bunch000010-$(ACC)acc$(TIME).bin -n $(ACC) -m 000010 $(FLAGS)
$(BINARY_DIR)md-$(MD_DATE)-hb1-m1-bunch000100-$(ACC)acc$(TIME).bin: $(DATA_CHECK)
	$(DATA_CHECK) -p MD-$(MD_DATE)/HB1/M1/ -o $(BINARY_DIR)md-$(MD_DATE)-hb1-m1-bunch000100-$(ACC)acc$(TIME).bin -n $(ACC) -m 000100 $(FLAGS)
$(BINARY_DIR)md-$(MD_DATE)-hb1-m1-bunch001000-$(ACC)acc$(TIME).bin: $(DATA_CHECK)
	$(DATA_CHECK) -p MD-$(MD_DATE)/HB1/M1/ -o $(BINARY_DIR)md-$(MD_DATE)-hb1-m1-bunch001000-$(ACC)acc$(TIME).bin -n $(ACC) -m 001000 $(FLAGS)
$(BINARY_DIR)md-$(MD_DATE)-hb1-m1-bunch010000-$(ACC)acc$(TIME).bin: $(DATA_CHECK)
	$(DATA_CHECK) -p MD-$(MD_DATE)/HB1/M1/ -o $(BINARY_DIR)md-$(MD_DATE)-hb1-m1-bunch010000-$(ACC)acc$(TIME).bin -n $(ACC) -m 010000 $(FLAGS)
$(BINARY_DIR)md-$(MD_DATE)-hb1-m1-bunch100000-$(ACC)acc$(TIME).bin: $(DATA_CHECK)
	$(DATA_CHECK) -p MD-$(MD_DATE)/HB1/M1/ -o $(BINARY_DIR)md-$(MD_DATE)-hb1-m1-bunch100000-$(ACC)acc$(TIME).bin -n $(ACC) -m 100000 $(FLAGS)

# binary HB2

$(BINARY_DIR)md-$(MD_DATE)-hb2-m1-bunch000001-$(ACC)acc$(TIME).bin: $(DATA_CHECK)
	$(DATA_CHECK) -p MD-$(MD_DATE)/HB2/M1/ -o $(BINARY_DIR)md-$(MD_DATE)-hb2-m1-bunch000001-$(ACC)acc$(TIME).bin -n $(ACC) -m 000001 $(FLAGS)
$(BINARY_DIR)md-$(MD_DATE)-hb2-m1-bunch000010-$(ACC)acc$(TIME).bin: $(DATA_CHECK)
	$(DATA_CHECK) -p MD-$(MD_DATE)/HB2/M1/ -o $(BINARY_DIR)md-$(MD_DATE)-hb2-m1-bunch000010-$(ACC)acc$(TIME).bin -n $(ACC) -m 000010 $(FLAGS)
$(BINARY_DIR)md-$(MD_DATE)-hb2-m1-bunch000100-$(ACC)acc$(TIME).bin: $(DATA_CHECK)
	$(DATA_CHECK) -p MD-$(MD_DATE)/HB2/M1/ -o $(BINARY_DIR)md-$(MD_DATE)-hb2-m1-bunch000100-$(ACC)acc$(TIME).bin -n $(ACC) -m 000100 $(FLAGS)
$(BINARY_DIR)md-$(MD_DATE)-hb2-m1-bunch001000-$(ACC)acc$(TIME).bin: $(DATA_CHECK)
	$(DATA_CHECK) -p MD-$(MD_DATE)/HB2/M1/ -o $(BINARY_DIR)md-$(MD_DATE)-hb2-m1-bunch001000-$(ACC)acc$(TIME).bin -n $(ACC) -m 001000 $(FLAGS)
$(BINARY_DIR)md-$(MD_DATE)-hb2-m1-bunch010000-$(ACC)acc$(TIME).bin: $(DATA_CHECK)
	$(DATA_CHECK) -p MD-$(MD_DATE)/HB2/M1/ -o $(BINARY_DIR)md-$(MD_DATE)-hb2-m1-bunch010000-$(ACC)acc$(TIME).bin -n $(ACC) -m 010000 $(FLAGS)
$(BINARY_DIR)md-$(MD_DATE)-hb2-m1-bunch100000-$(ACC)acc$(TIME).bin: $(DATA_CHECK)
	$(DATA_CHECK) -p MD-$(MD_DATE)/HB2/M1/ -o $(BINARY_DIR)md-$(MD_DATE)-hb2-m1-bunch100000-$(ACC)acc$(TIME).bin -n $(ACC) -m 100000 $(FLAGS)

# pictures VB1

$(PICTURES_DIR)md-$(MD_DATE)-vb1-m1-bunch000001-$(ACC)acc$(TIME).png: $(BINARY_DIR)md-$(MD_DATE)-vb1-m1-bunch000001-$(ACC)acc$(TIME).bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-$(MD_DATE)-vb1-m1-bunch000001-$(ACC)acc$(TIME).bin -b $(PICTURES_DIR)md-$(MD_DATE)-vb1-m1-bunch000001-$(ACC)acc$(TIME).png
$(PICTURES_DIR)md-$(MD_DATE)-vb1-m1-bunch000010-$(ACC)acc$(TIME).png: $(BINARY_DIR)md-$(MD_DATE)-vb1-m1-bunch000010-$(ACC)acc$(TIME).bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-$(MD_DATE)-vb1-m1-bunch000010-$(ACC)acc$(TIME).bin -b $(PICTURES_DIR)md-$(MD_DATE)-vb1-m1-bunch000010-$(ACC)acc$(TIME).png
$(PICTURES_DIR)md-$(MD_DATE)-vb1-m1-bunch000100-$(ACC)acc$(TIME).png: $(BINARY_DIR)md-$(MD_DATE)-vb1-m1-bunch000100-$(ACC)acc$(TIME).bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-$(MD_DATE)-vb1-m1-bunch000100-$(ACC)acc$(TIME).bin -b $(PICTURES_DIR)md-$(MD_DATE)-vb1-m1-bunch000100-$(ACC)acc$(TIME).png
$(PICTURES_DIR)md-$(MD_DATE)-vb1-m1-bunch001000-$(ACC)acc$(TIME).png: $(BINARY_DIR)md-$(MD_DATE)-vb1-m1-bunch001000-$(ACC)acc$(TIME).bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-$(MD_DATE)-vb1-m1-bunch001000-$(ACC)acc$(TIME).bin -b $(PICTURES_DIR)md-$(MD_DATE)-vb1-m1-bunch001000-$(ACC)acc$(TIME).png
$(PICTURES_DIR)md-$(MD_DATE)-vb1-m1-bunch010000-$(ACC)acc$(TIME).png: $(BINARY_DIR)md-$(MD_DATE)-vb1-m1-bunch010000-$(ACC)acc$(TIME).bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-$(MD_DATE)-vb1-m1-bunch010000-$(ACC)acc$(TIME).bin -b $(PICTURES_DIR)md-$(MD_DATE)-vb1-m1-bunch010000-$(ACC)acc$(TIME).png
$(PICTURES_DIR)md-$(MD_DATE)-vb1-m1-bunch100000-$(ACC)acc$(TIME).png: $(BINARY_DIR)md-$(MD_DATE)-vb1-m1-bunch100000-$(ACC)acc$(TIME).bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-$(MD_DATE)-vb1-m1-bunch100000-$(ACC)acc$(TIME).bin -b $(PICTURES_DIR)md-$(MD_DATE)-vb1-m1-bunch100000-$(ACC)acc$(TIME).png

# pictures VB2

$(PICTURES_DIR)md-$(MD_DATE)-vb2-m1-bunch000001-$(ACC)acc$(TIME).png: $(BINARY_DIR)md-$(MD_DATE)-vb2-m1-bunch000001-$(ACC)acc$(TIME).bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-$(MD_DATE)-vb2-m1-bunch000001-$(ACC)acc$(TIME).bin -b $(PICTURES_DIR)md-$(MD_DATE)-vb2-m1-bunch000001-$(ACC)acc$(TIME).png
$(PICTURES_DIR)md-$(MD_DATE)-vb2-m1-bunch000010-$(ACC)acc$(TIME).png: $(BINARY_DIR)md-$(MD_DATE)-vb2-m1-bunch000010-$(ACC)acc$(TIME).bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-$(MD_DATE)-vb2-m1-bunch000010-$(ACC)acc$(TIME).bin -b $(PICTURES_DIR)md-$(MD_DATE)-vb2-m1-bunch000010-$(ACC)acc$(TIME).png
$(PICTURES_DIR)md-$(MD_DATE)-vb2-m1-bunch000100-$(ACC)acc$(TIME).png: $(BINARY_DIR)md-$(MD_DATE)-vb2-m1-bunch000100-$(ACC)acc$(TIME).bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-$(MD_DATE)-vb2-m1-bunch000100-$(ACC)acc$(TIME).bin -b $(PICTURES_DIR)md-$(MD_DATE)-vb2-m1-bunch000100-$(ACC)acc$(TIME).png
$(PICTURES_DIR)md-$(MD_DATE)-vb2-m1-bunch001000-$(ACC)acc$(TIME).png: $(BINARY_DIR)md-$(MD_DATE)-vb2-m1-bunch001000-$(ACC)acc$(TIME).bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-$(MD_DATE)-vb2-m1-bunch001000-$(ACC)acc$(TIME).bin -b $(PICTURES_DIR)md-$(MD_DATE)-vb2-m1-bunch001000-$(ACC)acc$(TIME).png
$(PICTURES_DIR)md-$(MD_DATE)-vb2-m1-bunch010000-$(ACC)acc$(TIME).png: $(BINARY_DIR)md-$(MD_DATE)-vb2-m1-bunch010000-$(ACC)acc$(TIME).bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-$(MD_DATE)-vb2-m1-bunch010000-$(ACC)acc$(TIME).bin -b $(PICTURES_DIR)md-$(MD_DATE)-vb2-m1-bunch010000-$(ACC)acc$(TIME).png
$(PICTURES_DIR)md-$(MD_DATE)-vb2-m1-bunch100000-$(ACC)acc$(TIME).png: $(BINARY_DIR)md-$(MD_DATE)-vb2-m1-bunch100000-$(ACC)acc$(TIME).bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-$(MD_DATE)-vb2-m1-bunch100000-$(ACC)acc$(TIME).bin -b $(PICTURES_DIR)md-$(MD_DATE)-vb2-m1-bunch100000-$(ACC)acc$(TIME).png

# pictures HB1

$(PICTURES_DIR)md-$(MD_DATE)-hb1-m1-bunch000001-$(ACC)acc$(TIME).png: $(BINARY_DIR)md-$(MD_DATE)-hb1-m1-bunch000001-$(ACC)acc$(TIME).bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-$(MD_DATE)-hb1-m1-bunch000001-$(ACC)acc$(TIME).bin -b $(PICTURES_DIR)md-$(MD_DATE)-hb1-m1-bunch000001-$(ACC)acc$(TIME).png
$(PICTURES_DIR)md-$(MD_DATE)-hb1-m1-bunch000010-$(ACC)acc$(TIME).png: $(BINARY_DIR)md-$(MD_DATE)-hb1-m1-bunch000010-$(ACC)acc$(TIME).bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-$(MD_DATE)-hb1-m1-bunch000010-$(ACC)acc$(TIME).bin -b $(PICTURES_DIR)md-$(MD_DATE)-hb1-m1-bunch000010-$(ACC)acc$(TIME).png
$(PICTURES_DIR)md-$(MD_DATE)-hb1-m1-bunch000100-$(ACC)acc$(TIME).png: $(BINARY_DIR)md-$(MD_DATE)-hb1-m1-bunch000100-$(ACC)acc$(TIME).bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-$(MD_DATE)-hb1-m1-bunch000100-$(ACC)acc$(TIME).bin -b $(PICTURES_DIR)md-$(MD_DATE)-hb1-m1-bunch000100-$(ACC)acc$(TIME).png
$(PICTURES_DIR)md-$(MD_DATE)-hb1-m1-bunch001000-$(ACC)acc$(TIME).png: $(BINARY_DIR)md-$(MD_DATE)-hb1-m1-bunch001000-$(ACC)acc$(TIME).bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-$(MD_DATE)-hb1-m1-bunch001000-$(ACC)acc$(TIME).bin -b $(PICTURES_DIR)md-$(MD_DATE)-hb1-m1-bunch001000-$(ACC)acc$(TIME).png
$(PICTURES_DIR)md-$(MD_DATE)-hb1-m1-bunch010000-$(ACC)acc$(TIME).png: $(BINARY_DIR)md-$(MD_DATE)-hb1-m1-bunch010000-$(ACC)acc$(TIME).bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-$(MD_DATE)-hb1-m1-bunch010000-$(ACC)acc$(TIME).bin -b $(PICTURES_DIR)md-$(MD_DATE)-hb1-m1-bunch010000-$(ACC)acc$(TIME).png
$(PICTURES_DIR)md-$(MD_DATE)-hb1-m1-bunch100000-$(ACC)acc$(TIME).png: $(BINARY_DIR)md-$(MD_DATE)-hb1-m1-bunch100000-$(ACC)acc$(TIME).bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-$(MD_DATE)-hb1-m1-bunch100000-$(ACC)acc$(TIME).bin -b $(PICTURES_DIR)md-$(MD_DATE)-hb1-m1-bunch100000-$(ACC)acc$(TIME).png

# pictures HB2

$(PICTURES_DIR)md-$(MD_DATE)-hb2-m1-bunch000001-$(ACC)acc$(TIME).png: $(BINARY_DIR)md-$(MD_DATE)-hb2-m1-bunch000001-$(ACC)acc$(TIME).bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-$(MD_DATE)-hb2-m1-bunch000001-$(ACC)acc$(TIME).bin -b $(PICTURES_DIR)md-$(MD_DATE)-hb2-m1-bunch000001-$(ACC)acc$(TIME).png
$(PICTURES_DIR)md-$(MD_DATE)-hb2-m1-bunch000010-$(ACC)acc$(TIME).png: $(BINARY_DIR)md-$(MD_DATE)-hb2-m1-bunch000010-$(ACC)acc$(TIME).bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-$(MD_DATE)-hb2-m1-bunch000010-$(ACC)acc$(TIME).bin -b $(PICTURES_DIR)md-$(MD_DATE)-hb2-m1-bunch000010-$(ACC)acc$(TIME).png
$(PICTURES_DIR)md-$(MD_DATE)-hb2-m1-bunch000100-$(ACC)acc$(TIME).png: $(BINARY_DIR)md-$(MD_DATE)-hb2-m1-bunch000100-$(ACC)acc$(TIME).bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-$(MD_DATE)-hb2-m1-bunch000100-$(ACC)acc$(TIME).bin -b $(PICTURES_DIR)md-$(MD_DATE)-hb2-m1-bunch000100-$(ACC)acc$(TIME).png
$(PICTURES_DIR)md-$(MD_DATE)-hb2-m1-bunch001000-$(ACC)acc$(TIME).png: $(BINARY_DIR)md-$(MD_DATE)-hb2-m1-bunch001000-$(ACC)acc$(TIME).bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-$(MD_DATE)-hb2-m1-bunch001000-$(ACC)acc$(TIME).bin -b $(PICTURES_DIR)md-$(MD_DATE)-hb2-m1-bunch001000-$(ACC)acc$(TIME).png
$(PICTURES_DIR)md-$(MD_DATE)-hb2-m1-bunch010000-$(ACC)acc$(TIME).png: $(BINARY_DIR)md-$(MD_DATE)-hb2-m1-bunch010000-$(ACC)acc$(TIME).bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-$(MD_DATE)-hb2-m1-bunch010000-$(ACC)acc$(TIME).bin -b $(PICTURES_DIR)md-$(MD_DATE)-hb2-m1-bunch010000-$(ACC)acc$(TIME).png
$(PICTURES_DIR)md-$(MD_DATE)-hb2-m1-bunch100000-$(ACC)acc$(TIME).png: $(BINARY_DIR)md-$(MD_DATE)-hb2-m1-bunch100000-$(ACC)acc$(TIME).bin
	$(DATA_CHECK) -i $(BINARY_DIR)md-$(MD_DATE)-hb2-m1-bunch100000-$(ACC)acc$(TIME).bin -b $(PICTURES_DIR)md-$(MD_DATE)-hb2-m1-bunch100000-$(ACC)acc$(TIME).png

clean:
	rm -f $(ALL)
