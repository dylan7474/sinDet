# Makefile for C Sine Wave Detector
# Matches the style and compilation flags from the user's document
# Assumes SDL2 and FFTW3 development libraries are installed

CC = gcc
TARGET = sinewave_detector
SRCS = main.c
CFLAGS = -Wall -O2 `sdl2-config --cflags` -I/usr/include/fftw3
LDFLAGS = `sdl2-config --libs` -lfftw3 -lm

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) $(SRCS) -o $(TARGET) $(LDFLAGS)

clean:
	rm -f $(TARGET)
