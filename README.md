# sinDet

sinDet is a real-time sine wave detector. It uses SDL2 for audio capture and display and FFTW3 for frequency analysis to identify and track sine components in incoming audio. The detector can lock onto multiple tones simultaneously, tolerating volume fluctuations much like a phase locked loop. A basic spectrum view visualizes the incoming audio so you can see what the application is hearing.

## Building

Run the optional configuration script to verify required tools and libraries:

```sh
./configure
```

### Linux

With dependencies installed, build using the default makefile:

```sh
make
```

### Windows

On Windows, use the alternative makefile:

```sh
make -f Makefile.win
```

## Controls

- **Up/Down Arrow Keys**: Increase or decrease how long a tone must persist before it is reported.
- **Left/Right Arrow Keys**: Decrease or increase input gain to adjust detection sensitivity.
- **Esc**: Exit the application.

## Roadmap

- Cross-platform packaging and binary releases.
- Configurable detection thresholds and audio devices.
- Frequency spectrum visualization and logging improvements.

