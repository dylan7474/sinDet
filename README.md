# sinDet

sinDet is a simple real-time sine wave detector. It uses SDL2 for audio capture and display and FFTW3 for frequency analysis to show the strongest sine component in incoming audio. A basic spectrum view visualizes the incoming audio so you can see what the application is hearing.

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
- **Esc**: Exit the application.

## Roadmap

- Cross-platform packaging and binary releases.
- Configurable detection thresholds and audio devices.
- Frequency spectrum visualization and logging improvements.

