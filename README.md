# sinDet

A simple real-time sine wave detector that uses SDL2 for audio capture and display and FFTW3 for frequency analysis.

## Building

Ensure SDL2, SDL2_ttf, and FFTW3 development libraries are installed, then run:

```
make
```

## Usage

Execute the program with `./sinewave_detector`. Use the UP and DOWN arrow keys to adjust how long a tone must be present before it is reported.

Each audio chunk is multiplied by a Hann window before the FFT is performed. Purity is calculated as the ratio of the peak spectral power to the total power, providing more reliable sine wave detection.

