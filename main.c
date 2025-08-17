#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <fftw3.h>
#include <signal.h>

// Include the separate font header file that you have.
// We will assume the font data is provided in this header file as 'font_data'.
#include "font.h"

// --- Configuration Constants ---
#define SAMPLE_RATE 44100
#define CHUNK_SIZE 2048
#define FFT_SIZE CHUNK_SIZE
#define MAX_AMPLITUDE 32768.0 // Maximum value for a 16-bit signed integer
#define DETECT_THRESHOLD 0.7   // A value from 0.0 to 1.0 for sine wave purity
#define FREQUENCY_TOLERANCE 5.0 // Tolerance in Hz to avoid flickering output
#define SINE_WAVE_MIN_HZ 20
#define SINE_WAVE_MAX_HZ 20000
#define FONT_SIZE 24

// --- Global Variables ---
static SDL_AudioDeviceID deviceId = 0;
static double pcm_buffer[CHUNK_SIZE];
static fftw_complex* out;
static fftw_plan p;
static double freq_resolution;

static SDL_Window* window = NULL;
static SDL_Renderer* renderer = NULL;
static TTF_Font* font = NULL;

// Global state variables for graphical output
static double detected_freq = 0.0;
static double detected_magnitude = 0.0;
static bool is_detecting_sine = false;
static bool keep_running = true;

// --- Function Prototypes ---
void log_error(const char* msg);
void audio_callback(void* userdata, Uint8* stream, int len);
void render_text(const char* text, int x, int y, SDL_Color color);
void cleanup();

int main(int argc, char* argv[]) {
    // --- 1. Initialization ---
    printf("Initializing SDL...\n");
    // Initialize both Audio and Video subsystems
    if (SDL_Init(SDL_INIT_AUDIO | SDL_INIT_VIDEO) < 0) {
        log_error("Failed to initialize SDL");
        return 1;
    }
    // Initialize the SDL_ttf library for text rendering
    if (TTF_Init() == -1) {
        log_error("Failed to initialize SDL_ttf");
        SDL_Quit();
        return 1;
    }

    // --- 2. Window and Renderer Setup ---
    window = SDL_CreateWindow("Sine Wave Detector", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 800, 600, SDL_WINDOW_FULLSCREEN);
    if (!window) {
        log_error("Failed to create window");
        cleanup();
        return 1;
    }
    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) {
        log_error("Failed to create renderer");
        cleanup();
        return 1;
    }

    // --- 3. Font Setup ---
    // Load the font from the font.h file
    SDL_RWops* rw = SDL_RWFromConstMem(font_data, sizeof(font_data));
    if (!rw) {
        log_error("Failed to create SDL_RWops from font data");
        cleanup();
        return 1;
    }
    font = TTF_OpenFontRW(rw, 1, FONT_SIZE);
    if (!font) {
        log_error("Failed to load font from RWops");
        cleanup();
        return 1;
    }
    printf("Successfully initialized graphical interface.\n");

    // --- 4. FFT Setup ---
    printf("Setting up FFTW3...\n");
    out = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * (FFT_SIZE / 2 + 1));
    if (!out) {
        log_error("FFTW memory allocation failed for output.");
        cleanup();
        return 1;
    }
    p = fftw_plan_dft_r2c_1d(FFT_SIZE, pcm_buffer, out, FFTW_ESTIMATE);
    freq_resolution = (double)SAMPLE_RATE / (double)FFT_SIZE;
    printf("Frequency resolution: %.2f Hz\n", freq_resolution);
    
    // --- 5. Audio Device Setup ---
    printf("Opening audio device...\n");
    SDL_AudioSpec want, have;
    SDL_zero(want);
    want.freq = SAMPLE_RATE;
    want.format = AUDIO_S16SYS; 
    want.channels = 1;          
    want.samples = CHUNK_SIZE;
    want.callback = audio_callback;

    deviceId = SDL_OpenAudioDevice(NULL, 1, &want, &have, 0);
    if (deviceId == 0) {
        log_error("Failed to open audio device");
        cleanup();
        return 1;
    }
    
    printf("Successfully opened audio device.\n");
    SDL_PauseAudioDevice(deviceId, 0); // Start capturing

    // --- 6. Main Loop with Event Handling and Rendering ---
    SDL_Event event;
    while (keep_running) {
        // Process all events in the queue
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                keep_running = false;
            } else if (event.type == SDL_KEYDOWN) {
                // Exit on Escape key press
                if (event.key.keysym.sym == SDLK_ESCAPE) {
                    keep_running = false;
                }
            }
        }
        
        // Clear the screen with a dark gray color
        SDL_SetRenderDrawColor(renderer, 50, 50, 50, 255);
        SDL_RenderClear(renderer);

        // Render status text
        SDL_Color color_white = {255, 255, 255, 255};
        render_text("Listening for sine waves...", 100, 100, color_white);
        render_text("Press ESC to exit.", 100, 150, color_white);

        // Render detection result
        char output_text[100];
        SDL_Color result_color;

        if (is_detecting_sine) {
            sprintf(output_text, "Sine wave detected! Freq: %.2f Hz | Purity: %.2f%%", detected_freq, detected_magnitude);
            result_color = (SDL_Color){0, 255, 0, 255}; // Green
        } else {
            sprintf(output_text, "No pure sine wave detected. Listening...");
            result_color = (SDL_Color){255, 255, 0, 255}; // Yellow
        }
        render_text(output_text, 100, 300, result_color);
        
        // Update the screen
        SDL_RenderPresent(renderer);

        SDL_Delay(10);
    }
    
    // --- 7. Cleanup ---
    cleanup();

    return 0;
}

// --- Audio Callback Function ---
// This function is called by SDL whenever it has a new chunk of audio data
void audio_callback(void* userdata, Uint8* stream, int len) {
    Sint16* pcm_stream = (Sint16*)stream;
    
    for (int i = 0; i < CHUNK_SIZE; ++i) {
        pcm_buffer[i] = (double)pcm_stream[i] / MAX_AMPLITUDE; 
    }
    fftw_execute(p);

    double max_magnitude = 0.0;
    int max_index = -1;
    double total_power = 0.0;

    for (int i = 0; i < FFT_SIZE / 2; ++i) {
        double magnitude = sqrt(out[i][0] * out[i][0] + out[i][1] * out[i][1]);
        total_power += magnitude;

        if (magnitude > max_magnitude) {
            max_magnitude = magnitude;
            max_index = i;
        }
    }

    if (max_index != -1 && total_power > 0) {
        double current_detected_freq = max_index * freq_resolution;
        double purity = (max_magnitude / total_power) * 100;

        if (purity > DETECT_THRESHOLD &&
            current_detected_freq >= SINE_WAVE_MIN_HZ &&
            current_detected_freq <= SINE_WAVE_MAX_HZ) {
            
            // Debounce the output to avoid flickering text
            if (fabs(current_detected_freq - detected_freq) > FREQUENCY_TOLERANCE) {
                detected_freq = current_detected_freq;
                detected_magnitude = purity;
            }
            is_detecting_sine = true;
        } else {
            is_detecting_sine = false;
        }
    }
}

// --- Helper Functions ---
void render_text(const char* text, int x, int y, SDL_Color color) {
    SDL_Surface* surface = TTF_RenderText_Solid(font, text, color);
    if (!surface) {
        return;
    }
    SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);
    if (!texture) {
        SDL_FreeSurface(surface);
        return;
    }

    SDL_Rect rect;
    rect.x = x;
    rect.y = y;
    rect.w = surface->w;
    rect.h = surface->h;

    SDL_RenderCopy(renderer, texture, NULL, &rect);

    SDL_FreeSurface(surface);
    SDL_DestroyTexture(texture);
}

void log_error(const char* msg) {
    fprintf(stderr, "ERROR: %s - %s\n", msg, SDL_GetError());
}

void cleanup() {
    if (deviceId) {
        SDL_CloseAudioDevice(deviceId);
    }
    if (p) {
        fftw_destroy_plan(p);
        fftw_free(out);
    }
    if (font) {
        TTF_CloseFont(font);
    }
    if (renderer) {
        SDL_DestroyRenderer(renderer);
    }
    if (window) {
        SDL_DestroyWindow(window);
    }
    TTF_Quit();
    SDL_Quit();
}
