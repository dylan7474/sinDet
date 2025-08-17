#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <fftw3.h>
#include <signal.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Include the separate font header file that you have.
// We will assume the font data is provided in this header file.
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
#define FONT_SIZE 12

// --- Global Variables ---
static SDL_AudioDeviceID deviceId = 0;
static double pcm_buffer[CHUNK_SIZE];
static fftw_complex* out;
static fftw_plan p;
static double freq_resolution;
static double hann_window[FFT_SIZE];
static double magnitudes[FFT_SIZE / 2]; // Stores normalized spectrum magnitudes for visualization

static SDL_Window* window = NULL;
static SDL_Renderer* renderer = NULL;
static TTF_Font* font = NULL;

// Global state variables for graphical output
static double detected_freq = 0.0;
static double detected_purity = 0.0;
static bool is_detecting_sine = false;
static bool keep_running = true;

// Logging support
#define MAX_LOG_LINES 20
#define LINE_SPACING (FONT_SIZE + 4)
static char log_lines[MAX_LOG_LINES][256];
static SDL_Color log_colors[MAX_LOG_LINES];
static int log_count = 0;

// Detection persistence control
static int persistence_threshold_ms = 200; // default 0.2s
static Uint32 detection_start_time = 0;

// --- Function Prototypes ---
void log_error(const char* msg);
void audio_callback(void* userdata, Uint8* stream, int len);
void render_text(const char* text, int x, int y, SDL_Color color);
void add_log_line(const char* text, SDL_Color color);
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
    // Load the font from the embedded font data in font.h
    SDL_RWops* rw = SDL_RWFromConstMem(font_ttf, sizeof(font_ttf));
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

    for (int i = 0; i < FFT_SIZE; ++i) {
        hann_window[i] = 0.5 * (1.0 - cos((2.0 * M_PI * i) / (FFT_SIZE - 1)));
    }
    
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
                if (event.key.keysym.sym == SDLK_ESCAPE) {
                    keep_running = false;
                } else if (event.key.keysym.sym == SDLK_UP) {
                    persistence_threshold_ms += 50;
                    char msg[64];
                    sprintf(msg, "Persistence set to %d ms", persistence_threshold_ms);
                    add_log_line(msg, (SDL_Color){200, 200, 200, 255});
                } else if (event.key.keysym.sym == SDLK_DOWN) {
                    if (persistence_threshold_ms > 50) {
                        persistence_threshold_ms -= 50;
                        char msg[64];
                        sprintf(msg, "Persistence set to %d ms", persistence_threshold_ms);
                        add_log_line(msg, (SDL_Color){200, 200, 200, 255});
                    }
                }
            }
        }

        static bool last_detection = false;
        static double last_logged_freq = 0.0;
        if (is_detecting_sine) {
            if (!last_detection || fabs(detected_freq - last_logged_freq) > FREQUENCY_TOLERANCE) {
                char log_text[128];
                sprintf(log_text, "Detected %.2f Hz (%.2f%%)", detected_freq, detected_purity * 100.0);
                add_log_line(log_text, (SDL_Color){0, 255, 0, 255});
                last_logged_freq = detected_freq;
            }
        } else if (last_detection) {
            add_log_line("Detection lost", (SDL_Color){255, 255, 0, 255});
        }
        last_detection = is_detecting_sine;
        
        // Clear the screen with a dark gray color
        SDL_SetRenderDrawColor(renderer, 50, 50, 50, 255);
        SDL_RenderClear(renderer);

        // Render status text
        SDL_Color color_white = {255, 255, 255, 255};
        render_text("Listening for sine waves...", 100, 100, color_white);
        render_text("Press ESC to exit.", 100, 150, color_white);
        char persist_text[80];
        sprintf(persist_text, "Detection persistence: %d ms (UP/DOWN adjust)", persistence_threshold_ms);
        render_text(persist_text, 100, 200, color_white);

        // Render detection result
        char output_text[100];
        SDL_Color result_color;

        if (is_detecting_sine) {
            sprintf(output_text, "Sine wave detected! Freq: %.2f Hz | Purity: %.2f%%", detected_freq, detected_purity * 100.0);
            result_color = (SDL_Color){0, 255, 0, 255}; // Green
        } else {
            sprintf(output_text, "No pure sine wave detected. Listening...");
            result_color = (SDL_Color){255, 255, 0, 255}; // Yellow
        }
        render_text(output_text, 100, 250, result_color);

        // Render log lines
        for (int i = 0; i < log_count; ++i) {
            render_text(log_lines[i], 100, 300 + i * LINE_SPACING, log_colors[i]);
        }

        // Render simple frequency spectrum visualization
        int w, h;
        SDL_GetRendererOutputSize(renderer, &w, &h);
        int spectrum_height = 100;
        SDL_SetRenderDrawColor(renderer, 0, 128, 255, 255);
        SDL_LockAudioDevice(deviceId);
        for (int i = 0; i < FFT_SIZE / 2; ++i) {
            int x = (int)((double)i / (FFT_SIZE / 2) * w);
            int bar_height = (int)(magnitudes[i] * spectrum_height);
            SDL_RenderDrawLine(renderer, x, h - 1, x, h - 1 - bar_height);
        }
        SDL_UnlockAudioDevice(deviceId);
        
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
        pcm_buffer[i] = ((double)pcm_stream[i] / MAX_AMPLITUDE) * hann_window[i];
    }
    fftw_execute(p);

    double max_power = 0.0;
    int max_index = -1;
    double total_power = 0.0;

    for (int i = 0; i < FFT_SIZE / 2; ++i) {
        double real = out[i][0];
        double imag = out[i][1];
        double power = real * real + imag * imag;
        magnitudes[i] = power;
        total_power += power;

        if (power > max_power) {
            max_power = power;
            max_index = i;
        }
    }

    if (max_index != -1 && total_power > 0) {
        double current_detected_freq = max_index * freq_resolution;
        double peak_power = 0.0;
        for (int j = -1; j <= 1; ++j) {
            int idx = max_index + j;
            if (idx >= 0 && idx < FFT_SIZE / 2) {
                peak_power += magnitudes[idx];
            }
        }
        double purity = peak_power / total_power;

        Uint32 now = SDL_GetTicks();
        if (purity > DETECT_THRESHOLD &&
            current_detected_freq >= SINE_WAVE_MIN_HZ &&
            current_detected_freq <= SINE_WAVE_MAX_HZ) {

            if (detection_start_time == 0) {
                detection_start_time = now;
            }
            Uint32 elapsed = now - detection_start_time;
            if (elapsed >= (Uint32)persistence_threshold_ms) {
                if (fabs(current_detected_freq - detected_freq) > FREQUENCY_TOLERANCE) {
                    detected_freq = current_detected_freq;
                    detected_purity = purity;
                }
                is_detecting_sine = true;
            } else {
                is_detecting_sine = false;
            }
        } else {
            detection_start_time = 0;
            is_detecting_sine = false;
        }
    }

    if (max_power > 0) {
        for (int i = 0; i < FFT_SIZE / 2; ++i) {
            magnitudes[i] /= max_power;
        }
    }
}

// --- Helper Functions ---
void add_log_line(const char* text, SDL_Color color) {
    if (log_count < MAX_LOG_LINES) {
        strncpy(log_lines[log_count], text, sizeof(log_lines[log_count]) - 1);
        log_lines[log_count][sizeof(log_lines[log_count]) - 1] = '\0';
        log_colors[log_count] = color;
        log_count++;
    } else {
        for (int i = 1; i < MAX_LOG_LINES; ++i) {
            strcpy(log_lines[i - 1], log_lines[i]);
            log_colors[i - 1] = log_colors[i];
        }
        strncpy(log_lines[MAX_LOG_LINES - 1], text, sizeof(log_lines[MAX_LOG_LINES - 1]) - 1);
        log_lines[MAX_LOG_LINES - 1][sizeof(log_lines[MAX_LOG_LINES - 1]) - 1] = '\0';
        log_colors[MAX_LOG_LINES - 1] = color;
    }
}

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
