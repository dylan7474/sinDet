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

#define VIS_HEIGHT 150         // Height of the visualization area
#define VIS_PADDING 20         // Padding for the visualization

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

// Sine tracking structure
#define MAX_TRACKED_SINES 5
typedef struct {
    double freq;
    double purity;
    Uint32 start_time;
    Uint32 last_seen;
    bool active;
} SineTrack;

static SineTrack tracks[MAX_TRACKED_SINES];
static bool keep_running = true;

// Logging support
#define MAX_LOG_LINES 20
#define LINE_SPACING (FONT_SIZE + 4)
static char log_lines[MAX_LOG_LINES][256];
static SDL_Color log_colors[MAX_LOG_LINES];
static int log_count = 0;

// Detection persistence control
static int persistence_threshold_ms = 200; // default 0.2s

// --- Function Prototypes ---
void log_error(const char* msg);
void audio_callback(void* userdata, Uint8* stream, int len);
void render_text(const char* text, int x, int y, SDL_Color color);
void add_log_line(const char* text, SDL_Color color);
void update_track(double freq, double purity, Uint32 now);
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
    int window_width = 800, window_height = 600;
    window = SDL_CreateWindow("Sine Wave Detector", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, window_width, window_height, SDL_WINDOW_FULLSCREEN_DESKTOP);
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

    SDL_GetWindowSize(window, &window_width, &window_height);

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

        SineTrack snapshot[MAX_TRACKED_SINES];
        SDL_LockAudioDevice(deviceId);
        memcpy(snapshot, tracks, sizeof(tracks));
        SDL_UnlockAudioDevice(deviceId);

        static bool prev_active[MAX_TRACKED_SINES] = {false};
        static double prev_freq[MAX_TRACKED_SINES] = {0.0};
        for (int i = 0; i < MAX_TRACKED_SINES; ++i) {
            if (snapshot[i].active) {
                if (!prev_active[i] || fabs(snapshot[i].freq - prev_freq[i]) > FREQUENCY_TOLERANCE) {
                    char log_text[128];
                    sprintf(log_text, "Detected %.2f Hz (%.2f%% purity)", snapshot[i].freq, snapshot[i].purity);
                    add_log_line(log_text, (SDL_Color){0, 255, 0, 255});
                }
                prev_active[i] = true;
                prev_freq[i] = snapshot[i].freq;
            } else if (prev_active[i]) {
                char log_text[128];
                sprintf(log_text, "Lost %.2f Hz", prev_freq[i]);
                add_log_line(log_text, (SDL_Color){255, 255, 0, 255});
                prev_active[i] = false;
            }
        }
        
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
        int line_y = 250;
        int active_count = 0;
        for (int i = 0; i < MAX_TRACKED_SINES; ++i) {
            if (snapshot[i].active) {
                char output_text[100];
                sprintf(output_text, "Sine wave detected! Freq: %.2f Hz | Purity: %.2f%%", snapshot[i].freq, snapshot[i].purity);
                render_text(output_text, 100, line_y, (SDL_Color){0, 255, 0, 255});
                line_y += LINE_SPACING;
                active_count++;
            }
        }
        if (active_count == 0) {
            render_text("No pure sine wave detected. Listening...", 100, line_y, (SDL_Color){255, 255, 0, 255});
            line_y += LINE_SPACING;
        }

        // Render log lines
        for (int i = 0; i < log_count; ++i) {
            render_text(log_lines[i], 100, 300 + i * LINE_SPACING, log_colors[i]);
        }

        // --- Render frequency spectrum visualization ---
        int vis_y_start = window_height - VIS_HEIGHT - VIS_PADDING;
        int vis_y_end = window_height - VIS_PADDING;
        int vis_width = window_width - VIS_PADDING * 2;

        // Draw background for visualization
        SDL_Rect vis_bg = {VIS_PADDING, vis_y_start, vis_width, VIS_HEIGHT};
        SDL_SetRenderDrawColor(renderer, 30, 30, 30, 255);
        SDL_RenderFillRect(renderer, &vis_bg);

        // Draw frequency line graph
        SDL_SetRenderDrawColor(renderer, 0, 128, 255, 255);
        SDL_Point points[FFT_SIZE / 2];
        SDL_LockAudioDevice(deviceId); // Lock audio to safely access magnitudes
        for (int i = 0; i < FFT_SIZE / 2; ++i) {
            int x = VIS_PADDING + (int)((double)i / (FFT_SIZE / 2) * vis_width);
            int bar_height = (int)(magnitudes[i] * VIS_HEIGHT);
            points[i].x = x;
            points[i].y = vis_y_end - bar_height;
        }
        SDL_UnlockAudioDevice(deviceId);
        SDL_RenderDrawLines(renderer, points, FFT_SIZE / 2);

        // Highlight detected frequencies
        for (int i = 0; i < MAX_TRACKED_SINES; ++i) {
            if (snapshot[i].active) {
                int freq_bin = (int)(snapshot[i].freq / freq_resolution);
                if (freq_bin >= 0 && freq_bin < FFT_SIZE / 2) {
                    int x = VIS_PADDING + (int)((double)freq_bin / (FFT_SIZE / 2) * vis_width);
                    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255); // Red highlight
                    SDL_RenderDrawLine(renderer, x, vis_y_start, x, vis_y_end);
                }
            }
        }
        // --- End of visualization ---
        
        // Update the screen
        SDL_RenderPresent(renderer);

        SDL_Delay(10);
    }
    
    // --- 7. Cleanup ---
    cleanup();

    return 0;
}

void update_track(double freq, double purity, Uint32 now) {
    int match = -1;
    for (int i = 0; i < MAX_TRACKED_SINES; ++i) {
        if (tracks[i].start_time != 0 && fabs(tracks[i].freq - freq) <= FREQUENCY_TOLERANCE) {
            match = i;
            break;
        }
    }
    if (match == -1) {
        for (int i = 0; i < MAX_TRACKED_SINES; ++i) {
            if (tracks[i].start_time == 0) {
                match = i;
                break;
            }
        }
    }
    if (match != -1) {
        if (tracks[match].start_time == 0) {
            tracks[match].freq = freq;
            tracks[match].purity = purity * 100.0;
            tracks[match].start_time = now;
            tracks[match].last_seen = now;
            tracks[match].active = false;
        } else {
            tracks[match].freq = tracks[match].freq * 0.9 + freq * 0.1;
            tracks[match].purity = purity * 100.0;
            tracks[match].last_seen = now;
        }
    }
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
    double total_power = 0.0;

    double powers[FFT_SIZE / 2];
    for (int i = 0; i < FFT_SIZE / 2; ++i) {
        double real = out[i][0];
        double imag = out[i][1];
        double power = real * real + imag * imag;
        powers[i] = power;
        total_power += power;
        if (power > max_power) {
            max_power = power;
        }
    }

    if (max_power > 0) {
        for (int i = 0; i < FFT_SIZE / 2; ++i) {
            magnitudes[i] = powers[i] / max_power;
        }
    }

    // Find top peaks
    int top_indices[MAX_TRACKED_SINES];
    double top_powers[MAX_TRACKED_SINES];
    for (int i = 0; i < MAX_TRACKED_SINES; ++i) {
        top_indices[i] = -1;
        top_powers[i] = 0.0;
    }

    for (int i = 0; i < FFT_SIZE / 2; ++i) {
        double power = powers[i];
        // insert into top arrays if large
        for (int j = 0; j < MAX_TRACKED_SINES; ++j) {
            if (power > top_powers[j]) {
                for (int k = MAX_TRACKED_SINES - 1; k > j; --k) {
                    top_powers[k] = top_powers[k - 1];
                    top_indices[k] = top_indices[k - 1];
                }
                top_powers[j] = power;
                top_indices[j] = i;
                break;
            }
        }
    }

    Uint32 now = SDL_GetTicks();
    for (int i = 0; i < MAX_TRACKED_SINES; ++i) {
        int idx = top_indices[i];
        if (idx == -1 || total_power == 0.0) {
            continue;
        }
        double freq = idx * freq_resolution;
        double peak_power = 0.0;
        for (int j = -1; j <= 1; ++j) {
            int n = idx + j;
            if (n >= 0 && n < FFT_SIZE / 2) {
                peak_power += powers[n];
            }
        }
        double purity = peak_power / total_power;
        if (purity > DETECT_THRESHOLD &&
            freq >= SINE_WAVE_MIN_HZ &&
            freq <= SINE_WAVE_MAX_HZ) {
            update_track(freq, purity, now);
        }
    }

    // update track states
    for (int i = 0; i < MAX_TRACKED_SINES; ++i) {
        if (tracks[i].start_time != 0 && !tracks[i].active) {
            if (now - tracks[i].start_time >= (Uint32)persistence_threshold_ms) {
                tracks[i].active = true;
                tracks[i].last_seen = now;
            }
        } else if (tracks[i].active) {
            if (now - tracks[i].last_seen >= (Uint32)persistence_threshold_ms) {
                tracks[i].active = false;
                tracks[i].start_time = 0;
            }
        }
    }
}

// --- Helper Functions ---
void add_log_line(const char* text, SDL_Color color) {
    SDL_LockAudioDevice(deviceId); // Prevent race condition with audio thread
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
    SDL_UnlockAudioDevice(deviceId);
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
