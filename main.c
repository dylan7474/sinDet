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
#define PEAK_SUPPRESS_BINS 2    // Number of neighbouring bins to suppress around a detected peak
#define SINE_WAVE_MIN_HZ 20
#define SINE_WAVE_MAX_HZ 20000
#define FONT_SIZE 12

#define VIS_HEIGHT 150         // Height of the visualization area
#define VIS_PADDING 20         // Padding for the visualization
#define AVERAGING_ALPHA 0.1     // Smoothing factor for optional averaging filter
#define CONFIG_FILE "sinDet.cfg"
#define MORSE_BUFFER_SIZE 256
#define DOT_EST_ALPHA 0.2

// --- Global Variables ---
static SDL_AudioDeviceID deviceId = 0;
static double pcm_buffer[CHUNK_SIZE];
static fftw_complex* out;
static fftw_plan p;
static double freq_resolution;
static double hann_window[FFT_SIZE];
static double magnitudes[FFT_SIZE / 2]; // Stores normalized spectrum magnitudes for visualization
static double avg_powers[FFT_SIZE / 2]; // Smoothed power spectrum when averaging filter is enabled
static bool averaging_enabled = false;  // Toggle for averaging filter

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
    Uint32 tone_start;
    bool active;
} SineTrack;

static SineTrack tracks[MAX_TRACKED_SINES];
static bool keep_running = true;

static double estimated_dot_ms = 120.0;
static char morse_buffer[MORSE_BUFFER_SIZE];
static int morse_length = 0;

// Logging support
#define MAX_LOG_LINES 20
#define LINE_SPACING (FONT_SIZE + 4)
typedef struct {
    char text[256];
    SDL_Color color;
    Uint32 expire_time; // 0 means persistent
    int track_id;       // index of associated track or -1
} LogEntry;
static LogEntry log_entries[MAX_LOG_LINES];
static int log_count = 0;

// Detection persistence control
static int persistence_threshold_ms = 200; // default 0.2s
// Input gain control (dB)
static double input_gain_db = 0.0; // 0 dB default
// Band-pass filter settings
static double bandpass_low_hz = SINE_WAVE_MIN_HZ;
static double bandpass_high_hz = SINE_WAVE_MAX_HZ;
// Squelch settings
static bool squelch_enabled = false;
static double squelch_threshold = 0.02; // normalized 0.0-1.0

// --- Function Prototypes ---
void log_error(const char* msg);
void audio_callback(void* userdata, Uint8* stream, int len);
void render_text(const char* text, int x, int y, SDL_Color color);
void add_log_line(const char* text, SDL_Color color, Uint32 expire_time, int track_id);
void prune_expired_logs(Uint32 now);
void update_track(double freq, double purity, Uint32 now);
void cleanup();
void sdl_log_filter(void* userdata, int category, SDL_LogPriority priority, const char* message);
void save_config(void);
void load_config(void);

int main(int argc, char* argv[]) {
    // --- 1. Initialization ---
    // Suppress less important SDL log messages such as unrecognized key warnings
    SDL_LogSetOutputFunction(sdl_log_filter, NULL);
    SDL_LogSetAllPriority(SDL_LOG_PRIORITY_ERROR);
    load_config();
    SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "Initializing SDL...");
    // Initialize both Audio and Video subsystems
    if (SDL_Init(SDL_INIT_AUDIO | SDL_INIT_VIDEO) < 0) {
        log_error("Failed to initialize SDL");
        return 1;
    }
    // SDL_Init may reset log settings; reapply the custom filter and priority
    SDL_LogSetOutputFunction(sdl_log_filter, NULL);
    SDL_LogSetAllPriority(SDL_LOG_PRIORITY_ERROR);
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
    SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "Successfully initialized graphical interface.");

    // --- 4. FFT Setup ---
    SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "Setting up FFTW3...");
    out = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * (FFT_SIZE / 2 + 1));
    if (!out) {
        log_error("FFTW memory allocation failed for output.");
        cleanup();
        return 1;
    }
    p = fftw_plan_dft_r2c_1d(FFT_SIZE, pcm_buffer, out, FFTW_ESTIMATE);
    freq_resolution = (double)SAMPLE_RATE / (double)FFT_SIZE;
    SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "Frequency resolution: %.2f Hz", freq_resolution);

    for (int i = 0; i < FFT_SIZE; ++i) {
        hann_window[i] = 0.5 * (1.0 - cos((2.0 * M_PI * i) / (FFT_SIZE - 1)));
    }
    
    // --- 5. Audio Device Setup ---
    SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "Opening audio device...");
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
    
    SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "Successfully opened audio device.");
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
                } else if (event.key.keysym.sym == SDLK_DOWN) {
                    if (persistence_threshold_ms > 50) {
                        persistence_threshold_ms -= 50;
                    }
                } else if (event.key.keysym.sym == SDLK_RIGHT) {
                    input_gain_db += 1.0;
                } else if (event.key.keysym.sym == SDLK_LEFT) {
                    input_gain_db -= 1.0;
                } else if (event.key.keysym.sym == SDLK_z) {
                    if (bandpass_low_hz > SINE_WAVE_MIN_HZ) {
                        bandpass_low_hz -= 10.0;
                        if (bandpass_low_hz < SINE_WAVE_MIN_HZ) bandpass_low_hz = SINE_WAVE_MIN_HZ;
                    }
                } else if (event.key.keysym.sym == SDLK_x) {
                    if (bandpass_low_hz < bandpass_high_hz - 10.0) {
                        bandpass_low_hz += 10.0;
                    }
                } else if (event.key.keysym.sym == SDLK_c) {
                    if (bandpass_high_hz > bandpass_low_hz + 10.0) {
                        bandpass_high_hz -= 10.0;
                    }
                } else if (event.key.keysym.sym == SDLK_v) {
                    if (bandpass_high_hz < SINE_WAVE_MAX_HZ) {
                        bandpass_high_hz += 10.0;
                        if (bandpass_high_hz > SINE_WAVE_MAX_HZ) bandpass_high_hz = SINE_WAVE_MAX_HZ;
                    }
                } else if (event.key.keysym.sym == SDLK_a) {
                    averaging_enabled = !averaging_enabled;
                    char log_text[128];
                    sprintf(log_text, "Averaging %s", averaging_enabled ? "ON" : "OFF");
                    Uint32 expire = SDL_GetTicks() + 2000;
                    add_log_line(log_text, (SDL_Color){0, 255, 255, 255}, expire, -1);
                } else if (event.key.keysym.sym == SDLK_s) {
                    squelch_enabled = !squelch_enabled;
                    char log_text[128];
                    sprintf(log_text, "Squelch %s", squelch_enabled ? "ON" : "OFF");
                    Uint32 expire = SDL_GetTicks() + 2000;
                    add_log_line(log_text, (SDL_Color){0, 255, 255, 255}, expire, -1);
                } else if (event.key.keysym.sym == SDLK_d) {
                    if (squelch_threshold > 0.0) {
                        squelch_threshold -= 0.01;
                        if (squelch_threshold < 0.0) squelch_threshold = 0.0;
                    }
                } else if (event.key.keysym.sym == SDLK_f) {
                    if (squelch_threshold < 1.0) {
                        squelch_threshold += 0.01;
                        if (squelch_threshold > 1.0) squelch_threshold = 1.0;
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
                    add_log_line(log_text, (SDL_Color){0, 255, 0, 255}, 0, i);
                }
                prev_active[i] = true;
                prev_freq[i] = snapshot[i].freq;
            } else if (prev_active[i]) {
                char log_text[128];
                sprintf(log_text, "Lost %.2f Hz", prev_freq[i]);
                Uint32 expire = SDL_GetTicks() + 3000;
                add_log_line(log_text, (SDL_Color){255, 255, 0, 255}, expire, i);
                for (int j = log_count - 1; j >= 0; --j) {
                    if (log_entries[j].track_id == i && log_entries[j].expire_time == 0) {
                        log_entries[j].expire_time = expire;
                        break;
                    }
                }
                prev_active[i] = false;
            }
        }
        
        // Clear the screen with a dark gray color
        SDL_SetRenderDrawColor(renderer, 50, 50, 50, 255);
        SDL_RenderClear(renderer);

        // Render status text and controls
        SDL_Color color_white = {255, 255, 255, 255};
        render_text("ESC: exit", 100, 80, color_white);
        render_text("UP/DOWN: adjust persistence", 100, 100, color_white);
        render_text("LEFT/RIGHT: adjust gain", 100, 120, color_white);
        render_text("Z/X: low cutoff  C/V: high cutoff", 100, 140, color_white);
        render_text("A: toggle averaging", 100, 160, color_white);
        render_text("S/D/F: squelch toggle/adjust", 100, 180, color_white);
        char persist_text[80];
        sprintf(persist_text, "Persistence: %d ms", persistence_threshold_ms);
        render_text(persist_text, 100, 200, color_white);
        char gain_text[80];
        sprintf(gain_text, "Gain: %.1f dB", input_gain_db);
        render_text(gain_text, 100, 220, color_white);
        char band_text[120];
        sprintf(band_text, "Band-pass: %.0f-%.0f Hz", bandpass_low_hz, bandpass_high_hz);
        render_text(band_text, 100, 240, color_white);
        char avg_text[80];
        sprintf(avg_text, "Averaging: %s", averaging_enabled ? "ON" : "OFF");
        render_text(avg_text, 100, 260, color_white);
        char squelch_text[80];
        sprintf(squelch_text, "Squelch: %s (%.0f%%)", squelch_enabled ? "ON" : "OFF", squelch_threshold * 100.0);
        render_text(squelch_text, 100, 280, color_white);
        // Render detection result
        int line_y = 300;
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

        // Render Morse buffer and estimated speed
        char morse_snapshot[MORSE_BUFFER_SIZE];
        double dot_snapshot;
        SDL_LockAudioDevice(deviceId);
        strncpy(morse_snapshot, morse_buffer, MORSE_BUFFER_SIZE);
        morse_snapshot[MORSE_BUFFER_SIZE - 1] = '\0';
        dot_snapshot = estimated_dot_ms;
        SDL_UnlockAudioDevice(deviceId);
        render_text(morse_snapshot, 100, line_y, color_white);
        line_y += LINE_SPACING;
        char dotlen_text[80];
        sprintf(dotlen_text, "Dot length: %.0f ms", dot_snapshot);
        render_text(dotlen_text, 100, line_y, color_white);
        line_y += LINE_SPACING;

        // Render log lines
        prune_expired_logs(SDL_GetTicks());
        for (int i = 0; i < log_count; ++i) {
            render_text(log_entries[i].text, 100, 400 + i * LINE_SPACING, log_entries[i].color);
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

        // Highlight band-pass region and block-color out-of-band areas
        SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
        int band_start = VIS_PADDING + (int)((bandpass_low_hz / (SAMPLE_RATE / 2.0)) * vis_width);
        int band_end = VIS_PADDING + (int)((bandpass_high_hz / (SAMPLE_RATE / 2.0)) * vis_width);
        if (band_start < VIS_PADDING) band_start = VIS_PADDING;
        if (band_end > VIS_PADDING + vis_width) band_end = VIS_PADDING + vis_width;

        int vis_left = VIS_PADDING;
        int vis_right = VIS_PADDING + vis_width;
        if (band_start > vis_left) {
            SDL_Rect left_rect = {vis_left, vis_y_start, band_start - vis_left, VIS_HEIGHT};
            SDL_SetRenderDrawColor(renderer, 255, 0, 0, 50);
            SDL_RenderFillRect(renderer, &left_rect);
        }
        if (band_end < vis_right) {
            SDL_Rect right_rect = {band_end, vis_y_start, vis_right - band_end, VIS_HEIGHT};
            SDL_SetRenderDrawColor(renderer, 255, 0, 0, 50);
            SDL_RenderFillRect(renderer, &right_rect);
        }

        SDL_Rect band_rect = {band_start, vis_y_start, band_end - band_start, VIS_HEIGHT};
        SDL_SetRenderDrawColor(renderer, 0, 255, 0, 50);
        SDL_RenderFillRect(renderer, &band_rect);
        SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_NONE);

        int squelch_y = vis_y_end - (int)(squelch_threshold * VIS_HEIGHT);
        if (squelch_y < vis_y_start) squelch_y = vis_y_start;
        if (squelch_y > vis_y_end) squelch_y = vis_y_end;
        SDL_Color sq_color = squelch_enabled ? (SDL_Color){255, 255, 0, 255} : (SDL_Color){100, 100, 100, 255};
        SDL_SetRenderDrawColor(renderer, sq_color.r, sq_color.g, sq_color.b, sq_color.a);
        SDL_RenderDrawLine(renderer, VIS_PADDING, squelch_y, VIS_PADDING + vis_width, squelch_y);

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
    save_config();
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
            tracks[match].tone_start = 0;
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
    double gain = pow(10.0, input_gain_db / 20.0);
    for (int i = 0; i < CHUNK_SIZE; ++i) {
        pcm_buffer[i] = ((double)pcm_stream[i] / MAX_AMPLITUDE) * gain * hann_window[i];
    }
    fftw_execute(p);

    double total_power = 0.0;

    double powers[FFT_SIZE / 2];
    for (int i = 0; i < FFT_SIZE / 2; ++i) {
        double real = out[i][0];
        double imag = out[i][1];
        double power = real * real + imag * imag;
        double freq = i * freq_resolution;
        if (freq < bandpass_low_hz || freq > bandpass_high_hz) {
            power = 0.0; // Apply band-pass filter in frequency domain
        }
        if (averaging_enabled) {
            avg_powers[i] = AVERAGING_ALPHA * power + (1.0 - AVERAGING_ALPHA) * avg_powers[i];
            power = avg_powers[i];
        } else {
            avg_powers[i] = power;
        }
        powers[i] = power;
    }

    /*
     * Normalize spectrum magnitudes against the theoretical maximum power of a
     * full-scale sine wave so that input gain changes are reflected in the
     * visualization. Previously the magnitudes were normalised by the maximum
     * power of the current frame, which hid overall amplitude variations.
     *
     * For a Hann-windowed, full-scale sine wave the peak power is roughly
     * (FFT_SIZE/4)^2.  Scaling by this constant keeps magnitudes in the
     * 0.0-1.0 range while allowing gain adjustments to impact the display.
     */
    double max_possible_power = (FFT_SIZE / 4.0) * (FFT_SIZE / 4.0);
    total_power = 0.0;
    for (int i = 0; i < FFT_SIZE / 2; ++i) {
        double norm = powers[i] / max_possible_power;
        if (norm > 1.0) {
            norm = 1.0;
        }
        if (squelch_enabled && norm < squelch_threshold) {
            powers[i] = 0.0;
            norm = 0.0;
        }
        magnitudes[i] = norm;
        total_power += powers[i];
    }

    // Find top peaks while merging nearby bins to avoid duplicate detections
    int top_indices[MAX_TRACKED_SINES];
    for (int i = 0; i < MAX_TRACKED_SINES; ++i) {
        top_indices[i] = -1;
    }

    bool used[FFT_SIZE / 2] = {false};
    for (int p = 0; p < MAX_TRACKED_SINES; ++p) {
        int best = -1;
        double best_power = 0.0;
        for (int i = 1; i < FFT_SIZE / 2 - 1; ++i) {
            if (used[i]) continue;
            double power = powers[i];
            if (power > best_power && power > powers[i - 1] && power >= powers[i + 1]) {
                best_power = power;
                best = i;
            }
        }
        if (best == -1) {
            break;
        }
        top_indices[p] = best;
        for (int k = best - PEAK_SUPPRESS_BINS; k <= best + PEAK_SUPPRESS_BINS; ++k) {
            if (k >= 0 && k < FFT_SIZE / 2) {
                used[k] = true;
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
            freq >= bandpass_low_hz &&
            freq <= bandpass_high_hz) {
            update_track(freq, purity, now);
        }
    }

    // update track states
    for (int i = 0; i < MAX_TRACKED_SINES; ++i) {
        if (tracks[i].start_time != 0 && !tracks[i].active) {
            if (now - tracks[i].start_time >= (Uint32)persistence_threshold_ms) {
                tracks[i].active = true;
                tracks[i].last_seen = now;
                tracks[i].tone_start = now;
            }
        } else if (tracks[i].active) {
            if (now - tracks[i].last_seen >= (Uint32)persistence_threshold_ms) {
                Uint32 tone_duration = tracks[i].last_seen - tracks[i].tone_start;
                char symbol;
                if (tone_duration < estimated_dot_ms * 2.0) {
                    symbol = '.';
                    estimated_dot_ms = (1.0 - DOT_EST_ALPHA) * estimated_dot_ms + DOT_EST_ALPHA * tone_duration;
                } else {
                    symbol = '-';
                    estimated_dot_ms = (1.0 - DOT_EST_ALPHA) * estimated_dot_ms + DOT_EST_ALPHA * (tone_duration / 3.0);
                }
                if (morse_length < MORSE_BUFFER_SIZE - 1) {
                    morse_buffer[morse_length++] = symbol;
                    morse_buffer[morse_length] = '\0';
                }
                tracks[i].active = false;
                tracks[i].start_time = 0;
                tracks[i].tone_start = 0;
            }
        }
    }
}

// --- Helper Functions ---
void add_log_line(const char* text, SDL_Color color, Uint32 expire_time, int track_id) {
    SDL_LockAudioDevice(deviceId); // Prevent race condition with audio thread
    if (log_count < MAX_LOG_LINES) {
        strncpy(log_entries[log_count].text, text, sizeof(log_entries[log_count].text) - 1);
        log_entries[log_count].text[sizeof(log_entries[log_count].text) - 1] = '\0';
        log_entries[log_count].color = color;
        log_entries[log_count].expire_time = expire_time;
        log_entries[log_count].track_id = track_id;
        log_count++;
    } else {
        for (int i = 1; i < MAX_LOG_LINES; ++i) {
            log_entries[i - 1] = log_entries[i];
        }
        strncpy(log_entries[MAX_LOG_LINES - 1].text, text, sizeof(log_entries[MAX_LOG_LINES - 1].text) - 1);
        log_entries[MAX_LOG_LINES - 1].text[sizeof(log_entries[MAX_LOG_LINES - 1].text) - 1] = '\0';
        log_entries[MAX_LOG_LINES - 1].color = color;
        log_entries[MAX_LOG_LINES - 1].expire_time = expire_time;
        log_entries[MAX_LOG_LINES - 1].track_id = track_id;
    }
    SDL_UnlockAudioDevice(deviceId);
}

void prune_expired_logs(Uint32 now) {
    SDL_LockAudioDevice(deviceId);
    int dst = 0;
    for (int i = 0; i < log_count; ++i) {
        if (log_entries[i].expire_time && now >= log_entries[i].expire_time) {
            continue;
        }
        if (dst != i) {
            log_entries[dst] = log_entries[i];
        }
        dst++;
    }
    log_count = dst;
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

void save_config(void) {
    FILE* f = fopen(CONFIG_FILE, "w");
    if (!f) {
        return;
    }
    fprintf(f, "persistence_threshold_ms=%d\n", persistence_threshold_ms);
    fprintf(f, "input_gain_db=%.2f\n", input_gain_db);
    fprintf(f, "bandpass_low_hz=%.2f\n", bandpass_low_hz);
    fprintf(f, "bandpass_high_hz=%.2f\n", bandpass_high_hz);
    fprintf(f, "averaging_enabled=%d\n", averaging_enabled ? 1 : 0);
    fprintf(f, "squelch_enabled=%d\n", squelch_enabled ? 1 : 0);
    fprintf(f, "squelch_threshold=%.2f\n", squelch_threshold);
    fclose(f);
}

void load_config(void) {
    FILE* f = fopen(CONFIG_FILE, "r");
    if (!f) {
        return;
    }
    char line[128];
    while (fgets(line, sizeof(line), f)) {
        int i;
        double d;
        if (sscanf(line, "persistence_threshold_ms=%d", &i) == 1) {
            persistence_threshold_ms = i;
        } else if (sscanf(line, "input_gain_db=%lf", &d) == 1) {
            input_gain_db = d;
        } else if (sscanf(line, "bandpass_low_hz=%lf", &d) == 1) {
            bandpass_low_hz = d;
        } else if (sscanf(line, "bandpass_high_hz=%lf", &d) == 1) {
            bandpass_high_hz = d;
        } else if (sscanf(line, "averaging_enabled=%d", &i) == 1) {
            averaging_enabled = i ? true : false;
        } else if (sscanf(line, "squelch_enabled=%d", &i) == 1) {
            squelch_enabled = i ? true : false;
        } else if (sscanf(line, "squelch_threshold=%lf", &d) == 1) {
            squelch_threshold = d;
        }
    }
    fclose(f);
}

void sdl_log_filter(void* userdata, int category, SDL_LogPriority priority, const char* message) {
    if (strstr(message, "not recognized by SDL") != NULL) {
        return; // suppress unrecognized key warnings
    }
    if (priority >= SDL_LOG_PRIORITY_ERROR) {
        fprintf(stderr, "%s\n", message);
    }
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
