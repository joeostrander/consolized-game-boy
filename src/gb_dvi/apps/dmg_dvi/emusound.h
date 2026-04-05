#ifndef EMUSOUND_H
#define EMUSOUND_H

#include <stdbool.h>
#include <stdint.h>

#define SAMPLE_FREQ         32000
// Use power of 2 for efficient modulo operations
// 32000 Hz divides perfectly: 32000/50 = 640, 640/5 = 128 samples per tick (with 4ms tick)
#define NUMSAMPLES          (SAMPLE_FREQ / 50)   // 640 samples per frame at 50Hz @ 32000
#define ZEROSOUND 0                     // Zero point for sound

// Audio processing cadence in milliseconds (matches ADC chunk size)
#define AUDIO_TICK_MS       4

// ADC buffer size MUST match PIO chunk size for synchronization!
// 128 samples @ 32000 Hz = 4ms (matches PIO callback interval)
// Balanced to limit IRQ load while avoiding long-latency audio buzz
#define ADC_CHUNK_SIZE      (128)       // Must match TICK_SAMPLES
#define AUDIO_BUFFER_SIZE   (2048)      // HDMI ring buffer (separate from ADC)

void emu_sndInit(bool playSound, bool reset, audio_ring_t* audio_ring, int16_t* sample_buff);  // JOE ADDED audio_ring, sample buffer
// void emu_generateSoundSamples(void);
void emu_silenceSound(void);
uint16_t emu_SoundSampleRate(void);
// Enable manual ticking (skip repeating timer) when true; call emu_audio_manual_tick() periodically
void emu_audio_set_manual_tick(bool manual_mode);
void emu_audio_manual_tick(void);
// Apply software gain multiplier to captured audio (default 1.0f)
void emu_audio_set_gain(float gain);
float emu_audio_get_gain(void);
// Enable single-pole low-pass filter; pass cutoff_hz<=0 to disable (no filtering)
void emu_audio_set_lowpass(float cutoff_hz);
// Enable single-pole high-pass filter (DC blocker); pass cutoff_hz<=0 to disable.
void emu_audio_set_highpass(float cutoff_hz);
float emu_audio_get_highpass(void);



#endif