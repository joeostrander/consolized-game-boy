#include "pico.h"
#include <stdlib.h>
#include <limits.h>
#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/irq.h"
#include "hardware/timer.h"
// Map timer IRQ across RP2040/RP2350 naming
#if defined(TIMER_IRQ_0)
#define AUDIO_TIMER_IRQ TIMER_IRQ_0
#elif defined(TIMER0_IRQ_0)
#define AUDIO_TIMER_IRQ TIMER0_IRQ_0
#else
#define AUDIO_TIMER_IRQ -1
#endif
#include "hardware/clocks.h"
#include "pico/sync.h"

#include "audio_ring.h"
#include "emusound.h"

// Set to 1 to test with sine wave, 0 for real microphone input
#define TEST_WITH_SINE_WAVE 0

static bool __time_critical_func(audio_timer_callback)(__unused repeating_timer_t* rt);
static void __time_critical_func(audio_service_tick)(void);

semaphore_t timer_sem;


static volatile bool first = true;   // True if the first buffer is playing
static bool genSound = false;
static float audio_gain = 1.0f;
static int32_t audio_gain_q12 = (1 << 12); // Fixed-point gain for fast path
static float lp_alpha = 1.0f;        // 1.0f => no filtering
static float lp_state = 0.0f;        // filter memory
static float hp_alpha = 0.0f;        // 0.0f => disabled
static float hp_x_prev = 0.0f;
static float hp_y_prev = 0.0f;
static float hp_cutoff_hz = 0.0f;

static void beginAudio(void);

#define TWOFIVEHZMS     40    // ms between 25 HZ ticks
#define FIFTYHZMS       20    // ms between 50 HZ ticks
#define ONEHUNDRDHZMS   10    // ms between 100 HZ ticks
#define TWOHUNDRDHZMS   5    // ms between 200 HZ ticks
#define FIVEHUNDRDHZMS  2    // ms between 500 HZ ticks
#define TICKMS          4     // ms ticks to service hdmi audio ring buffer
//#define TICKCOUNT       (TWOFIVEHZMS/TICKMS)  // number of ring buffer ticks per 50Hz tick
#define TICKCOUNT       (FIFTYHZMS/TICKMS)  // number of ring buffer ticks per 50Hz tick
// #define TICKCOUNT       (FIVEHUNDRDHZMS/TICKMS)  // number of ring buffer ticks per 200Hz tick
// #define TICKCOUNT       (TWOHUNDRDHZMS/TICKMS)  // number of ring buffer ticks per 200Hz tick
// #define TICKCOUNT       (ONEHUNDRDHZMS/TICKMS)  // number of ring buffer ticks per 100Hz tick
#define TICK_SAMPLES    (NUMSAMPLES / TICKCOUNT)

repeating_timer_t audio_timer;
audio_ring_t* ring;
int16_t* samples;
audio_sample_t* hdmi_buffer;
int hdmi_buffer_size;
static bool audio_manual_mode = false;  // When true, caller drives ticks instead of repeating timer

#ifdef TIME_SPARE
int32_t sound_count = 0;
int64_t int_count = 0;
#endif



uint16_t emu_SoundSampleRate(void)
{
  return SAMPLE_FREQ;
}

void emu_sndInit(bool playSound, bool reset, audio_ring_t* audio_ring, int16_t* sample_buff)  // JOE ADDED audio_ring
{
  genSound = playSound;

  ring = audio_ring;
  samples = sample_buff;

  beginAudio();
}

void emu_audio_set_manual_tick(bool manual_mode)
{
    audio_manual_mode = manual_mode;
}

void emu_audio_manual_tick(void)
{
    audio_service_tick();
}

void emu_audio_set_gain(float gain)
{
  // Clamp to a sane positive range to avoid runaway clipping
  if (gain < 0.0f) {
    gain = 0.0f;
  }
  if (gain > 16.0f) {
    gain = 16.0f;
  }
  audio_gain = gain;

  // Keep an integer gain copy for the low-CPU fast path.
  int32_t q12 = (int32_t)(gain * 4096.0f + 0.5f);
  if (q12 < 0) {
    q12 = 0;
  }
  audio_gain_q12 = q12;
}

float emu_audio_get_gain(void)
{
  return audio_gain;
}

void emu_audio_set_lowpass(float cutoff_hz)
{
  if (cutoff_hz <= 0.0f) {
    lp_alpha = 1.0f; // disable filtering
    return;
  }
  // Single-pole LPF: alpha = dt / (RC + dt), RC = 1/(2*pi*fc)
  const float dt = 1.0f / (float)SAMPLE_FREQ;
  const float rc = 1.0f / (6.2831853f * cutoff_hz);
  float alpha = dt / (rc + dt);
  if (alpha < 0.0f) alpha = 0.0f;
  if (alpha > 1.0f) alpha = 1.0f;
  lp_alpha = alpha;
}

void emu_audio_set_highpass(float cutoff_hz)
{
  if (cutoff_hz <= 0.0f) {
    hp_alpha = 0.0f;
    hp_cutoff_hz = 0.0f;
    hp_x_prev = 0.0f;
    hp_y_prev = 0.0f;
    return;
  }

  // Single-pole HPF: y[n] = a * (y[n-1] + x[n] - x[n-1]), a = RC/(RC + dt)
  const float dt = 1.0f / (float)SAMPLE_FREQ;
  const float rc = 1.0f / (6.2831853f * cutoff_hz);
  float alpha = rc / (rc + dt);
  if (alpha < 0.0f) alpha = 0.0f;
  if (alpha > 1.0f) alpha = 1.0f;

  hp_alpha = alpha;
  hp_cutoff_hz = cutoff_hz;
}

float emu_audio_get_highpass(void)
{
  return hp_cutoff_hz;
}

const int16_t sine[32] = {
    0x8000,0x98f8,0xb0fb,0xc71c,0xda82,0xea6d,0xf641,0xfd89,
    0xffff,0xfd89,0xf641,0xea6d,0xda82,0xc71c,0xb0fb,0x98f8,
    0x8000,0x6707,0x4f04,0x38e3,0x257d,0x1592,0x9be,0x276,
    0x0,0x276,0x9be,0x1592,0x257d,0x38e3,0x4f04,0x6707
};

static bool __time_critical_func(audio_timer_callback)(__unused repeating_timer_t* rt)
{
  audio_service_tick();
  return true;
}

static void __time_critical_func(audio_service_tick)(void)
{
    static uint32_t call_count = 0;
    
    // Process chunk of samples from ADC double-buffer
    // ADC fills one buffer while we read from the other
    // IMPORTANT: Always process exactly TICK_SAMPLES (128), never more!
    // The ADC buffer is only TICK_SAMPLES, so we can't read beyond that.
    
    int size = get_write_size(ring, true);
    if (size >= TICK_SAMPLES)
    {
        int audio_offset = get_write_offset(ring);
        
        const bool filters_enabled = (hp_alpha > 0.0f) || (lp_alpha < 0.9999f);
        
        // ALWAYS process exactly TICK_SAMPLES - no doubling!
        // Our ADC buffer is only TICK_SAMPLES deep, reading more would be garbage data
        size = TICK_SAMPLES;
        
        // Process chunk of samples (exactly TICK_SAMPLES)
        for (int c = 0; c < size; c++)
        {
#if TEST_WITH_SINE_WAVE
            // Test with sine wave for debugging
            int32_t capture_value = sine[c % 32];
            int32_t filtered = capture_value;
#else
            // ADC is 12-bit (0-4095), convert to 16-bit signed (-32768 to 32767)
            int32_t capture_value = ((int32_t)((uint16_t)samples[c]) << 4) - 32768;
            // Apply software gain with saturation
            int32_t scaled;
            if (audio_gain_q12 == (1 << 12)) {
              scaled = capture_value;
            } else {
              scaled = (capture_value * audio_gain_q12) >> 12;
            }
            if (scaled > INT16_MAX) {
              scaled = INT16_MAX;
            }
            if (scaled < INT16_MIN) {
              scaled = INT16_MIN;
            }

            int32_t filtered = scaled;
            if (filters_enabled) {
              // Optional single-pole high-pass to remove DC offset and slow bias drift.
              float hp_out = (float)scaled;
              if (hp_alpha > 0.0f) {
                hp_out = hp_alpha * (hp_y_prev + hp_out - hp_x_prev);
                hp_x_prev = (float)scaled;
                hp_y_prev = hp_out;
              }

              // Optional single-pole low-pass to reduce hiss; lp_alpha==1 => bypass
              float filtered_f = (lp_state + lp_alpha * (hp_out - lp_state));
              lp_state = filtered_f;
              filtered = (int32_t)filtered_f;
              if (filtered > INT16_MAX) filtered = INT16_MAX;
              if (filtered < INT16_MIN) filtered = INT16_MIN;
            }
#endif
            
            hdmi_buffer[audio_offset].channels[0] = (int16_t)filtered;
            hdmi_buffer[audio_offset].channels[1] = (int16_t)filtered;
            audio_offset = (audio_offset + 1) & (hdmi_buffer_size-1);
        }          

        set_write_offset(ring, audio_offset);
    }
    
    // 50Hz semaphore for compatibility with existing code
    if (++call_count >= TICKCOUNT)
    {
        call_count = 0;
        first = !first;
        sem_release(&timer_sem);
    }

}

static void beginAudio(void)
{
  static bool begun = false;

  if (!begun) // Only begin once...
  {
    begun = true;
    sem_init(&timer_sem, 0, 1);    // Create the timer callback
    emu_silenceSound();    hdmi_buffer = ring->buffer;
    hdmi_buffer_size = ring->size;

    // Use CPU timer for audio chunking
    if (!audio_manual_mode) 
    {
        printf("Initializing CPU timer for audio chunking at %d ms intervals...\n", TICKMS);
        add_repeating_timer_ms(-TICKMS, audio_timer_callback, NULL, &audio_timer);

        // Lower timer IRQ priority so VSYNC GPIO (IO_IRQ_BANK0) wins
        if (AUDIO_TIMER_IRQ >= 0) 
        {
          irq_set_priority(AUDIO_TIMER_IRQ, 0xC0);
        }
    } 
    else 
    {
        printf("Audio manual-tick mode enabled; no repeating timer started\n");
    }

    printf("sound initialized\n");
  }
  else
  {
    emu_silenceSound();
  }
}

void emu_silenceSound(void)
{
//   if (soundBuffer16)
//   {
    // // Set buffers to silence
    // for (int i = 0; i < (NUMSAMPLES<<2); ++i)
    // {
    //   soundBuffer16[i] = ZEROSOUND;
    // }
//   }
}