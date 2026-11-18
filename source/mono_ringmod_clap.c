/* Mono Ringmod: the Monomachine's FX-RINGMOD machine as a CLAP effect.
 *
 * The ring modulator itself is mono_ringmod.h, a fixed-point port of OS
 * 1.32B. This file supplies what surrounds it on the hardware and in a DAW:
 *  - the carrier's pitch, which on the unit is the note the track plays:
 *    here the Pitch knob, moved by incoming notes when Note Tracking is on
 *    (a note n shifts it by n - 60 semitones);
 *  - the EXT carrier, which on the unit reads the external inputs one
 *    sample behind the machine's input: here the sidechain input;
 *  - the main CPU's handling of a knob: every 384 frames the word moves a
 *    quarter of the way to the knob's value; modulation is added on top,
 *    unsmoothed, every 48 frames (as the Monomachine's LFOs are);
 *  - a few silent blocks at every reset, past the machine's first block;
 *  - sample-rate conversion when the host does not run at 44.1 kHz.
 * SPDX-License-Identifier: GPL-3.0-only */
#include <clap/clap.h>
#include <math.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "mono_ringmod.h"

/* Pitch and Note Tracking, then the core's controls in its order (the machine's knobs). */
enum { PITCH, TRACK, WAVE, NPARAM = WAVE + MQ_CONTROLS };
static const struct { const char *name; double max, def; } info[NPARAM] = {
   {"Pitch", 127, 60}, {"Note Tracking", 1, 1}, {"Wave", 127, 0}, {"Ext", 127, 0}, {"Mix", 127, 0}, {"Input", 127, 64},
};
#define WARM 4      /* silent blocks at reset */
#define DELAY (MQ_FRAMES + 15)   /* the block, plus the machine's own delay (its 8 + 7 sample delay branch) */

static const char *const features[] = {
   CLAP_PLUGIN_FEATURE_AUDIO_EFFECT, "ring-modulator", CLAP_PLUGIN_FEATURE_STEREO, NULL
};
static const clap_plugin_descriptor_t descriptor = {
   .clap_version = CLAP_VERSION_INIT,
   .id = "works.reconstruction.monoringmod",
   .name = "Mono Ringmod",
   .vendor = "Independent DSP",
   .url = "", .manual_url = "", .support_url = "",
   .version = "0.1.0",
   .description = "The Monomachine FX-RINGMOD machine, ported from its DSP code",
   .features = features,
};

#define NATIVE 44100.0
#define HISTORY 4096        /* resampler input ring */
#define PHASES 128
#define QUEUE 8192

static double clamp(double x, double lo, double hi) { return x < lo ? lo : x > hi ? hi : x; }

/* ---- sample-rate conversion (windowed sinc, as in Mono Delay) -------------- */

typedef struct { double x[4]; } Frame;   /* main left, right; sidechain left, right */
typedef struct {
   float *kernel;
   Frame history[HISTORY];
   double ratio, offset;   /* output n is the input at n x ratio - offset */
   int half, taps;
   int64_t last;
   uint64_t index;
} Resampler;

static bool rs_prepare(Resampler *r, double from, double to) {
   r->ratio = from / to;
   r->offset = 0;
   r->half = (int)ceil(40.0 * fmax(1.0, r->ratio));
   r->taps = 2 * r->half;
   if (r->taps + 2 >= HISTORY) return false;
   r->kernel = calloc((size_t)(PHASES + 1) * r->taps, sizeof(float));
   if (!r->kernel) return false;
   const double cutoff = 0.95 * fmin(1.0, to / from), pi = 3.14159265358979323846;
   for (int p = 0; p <= PHASES; ++p) {
      float *k = r->kernel + (size_t)p * r->taps;
      double sum = 0.0;
      for (int i = 0; i < r->taps; ++i) {
         const double d = (double)(i - r->half + 1) - (double)p / PHASES, x = d / r->half, u = pi * cutoff * d;
         const double window = fabs(x) <= 1.0 ? 0.42 + 0.5 * cos(pi * x) + 0.08 * cos(2.0 * pi * x) : 0.0;
         k[i] = (float)(cutoff * (fabs(u) < 1e-14 ? 1.0 : sin(u) / u) * window);
         sum += k[i];
      }
      for (int i = 0; i < r->taps; ++i) k[i] /= (float)sum;
   }
   return true;
}
static void rs_reset(Resampler *r) { memset(r->history, 0, sizeof r->history); r->last = -1; r->index = 0; }
static void rs_push(Resampler *r, Frame f) { r->history[(size_t)(++r->last) & (HISTORY - 1)] = f; }
static double rs_pos(const Resampler *r) { return (double)r->index * r->ratio - r->offset; }
static bool rs_ready(const Resampler *r) { return (int64_t)floor(rs_pos(r)) + r->half <= r->last; }
static Frame rs_pop(Resampler *r) {
   const double pos = rs_pos(r), pp = (pos - floor(pos)) * PHASES;
   r->index++;
   const int64_t base = (int64_t)floor(pos);
   const int phase = (int)fmin(PHASES - 1, (int)pp);
   const float *a = r->kernel + (size_t)phase * r->taps, *b = a + r->taps;
   Frame out = {{0, 0, 0, 0}};
   for (int i = 0; i < r->taps; ++i) {
      const int64_t at = base - r->half + 1 + i;
      if (at < 0) continue;
      const Frame *f = &r->history[(size_t)at & (HISTORY - 1)];
      const double w = a[i] + (b[i] - a[i]) * (pp - phase);
      for (int c = 0; c < 4; ++c) out.x[c] += f->x[c] * w;
   }
   return out;
}

/* ---- plug-in --------------------------------------------------------------- */

typedef struct {
   clap_plugin_t plugin;
   const clap_host_t *host;
   _Atomic uint64_t value[NPARAM];    /* parameter values (double bits): host thread and audio thread */
   double mod[NPARAM];
   /* audio thread */
   bool active, direct, settled;
   uint32_t latency, warmup, blocks;
   uint64_t frames, qread, qwrite;
   int pos;
   int32_t knob[MQ_CONTROLS], word[MQ_CONTROLS], in[2][MQ_FRAMES], ext[2][MQ_FRAMES], ext_last[2], note_now;
   MqPitch glide;
   int note;                          /* the last note received */
   double out[2][MQ_FRAMES];
   MonoRingmod core;
   Resampler up, down;
   Frame queue[QUEUE];
} Plugin;

static double bits_to(uint64_t u) { double x; memcpy(&x, &u, sizeof x); return x; }
static uint64_t to_bits(double x) { uint64_t u; memcpy(&u, &x, sizeof u); return u; }
static double value(const Plugin *s, int id) { return bits_to(atomic_load_explicit(&s->value[id], memory_order_relaxed)); }
static void set_value(Plugin *s, int id, double x) { atomic_store_explicit(&s->value[id], to_bits(x), memory_order_relaxed); }

/* The note word the Monomachine's CPU sends for a note, (note << 11) / 12, with fractional notes in between. */
static int32_t note_word(double note) { return (int32_t)floor(clamp(note, 0, 127) * 2048.0 / 12.0); }
static int32_t pitch_index(double note) { return (int32_t)clamp(note_word(note) + 340, 0, MQ_PITCH_MAX); }   /* settled */

/* Start of a control frame (three blocks): the words the main CPU sends, and the pitch. */
static void control_frame(Plugin *s) {
   const uint32_t frame = s->blocks / 3;
   for (int i = 0; i < MQ_CONTROLS; ++i) {
      const int p = WAVE + i;
      const int32_t target = (int32_t)lrint(clamp(value(s, p), 0, 127) * 65536.0);
      if (!s->settled) s->knob[i] = target;
      else if ((frame & 7) == 1) s->knob[i] = (3 * s->knob[i] + target) >> 2;
      s->word[i] = s->mod[p] == 0 ? s->knob[i] : (int32_t)lrint(clamp(s->knob[i] + s->mod[p] * 65536.0, 0, 0x7effff));
   }
   const bool track = value(s, TRACK) + s->mod[TRACK] >= 0.5;
   s->note_now = note_word(clamp(clamp(value(s, PITCH) + s->mod[PITCH], 0, 127) + (track ? s->note - 60 : 0), 0, 127));   /* the unit's notes */
   if (!s->settled) { mq_reset(&s->core); mq_pitch_reset(&s->glide, s->note_now); }
   s->settled = true;
}

static int32_t word_of(double x) { return (int32_t)lrint(clamp(x, -1.0, 0.99999988) * 8388608.0); }

/* One frame at 44.1 kHz: in (main, then sidechain) goes into the core's
 * block, out is the ring modulator's output from the previous block (16
 * frames of latency). */
static void native_frame(Plugin *s, const double in[4], double out[2]) {
   if (s->pos == 0 && s->blocks++ % 3 == 0) control_frame(s);
   for (int c = 0; c < 2; ++c) {
      s->in[c][s->pos] = word_of(in[c]);
      s->ext[c][s->pos] = s->ext_last[c];   /* one sample behind */
      s->ext_last[c] = word_of(in[2 + c]);
      out[c] = s->out[c][s->pos];
   }
   if (++s->pos < MQ_FRAMES) return;
   int32_t y[2][MQ_FRAMES];
   mq_block(&s->core, s->word, mq_pitch(&s->glide, s->note_now), (const int32_t (*)[MQ_FRAMES])s->in, (const int32_t (*)[MQ_FRAMES])s->ext, y);
   for (int c = 0; c < 2; ++c)
      for (int i = 0; i < MQ_FRAMES; ++i) s->out[c][i] = y[c][i] / 8388608.0;
   s->pos = 0;
}

/* The ring modulator's output for one host frame, latency frames late. */
static void host_frame(Plugin *s, const double in[4], double out[2]) {
   if (s->direct) { native_frame(s, in, out); return; }
   rs_push(&s->up, (Frame){{in[0], in[1], in[2], in[3]}});
   while (rs_ready(&s->up)) {
      const Frame f = rs_pop(&s->up);
      Frame y;
      native_frame(s, f.x, y.x);
      rs_push(&s->down, y);
      while (rs_ready(&s->down))
         if (s->qwrite - s->qread < QUEUE) s->queue[(s->qwrite++) & (QUEUE - 1)] = rs_pop(&s->down);
         else rs_pop(&s->down);
   }
   out[0] = out[1] = 0;
   if (s->frames++ >= s->warmup && s->qread != s->qwrite) {
      const Frame f = s->queue[(s->qread++) & (QUEUE - 1)];
      out[0] = f.x[0];
      out[1] = f.x[1];
   }
}

static void reset(Plugin *s) {
   s->settled = false;
   s->blocks = 0;
   s->pos = 0;
   s->frames = s->qread = s->qwrite = 0;
   memset(s->in, 0, sizeof s->in);
   memset(s->ext, 0, sizeof s->ext);
   memset(s->ext_last, 0, sizeof s->ext_last);
   memset(s->out, 0, sizeof s->out);
   s->note = 60;
   rs_reset(&s->up);
   rs_reset(&s->down);
   const double silence[4] = {0, 0, 0, 0};
   double ignored[2];
   for (int n = 0; n < WARM * MQ_FRAMES; ++n) native_frame(s, silence, ignored);
}

static bool plug_init(const clap_plugin_t *p) { (void)p; return true; }
static void plug_destroy(const clap_plugin_t *p) {
   Plugin *s = p->plugin_data;
   free(s->up.kernel);
   free(s->down.kernel);
   free(s);
}
static bool plug_activate(const clap_plugin_t *p, double rate, uint32_t min, uint32_t max) {
   (void)min; (void)max;
   Plugin *s = p->plugin_data;
   if (s->active || !isfinite(rate) || rate < 8000 || rate > 768000) return false;
   s->direct = rate == NATIVE;
   uint32_t latency = DELAY, warmup = 0;
   if (!s->direct) {
      if (!rs_prepare(&s->up, rate, NATIVE) || !rs_prepare(&s->down, NATIVE, rate)) {
         free(s->up.kernel); free(s->down.kernel); s->up.kernel = s->down.kernel = NULL;
         return false;
      }
      warmup = (uint32_t)(s->up.half + ceil(s->down.half * rate / NATIVE) + 2);
      /* the block is 16 x rate / 44100 host frames: read the output that much earlier that it is whole */
      const uint32_t block = (uint32_t)ceil(DELAY * rate / NATIVE - 1e-9);
      s->down.offset = block * NATIVE / rate - DELAY;
      latency = warmup + block;
   }
   s->warmup = warmup;
   reset(s);
   s->active = true;
   if (latency != s->latency) {
      s->latency = latency;
      const clap_host_latency_t *hl = s->host->get_extension ? s->host->get_extension(s->host, CLAP_EXT_LATENCY) : NULL;
      if (hl && hl->changed) hl->changed(s->host);
   }
   return true;
}
static void plug_deactivate(const clap_plugin_t *p) {
   Plugin *s = p->plugin_data;
   free(s->up.kernel); free(s->down.kernel);
   s->up.kernel = s->down.kernel = NULL;
   s->active = false;
}
static bool plug_start(const clap_plugin_t *p) { (void)p; return true; }
static void plug_stop(const clap_plugin_t *p) { (void)p; }
static void plug_reset(const clap_plugin_t *p) { reset(p->plugin_data); }
static void plug_on_main(const clap_plugin_t *p) { (void)p; }

static void event(Plugin *s, const clap_event_header_t *h) {
   if (!h || h->space_id != CLAP_CORE_EVENT_SPACE_ID) return;
   if (h->type == CLAP_EVENT_PARAM_VALUE && h->size >= sizeof(clap_event_param_value_t)) {
      const clap_event_param_value_t *e = (const void *)h;
      if (e->param_id < NPARAM && e->note_id == -1 && e->port_index == -1 && e->channel == -1 && e->key == -1 && isfinite(e->value))
         set_value(s, (int)e->param_id, clamp(e->value, 0, info[e->param_id].max));
   } else if (h->type == CLAP_EVENT_PARAM_MOD && h->size >= sizeof(clap_event_param_mod_t)) {
      const clap_event_param_mod_t *e = (const void *)h;
      if (e->param_id < NPARAM && e->note_id == -1 && e->port_index == -1 && e->channel == -1 && e->key == -1 && isfinite(e->amount))
         s->mod[e->param_id] = clamp(e->amount, -info[e->param_id].max, info[e->param_id].max);
   } else if (h->type == CLAP_EVENT_NOTE_ON && h->size >= sizeof(clap_event_note_t)) {
      const clap_event_note_t *e = (const void *)h;
      if (e->key >= 0 && e->key < 128) s->note = e->key;
   } else if (h->type == CLAP_EVENT_MIDI && h->size >= sizeof(clap_event_midi_t)) {
      const clap_event_midi_t *e = (const void *)h;
      if ((e->data[0] & 0xf0) == 0x90 && e->data[2] > 0) s->note = e->data[1] & 0x7f;
   }
}

static clap_process_status plug_process(const clap_plugin_t *p, const clap_process_t *pr) {
   Plugin *s = p->plugin_data;
   if (!s->active || !pr || pr->audio_inputs_count < 1 || pr->audio_outputs_count < 1) return CLAP_PROCESS_ERROR;
   const clap_audio_buffer_t *in = &pr->audio_inputs[0], *side = pr->audio_inputs_count > 1 ? &pr->audio_inputs[1] : NULL;
   clap_audio_buffer_t *out = &pr->audio_outputs[0];
   if (in->channel_count < 2 || out->channel_count < 2 || ((!in->data32 || !out->data32) && (!in->data64 || !out->data64)))
      return CLAP_PROCESS_ERROR;
   const clap_input_events_t *ev = pr->in_events;
   const uint32_t count = ev && ev->size && ev->get ? ev->size(ev) : 0;
   uint32_t next = 0;
   for (uint32_t n = 0; n < pr->frames_count; ++n) {
      for (; next < count; ++next) {
         const clap_event_header_t *h = ev->get(ev, next);
         if (h && h->time > n) break;
         event(s, h);
      }
      double x[4], y[2];
      for (int c = 0; c < 4; ++c) {   /* the sidechain may be absent, mono or silent */
         const clap_audio_buffer_t *b = c < 2 ? in : side;
         const uint32_t k = b && b->channel_count ? (uint32_t)(c & 1) % b->channel_count : 0;
         x[c] = !b || !b->channel_count ? 0 : b->data32 ? b->data32[k][n] : b->data64 ? b->data64[k][n] : 0;
         if (!isfinite(x[c])) x[c] = 0;
      }
      host_frame(s, x, y);
      for (int c = 0; c < 2; ++c)
         if (out->data32) out->data32[c][n] = (float)y[c]; else out->data64[c][n] = y[c];
   }
   out->constant_mask = 0;
   return CLAP_PROCESS_CONTINUE;
}

/* ---- extensions ------------------------------------------------------------ */

static uint32_t ports_count(const clap_plugin_t *p, bool input) { (void)p; return input ? 2 : 1; }
static bool ports_get(const clap_plugin_t *p, uint32_t i, bool input, clap_audio_port_info_t *port) {
   if (i >= ports_count(p, input) || !port) return false;
   memset(port, 0, sizeof *port);
   port->id = i;
   snprintf(port->name, sizeof port->name, "%s", !input ? "Stereo Out" : i ? "Sidechain (EXT)" : "Stereo In");
   port->flags = (i ? 0 : CLAP_AUDIO_PORT_IS_MAIN) | CLAP_AUDIO_PORT_SUPPORTS_64BITS | CLAP_AUDIO_PORT_REQUIRES_COMMON_SAMPLE_SIZE;
   port->channel_count = 2;
   port->port_type = CLAP_PORT_STEREO;
   port->in_place_pair = 0;
   return true;
}
static const clap_plugin_audio_ports_t ports = {ports_count, ports_get};

static uint32_t params_count(const clap_plugin_t *p) { (void)p; return NPARAM; }
static bool params_info(const clap_plugin_t *p, uint32_t i, clap_param_info_t *pi) {
   (void)p;
   if (i >= NPARAM || !pi) return false;
   memset(pi, 0, sizeof *pi);
   pi->id = i;
   pi->flags = CLAP_PARAM_IS_AUTOMATABLE | CLAP_PARAM_IS_MODULATABLE | CLAP_PARAM_REQUIRES_PROCESS | (i == TRACK ? CLAP_PARAM_IS_STEPPED : 0);
   snprintf(pi->name, sizeof pi->name, "%s", info[i].name);
   pi->min_value = 0;
   pi->max_value = info[i].max;
   pi->default_value = info[i].def;
   return true;
}
static bool params_get(const clap_plugin_t *p, clap_id id, double *v) {
   if (id >= NPARAM || !v) return false;
   *v = value(p->plugin_data, (int)id);
   return true;
}
/* The carrier's frequency at a pitch index: 44100 over the square's period, from the core's arithmetic. */
static double carrier_hz(int32_t pitch) {
   const mq_acc f0 = mq_sat48(mq_freq(pitch)), f = f0 >= mq_w(0x4074) ? mq_w(0x4074) : f0;
   const int t = mq_clb(f);
   const mq_acc period = mq_sat48(mq_normf(mq_w(mq_div(mq_w(0x2b1100), mq_sat(mq_normf(f, t)))), t + 0x1d));
   return NATIVE * 16777216.0 / (double)period;
}

/* The knob value with what it means. */
static bool params_to_text(const clap_plugin_t *p, clap_id id, double v, char *out, uint32_t cap) {
   (void)p;
   if (id >= NPARAM || !out || !cap || !isfinite(v)) return false;
   const int32_t w = (int32_t)lrint(clamp(v, 0, 127) * 65536.0);
   if (id == PITCH)   /* the carrier at this knob, before any tracked note */
      snprintf(out, cap, "%.1f (%.1f Hz)", v, carrier_hz(pitch_index(v)));
   else if (id == TRACK) snprintf(out, cap, "%s", v >= 0.5 ? "On" : "Off");
   else if (id == WAVE + MQ_INPUT && w == 0) snprintf(out, cap, "%.1f (-inf dB)", v);
   else if (id == WAVE + MQ_INPUT) snprintf(out, cap, "%.1f (%+.1f dB)", v, 20.0 * log10(4.0 * pow(w / 8388608.0, 2)));
   else snprintf(out, cap, "%.1f", v);
   return true;
}
static bool params_from_text(const clap_plugin_t *p, clap_id id, const char *text, double *v) {
   (void)p;
   if (id >= NPARAM || !text || !v) return false;
   if (id == TRACK && (!strcmp(text, "On") || !strcmp(text, "Off"))) { *v = text[1] == 'n'; return true; }
   char *end = NULL;
   const double x = strtod(text, &end);
   if (end == text || !isfinite(x)) return false;
   *v = clamp(x, 0, info[id].max);
   return true;
}
static void params_flush(const clap_plugin_t *p, const clap_input_events_t *in, const clap_output_events_t *out) {
   (void)out;
   const uint32_t n = in && in->size && in->get ? in->size(in) : 0;
   for (uint32_t i = 0; i < n; ++i) event(p->plugin_data, in->get(in, i));
}
static const clap_plugin_params_t params = {params_count, params_info, params_get, params_to_text, params_from_text, params_flush};

static const uint8_t magic[8] = {'M', 'R', 'N', 'G', 0, 0, 0, 1};
static bool state_save(const clap_plugin_t *p, const clap_ostream_t *st) {
   uint8_t buf[8 + 8 * NPARAM];
   memcpy(buf, magic, 8);
   for (int i = 0; i < NPARAM; ++i)
      for (int b = 0; b < 8; ++b) buf[8 + 8 * i + b] = (uint8_t)(to_bits(value(p->plugin_data, i)) >> (8 * b));
   for (uint64_t done = 0; done < sizeof buf;) {
      const int64_t n = st && st->write ? st->write(st, buf + done, sizeof buf - done) : -1;
      if (n <= 0) return false;
      done += (uint64_t)n;
   }
   return true;
}
static bool state_load(const clap_plugin_t *p, const clap_istream_t *st) {
   Plugin *s = p->plugin_data;
   uint8_t buf[8 + 8 * NPARAM];
   for (uint64_t done = 0; done < sizeof buf;) {
      const int64_t n = st && st->read ? st->read(st, buf + done, sizeof buf - done) : -1;
      if (n <= 0) return false;
      done += (uint64_t)n;
   }
   if (memcmp(buf, magic, 8)) return false;
   double v[NPARAM];
   for (int i = 0; i < NPARAM; ++i) {
      uint64_t u = 0;
      for (int b = 0; b < 8; ++b) u |= (uint64_t)buf[8 + 8 * i + b] << (8 * b);
      v[i] = bits_to(u);
      if (!isfinite(v[i]) || v[i] < 0 || v[i] > info[i].max) return false;
   }
   for (int i = 0; i < NPARAM; ++i) set_value(s, i, v[i]);
   const clap_host_params_t *hp = s->host->get_extension ? s->host->get_extension(s->host, CLAP_EXT_PARAMS) : NULL;
   if (hp && hp->rescan) hp->rescan(s->host, CLAP_PARAM_RESCAN_VALUES);
   return true;
}
static const clap_plugin_state_t state = {state_save, state_load};

static uint32_t remote_count(const clap_plugin_t *p) { (void)p; return 1; }
static bool remote_get(const clap_plugin_t *p, uint32_t i, clap_remote_controls_page_t *page) {
   (void)p;
   if (i || !page) return false;
   memset(page, 0, sizeof *page);
   snprintf(page->section_name, sizeof page->section_name, "Ringmod");
   snprintf(page->page_name, sizeof page->page_name, "Main");
   for (int k = 0; k < CLAP_REMOTE_CONTROLS_COUNT; ++k) page->param_ids[k] = k < NPARAM ? (clap_id)k : CLAP_INVALID_ID;
   return true;
}
static const clap_plugin_remote_controls_t remote = {remote_count, remote_get};
static uint32_t tail_get(const clap_plugin_t *p) { return ((const Plugin *)p->plugin_data)->latency + 2048; }   /* the allpasses ring out */
static const clap_plugin_tail_t tail = {tail_get};
static uint32_t latency_get(const clap_plugin_t *p) { return ((const Plugin *)p->plugin_data)->latency; }
static uint32_t notes_count(const clap_plugin_t *p, bool input) { (void)p; return input ? 1 : 0; }
static bool notes_get(const clap_plugin_t *p, uint32_t i, bool input, clap_note_port_info_t *port) {
   (void)p;
   if (i || !input || !port) return false;
   memset(port, 0, sizeof *port);
   port->supported_dialects = CLAP_NOTE_DIALECT_CLAP | CLAP_NOTE_DIALECT_MIDI;
   port->preferred_dialect = CLAP_NOTE_DIALECT_CLAP;
   snprintf(port->name, sizeof port->name, "Notes (pitch)");
   return true;
}
static const clap_plugin_note_ports_t notes = {notes_count, notes_get};
static const clap_plugin_latency_t latency = {latency_get};

static const void *plug_extension(const clap_plugin_t *p, const char *id) {
   (void)p;
   if (!id) return NULL;
   if (!strcmp(id, CLAP_EXT_AUDIO_PORTS)) return &ports;
   if (!strcmp(id, CLAP_EXT_PARAMS)) return &params;
   if (!strcmp(id, CLAP_EXT_STATE)) return &state;
   if (!strcmp(id, CLAP_EXT_REMOTE_CONTROLS) || !strcmp(id, CLAP_EXT_REMOTE_CONTROLS_COMPAT)) return &remote;
   if (!strcmp(id, CLAP_EXT_TAIL)) return &tail;
   if (!strcmp(id, CLAP_EXT_LATENCY)) return &latency;
   if (!strcmp(id, CLAP_EXT_NOTE_PORTS)) return &notes;
   return NULL;
}

/* ---- factory and entry ----------------------------------------------------- */

static uint32_t factory_count(const clap_plugin_factory_t *f) { (void)f; return 1; }
static const clap_plugin_descriptor_t *factory_desc(const clap_plugin_factory_t *f, uint32_t i) { (void)f; return i ? NULL : &descriptor; }
static const clap_plugin_t *factory_create(const clap_plugin_factory_t *f, const clap_host_t *host, const char *id) {
   (void)f;
   if (!host || !id || strcmp(id, descriptor.id) || !clap_version_is_compatible(host->clap_version)) return NULL;
   Plugin *s = calloc(1, sizeof *s);
   if (!s) return NULL;
   s->host = host;
   s->latency = DELAY;
   for (int i = 0; i < NPARAM; ++i) atomic_init(&s->value[i], to_bits(info[i].def));
   s->plugin = (clap_plugin_t){
      .desc = &descriptor, .plugin_data = s, .init = plug_init, .destroy = plug_destroy,
      .activate = plug_activate, .deactivate = plug_deactivate, .start_processing = plug_start,
      .stop_processing = plug_stop, .reset = plug_reset, .process = plug_process,
      .get_extension = plug_extension, .on_main_thread = plug_on_main,
   };
   return &s->plugin;
}
static const clap_plugin_factory_t factory = {factory_count, factory_desc, factory_create};

static bool entry_init(const char *path) { (void)path; mq_tables(); return true; }   /* read-only from here on */
static void entry_deinit(void) {}
static const void *entry_factory(const char *id) { return id && !strcmp(id, CLAP_PLUGIN_FACTORY_ID) ? &factory : NULL; }
CLAP_EXPORT const clap_plugin_entry_t clap_entry = {CLAP_VERSION_INIT, entry_init, entry_deinit, entry_factory};
