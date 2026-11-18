/* Loads the built plug-in like a host and checks what the core test cannot:
 * the wrapper around mono_ringmod.h. usage: clap_test "build/Mono Ringmod.clap"
 *  - at 44.1 kHz the output is the core's after its silent start, 16 frames
 *    late, exactly, with the sidechain as the EXT carrier one sample late
 *  - with Mix 0 and the same signal on both channels, the output is the
 *    input (x 0.99) after the reported latency, which includes the machine's
 *    own 15 samples, at 44.1, 48 and 96 kHz
 *  - at 44.1 and 48 kHz, Pitch 69 puts the carrier at 440 Hz
 *  - a note moves the carrier when Note Tracking is on, and not when it is off
 *  - with EXT up, the sidechain is the carrier
 *  - state round trip, rejected bad states, parameter text */
#include <clap/clap.h>
#include <dlfcn.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "../source/mono_ringmod.h"

#define CHECK(c, ...) do { if (!(c)) { printf("FAIL line %d: ", __LINE__); printf(__VA_ARGS__); puts(""); exit(1); } } while (0)
#define N (192 * 512)   /* whole process calls */
#define NP 6            /* Pitch, Note Tracking, Wave, Ext, Mix, Input */
#define PI 3.14159265358979323846

typedef union { clap_event_header_t h; clap_event_param_value_t v; clap_event_note_t n; } Event;
typedef struct { clap_input_events_t api; Event v[NP + 1]; uint32_t n; } Events;
static uint32_t ev_size(const clap_input_events_t *e) { return ((const Events *)e->ctx)->n; }
static const clap_event_header_t *ev_get(const clap_input_events_t *e, uint32_t i) { return &((const Events *)e->ctx)->v[i].h; }
static const void *host_ext(const clap_host_t *h, const char *id) { (void)h; (void)id; return NULL; }
static void host_cb(const clap_host_t *h) { (void)h; }
static const clap_host_t host = {CLAP_VERSION_INIT, NULL, "test", "", "", "1", host_ext, host_cb, host_cb, host_cb};
static const clap_plugin_factory_t *factory;

typedef struct { uint8_t data[256]; uint64_t size, pos; } Buffer;
static int64_t buf_write(const clap_ostream_t *s, const void *p, uint64_t n) { Buffer *b = s->ctx; if (b->size + n > sizeof b->data) return -1; memcpy(b->data + b->size, p, n); b->size += n; return (int64_t)n; }
static int64_t buf_read(const clap_istream_t *s, void *p, uint64_t n) { Buffer *b = s->ctx; if (n > b->size - b->pos) n = b->size - b->pos; memcpy(p, b->data + b->pos, n); b->pos += n; return (int64_t)n; }

static const clap_plugin_t *make(const double v[NP], double rate) {
   const clap_plugin_t *p = factory->create_plugin(factory, &host, "works.reconstruction.monoringmod");
   CHECK(p && p->init(p), "create");
   const clap_plugin_params_t *params = p->get_extension(p, CLAP_EXT_PARAMS);
   Events ev = {{NULL, ev_size, ev_get}, {{{0}}}, NP};
   for (uint32_t i = 0; i < NP; ++i)
      ev.v[i].v = (clap_event_param_value_t){{sizeof ev.v[i].v, 0, CLAP_CORE_EVENT_SPACE_ID, CLAP_EVENT_PARAM_VALUE, 0}, i, NULL, -1, -1, -1, -1, v[i]};
   ev.api.ctx = &ev;
   params->flush(p, &ev.api, NULL);
   CHECK(p->activate(p, rate, 1, 512) && p->start_processing(p), "activate");
   return p;
}
/* Runs N frames of in (and the sidechain) through p, 512 at a time; a note-on (key >= 0) with the first call. */
static void run(const clap_plugin_t *p, double in[2][N], double side[2][N], double out[2][N], int key) {
   Events ev = {{NULL, ev_size, ev_get}, {{{0}}}, 0};
   ev.api.ctx = &ev;
   if (key >= 0) {
      ev.v[0].n = (clap_event_note_t){{sizeof ev.v[0].n, 0, CLAP_CORE_EVENT_SPACE_ID, CLAP_EVENT_NOTE_ON, 0}, -1, 0, 0, (int16_t)key, 1.0};
      ev.n = 1;
   }
   for (uint32_t at = 0; at < N; at += 512) {
      double *i[2] = {in[0] + at, in[1] + at}, *s[2] = {side[0] + at, side[1] + at}, *o[2] = {out[0] + at, out[1] + at};
      clap_audio_buffer_t ai[2] = {{NULL, i, 2, 0, 0}, {NULL, s, 2, 0, 0}}, ao = {NULL, o, 2, 0, 0};
      clap_process_t pr = {0};
      pr.frames_count = 512; pr.audio_inputs = ai; pr.audio_outputs = &ao;
      pr.audio_inputs_count = 2; pr.audio_outputs_count = 1; pr.in_events = &ev.api;
      CHECK(p->process(p, &pr) == CLAP_PROCESS_CONTINUE, "process");
      ev.n = 0;
   }
}
static uint32_t latency_of(const clap_plugin_t *p) { return ((const clap_plugin_latency_t *)p->get_extension(p, CLAP_EXT_LATENCY))->get(p); }
static void done(const clap_plugin_t *p) { p->stop_processing(p); p->deactivate(p); p->destroy(p); }
/* The frequency of the left output over the second half, from its upward crossings of its mean. */
static double frequency(double out[2][N], double rate) {
   double mean = 0;
   for (int n = N / 2; n < N; ++n) mean += out[0][n];
   mean /= N / 2;
   int first = -1, last = -1, count = 0;
   for (int n = N / 2 + 1; n < N; ++n)
      if (out[0][n - 1] < mean && out[0][n] >= mean) { if (first < 0) first = n; last = n; ++count; }
   return count > 1 ? (count - 1) * rate / (last - first) : 0;
}

static double in[2][N], side[2][N], out[2][N];
int main(int argc, char **argv) {
   CHECK(argc == 2, "usage: clap_test plugin.clap");
   char path[1024];
   snprintf(path, sizeof path, "%s/Contents/MacOS/Mono Ringmod", argv[1]);
   void *lib = dlopen(path, RTLD_NOW | RTLD_LOCAL);
   CHECK(lib, "%s", dlerror());
   const clap_plugin_entry_t *entry = dlsym(lib, "clap_entry");
   CHECK(entry && entry->init(path), "entry");
   factory = entry->get_factory(CLAP_PLUGIN_FACTORY_ID);
   mq_tables();
   uint32_t seed = 7;
   for (int c = 0; c < 2; ++c)
      for (int n = 0; n < N; ++n) {
         seed = seed * 1664525u + 1013904223u; in[c][n] = (double)((int32_t)seed >> 9) / 8388608.0;
         seed = seed * 1664525u + 1013904223u; side[c][n] = (double)((int32_t)seed >> 10) / 8388608.0;
      }

   /* 44.1 kHz: the core itself, after the plug-in's 4 silent blocks */
   const double sets[2][NP] = {{60, 1, 0, 0, 100, 64}, {45, 0, 90, 70, 127, 110}};
   for (int t = 0; t < 2; ++t) {
      const clap_plugin_t *p = make(sets[t], 44100);
      CHECK(latency_of(p) == 31, "latency %u", latency_of(p));
      run(p, in, side, out, -1);
      done(p);
      static MonoRingmod m;
      MqPitch g;
      mq_reset(&m);
      int32_t ctl[MQ_CONTROLS];
      for (int k = 0; k < MQ_CONTROLS; ++k) ctl[k] = (int32_t)sets[t][2 + k] << 16;
      const int32_t word = ((int)sets[t][0] << 11) / 12;
      mq_pitch_reset(&g, word);
      const int32_t z[2][16] = {{0}};
      int32_t y[2][16];
      for (int b = 0; b < 4; ++b) mq_block(&m, ctl, mq_pitch(&g, word), z, z, y);
      for (int at = 0; at + 32 <= N; at += 16) {
         int32_t x[2][16], e[2][16];
         for (int c = 0; c < 2; ++c)
            for (int n = 0; n < 16; ++n) {
               x[c][n] = (int32_t)lrint(in[c][at + n] * 8388608.0);
               e[c][n] = at + n ? (int32_t)lrint(side[c][at + n - 1] * 8388608.0) : 0;
            }
         mq_block(&m, ctl, mq_pitch(&g, word), (const int32_t (*)[16])x, (const int32_t (*)[16])e, y);
         for (int c = 0; c < 2; ++c) for (int n = 0; n < 16; ++n)
            CHECK(out[c][at + 16 + n] == y[c][n] / 8388608.0, "set %d frame %d: %.9f, core %.9f", t, at + 16 + n, out[c][at + 16 + n], y[c][n] / 8388608.0);
      }
   }
   puts("PASS 44.1 kHz output is the core's after its silent start, bit for bit, with the sidechain as EXT");

   /* Mix 0: the machine's delay and allpass branches line up, so a signal on both channels comes through */
   for (int c = 0; c < 2; ++c) for (int n = 0; n < N; ++n) { in[c][n] = 0.3 * sin(n * 0.011) + 0.2 * sin(n * 0.31) + 0.1 * sin(n * 1.1); side[c][n] = 0; }
   const double all[3] = {44100, 48000, 96000};
   for (int r = 0; r < 3; ++r) {
      const double v[NP] = {60, 1, 0, 0, 0, 64};
      const clap_plugin_t *p = make(v, all[r]);
      const uint32_t lat = latency_of(p);
      run(p, in, side, out, -1);
      done(p);
      double worst = 0;
      for (int c = 0; c < 2; ++c) for (uint32_t n = 4096; n + lat < N; ++n) worst = fmax(worst, fabs(out[c][n + lat] - 0.99 * in[c][n]));
      CHECK(20 * log10(worst) < -50, "Mix 0 at %.0f Hz: %.1f dB off", all[r], 20 * log10(worst));
      printf("PASS Mix 0 at %.0f Hz: the input after the reported latency (%u frames, worst error %.1f dB)\n", all[r], lat, 20 * log10(worst));
   }

   /* the carrier, heard through a constant input with Mix 127 */
   for (int c = 0; c < 2; ++c) for (int n = 0; n < N; ++n) { in[c][n] = 0.25; side[c][n] = 0; }
   const double rates[2] = {44100, 48000};
   for (int r = 0; r < 2; ++r) {
      const double v[NP] = {69, 1, 0, 0, 127, 64};
      const clap_plugin_t *p = make(v, rates[r]);
      run(p, in, side, out, -1);
      done(p);
      const double f = frequency(out, rates[r]);
      CHECK(fabs(f - 440) < 1.5, "carrier %.2f Hz at %.0f Hz", f, rates[r]);
      printf("PASS Pitch 69: carrier at %.2f Hz at %.0f Hz\n", f, rates[r]);
   }
   for (int track = 1; track >= 0; --track) {
      const double v[NP] = {60, track, 0, 0, 127, 64};
      const clap_plugin_t *p = make(v, 44100);
      run(p, in, side, out, 81);
      done(p);
      const double f = frequency(out, 44100), want = track ? 880 : 261.63;
      CHECK(fabs(f / want - 1) < 0.005, "note 81, tracking %d: %.2f Hz", track, f);
      printf("PASS note 81 with Pitch 60, Note Tracking %s: carrier at %.2f Hz\n", track ? "on" : "off", f);
   }
   for (int c = 0; c < 2; ++c) for (int n = 0; n < N; ++n) side[c][n] = 0.5 * sin(2 * PI * 1000 * n / 44100.0);
   {
      const double v[NP] = {69, 1, 0, 127, 127, 64};
      const clap_plugin_t *p = make(v, 44100);
      run(p, in, side, out, -1);
      done(p);
      const double f = frequency(out, 44100);
      CHECK(fabs(f - 1000) < 2, "EXT carrier %.2f Hz", f);
      printf("PASS Ext 127: the sidechain's 1 kHz is the carrier (%.2f Hz)\n", f);
   }

   /* state and text */
   {
      const double v[NP] = {10, 0, 30, 40, 50.5, 70};
      const clap_plugin_t *a = make(v, 44100), *b = make((const double[NP]){0}, 44100);
      const clap_plugin_state_t *sa = a->get_extension(a, CLAP_EXT_STATE), *sb = b->get_extension(b, CLAP_EXT_STATE);
      Buffer buf = {{0}, 0, 0};
      clap_ostream_t os = {&buf, buf_write};
      clap_istream_t is = {&buf, buf_read};
      CHECK(sa->save(a, &os) && sb->load(b, &is), "state");
      const clap_plugin_params_t *pb = b->get_extension(b, CLAP_EXT_PARAMS);
      for (clap_id i = 0; i < NP; ++i) { double x; CHECK(pb->get_value(b, i, &x) && x == v[i], "param %u after load", i); }
      buf.size -= 8; buf.pos = 0;
      CHECK(!sb->load(b, &is), "a short state must be refused");
      buf.size += 8; buf.data[0] = 'X'; buf.pos = 0;
      CHECK(!sb->load(b, &is), "a bad state must be refused");
      const char *want[NP] = {"69.0 (439.8 Hz)", "On", "64.0", "0.0", "127.0", "64.0 (+0.0 dB)"};
      const double at[NP] = {69, 1, 64, 0, 127, 64};
      char text[64];
      for (clap_id i = 0; i < NP; ++i)
         CHECK(pb->value_to_text(b, i, at[i], text, sizeof text) && !strcmp(text, want[i]), "text of %u: \"%s\", want \"%s\"", i, text, want[i]);
      double x;
      CHECK(pb->text_to_value(b, 1, "Off", &x) && x == 0, "tracking from text %f", x);
      CHECK(pb->text_to_value(b, 0, "72", &x) && x == 72, "pitch from text %f", x);
      done(a); done(b);
      puts("PASS state round trip, bad states refused, parameter text");
   }
   entry->deinit();
   return 0;
}
