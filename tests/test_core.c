/* Regression test for mono_ringmod.h that needs no firmware.
 *
 * The expected hashes come from Monomachine OS 1.32B itself: the tables were
 * read from the emulated DSP's memory, and the same stimulus was fed into the
 * firmware's RINGMOD machine running in an emulator (see the README), with
 * the sine's position set to the table's start before the first block as
 * mq_reset has it, and the notes through the kernel's pitch glide. The
 * stimulus plays notes over the whole range, moves every control, including
 * to the ends of their range, and feeds a separate external carrier. */
#include <stdio.h>
#include "../source/mono_ringmod.h"

static uint32_t seed = 1;
static uint32_t rnd(uint32_t n) { seed = seed * 1664525u + 1013904223u; return (seed >> 8) % n; }

/* Controls, note and input of one block. Notes change only every third block,
 * where the Monomachine's CPU sends a new one. */
static void stimulus(int block, int32_t ctl[MQ_CONTROLS], int *note, int32_t in[2][MQ_FRAMES], int32_t ext[2][MQ_FRAMES]) {
   static int level = 8, ext_level = 8;
   static const int32_t defaults[MQ_CONTROLS] = {32, 0, 100, 64};
   if (block == 0) {
      for (int k = 0; k < MQ_CONTROLS; ++k) ctl[k] = defaults[k] << 16;
      *note = 60;
   } else if (block % 61 == 0) {                    /* knob-like moves, also between steps and to the ends */
      const int k = (int)rnd(MQ_CONTROLS);
      ctl[k] = rnd(4) == 0 ? (rnd(2) ? 0x7fffff : 0) : (int32_t)rnd(0x7f0000);
      level = 8 + (int)rnd(8);
      ext_level = 8 + (int)rnd(10);
   }
   if (block % 3 == 0 && block % 150 == 0) *note = rnd(5) ? 24 + (int)rnd(85) : (int)rnd(128);
   for (int c = 0; c < 2; ++c)
      for (int n = 0; n < MQ_FRAMES; ++n) {
         seed = seed * 1664525u + 1013904223u;
         in[c][n] = block % 400 < 300 ? (int32_t)seed >> level : 0;
         seed = seed * 1664525u + 1013904223u;
         ext[c][n] = block % 500 < 350 ? (int32_t)seed >> ext_level : 0;
      }
}

static int32_t word_of(int note) { return (note << 11) / 12; }

static uint64_t hash = 1469598103934665603ULL;
static void mix(const int32_t *v, int n) {
   for (int i = 0; i < n; ++i)
      for (int b = 0; b < 4; ++b) hash = (hash ^ (((uint32_t)v[i] >> (8 * b)) & 0xff)) * 1099511628211ULL;
}

#define TEST_BLOCKS 12000

#ifndef TEST_CORE_NO_MAIN
int main(void) {
   static MonoRingmod q;
   MqPitch glide = {0, 0, 0};
   mq_tables();
   mq_reset(&q);
   mix(mq_sine, 0x2000); mix(mq_exp, 0x800); mix(mq_shape, 256); mix(mq_block_dc, 1600); mix(mq_impulse, 515);
   const uint64_t tables = hash;
   hash = 1469598103934665603ULL;
   int32_t ctl[MQ_CONTROLS], in[2][MQ_FRAMES], ext[2][MQ_FRAMES], out[2][MQ_FRAMES];
   int note = 60;
   for (int block = 0; block < TEST_BLOCKS; ++block) {
      stimulus(block, ctl, &note, in, ext);
      if (block == 0) mq_pitch_reset(&glide, word_of(note));
      mq_block(&q, ctl, mq_pitch(&glide, word_of(note)), in, ext, out);
      mix(out[0], MQ_FRAMES); mix(out[1], MQ_FRAMES);
   }
   const uint64_t want_tables = 0xcf262aa91e5627acULL, want_audio = 0x70503257325a2adfULL;
   printf("tables %016llx audio %016llx\n", (unsigned long long)tables, (unsigned long long)hash);
   if (tables != want_tables || hash != want_audio) { puts("FAIL: output differs from the firmware"); return 1; }
   puts("PASS core tables and audio match the firmware");
   return 0;
}
#endif
