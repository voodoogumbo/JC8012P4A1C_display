#pragma once
// PIE (the ESP32-P4's 128-bit SIMD extension) helpers for RGB565 strips: eight
// pixels per instruction in registers q0..q7.
//
// The compiler never uses the q registers, so each helper is one self-contained
// asm block that loads its own constants.
//
// Call these only from on_strip (the two renderers, one per core), never from
// on_frame or another task. Arduino core 3.3.11/3.3.12 (ESP-IDF v5.5.5) saves PIE
// state wrongly on a context switch (q3 lost, SAR fields swapped; fixed after IDF
// v5.5.6), so two PIE tasks on one core corrupt each other (white speckles). With
// one PIE task per core, FreeRTOS's lazy save never runs.
//
// Results: vadd.u16 saturates; vmul.u16 keeps the 32-bit product and shifts it
// right by SAR (set with esp.movx.w.sar) before taking the low 16 bits.
//
// Registers: PIE instructions only take address/scalar operands in x8..x15
// (s0, s1, a0..a5), so every operand is pinned to one of a0..a5.
//
// Alignment: esp.vld/vst.128 ignore the low 4 address bits, so every destination
// here must be 16-byte aligned (strip rows are: 1600 bytes apart, 128-aligned ring).
// Sprite sources may be unaligned: esp.ld.128.usar + esp.src.q funnel-shift two
// aligned loads into the eight pixels starting at any address.

#include <stdint.h>

// Constants for the blend: R+B field mask, G field mask, the background weight
// (12/32, so the sprite's premultiplied weight is 20/32), and zero.
static const uint16_t PIE_K[8] __attribute__((aligned(16))) = { 0xF81F, 0x07E0, 12, 0, 0, 0, 0, 0 };

// See-through: for n groups of 8 pixels,
//   d = pre ? pre + (d * 12/32 per channel) : d
// where pre is the sprite premultiplied by 20/32 (0 = transparent).
// Channel math: R and B scaled together (their products don't overlap), G alone;
// esp.vmul.u16 keeps the 32-bit product shifted right by SAR (5).
static inline void pie_blend_row(uint16_t *dst, const uint16_t *pre, int count) {
    register uint16_t *d asm("a0") = dst;
    register const uint16_t *sp asm("a1") = pre;
    register int n asm("a2") = count;
    register const uint16_t *k asm("a3") = PIE_K;
    register int sh asm("a4") = 5;
    asm volatile(
        "esp.vldbc.16.ip  q0, %[k], 2\n"     // q0 = 0xF81F
        "esp.vldbc.16.ip  q1, %[k], 2\n"     // q1 = 0x07E0
        "esp.vldbc.16.ip  q2, %[k], 2\n"     // q2 = 12
        "esp.zero.q       q3\n"
        "esp.movx.w.sar   %[sh]\n"
        "1:\n"
        "esp.vld.128.ip   q4, %[d], 0\n"     // q4 = background
        "esp.ld.128.usar.ip q5, %[s], 16\n"  // q5:q6 = two aligned blocks around pre
        "esp.vld.128.ip   q6, %[s], 0\n"
        "esp.src.q        q5, q5, q6\n"      // q5 = pre (8 px from any address)
        "esp.andq         q6, q4, q0\n"      // R+B * 12 >> 5
        "esp.vmul.u16     q6, q6, q2\n"
        "esp.andq         q6, q6, q0\n"
        "esp.andq         q7, q4, q1\n"      // G * 12 >> 5
        "esp.vmul.u16     q7, q7, q2\n"
        "esp.andq         q7, q7, q1\n"
        "esp.orq          q6, q6, q7\n"
        "esp.vadd.u16     q6, q6, q5\n"      // q6 = blended
        "esp.vcmp.eq.u16  q7, q5, q3\n"      // q7 = transparent lanes
        "esp.xorq         q5, q6, q4\n"      // keep the background there:
        "esp.andq         q5, q5, q7\n"      //   out = blend ^ ((blend ^ bg) & transparent)
        "esp.xorq         q6, q6, q5\n"
        "esp.vst.128.ip   q6, %[d], 16\n"
        "addi             %[n], %[n], -1\n"
        "bnez             %[n], 1b\n"
        : [d] "+r"(d), [s] "+r"(sp), [n] "+r"(n), [k] "+r"(k)
        : [sh] "r"(sh)
        : "memory");
}

// Opaque: d = spr ? spr : d, for n groups of 8 pixels (0 = transparent).
static inline void pie_opaque_row(uint16_t *dst, const uint16_t *spr, int count) {
    register uint16_t *d asm("a0") = dst;
    register const uint16_t *sp asm("a1") = spr;
    register int n asm("a2") = count;
    asm volatile(
        "esp.zero.q       q3\n"
        "1:\n"
        "esp.vld.128.ip   q4, %[d], 0\n"
        "esp.ld.128.usar.ip q5, %[s], 16\n"
        "esp.vld.128.ip   q6, %[s], 0\n"
        "esp.src.q        q5, q5, q6\n"
        "esp.vcmp.eq.u16  q7, q5, q3\n"
        "esp.xorq         q6, q5, q4\n"
        "esp.andq         q6, q6, q7\n"
        "esp.xorq         q5, q5, q6\n"
        "esp.vst.128.ip   q5, %[d], 16\n"
        "addi             %[n], %[n], -1\n"
        "bnez             %[n], 1b\n"
        : [d] "+r"(d), [s] "+r"(sp), [n] "+r"(n)
        :
        : "memory");
}

// The same row (n groups of 8 px, aligned) into 16 rows 1600 bytes apart:
// each source block is loaded once and stored 16 times.
static inline void pie_fill16(uint16_t *dst, const uint16_t *src, int count) {
    register uint16_t *d asm("a0") = dst;
    register const uint16_t *sp asm("a1") = src;
    register int n asm("a2") = count;
    register uint16_t *t asm("a3");
    asm volatile(
        "1:\n"
        "esp.vld.128.ip   q0, %[s], 16\n"
        "mv               %[t], %[d]\n"
        "esp.vst.128.ip   q0, %[t], 1600\n" "esp.vst.128.ip   q0, %[t], 1600\n"
        "esp.vst.128.ip   q0, %[t], 1600\n" "esp.vst.128.ip   q0, %[t], 1600\n"
        "esp.vst.128.ip   q0, %[t], 1600\n" "esp.vst.128.ip   q0, %[t], 1600\n"
        "esp.vst.128.ip   q0, %[t], 1600\n" "esp.vst.128.ip   q0, %[t], 1600\n"
        "esp.vst.128.ip   q0, %[t], 1600\n" "esp.vst.128.ip   q0, %[t], 1600\n"
        "esp.vst.128.ip   q0, %[t], 1600\n" "esp.vst.128.ip   q0, %[t], 1600\n"
        "esp.vst.128.ip   q0, %[t], 1600\n" "esp.vst.128.ip   q0, %[t], 1600\n"
        "esp.vst.128.ip   q0, %[t], 1600\n" "esp.vst.128.ip   q0, %[t], 1600\n"
        "addi             %[d], %[d], 16\n"
        "addi             %[n], %[n], -1\n"
        "bnez             %[n], 1b\n"
        : [d] "+r"(d), [s] "+r"(sp), [n] "+r"(n), [t] "=&r"(t)
        :
        : "memory");
}

// The same row (n groups of 8 px, aligned) into `rows` rows `stride` bytes apart.
static inline void pie_fill_rows(uint16_t *dst, const uint16_t *src, int count, int rows, int stride) {
    register uint16_t *d asm("a0") = dst;
    register const uint16_t *sp asm("a1") = src;
    register int n asm("a2") = count;
    register uint16_t *t asm("a3");
    register int st asm("a4") = stride;
    register int r asm("a5");
    asm volatile(
        "1:\n"
        "esp.vld.128.ip   q0, %[s], 16\n"
        "mv               %[t], %[d]\n"
        "mv               %[r], %[rows]\n"
        "2:\n"
        "esp.vst.128.xp   q0, %[t], %[st]\n"
        "addi             %[r], %[r], -1\n"
        "bnez             %[r], 2b\n"
        "addi             %[d], %[d], 16\n"
        "addi             %[n], %[n], -1\n"
        "bnez             %[n], 1b\n"
        : [d] "+r"(d), [s] "+r"(sp), [n] "+r"(n), [t] "=&r"(t), [r] "=&r"(r)
        : [st] "r"(st), [rows] "r"(rows)
        : "memory");
}

// Plain aligned copy, n groups of 8 px (both pointers 16-byte aligned).
static inline void pie_copy(uint16_t *dst, const uint16_t *src, int count) {
    register uint16_t *d asm("a0") = dst;
    register const uint16_t *sp asm("a1") = src;
    register int n asm("a2") = count;
    asm volatile(
        "1:\n"
        "esp.vld.128.ip   q0, %[s], 16\n"
        "esp.vst.128.ip   q0, %[d], 16\n"
        "addi             %[n], %[n], -1\n"
        "bnez             %[n], 1b\n"
        : [d] "+r"(d), [s] "+r"(sp), [n] "+r"(n)
        :
        : "memory");
}
