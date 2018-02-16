/* Copyright (c) 2018
 * DIMA Research Group, TU Berlin
 * All rights reserved.
 *
 * Author: Tobias Behrens (tobias.behrens@dfki.de)
 *
 * All lines changed compared to the original file are marked with my surname.
 */

/* Copyright (c) 2015
 * The Trustees of Columbia University in the City of New York
 * All rights reserved.
 *
 * Author:  Orestis Polychroniou  (orestis@cs.columbia.edu)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <immintrin.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <assert.h>
#include <stdio.h>
#include <ctype.h>
#include <sched.h>
#include <time.h>

#include "inner_outer.h"


uint64_t thread_time(void)
{
	struct timespec t;
	// assert(clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t) == 0); // delete behrens
	assert(clock_gettime(CLOCK_MONOTONIC, &t) == 0);            // add behrens
	return t.tv_sec * 1000 * 1000 * 1000 + t.tv_nsec;
}

int hardware_threads(void)
{
	/* begin delete behrens
	 char name[64];
	 struct stat st;
	 int threads = -1;
	 do {
	 sprintf(name, "/sys/devices/system/cpu/cpu%d", ++threads);
	 } while (stat(name, &st) == 0);
	 return threads;
	 end delete behrens */
	return 60; // add behrens
}

void bind_thread(int thread, int threads)
{
	size_t size = CPU_ALLOC_SIZE(threads);
	cpu_set_t *cpu_set = CPU_ALLOC(threads);
	assert(cpu_set != NULL);
	CPU_ZERO_S(size, cpu_set);
	CPU_SET_S(thread, size, cpu_set);
	assert(pthread_setaffinity_np(pthread_self(), size, cpu_set) == 0);
	CPU_FREE(cpu_set);
}

void *mamalloc(size_t size)
{
	void *ptr = NULL;
	return posix_memalign(&ptr, 64, size) ? NULL : ptr;
}

void *align(const void *p)
{
	while (63 & (size_t) p) p++;
	return (void*) p;
}

int log_2(size_t n)
{
	size_t b = 1;
	int p = 0;
	while (b < n) {
		b += b;
		p++;
	}
	assert(b == n);
	return p;
}

uint64_t sum(const uint32_t *data, size_t size)
{
	size_t i;
	uint64_t sum = 0;
	for (i = 0 ; i != size ; ++i)
		sum += data[i];
	return sum;
}

typedef struct {
	uint32_t key;
	uint32_t val;
} bucket_t;

void build(const uint32_t *keys, size_t size, bucket_t *table,
           int log_table_bytes, uint32_t table_factor,
           uint32_t invalid_key, uint32_t payload_factor)
{
	size_t i;
	const size_t buckets = (((size_t) 1) << log_table_bytes) / sizeof(table[0]);
	const uint8_t shift = 32 - log_2(buckets);
	for (i = 0 ; i != buckets ; ++i)
		table[i].key = invalid_key;
	for (i = 0 ; i != size ; ++i) {
		uint32_t key = keys[i];
		uint32_t val = key * payload_factor;
		size_t h = ((uint32_t) (key * table_factor)) >> shift;
		while (table[h].key != invalid_key)
			h = (h + 1) & (buckets - 1);
		table[h].key = key;
		table[h].val = val;
	}
}

#ifdef __MIC
#define WIDTH 16
#else
#define WIDTH 4
#endif

typedef struct {
	uint32_t key[WIDTH];
	uint32_t val[WIDTH];
} bucket_wide_t;

void build_wide(const uint32_t *keys, size_t size, bucket_wide_t *table,
                int log_table_bytes, uint32_t table_factor,
                uint32_t invalid_key, uint32_t payload_factor)
{
	size_t i, j;
	const size_t buckets = (((size_t) 1) << log_table_bytes) / sizeof(table[0]);
	const uint8_t shift = 32 - log_2(buckets);
	for (i = 0 ; i != buckets ; ++i)
		for (j = 0 ; j != WIDTH ; ++j)
			table[i].key[j] = invalid_key;
	for (i = 0 ; i != size ; ++i) {
		uint32_t key = keys[i];
		uint32_t val = key * payload_factor;
		size_t h = ((uint32_t) (key * table_factor)) >> shift;
		for (;;) {
			for (j = 0 ; j != WIDTH ; ++j)
				if (table[h].key[j] == invalid_key) break;
			if (j != WIDTH) break;
			h = (h + 1) & (buckets - 1);
		}
		table[h].key[j] = key;
		table[h].val[j] = val;
	}
}

size_t probe_sum(const uint32_t *keys, size_t size,
                 const bucket_t *table, int log_table_bytes,
                 uint32_t table_factor, uint32_t invalid_key,
                 uint64_t *payload_checksum)
{
	size_t i, j;
	uint64_t sum = 0;
	assert(sizeof(table[0]) == 8);
	const size_t buckets = ((size_t) 1) << (log_table_bytes - 3);
	const uint8_t shift = 32 - log_2(buckets);
	const uint64_t *table_64 = (const uint64_t*) table;
	for (i = j = 0 ; i != size ; ++i) {
		uint32_t key = keys[i];
		size_t h = ((uint32_t) (key * table_factor)) >> shift;
		uint64_t tab = table_64[h];
		while (invalid_key != (uint32_t) tab) {
			if (key == (uint32_t) tab) {
				sum += tab >> 32;
				j++;
				break;
			}
			h = (h + 1) & (buckets - 1);
			tab = table_64[h];
		}
	}
	*payload_checksum = sum;
	return j;
}

inline void store(uint32_t *p, uint32_t v)
{
#ifndef __MIC
	_mm_stream_si32((int*) p, v);
#else
	*p = v;
#endif
}

size_t probe(const uint32_t *keys, uint32_t *vals, size_t size,
             const bucket_t *table, int log_table_bytes,
             uint32_t table_factor, uint32_t invalid_key)
{
	size_t i;
	uint32_t *vals_orig = vals;
	assert(sizeof(table[0]) == 8);
	const size_t buckets = ((size_t) 1) << (log_table_bytes - 3);
	const uint8_t shift = 32 - log_2(buckets);
	const uint64_t *table_64 = (const uint64_t*) table;
	for (i = 0 ; i != size ; ++i) {
		uint32_t key = keys[i];
		size_t h = ((uint32_t) (key * table_factor)) >> shift;
		uint64_t tab = table_64[h];
		if (key == (uint32_t) tab)
			store(vals++, tab >> 32);
		else while (invalid_key != (uint32_t) tab) {
			h = (h + 1) & (buckets - 1);
			tab = table_64[h];
			if (key == (uint32_t) tab) {
				store(vals++, tab >> 32);
				break;
			}
		}
	}
	return vals - vals_orig;
}

#ifndef __MIC

inline uint32_t _mm_reduce_or_epi32(__m128i x)
{
	x = _mm_or_si128(x, _mm_shuffle_epi32(x, _MM_SHUFFLE(1,0,3,2)));
	x = _mm_or_si128(x, _mm_shuffle_epi32(x, _MM_SHUFFLE(2,3,0,1)));
	return _mm_cvtsi128_si32(x);
}

size_t probe_hor(const uint32_t *keys, uint32_t *vals, size_t size,
                 const bucket_wide_t *table, int log_table_bytes,
                 uint32_t table_factor, uint32_t invalid_key)
{
	size_t i, j, p = 0;
	assert(sizeof(table[0]) == 32);
	uint32_t *vals_orig = vals;
	const size_t buckets = ((size_t) 1) << (log_table_bytes - 5);
	const size_t hash_mask = (buckets - 1) << 5;
	const __m128i shift = _mm_cvtsi32_si128(32 - log_2(buckets));
	const __m128i empty = _mm_set1_epi32(invalid_key);
	const __m128i factor = _mm_set1_epi32(table_factor);
	const void *table_void = (const void*) table;
	for (i = 0 ; i != size ; i += 4) {
		__m128i key = _mm_load_si128((__m128i*) &keys[i]);
		__m128i h = _mm_mullo_epi32(key, factor);
		h = _mm_srl_epi32(h, shift);
		h = _mm_slli_epi32(h, 5);
		for (j = 0 ; j != 4 ; ++j) {
			asm("movd	%1, %%eax" : "=a"(p) : "x"(h), "0"(p));
			__m128i key_x4 = _mm_shuffle_epi32(key, 0);
			for (;;) {
				bucket_wide_t *b = (bucket_wide_t*) (table_void + p);
				__m128i tab = _mm_load_si128((__m128i*) &b->key);
				__m128i out = _mm_cmpeq_epi32(tab, key_x4);
				if (!_mm_testz_si128(out, out)) {
					tab = _mm_load_si128((__m128i*) &b->val);
					tab = _mm_and_si128(tab, out);
					uint32_t val = _mm_reduce_or_epi32(tab);
					_mm_stream_si32((int*) vals++, val);
					break;
				}
				__m128i inv = _mm_cmpeq_epi32(tab, empty);
				if (!_mm_testz_si128(inv, inv)) break;
				p = (p + 32) & hash_mask;
			}
			key = _mm_srli_si128(key, 4);
			h = _mm_srli_si128(h, 4);
		}
	}
	return vals - vals_orig;
}

#else

size_t probe_hor(const uint32_t *keys, uint32_t *vals, size_t size,
                 const bucket_wide_t *table, int log_table_bytes,
                 uint32_t table_factor, uint32_t invalid_key)
{
	size_t i, j;
	uint32_t *vals_orig = vals;
	uint32_t buf[31];
	uint32_t *h_buf = align((void*) buf);
	assert(sizeof(table[0]) == 128);
	const size_t buckets = ((size_t) 1) << (log_table_bytes - 7);
	const size_t hash_mask = (buckets - 1) << 7;
	const __mmask16 z = _mm512_int2mask(0);
	const __m512i shift = _mm512_set1_epi32(32 - log_2(buckets));
	const __m512i empty = _mm512_set1_epi32(invalid_key);
	const __m512i factor = _mm512_set1_epi32(table_factor);
	const void *table_void = (const void*) table;
	for (i = 0 ; i != size ; i += 16) {
		__m512i key = _mm512_load_epi32((__m512i*) &keys[i]);
		__m512i h = _mm512_mullo_epi32(key, factor);
		h = _mm512_srlv_epi32(h, shift);
		h = _mm512_slli_epi32(h, 7);
		_mm512_store_epi32(h_buf, h);
		for (j = 0 ; j != 16 ; ++j) {
			size_t p = h_buf[j];
			__m512i key_x16 = _mm512_extload_epi32(&keys[i + j], _MM_UPCONV_EPI32_NONE,
			                                       _MM_BROADCAST_1X16, _MM_HINT_NONE);
			for (;;) {
				bucket_wide_t *b = (bucket_wide_t*) (table_void + p);
				__m512i tab = _mm512_load_epi32(&b->key);
				__mmask16 k = _mm512_cmpeq_epi32_mask(tab, key_x16);
				if (!_mm512_kortestz(k, k)) {
					p = _mm_tzcnt_64(_mm512_kconcatlo_64(z, k));
					*vals++ = b->val[p];
					break;
				}
				k = _mm512_cmpeq_epi32_mask(tab, empty);
				if (!_mm512_kortestz(k, k)) break;
				p = (p + 128) & hash_mask;
			}
		}
	}
	return vals - vals_orig;
}

#endif

#ifndef __MIC

inline __m256i _mm256_packlo_epi32(__m256i x, __m256i y)
{
	__m256 a = _mm256_castsi256_ps(x);
	__m256 b = _mm256_castsi256_ps(y);
	__m256 c = _mm256_shuffle_ps(a, b, _MM_SHUFFLE(2,0,2,0));
	__m256i z = _mm256_castps_si256(c);
	return _mm256_permute4x64_epi64(z, _MM_SHUFFLE(3,1,2,0));
}

inline __m256i _mm256_packhi_epi32(__m256i x, __m256i y)
{
	__m256 a = _mm256_castsi256_ps(x);
	__m256 b = _mm256_castsi256_ps(y);
	__m256 c = _mm256_shuffle_ps(a, b, _MM_SHUFFLE(3,1,3,1));
	__m256i z = _mm256_castps_si256(c);
	return _mm256_permute4x64_epi64(z, _MM_SHUFFLE(3,1,2,0));
}

const uint64_t perm[256] = {0x0706050403020100ull,
     0x0007060504030201ull, 0x0107060504030200ull, 0x0001070605040302ull,
     0x0207060504030100ull, 0x0002070605040301ull, 0x0102070605040300ull,
     0x0001020706050403ull, 0x0307060504020100ull, 0x0003070605040201ull,
     0x0103070605040200ull, 0x0001030706050402ull, 0x0203070605040100ull,
     0x0002030706050401ull, 0x0102030706050400ull, 0x0001020307060504ull,
     0x0407060503020100ull, 0x0004070605030201ull, 0x0104070605030200ull,
     0x0001040706050302ull, 0x0204070605030100ull, 0x0002040706050301ull,
     0x0102040706050300ull, 0x0001020407060503ull, 0x0304070605020100ull,
     0x0003040706050201ull, 0x0103040706050200ull, 0x0001030407060502ull,
     0x0203040706050100ull, 0x0002030407060501ull, 0x0102030407060500ull,
     0x0001020304070605ull, 0x0507060403020100ull, 0x0005070604030201ull,
     0x0105070604030200ull, 0x0001050706040302ull, 0x0205070604030100ull,
     0x0002050706040301ull, 0x0102050706040300ull, 0x0001020507060403ull,
     0x0305070604020100ull, 0x0003050706040201ull, 0x0103050706040200ull,
     0x0001030507060402ull, 0x0203050706040100ull, 0x0002030507060401ull,
     0x0102030507060400ull, 0x0001020305070604ull, 0x0405070603020100ull,
     0x0004050706030201ull, 0x0104050706030200ull, 0x0001040507060302ull,
     0x0204050706030100ull, 0x0002040507060301ull, 0x0102040507060300ull,
     0x0001020405070603ull, 0x0304050706020100ull, 0x0003040507060201ull,
     0x0103040507060200ull, 0x0001030405070602ull, 0x0203040507060100ull,
     0x0002030405070601ull, 0x0102030405070600ull, 0x0001020304050706ull,
     0x0607050403020100ull, 0x0006070504030201ull, 0x0106070504030200ull,
     0x0001060705040302ull, 0x0206070504030100ull, 0x0002060705040301ull,
     0x0102060705040300ull, 0x0001020607050403ull, 0x0306070504020100ull,
     0x0003060705040201ull, 0x0103060705040200ull, 0x0001030607050402ull,
     0x0203060705040100ull, 0x0002030607050401ull, 0x0102030607050400ull,
     0x0001020306070504ull, 0x0406070503020100ull, 0x0004060705030201ull,
     0x0104060705030200ull, 0x0001040607050302ull, 0x0204060705030100ull,
     0x0002040607050301ull, 0x0102040607050300ull, 0x0001020406070503ull,
     0x0304060705020100ull, 0x0003040607050201ull, 0x0103040607050200ull,
     0x0001030406070502ull, 0x0203040607050100ull, 0x0002030406070501ull,
     0x0102030406070500ull, 0x0001020304060705ull, 0x0506070403020100ull,
     0x0005060704030201ull, 0x0105060704030200ull, 0x0001050607040302ull,
     0x0205060704030100ull, 0x0002050607040301ull, 0x0102050607040300ull,
     0x0001020506070403ull, 0x0305060704020100ull, 0x0003050607040201ull,
     0x0103050607040200ull, 0x0001030506070402ull, 0x0203050607040100ull,
     0x0002030506070401ull, 0x0102030506070400ull, 0x0001020305060704ull,
     0x0405060703020100ull, 0x0004050607030201ull, 0x0104050607030200ull,
     0x0001040506070302ull, 0x0204050607030100ull, 0x0002040506070301ull,
     0x0102040506070300ull, 0x0001020405060703ull, 0x0304050607020100ull,
     0x0003040506070201ull, 0x0103040506070200ull, 0x0001030405060702ull,
     0x0203040506070100ull, 0x0002030405060701ull, 0x0102030405060700ull,
     0x0001020304050607ull, 0x0706050403020100ull, 0x0007060504030201ull,
     0x0107060504030200ull, 0x0001070605040302ull, 0x0207060504030100ull,
     0x0002070605040301ull, 0x0102070605040300ull, 0x0001020706050403ull,
     0x0307060504020100ull, 0x0003070605040201ull, 0x0103070605040200ull,
     0x0001030706050402ull, 0x0203070605040100ull, 0x0002030706050401ull,
     0x0102030706050400ull, 0x0001020307060504ull, 0x0407060503020100ull,
     0x0004070605030201ull, 0x0104070605030200ull, 0x0001040706050302ull,
     0x0204070605030100ull, 0x0002040706050301ull, 0x0102040706050300ull,
     0x0001020407060503ull, 0x0304070605020100ull, 0x0003040706050201ull,
     0x0103040706050200ull, 0x0001030407060502ull, 0x0203040706050100ull,
     0x0002030407060501ull, 0x0102030407060500ull, 0x0001020304070605ull,
     0x0507060403020100ull, 0x0005070604030201ull, 0x0105070604030200ull,
     0x0001050706040302ull, 0x0205070604030100ull, 0x0002050706040301ull,
     0x0102050706040300ull, 0x0001020507060403ull, 0x0305070604020100ull,
     0x0003050706040201ull, 0x0103050706040200ull, 0x0001030507060402ull,
     0x0203050706040100ull, 0x0002030507060401ull, 0x0102030507060400ull,
     0x0001020305070604ull, 0x0405070603020100ull, 0x0004050706030201ull,
     0x0104050706030200ull, 0x0001040507060302ull, 0x0204050706030100ull,
     0x0002040507060301ull, 0x0102040507060300ull, 0x0001020405070603ull,
     0x0304050706020100ull, 0x0003040507060201ull, 0x0103040507060200ull,
     0x0001030405070602ull, 0x0203040507060100ull, 0x0002030405070601ull,
     0x0102030405070600ull, 0x0001020304050706ull, 0x0607050403020100ull,
     0x0006070504030201ull, 0x0106070504030200ull, 0x0001060705040302ull,
     0x0206070504030100ull, 0x0002060705040301ull, 0x0102060705040300ull,
     0x0001020607050403ull, 0x0306070504020100ull, 0x0003060705040201ull,
     0x0103060705040200ull, 0x0001030607050402ull, 0x0203060705040100ull,
     0x0002030607050401ull, 0x0102030607050400ull, 0x0001020306070504ull,
     0x0406070503020100ull, 0x0004060705030201ull, 0x0104060705030200ull,
     0x0001040607050302ull, 0x0204060705030100ull, 0x0002040607050301ull,
     0x0102040607050300ull, 0x0001020406070503ull, 0x0304060705020100ull,
     0x0003040607050201ull, 0x0103040607050200ull, 0x0001030406070502ull,
     0x0203040607050100ull, 0x0002030406070501ull, 0x0102030406070500ull,
     0x0001020304060705ull, 0x0506070403020100ull, 0x0005060704030201ull,
     0x0105060704030200ull, 0x0001050607040302ull, 0x0205060704030100ull,
     0x0002050607040301ull, 0x0102050607040300ull, 0x0001020506070403ull,
     0x0305060704020100ull, 0x0003050607040201ull, 0x0103050607040200ull,
     0x0001030506070402ull, 0x0203050607040100ull, 0x0002030506070401ull,
     0x0102030506070400ull, 0x0001020305060704ull, 0x0405060703020100ull,
     0x0004050607030201ull, 0x0104050607030200ull, 0x0001040506070302ull,
     0x0204050607030100ull, 0x0002040506070301ull, 0x0102040506070300ull,
     0x0001020405060703ull, 0x0304050607020100ull, 0x0003040506070201ull,
     0x0103040506070200ull, 0x0001030405060702ull, 0x0203040506070100ull,
     0x0002030405060701ull, 0x0102030405060700ull, 0x0001020304050607ull};

size_t probe_ver(const uint32_t *keys, uint32_t *vals, size_t size,
                 const bucket_t *table, int log_table_bytes,
                 uint32_t table_factor, uint32_t invalid_key)
{
	size_t i = 0, o = 0, b = 0;
	assert(sizeof(table[0]) == 8);
	const size_t buckets = ((size_t) 1) << (log_table_bytes - 3);
	const __m128i shift = _mm_cvtsi32_si128(32 - log_2(buckets));
	const __m256i factor = _mm256_set1_epi32(table_factor);
	const __m256i empty = _mm256_set1_epi32(invalid_key);
	const __m256i buckets_minus_1 = _mm256_set1_epi32(buckets - 1);
	const __m256i mask_1 = _mm256_set1_epi32(1);
#if defined __INTEL_COMPILER &&  __INTEL_COMPILER < 1600
	const long *table_64 = (const long*) table;
#else
	const long long *table_64 = (const long long*) table;
#endif
	const size_t buf_size = 128;
	int buf_space[buf_size + 8 + 15];
	int *buf = align((void*) buf_space);
	__m256i key, off, inv = _mm256_set1_epi32(-1);
	while (i + 8 <= size) {
		// load new items (mask out reloads)
		__m256i new_key = _mm256_maskload_epi32((const int*) &keys[i], inv);
		key = _mm256_andnot_si256(inv, key);
		key = _mm256_or_si256(key, new_key);
		// hash
		__m256i h = _mm256_mullo_epi32(key, factor);
		off = _mm256_add_epi32(off, mask_1);
		off = _mm256_andnot_si256(inv, off);
		h = _mm256_srl_epi32(h, shift);
		h = _mm256_add_epi32(h, off);
		h = _mm256_and_si256(h, buckets_minus_1);
		// gather
		__m256i tab_lo = _mm256_i32gather_epi64(table_64, _mm256_castsi256_si128(h), 8);
		h = _mm256_permute4x64_epi64(h, _MM_SHUFFLE(1,0,3,2));
		__m256i tab_hi = _mm256_i32gather_epi64(table_64, _mm256_castsi256_si128(h), 8);
		__m256i tab_key = _mm256_packlo_epi32(tab_lo, tab_hi);
		__m256i tab_val = _mm256_packhi_epi32(tab_lo, tab_hi);
		// update count & sum
		inv = _mm256_cmpeq_epi32(tab_key, empty);
		__m256i out = _mm256_cmpeq_epi32(tab_key, key);
		inv = _mm256_or_si256(inv, out);
		// load permutation masks
		size_t j = _mm256_movemask_ps(_mm256_castsi256_ps(inv));
		size_t k = _mm256_movemask_ps(_mm256_castsi256_ps(out));
		__m128i perm_inv_comp = _mm_loadl_epi64((__m128i*) &perm[j]);
		__m128i perm_out_comp = _mm_loadl_epi64((__m128i*) &perm[k ^ 255]);
		__m256i perm_inv = _mm256_cvtepi8_epi32(perm_inv_comp);
		__m256i perm_out = _mm256_cvtepi8_epi32(perm_out_comp);
		// permutation for invalid
		inv = _mm256_permutevar8x32_epi32(inv, perm_inv);
		key = _mm256_permutevar8x32_epi32(key, perm_inv);
		off = _mm256_permutevar8x32_epi32(off, perm_inv);
		i += _mm_popcnt_u64(j);
		// permutation for output
		tab_val = _mm256_permutevar8x32_epi32(tab_val, perm_out);
		out = _mm256_permutevar8x32_epi32(out, perm_out);
		_mm256_maskstore_epi32(&buf[b], out, tab_val);
		b += _mm_popcnt_u64(k);
		// flush buffer
		if (b > buf_size) {
			size_t b_i = 0;
			do {
				__m256i x = _mm256_load_si256((__m256i*) &buf[b_i]);
				_mm256_stream_si256((__m256i*) &vals[o], x);
				b_i += 8;
				o += 8;
			} while (b_i != buf_size);
			__m256i x = _mm256_load_si256((__m256i*) &buf[b_i]);
			_mm256_store_si256((__m256i*) &buf[0], x);
			b -= buf_size;
		}
	}
	// clean buffer
	size_t b_i = 0;
	while (b_i != b) _mm_stream_si32(&((int*) vals)[o++], buf[b_i++]);
	// extract last keys
	uint32_t l_keys[8];
	_mm256_storeu_si256((__m256i*) l_keys, key);
	size_t j = _mm256_movemask_ps(_mm256_castsi256_ps(inv));
	j = 8 - _mm_popcnt_u64(j);
	i += j;
	while (i != size) l_keys[j++] = keys[i++];
	// process last keys
	const uint8_t s = 32 - log_2(buckets);
	for (i = 0 ; i != j ; ++i) {
		uint32_t k = l_keys[i];
		size_t h = ((uint32_t) (k * table_factor)) >> s;
		while (invalid_key != table[h].key) {
			if (k == table[h].key) {
				_mm_stream_si32(&((int*) vals)[o++], table[h].val);
				break;
			}
			h = (h + 1) & (buckets - 1);
		}
	}
	return o;
}

inline __m256i _mm256_i32gather_epi64_emu(const uint64_t *table, __m128i index)
{
	__m128i index_R = _mm_shuffle_epi32(index, _MM_SHUFFLE(1,0,3,2));
	__m128i i12 = _mm_cvtepi32_epi64(index);
	__m128i i34 = _mm_cvtepi32_epi64(index_R);
	size_t i1 = _mm_cvtsi128_si64(i12);
	size_t i3 = _mm_cvtsi128_si64(i34);
	__m128i d1 = _mm_loadl_epi64((__m128i*) &table[i1]);
	__m128i d3 = _mm_loadl_epi64((__m128i*) &table[i3]);
	i12 = _mm_srli_si128(i12, 8);
	i34 = _mm_srli_si128(i34, 8);
	size_t i2 = _mm_cvtsi128_si64(i12);
	size_t i4 = _mm_cvtsi128_si64(i34);
	__m128i d2 = _mm_loadl_epi64((__m128i*) &table[i2]);
	__m128i d4 = _mm_loadl_epi64((__m128i*) &table[i4]);
	__m256i d12 = _mm256_castsi128_si256(_mm_unpacklo_epi64(d1, d2));
	__m256i d34 = _mm256_castsi128_si256(_mm_unpacklo_epi64(d3, d4));
	return _mm256_permute2x128_si256(d12, d34, _MM_SHUFFLE(0,2,0,0));
}

size_t probe_ver_emu(const uint32_t *keys, uint32_t *vals, size_t size,
                     const bucket_t *table, int log_table_bytes,
                     uint32_t table_factor, uint32_t invalid_key)
{
	size_t i = 0, o = 0, b = 0;
	assert(sizeof(table[0]) == 8);
	const size_t buckets = ((size_t) 1) << (log_table_bytes - 3);
	const __m128i shift = _mm_cvtsi32_si128(32 - log_2(buckets));
	const __m256i factor = _mm256_set1_epi32(table_factor);
	const __m256i empty = _mm256_set1_epi32(invalid_key);
	const __m256i buckets_minus_1 = _mm256_set1_epi32(buckets - 1);
	const __m256i mask_1 = _mm256_set1_epi32(1);
	const uint64_t *table_64 = (const uint64_t*) table;
	const size_t buf_size = 128;
	int buf_space[buf_size + 8 + 15];
	int *buf = align((void*) buf_space);
	__m256i key, off, inv = _mm256_set1_epi32(-1);
	while (i + 8 <= size) {
		// load new items (mask out reloads)
		__m256i new_key = _mm256_maskload_epi32((const int*) &keys[i], inv);
		key = _mm256_andnot_si256(inv, key);
		key = _mm256_or_si256(key, new_key);
		// hash
		__m256i h = _mm256_mullo_epi32(key, factor);
		off = _mm256_add_epi32(off, mask_1);
		off = _mm256_andnot_si256(inv, off);
		h = _mm256_srl_epi32(h, shift);
		h = _mm256_add_epi32(h, off);
		h = _mm256_and_si256(h, buckets_minus_1);
		// gather
		__m256i tab_lo = _mm256_i32gather_epi64_emu(table_64, _mm256_castsi256_si128(h));
		h = _mm256_permute4x64_epi64(h, _MM_SHUFFLE(1,0,3,2));
		__m256i tab_hi = _mm256_i32gather_epi64_emu(table_64, _mm256_castsi256_si128(h));
		__m256i tab_key = _mm256_packlo_epi32(tab_lo, tab_hi);
		__m256i tab_val = _mm256_packhi_epi32(tab_lo, tab_hi);
		// update count & sum
		inv = _mm256_cmpeq_epi32(tab_key, empty);
		__m256i out = _mm256_cmpeq_epi32(tab_key, key);
		inv = _mm256_or_si256(inv, out);
		// load permutation masks
		size_t j = _mm256_movemask_ps(_mm256_castsi256_ps(inv));
		size_t k = _mm256_movemask_ps(_mm256_castsi256_ps(out));
		__m128i perm_inv_comp = _mm_loadl_epi64((__m128i*) &perm[j]);
		__m128i perm_out_comp = _mm_loadl_epi64((__m128i*) &perm[k ^ 255]);
		__m256i perm_inv = _mm256_cvtepi8_epi32(perm_inv_comp);
		__m256i perm_out = _mm256_cvtepi8_epi32(perm_out_comp);
		// permutation for invalid
		inv = _mm256_permutevar8x32_epi32(inv, perm_inv);
		key = _mm256_permutevar8x32_epi32(key, perm_inv);
		off = _mm256_permutevar8x32_epi32(off, perm_inv);
		i += _mm_popcnt_u64(j);
		// permutation for output
		tab_val = _mm256_permutevar8x32_epi32(tab_val, perm_out);
		out = _mm256_permutevar8x32_epi32(out, perm_out);
		_mm256_maskstore_epi32(&buf[b], out, tab_val);
		b += _mm_popcnt_u64(k);
		// flush buffer
		if (b > buf_size) {
			size_t b_i = 0;
			do {
				__m256i x = _mm256_load_si256((__m256i*) &buf[b_i]);
				_mm256_stream_si256((__m256i*) &vals[o], x);
				b_i += 8;
				o += 8;
			} while (b_i != buf_size);
			__m256i x = _mm256_load_si256((__m256i*) &buf[b_i]);
			_mm256_store_si256((__m256i*) &buf[0], x);
			b -= buf_size;
		}
	}
	// clean buffer
	size_t b_i = 0;
	while (b_i != b) _mm_stream_si32(&((int*) vals)[o++], buf[b_i++]);
	// extract last keys
	uint32_t l_keys[8];
	_mm256_storeu_si256((__m256i*) l_keys, key);
	size_t j = _mm256_movemask_ps(_mm256_castsi256_ps(inv));
	j = 8 - _mm_popcnt_u64(j);
	i += j;
	while (i != size) l_keys[j++] = keys[i++];
	// process last keys
	const uint8_t s = 32 - log_2(buckets);
	for (i = 0 ; i != j ; ++i) {
		uint32_t k = l_keys[i];
		size_t h = ((uint32_t) (k * table_factor)) >> s;
		while (invalid_key != table[h].key) {
			if (k == table[h].key) {
				_mm_stream_si32(&((int*) vals)[o++], table[h].val);
				break;
			}
			h = (h + 1) & (buckets - 1);
		}
	}
	return o;
}

#else

size_t probe_ver(const uint32_t *keys, uint32_t *vals, size_t size,
                 const bucket_t *table, int log_table_bytes,
                 uint32_t table_factor, uint32_t invalid_key)
{
	size_t i = 0, o = 0, b = 0;
	assert(sizeof(table[0]) == 8);
	const size_t buckets = ((size_t) 1) << (log_table_bytes - 3);
	const __mmask16 z = _mm512_int2mask(0);
	const __mmask16 blend_AAAA = _mm512_int2mask(0xAAAA);
	const __mmask16 blend_5555 = _mm512_int2mask(0x5555);
	const __m512i shift = _mm512_set1_epi32(32 - log_2(buckets));
	const __m512i factor = _mm512_set1_epi32(table_factor);
	const __m512i empty = _mm512_set1_epi32(invalid_key);
	const __m512i buckets_minus_1 = _mm512_set1_epi32(buckets - 1);
	const __m512i unpack = _mm512_set_epi32(15,13,11,9,7,5,3,1,14,12,10,8,6,4,2,0);
	const __m512i mask_1 = _mm512_set1_epi32(1);
	const size_t buf_size = 128;
	int buf_space[buf_size + 16 + 15];
	int *buf = align((void*) buf_space);
	__m512i key, off;
	__mmask16 k = _mm512_kxnor(k, k);
	while (i + 16 <= size) {
		// load new items (mask out reloads)
		key = _mm512_mask_loadunpacklo_epi32(key, k, &keys[i]);
		key = _mm512_mask_loadunpackhi_epi32(key, k, &keys[i + 16]);
		i += _mm_countbits_64(_mm512_kconcatlo_64(z, k));
		// hash & update offsets
		__m512i h = _mm512_mullo_epi32(key, factor);
		off = _mm512_add_epi32(off, mask_1);
		off = _mm512_mask_xor_epi32(off, k, off, off);
		h = _mm512_srlv_epi32(h, shift);
		h = _mm512_add_epi32(h, off);
		h = _mm512_and_epi32(h, buckets_minus_1);
		// load pairs from table
		__m512i tab_lo = _mm512_i32logather_epi64(h, table, 8);
		h = _mm512_permute4f128_epi32(h, _MM_PERM_BADC);
		__m512i tab_hi = _mm512_i32logather_epi64(h, table, 8);
		// split keys & payloads
		__m512i tab_key = _mm512_mask_blend_epi32(blend_AAAA, tab_lo, _mm512_swizzle_epi32(tab_hi, _MM_SWIZ_REG_CDAB));
		__m512i tab_val = _mm512_mask_blend_epi32(blend_5555, tab_hi, _mm512_swizzle_epi32(tab_lo, _MM_SWIZ_REG_CDAB));
		tab_key = _mm512_permutevar_epi32(unpack, tab_key);
		tab_val = _mm512_permutevar_epi32(unpack, tab_val);
		// find matches and write to buffer
		k = _mm512_cmpeq_epi32_mask(tab_key, key);
		_mm512_mask_packstorelo_epi32(&buf[b +  0], k, tab_val);
		_mm512_mask_packstorehi_epi32(&buf[b + 16], k, tab_val);
		b += _mm_countbits_64(_mm512_kconcatlo_64(z, k));
		// flush buffer
		if (b > buf_size) {
			b -= buf_size;
			size_t b_i = 0;
			do {
				__m512 x = _mm512_load_ps(&buf[b_i]);
				_mm512_storenrngo_ps(&vals[o], x);
				b_i += 16;
				o += 16;
			} while (b_i != buf_size);
			__m512 x = _mm512_load_ps(&buf[b_i]);
			_mm512_store_ps(&buf[0], x);
		}
		// what to replace
		k = _mm512_kor(k, _mm512_cmpeq_epi32_mask(tab_key, empty));
	}
	// clean buffer
	size_t b_i = 0;
	while (b_i != b) vals[o++] = buf[b_i++];
	// extract last keys
	uint32_t l_keys[32];
	k = _mm512_knot(k);
	_mm512_mask_packstorelo_epi32(&l_keys[0],  k, key);
	_mm512_mask_packstorehi_epi32(&l_keys[16], k, key);
	size_t j = _mm_countbits_64(_mm512_kconcatlo_64(z, k));
	while (i != size) l_keys[j++] = keys[i++];
	// process last keys
	const uint8_t s = 32 - log_2(buckets);
	for (i = 0 ; i != j ; ++i) {
		uint32_t k = l_keys[i];
		size_t h = ((uint32_t) (k * table_factor)) >> s;
		while (invalid_key != table[h].key) {
			if (k == table[h].key) {
				vals[o++] = table[h].val;
				break;
			}
			h = (h + 1) & (buckets - 1);
		}
	}
	return o;
}

size_t probe_ver_emu(const uint32_t *keys, uint32_t *vals, size_t size,
                     const bucket_t *table, int log_table_bytes,
                     uint32_t table_factor, uint32_t invalid_key)
{
	size_t e, i = 0, o = 0, b = 0;
	assert(sizeof(table[0]) == 8);
	const size_t buckets = ((size_t) 1) << (log_table_bytes - 3);
	const __mmask16 z = _mm512_int2mask(0);
	const __mmask16 blend_AAAA = _mm512_int2mask(0xAAAA);
	const __mmask16 blend_5555 = _mm512_int2mask(0x5555);
	const __m512i shift = _mm512_set1_epi32(32 - log_2(buckets));
	const __m512i factor = _mm512_set1_epi32(table_factor);
	const __m512i empty = _mm512_set1_epi32(invalid_key);
	const __m512i buckets_minus_1 = _mm512_set1_epi32(buckets - 1);
	const __m512i unpack = _mm512_set_epi32(15,13,11,9,7,5,3,1,14,12,10,8,6,4,2,0);
	const __m512i mask_1 = _mm512_set1_epi32(1);
	const size_t buf_size = 128;
	int buf_space[buf_size + 16 + 15];
	int *buf = align((void*) buf_space);
	const uint64_t *table_64 = (const uint64_t*) table;
	uint32_t index_space[16 + 15];
	uint32_t *index = align((void*) index_space);
	uint64_t data_space[16 + 7];
	uint64_t *data = align((void*) data_space);
	__m512i key, off;
	__mmask16 k = _mm512_kxnor(k, k);
	while (i + 16 <= size) {
		// load new items (mask out reloads)
		key = _mm512_mask_loadunpacklo_epi32(key, k, &keys[i]);
		key = _mm512_mask_loadunpackhi_epi32(key, k, &keys[i + 16]);
		i += _mm_countbits_64(_mm512_kconcatlo_64(z, k));
		// hash & update offsets
		__m512i h = _mm512_mullo_epi32(key, factor);
		off = _mm512_add_epi32(off, mask_1);
		off = _mm512_mask_xor_epi32(off, k, off, off);
		h = _mm512_srlv_epi32(h, shift);
		h = _mm512_add_epi32(h, off);
		h = _mm512_and_epi32(h, buckets_minus_1);
		// load pairs from table
		_mm512_store_epi32(index, h);
		for (e = 0 ; e != 16 ; ++e)
			data[e] = table_64[index[e]];
		__m512i tab_lo = _mm512_load_epi32(&data[0]);
		__m512i tab_hi = _mm512_load_epi32(&data[8]);
		// split keys & payloads
		__m512i tab_key = _mm512_mask_blend_epi32(blend_AAAA, tab_lo, _mm512_swizzle_epi32(tab_hi, _MM_SWIZ_REG_CDAB));
		__m512i tab_val = _mm512_mask_blend_epi32(blend_5555, tab_hi, _mm512_swizzle_epi32(tab_lo, _MM_SWIZ_REG_CDAB));
		tab_key = _mm512_permutevar_epi32(unpack, tab_key);
		tab_val = _mm512_permutevar_epi32(unpack, tab_val);
		// find matches and write to buffer
		k = _mm512_cmpeq_epi32_mask(tab_key, key);
		_mm512_mask_packstorelo_epi32(&buf[b +  0], k, tab_val);
		_mm512_mask_packstorehi_epi32(&buf[b + 16], k, tab_val);
		b += _mm_countbits_64(_mm512_kconcatlo_64(z, k));
		// flush buffer
		if (b > buf_size) {
			b -= buf_size;
			size_t b_i = 0;
			do {
				__m512 x = _mm512_load_ps(&buf[b_i]);
				_mm512_storenrngo_ps(&vals[o], x);
				b_i += 16;
				o += 16;
			} while (b_i != buf_size);
			__m512 x = _mm512_load_ps(&buf[b_i]);
			_mm512_store_ps(&buf[0], x);
		}
		// what to replace
		k = _mm512_kor(k, _mm512_cmpeq_epi32_mask(tab_key, empty));
	}
	// clean buffer
	size_t b_i = 0;
	while (b_i != b) vals[o++] = buf[b_i++];
	// extract last keys
	uint32_t l_keys[32];
	k = _mm512_knot(k);
	_mm512_mask_packstorelo_epi32(&l_keys[0],  k, key);
	_mm512_mask_packstorehi_epi32(&l_keys[16], k, key);
	size_t j = _mm_countbits_64(_mm512_kconcatlo_64(z, k));
	while (i != size) l_keys[j++] = keys[i++];
	// process last keys
	const uint8_t s = 32 - log_2(buckets);
	for (i = 0 ; i != j ; ++i) {
		uint32_t k = l_keys[i];
		size_t h = ((uint32_t) (k * table_factor)) >> s;
		while (invalid_key != table[h].key) {
			if (k == table[h].key) {
				vals[o++] = table[h].val;
				break;
			}
			h = (h + 1) & (buckets - 1);
		}
	}
	return o;
}

#endif

typedef struct {
	pthread_t id;
	int threads;
	int thread;
	int log_table_bytes;
	bucket_t      *table;
	bucket_wide_t *table_wide;
	uint32_t   table_factor;
	uint32_t payload_factor;
	size_t inner_size;
	size_t outer_size;
	size_t  join_size;
	uint32_t *inner;
	uint32_t *outer;
	uint64_t *times;
	pthread_barrier_t *barrier;
} info_t;

void *run(void *arg)
{
	info_t *d = (info_t*) arg;
	assert(pthread_equal(pthread_self(), d->id));
	bind_thread(d->thread, d->threads);
	size_t j, b = 0;
	// copy stuff
	int log_table_bytes = d->log_table_bytes;
	uint32_t *inner = d->inner;
	uint32_t *outer = d->outer;
	size_t inner_size = d->inner_size;
	size_t outer_size = d->outer_size;
	uint32_t payload_factor = d->payload_factor;
	uint32_t   table_factor = d->table_factor;
	bucket_t      *table      = d->table;
	bucket_wide_t *table_wide = d->table_wide;
	uint32_t invalid_key = inner[inner_size];
	// build tables
	if (d->thread == 0) {
		build(inner, inner_size, table, log_table_bytes,
		      table_factor, invalid_key, payload_factor);
		build_wide(inner, inner_size, table_wide, log_table_bytes,
		           table_factor, invalid_key, payload_factor);
		fprintf(stderr, "Table building finished!\n");
	}
	pthread_barrier_wait(&d->barrier[b++]);
	// begin and end of each table
	size_t beg = ((outer_size / d->threads) & -16) *  d->thread;
	size_t end = ((outer_size / d->threads) & -16) * (d->thread + 1);
	if (d->thread + 1 == d->threads) end = outer_size;
	// compute correct results
	uint64_t checksum = 0;
	size_t join_size = probe_sum(&outer[beg], end - beg, table, log_table_bytes,
	                             table_factor, invalid_key, &checksum);
	// initialize output space
	uint32_t *joined = mamalloc(join_size * sizeof(uint32_t));
	memset(joined, 0xDB, join_size * sizeof(uint32_t));
	// scalar plain
	pthread_barrier_wait(&d->barrier[b++]);
	// uint64_t t = thread_time(); // delete behrens
	uint64_t t0 = thread_time();   // add behrens
	j = probe(&outer[beg], joined, end - beg, table, log_table_bytes, table_factor, invalid_key);
	// t = thread_time() - t;    // delete behrens
	uint64_t t1 = thread_time(); // add behrens
	pthread_barrier_wait(&d->barrier[b++]);
	if (d->thread == 0) fprintf(stderr, "Scalar finished!\n");
	// d->times[0] = t; // delete behrens
	d->times[0] = t0;   // add behrens
	d->times[1] = t1;   // add behrens
	assert(j == join_size);
	assert(sum(joined, j) == checksum);
	// scalar branchless
	/* begin delete behrens
	 pthread_barrier_wait(&d->barrier[b++]);
	 t = thread_time();
	 j = probe_hor(&outer[beg], joined, end - beg, table_wide, log_table_bytes, table_factor, invalid_key);
	 t = thread_time() - t;
	 pthread_barrier_wait(&d->barrier[b++]);
	 if (d->thread == 0) fprintf(stderr, "Horizontal finished!\n");
	 d->times[1] = t;
	 assert(j == join_size);
	 assert(sum(joined, j) == checksum);
	 end delete behrens */
	// vector horizontal
	pthread_barrier_wait(&d->barrier[b++]);
	// t = thread_time();        // delete behrens
	uint64_t t2 = thread_time(); // add behrens
	j = probe_ver(&outer[beg], joined, end - beg, table, log_table_bytes, table_factor, invalid_key);
	// t = thread_time() - t;    // delete behrens
	uint64_t t3 = thread_time(); // add behrens
	pthread_barrier_wait(&d->barrier[b++]);
	if (d->thread == 0) fprintf(stderr, "Vectical finished!\n");
	// d->times[2] = t; // delete behrens
	d->times[2] = t2;   // add behrens
	d->times[3] = t3;   // add behrens
	assert(j == join_size);
	assert(sum(joined, j) == checksum);
	// vector horizontal
	/* begin delete behrens
	 pthread_barrier_wait(&d->barrier[b++]);
	 t = thread_time();
	 j = probe_ver_emu(&outer[beg], joined, end - beg, table, log_table_bytes, table_factor, invalid_key);
	 t = thread_time() - t;
	 pthread_barrier_wait(&d->barrier[b++]);
	 if (d->thread == 0) fprintf(stderr, "Vectical (emulated) finished!\n");
	 d->times[3] = t;
	 assert(j == join_size);
	 assert(sum(joined, j) == checksum);
	 end delete behrens */
	// cleanup
	free(joined);
	d->join_size = join_size;
	pthread_exit(NULL);
}

int parse_log_size(const char *s)
{
	char *p = NULL;
	int bits = 0;
	long long base = 1;
	long long num = strtol(s, &p, 10);
	while (base < num) {
		base += base;
		bits++;
	}
	assert(base == num);
	while (isspace(*p)) p++;
	if      (*p == 'K') bits += 10;
	else if (*p == 'M') bits += 20;
	else if (*p == 'G') bits += 30;
	else p--;
	p++;
	assert(*p == 'B');
	p++;
	while (isspace(*p)) p++;
	assert(*p == 0);
	return bits;
}

char *print_log_size(int bits)
{
	static char buf[16];
	if      (bits >= 30) sprintf(buf, "%d GB", 1 << (bits - 30));
	else if (bits >= 20) sprintf(buf, "%d MB", 1 << (bits - 20));
	else if (bits >= 10) sprintf(buf, "%d KB", 1 << (bits - 10));
	else                 sprintf(buf, "%d  B", 1 << bits);
	return buf;
}

int main(int argc, char **argv)
{
	int t, threads = hardware_threads();
	// get arguments
	int log_table_bytes = argc > 1 ? parse_log_size(argv[1]) : 14;
	size_t outer_size = argc > 2 ? atoll(argv[2]) : 1000 * 1000 * 1000;
	double selectivity = argc > 3 ? atof(argv[3]) : 0.9;
	double load_factor = argc > 4 ? atof(argv[4]) : 0.5;
	// extract inner size
	size_t buckets = (((size_t) 1) << log_table_bytes) / sizeof(bucket_t);
	size_t inner_size = load_factor * buckets;
	// print info
	fprintf(stderr, "Threads: %d\n", threads);
	fprintf(stderr, "Table size: %s (2^%d)\n", print_log_size(log_table_bytes), log_table_bytes);
	fprintf(stderr, "Outer tuples: %ld\n", outer_size);
	fprintf(stderr, "Inner tuples: %ld\n", inner_size);
	fprintf(stderr, "Selectivity: %.2f%%\n", selectivity * 100.0);
#ifdef __MIC
	fprintf(stderr, "Running on Phi!\n");
#else
	fprintf(stderr, "Running on Haswell!\n");
#endif
	// method names
	/* begin delete behrens
	 char *names[] = {"Scalar    ",
	 "Horizontal",
	 "Vertical  ",
	 "E.Vertical"};
	 end delete behrens */
	char *names[] = {"Scalar    ", "Vertical  "}; // add behrens
	int m, methods = sizeof(names) / sizeof(char*);
	// barriers
	int b, barriers = methods * 2 + 1;
	pthread_barrier_t barrier[barriers];
	for (b = 0 ; b != barriers ; ++b)
		pthread_barrier_init(&barrier[b], NULL, threads);
	// initialize data
	uint32_t *inner = NULL;
	uint32_t *outer = NULL;
	size_t join_size = inner_outer(inner_size, outer_size, selectivity, &inner, &outer);
	// init tables
	// uint32_t   table_factor = (rand() << 1) | 1; // delete behrens
	// uint32_t payload_factor = (rand() << 1) | 1; // delete behrens
	uint32_t   table_factor = 2654435761; // add behrens
	uint32_t payload_factor = 42;         // add behrens
	bucket_t      *table      = mamalloc(((size_t) 1) << log_table_bytes);
	bucket_wide_t *table_wide = mamalloc(((size_t) 1) << log_table_bytes);
	// run threads
	info_t info[threads];
	// uint64_t times[methods * threads];  // delete behrens
	uint64_t times[methods * 2 * threads]; // add behrens
	for (t = 0 ; t != threads ; ++t) {
		info[t].threads = threads;
		info[t].thread = t;
		info[t].log_table_bytes = log_table_bytes;
		info[t].table      = table;
		info[t].table_wide = table_wide;
		info[t].table_factor = table_factor;
		info[t].payload_factor = payload_factor;
		info[t].inner_size = inner_size;
		info[t].outer_size = outer_size;
		info[t].inner = inner;
		info[t].outer = outer;
		info[t].barrier = barrier;
		// info[t].times = &times[methods * t]; // del behrens
		info[t].times = &times[methods * 2 * t]; // add behrens
		pthread_create(&info[t].id, NULL, run, (void*) &info[t]);
	}
	size_t j = 0;
	for (t = 0 ; t != threads ; ++t) {
		pthread_join(info[t].id, NULL);
		j += info[t].join_size;
	}
	assert(j == join_size);
	// cleanup
	free(table);
	free(table_wide);
	free(inner);
	free(outer);
	for (b = 0 ; b != barriers ; ++b)
		pthread_barrier_destroy(&barrier[b]);
	// print times
	fflush(stderr);
	printf("%6s", print_log_size(log_table_bytes));
	for (m = 0 ; m != methods ; ++m) {
		uint64_t ns = 0;
		/* begin delete behrens
		 for (t = 0 ; t != threads ; ++t)
		 	ns += info[t].times[m];
		 ns /= threads;
		 printf(", %6.3f", outer_size * 1.0 / ns);
		 end delete behrens */
		// begin add behrens
		uint64_t begin_min = info[0].times[m*2+0];
		uint64_t end_max = info[0].times[m*2+1];
		for (t = 0 ; t != threads ; ++t)
		{
			// find minimum begin time
			if (info[t].times[m*2+0] < begin_min) {
				begin_min = info[t].times[m*2+0];
			}
			// find maximum end time
			if (info[t].times[m*2+1] > end_max) {
				end_max = info[t].times[m*2+1];
			}
		}
		ns = end_max - begin_min;
		printf("; %6.3f; %ld", outer_size * 1.0 / ns, ns);
		// end add behrens
	//	fprintf(stderr, "%s: %.2f btps (%ld ns)\n", names[m], outer_size * 1.0 / ns, ns);
	}
	printf("\n");
	return EXIT_SUCCESS;
}
