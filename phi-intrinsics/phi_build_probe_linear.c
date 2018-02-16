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
#include <sched.h>
#include <time.h>

#include "rand.h"

#define KERNEL_LOOPS 1 // add behrens

uint64_t thread_time(void)
{
	struct timespec b;
	// assert(clock_gettime(CLOCK_THREAD_CPUTIME_ID, &b) == 0); // delete behrens
	assert(clock_gettime(CLOCK_MONOTONIC, &b) == 0); // add behrens
	return b.tv_sec * 1000 * 1000 * 1000 + b.tv_nsec;
}

int hardware_threads(void)
{
	char name[64];
	struct stat st;
	int threads = -1;
	do {
		sprintf(name, "/sys/devices/system/cpu/cpu%d", ++threads);
	} while (stat(name, &st) == 0);
	return threads;
}

void bind_thread(int thread_id)
{
	int threads = hardware_threads();
	assert(thread_id >= 0 && thread_id < threads);
	size_t size = CPU_ALLOC_SIZE(threads);
	cpu_set_t *cpu_set = CPU_ALLOC(threads);
	assert(cpu_set != NULL);
	CPU_ZERO_S(size, cpu_set);
	CPU_SET_S(thread_id, size, cpu_set);
	assert(pthread_setaffinity_np(pthread_self(),
	       size, cpu_set) == 0);
	CPU_FREE(cpu_set);
}

void *mamalloc(size_t size)
{
	void *ptr = NULL;
	return posix_memalign(&ptr, 64, size) ? NULL : ptr;
}

void *align(const void *p)
{
	size_t i = 63 & (size_t) p;
	return (void*) (i ? p + 64 - i : p);
}

#ifdef __MIC // add behrens
void print_vector(const char *m, __m512i x)
{
	uint32_t space[31];
	uint32_t *arr = align(space);
	_mm512_store_epi32(arr, x);
	fprintf(stderr, "%s: [%3u", m, arr[0]);
	size_t i = 1;
	do {
		fprintf(stderr, ", %3u", arr[i]);
	} while (++i != 16);
	fprintf(stderr, "]\n");
}

void print_mask(__mmask16 m)
{
	uint32_t x = _mm512_mask2int(m);
	size_t i;
	for (i = 0 ; i != 16 ; ++i) {
		fputc('0' + (x & 1), stderr);
		x >>= 1;
	}
	fputc('\n', stderr);
}
#endif // add behrens

int prime(uint64_t x)
{
	uint64_t d;
	assert(x > 1);
	if (x % 2 == 0)
		return x == 2;
	for (d = 3 ; d * d <= x ; d += 2)
		if (x % d == 0)
			return 0;
	return 1;
}

void build_scalar(const uint32_t *keys, const uint32_t *vals, size_t size,
                  uint64_t *table, size_t buckets, uint32_t factor, uint32_t empty)
{
	size_t i, fudge_buckets = buckets * 1.2;
	for (int loops = 0; loops < KERNEL_LOOPS; ++loops) { // add behrens
	for (i = 0 ; i != fudge_buckets ; ++i)
		table[i] = empty;
	for (i = 0 ; i != size ; ++i) {
		uint32_t k = keys[i];
		uint64_t p = vals[i];
		p = (p << 32) | k;
		size_t h = (uint32_t) (k * factor);
		h = (h * buckets) >> 32;
		while (empty != (uint32_t) table[h]) h++;
		table[h] = p;
	}
	} // add behrens
	assert(table[fudge_buckets - 1] == empty);
}

size_t probe_scalar(const uint32_t *keys, const uint32_t *vals, size_t size,
                    const uint64_t *table, size_t buckets, uint32_t factor, uint32_t empty,
                    uint32_t *keys_out, uint32_t *vals_out, uint32_t *tabs_out)
{
	size_t i, o;
	for (i = o = 0 ; i != size ; ++i) {
		uint32_t k = keys[i];
		uint32_t v = vals[i];
		size_t h = (uint32_t) (k * factor);
		h = (h * buckets) >> 32;
		uint64_t t = table[h];
		if (empty != (uint32_t) t) do {
			if (k == (uint32_t) t) {
				tabs_out[o] = t >> 32;
				vals_out[o] = v;
				keys_out[o++] = k;
#ifdef _UNIQUE
				break;
#endif
			}
			t = table[++h];
		} while (empty != (uint32_t) t);
	}
	return o;
}

#ifdef __MIC // add behrens
void build_vector(const uint32_t *keys, const uint32_t *rids, size_t size,
                  uint64_t *table, size_t buckets, uint32_t factor, uint32_t empty)
{
	__m512i mask_1 = _mm512_set1_epi32(1);
	__m512i mask_empty = _mm512_set1_epi32(empty);
	__m512i mask_factor = _mm512_set1_epi32(factor);
	__m512i mask_buckets = _mm512_set1_epi32(buckets);
	__m512i mask_pack = _mm512_set_epi32(15, 7, 14, 6, 13, 5, 12, 4, 11, 3, 10, 2, 9, 1, 8, 0);
	__mmask16 blend_0000 = _mm512_int2mask(0x0000);
	__mmask16 blend_AAAA = _mm512_int2mask(0xAAAA);
	__mmask16 blend_5555 = _mm512_int2mask(0x5555);
	// reset table
	__m512i mask_half_empty = _mm512_set1_epi64(empty);
	size_t fudge_buckets = buckets * 1.2;
	size_t i = fudge_buckets;
	assert(table == align(table));
	while (i & 7)
		table[--i] = empty;
	while (i) {
		i -= 8;
		_mm512_store_epi64(&table[i], mask_half_empty);
	}
	// main loop
	size_t size_minus_16 = size - 16;
	__mmask16 k = _mm512_kxnor(k, k);
	__m512i key, rid, off;
	if (size >= 16) do {
		// replace invalid keys & payloads
		key = _mm512_mask_loadunpacklo_epi32(key, k, &keys[i]);
		key = _mm512_mask_loadunpackhi_epi32(key, k, &keys[i + 16]);
		rid = _mm512_mask_loadunpacklo_epi32(rid, k, &rids[i]);
		rid = _mm512_mask_loadunpackhi_epi32(rid, k, &rids[i + 16]);
		off = _mm512_mask_xor_epi32(off, k, off, off);
		i += _mm_countbits_64(_mm512_kconcatlo_64(blend_0000, k));
		// hash keys
		__m512i hash = _mm512_mullo_epi32(key, mask_factor);
		hash = _mm512_mulhi_epu32(hash, mask_buckets);
		hash = _mm512_add_epi32(hash, off);
		// load keys from table and detect conflicts
		__m512i tab = _mm512_i32gather_epi32(hash, table, 8);
		k = _mm512_cmpeq_epi32_mask(tab, mask_empty);
#ifndef _UNIQUE
		// detect conflicts before
		_mm512_mask_i32scatter_epi32(table, k, hash, mask_pack, 8);
		tab = _mm512_mask_i32gather_epi32(tab, k, hash, table, 8);
		k = _mm512_mask_cmpeq_epi32_mask(k, tab, mask_pack);
#endif
		// mix keys and payloads in pairs
		__m512i key_tmp = _mm512_permutevar_epi32(mask_pack, key);
		__m512i rid_tmp = _mm512_permutevar_epi32(mask_pack, rid);
		__m512i lo = _mm512_mask_blend_epi32(blend_AAAA, key_tmp, _mm512_swizzle_epi32(rid_tmp, _MM_SWIZ_REG_CDAB));
		__m512i hi = _mm512_mask_blend_epi32(blend_5555, rid_tmp, _mm512_swizzle_epi32(key_tmp, _MM_SWIZ_REG_CDAB));
		// store valid pairs
		_mm512_mask_i32loscatter_epi64(table, k, hash, lo, 8);
		__m512i hash_S = _mm512_permute4f128_epi32(hash, _MM_PERM_BADC);
		__mmask16 k_S = _mm512_kmerge2l1h(k, k);
		_mm512_mask_i32loscatter_epi64(table, k_S, hash_S, hi, 8);
#ifdef _UNIQUE
		// detect conflicts after
		__m512i back = _mm512_mask_i32gather_epi32(back, k, hash, table, 8);
		k = _mm512_mask_cmpeq_epi32_mask(k, back, key);
#endif
		off = _mm512_add_epi32(off, mask_1);
	} while (i <= size_minus_16);
	// save last items
	uint32_t keys_last[32];
	uint32_t rids_last[32];
	k = _mm512_knot(k);
	_mm512_mask_packstorelo_epi32(&keys_last[0],  k, key);
	_mm512_mask_packstorehi_epi32(&keys_last[16], k, key);
	_mm512_mask_packstorelo_epi32(&rids_last[0],  k, rid);
	_mm512_mask_packstorehi_epi32(&rids_last[16], k, rid);
	size_t j = _mm_countbits_64(_mm512_kconcatlo_64(blend_0000, k));
	for (; i != size ; ++i, ++j) {
		keys_last[j] = keys[i];
		rids_last[j] = rids[i];
	}
	// process last items in scalar code
	for (i = 0 ; i != j ; ++i) {
		uint32_t k = keys_last[i];
		uint64_t p = rids_last[i];
		p = (p << 32) | k;
		size_t h = (uint32_t) (k * factor);
		h = (h * buckets) >> 32;
		while (empty != (uint32_t) table[h]) h++;
		table[h] = p;
	}
	assert(table[fudge_buckets - 1] == empty);
}

size_t probe_vector(const uint32_t *keys, const uint32_t *rids, size_t size,
                    const uint64_t *table, size_t buckets, uint32_t factor, uint32_t empty,
                    uint32_t *keys_out, uint32_t *rids_out, uint32_t *tabs_out)
{
	assert(keys_out == align(keys_out));
	assert(rids_out == align(rids_out));
	// generate masks
	__m512i mask_1 = _mm512_set1_epi32(1);
	__m512i mask_empty = _mm512_set1_epi32(empty);
	__m512i mask_factor = _mm512_set1_epi32(factor);
	__m512i mask_buckets = _mm512_set1_epi32(buckets);
	__m512i mask_unpack = _mm512_set_epi32(15, 13, 11, 9, 7, 5, 3, 1, 14, 12, 10, 8, 6, 4, 2, 0);
	__mmask16 blend_0000 = _mm512_int2mask(0x0000);
	__mmask16 blend_AAAA = _mm512_int2mask(0xAAAA);
	__mmask16 blend_5555 = _mm512_int2mask(0x5555);
	// space for buffers
	size_t buffer_size = 256;
	uint32_t buffer_space[(buffer_size + 16) * 3 + 15];
	uint32_t *keys_buf = align(buffer_space);
	uint32_t *rids_buf = &keys_buf[buffer_size + 16];
	uint32_t *tabs_buf = &rids_buf[buffer_size + 16];
	// main loop
	const size_t size_vec = size - 16;
	size_t b, i = 0, o = 0, j = 0;
	__mmask16 k = _mm512_kxnor(k, k);
	__m512i key, rid, off;
	if (size >= 16) do {
		// replace invalid keys & payloads
		key = _mm512_mask_loadunpacklo_epi32(key, k, &keys[i]);
		key = _mm512_mask_loadunpackhi_epi32(key, k, &keys[i + 16]);
		rid = _mm512_mask_loadunpacklo_epi32(rid, k, &rids[i]);
		rid = _mm512_mask_loadunpackhi_epi32(rid, k, &rids[i + 16]);
		off = _mm512_mask_xor_epi32(off, k, off, off);
		i += _mm_countbits_64(_mm512_kconcatlo_64(blend_0000, k));
		// hash keys using either 1st or 2nd function
		__m512i hash = _mm512_mullo_epi32(key, mask_factor);
		hash = _mm512_mulhi_epu32(hash, mask_buckets);
		hash = _mm512_add_epi32(hash, off);
		// load keys from table and update offsets
		__m512i lo = _mm512_i32logather_epi64(hash, table, 8);
		hash = _mm512_permute4f128_epi32(hash, _MM_PERM_BADC);
		__m512i hi = _mm512_i32logather_epi64(hash, table, 8);
		// split keys and payloads
		__m512i tab_key = _mm512_mask_blend_epi32(blend_AAAA, lo, _mm512_swizzle_epi32(hi, _MM_SWIZ_REG_CDAB));
		__m512i tab_rid = _mm512_mask_blend_epi32(blend_5555, hi, _mm512_swizzle_epi32(lo, _MM_SWIZ_REG_CDAB));
		tab_key = _mm512_permutevar_epi32(mask_unpack, tab_key);
		tab_rid = _mm512_permutevar_epi32(mask_unpack, tab_rid);
		// compare
		__mmask16 m = _mm512_cmpeq_epi32_mask(tab_key, key);
		k = _mm512_cmpeq_epi32_mask(tab_key, mask_empty);
#ifdef _UNIQUE
		k = _mm512_kor(k, m);
#endif
		// pack store matches
		_mm512_mask_packstorelo_epi32(&keys_buf[j], m, key);
		_mm512_mask_packstorehi_epi32(&keys_buf[j + 16], m, key);
		_mm512_mask_packstorelo_epi32(&rids_buf[j], m, rid);
		_mm512_mask_packstorehi_epi32(&rids_buf[j + 16], m, rid);
		_mm512_mask_packstorelo_epi32(&tabs_buf[j], m, tab_rid);
		_mm512_mask_packstorehi_epi32(&tabs_buf[j + 16], m, tab_rid);
		j += _mm_countbits_64(_mm512_kconcatlo_64(blend_0000, m));
		if (j >= buffer_size) {
			j -= buffer_size;
			for (b = 0 ; b != buffer_size ; b += 16, o += 16) {
				__m512 x = _mm512_load_ps(&keys_buf[b]);
				__m512 y = _mm512_load_ps(&rids_buf[b]);
				__m512 z = _mm512_load_ps(&tabs_buf[b]);
				_mm512_storenrngo_ps(&keys_out[o], x);
				_mm512_storenrngo_ps(&rids_out[o], y);
				_mm512_storenrngo_ps(&tabs_out[o], z);
			}
			__m512 x = _mm512_load_ps(&keys_buf[b]);
			__m512 y = _mm512_load_ps(&rids_buf[b]);
			__m512 z = _mm512_load_ps(&tabs_buf[b]);
			_mm512_store_ps(keys_buf, x);
			_mm512_store_ps(rids_buf, y);
			_mm512_store_ps(tabs_buf, z);
		}
		off = _mm512_add_epi32(off, mask_1);
	} while (i <= size_vec);
	// flush last items
	for (b = 0 ; b != j ; ++b, ++o) {
		keys_out[o] = keys_buf[b];
		rids_out[o] = rids_buf[b];
		tabs_out[o] = tabs_buf[b];
	}
	// save last items
	uint32_t keys_last[32];
	uint32_t rids_last[32];
	uint32_t offs_last[32];
	k = _mm512_knot(k);
	_mm512_mask_packstorelo_epi32(&keys_last[0],  k, key);
	_mm512_mask_packstorehi_epi32(&keys_last[16], k, key);
	_mm512_mask_packstorelo_epi32(&rids_last[0],  k, rid);
	_mm512_mask_packstorehi_epi32(&rids_last[16], k, rid);
	_mm512_mask_packstorelo_epi32(&offs_last[0],  k, off);
	_mm512_mask_packstorehi_epi32(&offs_last[16], k, off);
	j = _mm_countbits_64(_mm512_kconcatlo_64(blend_0000, k));
	for (; i != size ; ++i, ++j) {
		keys_last[j] = keys[i];
		rids_last[j] = rids[i];
		offs_last[j] = 0;
	}
	// process last items in scalar code
	for (i = 0 ; i != j ; ++i) {
		uint32_t k = keys_last[i];
		uint32_t r = rids_last[i];
		size_t h = (uint32_t) (k * factor);
		h = (h * buckets) >> 32;
		h += offs_last[i];
		uint64_t t = table[h];
		while (empty != (uint32_t) t) {
			if (k == (uint32_t) t) {
				tabs_out[o] = t >> 32;
				rids_out[o] = r;
				keys_out[o++] = k;
#ifdef _UNIQUE
				break;
#endif
			}
			t = table[++h];
		}
	}
	return o;
}
#endif // add behrens

int uint32_cmp(const void *x, const void *y)
{
	uint32_t a = *((uint32_t*) x);
	uint32_t b = *((uint32_t*) y);
	return a < b ? -1 : a > b ? 1 : 0;
}

void shuffle(uint32_t *data, size_t size, rand32_t *gen)
{
	size_t i, j;
	for (i = 0 ; i != size ; ++i) {
		j = rand32_next(gen);
		j *= size - i;
		j >>= 32;
		j += i;
		uint32_t temp = data[i];
		data[i] = data[j];
		data[j] = temp;
	}
}

void generate_unique(uint32_t *data, size_t size, size_t limit, rand32_t *gen)
{
	size_t i, j, extra_size = size;
	do {
		extra_size *= 1.1;
		assert(extra_size <= limit);
		for (i = 0 ; i != extra_size ; ++i)
			data[i] = rand32_next(gen);
		qsort(data, extra_size, sizeof(uint32_t), uint32_cmp);
		uint32_t p_key = data[0];
		for (i = j = 0 ; i != extra_size ; ++i) {
			uint32_t n_key = data[i];
			if (n_key != p_key) {
				data[j++] = p_key;
				p_key = n_key;
			}
		}
		data[j++] = p_key;
	} while (j < size);
	shuffle(data, size, gen);
}

size_t min(size_t x, size_t y) { return x < y ? x : y; }

typedef struct {
	pthread_t id;
	int cpu;
	int seed;
	size_t repeats;
	size_t inner_tuples;
	size_t outer_tuples;
	size_t inner_distinct;
	size_t outer_distinct;
	size_t joined_total;
	double selectivity;
	double load_factor;
	// uint64_t times[2]; // delete behrens
	uint64_t times[4];  // add behrens
	pthread_barrier_t *barrier;
} info_t;

void *run(void *arg)
{
	info_t *d = (info_t*) arg;
	assert(pthread_equal(pthread_self(), d->id));
	bind_thread(d->cpu);
	size_t i, j, r, repeats = d->repeats;
	size_t outer_tuples = d->outer_tuples;
	size_t inner_tuples = d->inner_tuples;
	size_t outer_distinct = d->outer_distinct;
	size_t inner_distinct = d->inner_distinct;
	size_t join_distinct = d->selectivity * min(inner_distinct, outer_distinct);
	size_t distinct = outer_distinct + inner_distinct - join_distinct;
	double outer_repeats = outer_tuples * 1.0 / outer_distinct;
	double inner_repeats = inner_tuples * 1.0 / inner_distinct;
	size_t join_tuples = outer_repeats * inner_repeats * join_distinct * 1.05 + 16;
	join_tuples &= -16;
	rand32_t *gen = rand32_init(d->seed);
	// uint32_t inner_factor = rand32_next(gen) | 1; // delete behrens
	// uint32_t outer_factor = rand32_next(gen) | 1; // delete behrens
	uint32_t inner_factor = 42; // add behrens
	// space for unique keys and space for table
	size_t buckets = inner_tuples / d->load_factor;
	size_t *joined = malloc(repeats * sizeof(size_t));
	uint32_t *empty = malloc(repeats * sizeof(uint32_t));
	uint64_t *table = mamalloc(buckets * 1.2 * sizeof(uint64_t));
	size_t distinct_keys_space = distinct * 1.5;
	uint32_t *distinct_keys = malloc(distinct_keys_space * sizeof(uint32_t));
#ifdef _BUILD_ONLY
	size_t random_bucket = rand32_next(gen) % buckets;
	uint64_t random_checksum = 0;
#endif
	// generate inputs
	uint32_t *inner_keys = mamalloc(inner_tuples * repeats * sizeof(uint32_t));
	uint32_t *inner_rids = mamalloc(inner_tuples * repeats * sizeof(uint32_t));
#ifndef _BUILD_ONLY // add behrens
	uint32_t *outer_keys = mamalloc(outer_tuples * repeats * sizeof(uint32_t));
	uint32_t *outer_rids = mamalloc(outer_tuples * repeats * sizeof(uint32_t));
#endif // add behrens
	for (r = 0 ; r != repeats ; ++r) {
		// generate unique keys
		generate_unique(distinct_keys, distinct + 1, distinct_keys_space, gen);
		empty[r] = distinct_keys[distinct];
		// copy unique keys for inner side
		uint32_t *keys = &inner_keys[r * inner_tuples];
		// uint32_t *rids = &inner_rids[r * inner_tuples]; // delete behrens
		for (i = 0 ; i != inner_distinct ; ++i) {
			size_t j = i + outer_distinct - join_distinct;
			keys[i] = distinct_keys[j];
		}
		for (; i != inner_tuples ; ++i) {
			uint64_t j = rand32_next(gen);
			j *= inner_distinct;
			j >>= 32;
			j += outer_distinct - join_distinct;
			keys[i] = distinct_keys[j];
		}
		shuffle(keys, inner_tuples, gen);
		// generate keys for outer side
#ifndef _BUILD_ONLY // add behrens
		keys = &outer_keys[r * outer_tuples];
		rids = &outer_rids[r * outer_tuples];
		for (i = 0 ; i != outer_distinct ; ++i)
			keys[i] = distinct_keys[i];
		for (; i != outer_tuples ; ++i) {
			uint64_t j = rand32_next(gen);
			j *= outer_distinct;
			j >>= 32;
			keys[i] = distinct_keys[j];
		}
		shuffle(keys, outer_tuples, gen);
#endif // add behrens
	}
	for (i = 0 ; i != inner_tuples * repeats ; ++i)
		inner_rids[i] = inner_keys[i] * inner_factor;
#ifndef _BUILD_ONLY // add behrens
	for (i = 0 ; i != outer_tuples * repeats ; ++i)
		outer_rids[i] = outer_keys[i] * outer_factor;
#endif // add behrens
	// generate outputs
	uint32_t *joined_keys = mamalloc(join_tuples * repeats * sizeof(uint32_t));
	uint32_t *joined_rids = mamalloc(join_tuples * repeats * sizeof(uint32_t));
	uint32_t *joined_tabs = mamalloc(join_tuples * repeats * sizeof(uint32_t));
	for (i = 0 ; i != join_tuples * repeats ; ++i) {
		joined_keys[i] = 0xCAFEBABE;
		joined_rids[i] = 0xDEADBEEF;
		joined_tabs[i] = 0xDBBEBEDB;
	}
	free(distinct_keys);
	// start experiments
	pthread_barrier_wait(&d->barrier[0]);
	uint64_t t1 = thread_time();
	for (r = 0 ; r != repeats ; ++r) {
		// uint32_t factor = rand32_next(gen) | 1; // delete behrens
		uint32_t factor = 2654435761; // add behrens
		build_scalar(&inner_keys[r * inner_tuples],
		             &inner_rids[r * inner_tuples],
		             inner_tuples, table, buckets, factor, empty[r]);
#ifndef _BUILD_ONLY
		j = probe_scalar(&outer_keys[r * outer_tuples],
		                 &outer_rids[r * outer_tuples],
		                 outer_tuples, table, buckets, factor, empty[r],
		                 &joined_keys[r * join_tuples],
		                 &joined_rids[r * join_tuples],
		                 &joined_tabs[r * join_tuples]);
		if (j > join_tuples)
			fprintf(stderr, "%ld: %ld > %ld\n", r, j, join_tuples);
		assert(j <= join_tuples);
		joined[r] = j;
#else
		random_checksum += table[random_bucket];
#endif
	}
	// t1 = thread_time() - t1;  // delete behrens
	uint64_t t2 = thread_time(); // add behrens
//	if (!d->cpu) fprintf(stderr, "Scalar done!\n");
	pthread_barrier_wait(&d->barrier[1]);
#ifndef _BUILD_ONLY
	for (r = 0 ; r != repeats ; ++r) {
		uint32_t *keys = &joined_keys[r * join_tuples];
		uint32_t *rids = &joined_rids[r * join_tuples];
		uint32_t *tabs = &joined_tabs[r * join_tuples];
		j = joined[r];
		for (i = 0 ; i != j ; ++i) {
			assert(rids[i] == keys[i] * outer_factor);
			assert(tabs[i] == keys[i] * inner_factor);
		}
	}
#endif
	pthread_barrier_wait(&d->barrier[2]);
	// uint64_t t2 = thread_time(); // delete behrens
	uint64_t t3 = thread_time();	// add behrens
#ifdef __MIC // add behrens
	for (r = 0 ; r != repeats ; ++r) {
		// uint32_t factor = rand32_next(gen) | 1; // delete behrens
		uint32_t factor = 2654435761; // add behrens
		build_vector(&inner_keys[r * inner_tuples],
		             &inner_rids[r * inner_tuples],
		             inner_tuples, table, buckets, factor, empty[r]);
#ifndef _BUILD_ONLY
		j = probe_vector(&outer_keys[r * outer_tuples],
		                 &outer_rids[r * outer_tuples],
		                 outer_tuples, table, buckets, factor, empty[r],
		                 &joined_keys[r * join_tuples],
		                 &joined_rids[r * join_tuples],
		                 &joined_tabs[r * join_tuples]);
		assert(joined[r] == j);
#else
		random_checksum += table[random_bucket];
#endif
	}
#endif
	// t2 = thread_time() - t2;  // delete behrens
	uint64_t t4 = thread_time(); // add behrens
	//	if (!d->cpu) fprintf(stderr, "Vector done!\n");
	pthread_barrier_wait(&d->barrier[3]);
#ifdef __MIC // add behrens
#ifndef _BUILD_ONLY
	for (r = 0 ; r != repeats ; ++r) {
		uint32_t *keys = &joined_keys[r * join_tuples];
		uint32_t *rids = &joined_rids[r * join_tuples];
		uint32_t *tabs = &joined_tabs[r * join_tuples];
		j = joined[r];
		for (i = 0 ; i != j ; ++i) {
			assert(rids[i] == keys[i] * outer_factor);
			assert(tabs[i] == keys[i] * inner_factor);
		}
	}
#else
	if (random_checksum == 666)
		fprintf(stderr, "666 the number of the beast!\n");
#endif
#endif // add behrens
	size_t joined_total = 0;
	for (r = 0 ; r != repeats ; ++r)
		joined_total += joined[r];
	d->joined_total = joined_total;
	d->times[0] = t1;
	d->times[1] = t2;
	d->times[2] = t3; // add behrens
	d->times[3] = t4; // add behrens
	free(gen);
	free(table);
	free(empty);
	free(joined);
	free(inner_keys);
#ifndef _BUILD_ONLY // add behrens
	free(outer_keys);
#endif // add behrens
	free(inner_rids);
#ifndef _BUILD_ONLY // add behrens
	free(outer_rids);
#endif // add behrens
	free(joined_keys);
	free(joined_rids);
	free(joined_tabs);
	pthread_exit(NULL);
}

int main(int argc, char **argv)
{
	// get arguments
	int t, threads = argc > 1 ? atoi(argv[1]) : hardware_threads();
	double gigs = argc > 2 ? atof(argv[2]) : 10;
	size_t outer_tuples = argc > 3 ? atoll(argv[3]) : 2048;
	size_t inner_tuples = argc > 4 ? atoll(argv[4]) : 256;
#ifdef _UNIQUE
	double selectivity = argc > 5 ? atof(argv[5]) : 0.99;
	double load_factor = argc > 6 ? atof(argv[6]) : 0.5;
	size_t outer_distinct = argc > 7 ? atoll(argv[7]) : min(outer_tuples, inner_tuples);
	size_t inner_distinct = inner_tuples;
#else
	double selectivity = argc > 5 ? atof(argv[5]) : 0.9;
	double load_factor = argc > 6 ? atof(argv[6]) : 0.5;
	size_t outer_distinct = argc > 7 ? atoll(argv[7]) : min(outer_tuples, inner_tuples) * 0.9;
	size_t inner_distinct = argc > 8 ? atoll(argv[8]) : min(outer_tuples, inner_tuples) * 0.9;
#endif
	// test arguments
	assert(inner_distinct <= inner_tuples);
	assert(outer_distinct <= outer_tuples);
	assert(selectivity >= 0.0 && selectivity <= 1.0);
	assert(load_factor >= 0.1 && load_factor <= 0.9);
	assert(threads > 0 && threads <= hardware_threads());
	// approximate result size
	double outer_repeats = outer_tuples * 1.0 / outer_distinct;
	double inner_repeats = inner_tuples * 1.0 / inner_distinct;
	size_t join_distinct = min(outer_distinct, inner_distinct) * selectivity;
	double join_tuples = outer_repeats * inner_repeats * join_distinct;
	// compute space
	size_t space_per_repeat = outer_tuples * 8 + inner_tuples * 8 + join_tuples * 12;
	size_t repeats = gigs * 1024 * 1024 * 1024 / space_per_repeat;
	repeats -= repeats % threads;
	repeats = threads; // add behrens
	// print info
	fprintf(stderr, "Space: %.2f GB\n", gigs);
	fprintf(stderr, "Threads: %d\n", threads);
	fprintf(stderr, "Inner: %5ld tuples / %5ld distinct / %.1f repeats)\n", inner_tuples, inner_distinct, inner_repeats);
	fprintf(stderr, "Outer: %5ld tuples / %5ld distinct / %.1f repeats)\n", outer_tuples, outer_distinct, outer_repeats);
	fprintf(stderr, "Distinct selectivity: %.1f%%\n", selectivity * 100);
	fprintf(stderr, "Join repeats: %ld (%ld per thread)\n", repeats, repeats / threads);
	fprintf(stderr, "Joined tuples (expected): %.1f\n", join_tuples);
#ifdef _UNIQUE
	fprintf(stderr, "Key join (unique inner tuples)!\n");
#endif
#ifdef _BUILD_ONLY
	fprintf(stderr, "Build only!\n");
#endif
	// run threads
	int b, barriers = 4;
	pthread_barrier_t barrier[barriers];
	for (b = 0 ; b != barriers ; ++b)
		pthread_barrier_init(&barrier[b], NULL, threads);
	srand(time(NULL));
	info_t info[threads];
	for (t = 0 ; t != threads ; ++t) {
		info[t].cpu = t;
		info[t].seed = rand();
		info[t].repeats = repeats / threads;
		info[t].outer_tuples = outer_tuples;
		info[t].inner_tuples = inner_tuples;
		info[t].outer_distinct = outer_distinct;
		info[t].inner_distinct = inner_distinct;
		info[t].selectivity = selectivity;
		info[t].load_factor = load_factor;
		info[t].selectivity = selectivity;
		info[t].barrier = barrier;
		pthread_create(&info[t].id, NULL, run, (void*) &info[t]);
	}
	// assemble results
	uint64_t sc_begin_min, sc_end_max; // add behrens
	uint64_t vc_begin_min, vc_end_max; // add behrens
	uint64_t scalar_time = 0;
	uint64_t vector_time = 0;
	size_t joined_total = 0;
	for (t = 0 ; t != threads ; ++t) {
		pthread_join(info[t].id, NULL);
		// scalar_time += info[t].times[0]; // delete behrens
		// vector_time += info[t].times[1]; // delete behrens
		// begin add behrens
		if (t == 0) {
			sc_begin_min = info[t].times[0];
			sc_end_max = info[t].times[1];
			vc_begin_min = info[t].times[2];
			vc_end_max = info[t].times[3];
		} else {
			if (info[t].times[0] < sc_begin_min) {
				sc_begin_min = info[t].times[0];
			}
			if (info[t].times[1] > sc_end_max) {
				sc_end_max = info[t].times[1];
			}
			if (info[t].times[2] < vc_begin_min) {
				vc_begin_min = info[t].times[2];
			}
			if (info[t].times[3] > vc_end_max) {
				vc_end_max = info[t].times[3];
			}
		}
		// end add behrens
		joined_total += info[t].joined_total;
	}
	scalar_time = sc_end_max - sc_begin_min; // add behrens
	vector_time = vc_end_max - vc_begin_min; // add behrens
	for (b = 0 ; b != barriers ; ++b)
		pthread_barrier_destroy(&barrier[b]);
	fprintf(stderr, "Joined tuples (measured): %.1f\n", joined_total * 1.0 / repeats);
	fprintf(stderr, "Scalar: %5.3f sec\n", (scalar_time / threads) / 1000000000.0);
	fprintf(stderr, "Vector: %5.3f sec\n", (vector_time / threads) / 1000000000.0);
	fprintf(stderr, "Speedup: %.1fX\n", scalar_time * 1.0 / vector_time);
	/* begin delete behrens
	 printf("LP, %ld, %ld, %.3f, %.3f\n", outer_tuples, inner_tuples,
	 repeats * (outer_tuples + inner_tuples) * 1.0 / (scalar_time / threads),
	 repeats * (outer_tuples + inner_tuples) * 1.0 / (vector_time / threads));
	 end delete behrens */
	// begin add behrens
#ifdef __MIC
	printf( "%ld; %.6f; %ld; %.6f; %ld\n", (inner_tuples * 2),
		   (threads * KERNEL_LOOPS * inner_tuples * 1.0) / scalar_time, scalar_time,
		   (threads * KERNEL_LOOPS * inner_tuples * 1.0) / vector_time, vector_time);
#else
	printf( "%ld; %.6f; %ld\n", (inner_tuples * 2),
		   (threads * KERNEL_LOOPS * inner_tuples * 1.0) / scalar_time, scalar_time);
#endif
	// end add behrens
	return EXIT_SUCCESS;
}
