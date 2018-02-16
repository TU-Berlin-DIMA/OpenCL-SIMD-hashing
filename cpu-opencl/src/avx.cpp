/* Copyright (c) 2018
 * DIMA Research Group, TU Berlin
 * All rights reserved.
 *
 * Author: Tobias Behrens (tobias.behrens@dfki.de)
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

#include "avx.hpp"

namespace avx {

#ifdef __AVX2__

/**
 * Selective Load with Intrinsics
 */
__m256i load(const uint32_t *input, const uint64_t in_count) {

  __m256i vec = _mm256_set1_epi32(0);
  uint64_t bound = in_count - 6;
  uint64_t pos_rd = 0;
  uint8_t mask = 255;

  while (pos_rd < bound) {

    // load mask from lookup-table and permute vector
    __m128i perm_in_comp = _mm_loadl_epi64((__m128i *)&perm[mask]);
    __m256i perm_in = _mm256_cvtepi8_epi32(perm_in_comp);
    vec = _mm256_permutevar8x32_epi32(vec, perm_in);

    // mask-load new items from input
    uint8_t item_count = __builtin_popcount(mask);
    __m256i mask_vec = fence[item_count];
    __m256i new_vec = _mm256_maskload_epi32((int *)&input[pos_rd], mask_vec);

    // combine vector with new items
    vec = _mm256_andnot_si256(mask_vec, vec);
    vec = _mm256_or_si256(vec, new_vec);

    // increment reading position and change mask
    pos_rd += item_count;
    mask++;
  }
  // return -> function content will not be optimized away...
  return vec;
}

/**
 * Selective Store with Intrinsics
 */
void store(const uint32_t *output, const uint64_t out_count) {

  uint8_t mask = 255;
  uint64_t pos_wr = 0;
  uint64_t bound = out_count - 6;
  __m256i vec = _mm256_set_epi32(0, 1, 2, 3, 4, 5, 6, 7);

  while (pos_wr < bound) {

    // load mask from lookup-table and permute vector
    __m128i perm_out_comp = _mm_loadl_epi64((__m128i *)&perm[mask]);
    __m256i perm_out = _mm256_cvtepi8_epi32(perm_out_comp);
    vec = _mm256_permutevar8x32_epi32(vec, perm_out);

    // mask-store items from vector
    uint8_t item_count = __builtin_popcount(mask);
    _mm256_maskstore_epi32((int32_t *)&output[pos_wr], fence[item_count], vec);

    // increment writing position and change mask
    pos_wr += item_count;
    mask++;
  }
}

/**
 * Gather with Intrinsics
 */
__m256i gather(const uint64_t *tab, const uint32_t count,
               const uint64_t bound) {

  const __m256i change = _mm256_set1_epi32(count / 32);
  const __m256i ccount = _mm256_set1_epi32(count - 1);

  __m256i mask = _mm256_mullo_epi32(change, _mm256_set1_epi32(3));
  mask = _mm256_mullo_epi32(mask, _mm256_set_epi32(1, 2, 3, 4, 5, 6, 7, 8));

  uint64_t pos = 0;
  __m256i sum = _mm256_set1_epi32(0);
  __m256i vec_key = _mm256_set1_epi32(0);
  __m256i vec_val = _mm256_set1_epi32(0);

  while (pos < bound) {

    // gather buckets from hash table
    __m256i vec_lo = _mm256_i32gather_epi64((long long *)tab,
                                            _mm256_castsi256_si128(mask), 8);
    __m256i mask_shuf = _mm256_permute4x64_epi64(mask, _MM_SHUFFLE(1, 0, 3, 2));
    __m256i vec_hi = _mm256_i32gather_epi64(
        (long long *)tab, _mm256_castsi256_si128(mask_shuf), 8);

    // pack buckets in a suitable format
    vec_key = _mm256_packlo_epi32(vec_lo, vec_hi);
    vec_val = _mm256_packhi_epi32(vec_lo, vec_hi);

    // change mask
    mask = _mm256_add_epi32(mask, change);
    mask = _mm256_and_si256(mask, ccount);

    sum = _mm256_add_epi32(sum, vec_key); // otherwise function content
    sum = _mm256_add_epi32(sum, vec_val); // is optmized away...

    pos += 8;
  }
  // return -> function content will not be optimized away...
  return sum;
}

/**
 * Scatter with Intrinsics
 */
void scatter(const uint64_t *out, const uint32_t count, const uint64_t bound) {

  const __m256i change = _mm256_set1_epi32(count / 32);
  const __m256i ccount = _mm256_set1_epi32(count - 1);

  __m256i vec_key = _mm256_set_epi32(0, 1, 2, 3, 4, 5, 6, 7);
  __m256i vec_val = _mm256_set_epi32(0, 1, 2, 3, 4, 5, 6, 7);
  __m256i mask = _mm256_mullo_epi32(change, _mm256_set1_epi32(3));
  mask = _mm256_mullo_epi32(mask, _mm256_set_epi32(1, 2, 3, 4, 5, 6, 7, 8));

  uint64_t pos = 0;
  while (pos < bound) {

    // unpack keys and payload to bucket format
    __m256i vec_lo = _mm256_unpacklo_epi32(vec_key, vec_val);
    __m256i vec_hi = _mm256_unpackhi_epi32(vec_key, vec_val);

    // scatter items to hash table
    _mm256_i32scatter_epi64_emu(out, vec_lo, mask);
    __m256i mask_shuf = _mm256_permute4x64_epi64(mask, _MM_SHUFFLE(2, 3, 0, 1));
    _mm256_i32scatter_epi64_emu(out, vec_hi, mask_shuf);

    // change mask
    mask = _mm256_add_epi32(mask, change);
    mask = _mm256_and_si256(mask, ccount);

    pos += 8;
  }
}

inline void _mm256_i32scatter_epi64_emu(const uint64_t *table, __m256i vec,
                                        __m256i index) {

  __m256i index_R = _mm256_permute4x64_epi64(index, _MM_SHUFFLE(1, 0, 3, 2));
  __m256i vec_R = _mm256_permute4x64_epi64(vec, _MM_SHUFFLE(1, 0, 3, 2));

  // store bucket 0 and 2
  __m128i i01 = _mm_cvtepi32_epi64(_mm256_castsi256_si128(index));
  __m128i i23 = _mm_cvtepi32_epi64(_mm256_castsi256_si128(index_R));
  __m128i v01 = _mm256_castsi256_si128(vec);
  __m128i v23 = _mm256_castsi256_si128(vec_R);
  size_t i0 = _mm_cvtsi128_si64(i01);
  size_t i2 = _mm_cvtsi128_si64(i23);
  _mm_storel_epi64((__m128i *)&table[i0], v01);
  _mm_storel_epi64((__m128i *)&table[i2], v23);

  // store bucket 1 and 3
  i01 = _mm_srli_si128(i01, 8);
  i23 = _mm_srli_si128(i23, 8);
  v01 = _mm_srli_si128(v01, 8);
  v23 = _mm_srli_si128(v23, 8);
  size_t i1 = _mm_cvtsi128_si64(i01);
  size_t i3 = _mm_cvtsi128_si64(i23);
  _mm_storel_epi64((__m128i *)&table[i1], v01);
  _mm_storel_epi64((__m128i *)&table[i3], v23);
}

/**
 * functions by Orestis Polychroniou
 */
inline __m256i _mm256_packlo_epi32(__m256i x, __m256i y) {
  __m256 a = _mm256_castsi256_ps(x);
  __m256 b = _mm256_castsi256_ps(y);
  __m256 c = _mm256_shuffle_ps(a, b, _MM_SHUFFLE(2, 0, 2, 0));
  __m256i z = _mm256_castps_si256(c);
  return _mm256_permute4x64_epi64(z, _MM_SHUFFLE(3, 1, 2, 0));
}

inline __m256i _mm256_packhi_epi32(__m256i x, __m256i y) {
  __m256 a = _mm256_castsi256_ps(x);
  __m256 b = _mm256_castsi256_ps(y);
  __m256 c = _mm256_shuffle_ps(a, b, _MM_SHUFFLE(3, 1, 3, 1));
  __m256i z = _mm256_castps_si256(c);
  return _mm256_permute4x64_epi64(z, _MM_SHUFFLE(3, 1, 2, 0));
}

#endif // __AVX2__

} // namespace avx
