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

struct Bucket {
  uint key;
  uint val;
};
typedef struct Bucket Bucket;

kernel __attribute__((vec_type_hint(uint8))) void
gather(global Bucket *tab, const uint count, const ulong bound,
       global Bucket *output) {

  uint div = count / 32;
  uint min = count - 1;
  uint8 change = (uint8)(div, div, div, div, div, div, div, div);
  uint8 ccount = (uint8)(min, min, min, min, min, min, min, min);

  uint8 mask = (uint8)(1, 2, 3, 4, 5, 6, 7, 8) *
               (uint8)(3, 3, 3, 3, 3, 3, 3, 3) * change;

  ulong pos = 0;
  uint8 sum = 0;
  uint8 vec_key = (uint8)(0, 0, 0, 0, 0, 0, 0, 0);
  uint8 vec_val = (uint8)(0, 0, 0, 0, 0, 0, 0, 0);

  while (pos < bound) {

    // gather
    vec_key.s0 = tab[mask.s0].key;
    vec_key.s1 = tab[mask.s1].key;
    vec_key.s2 = tab[mask.s2].key;
    vec_key.s3 = tab[mask.s3].key;
    vec_key.s4 = tab[mask.s4].key;
    vec_key.s5 = tab[mask.s5].key;
    vec_key.s6 = tab[mask.s6].key;
    vec_key.s7 = tab[mask.s7].key;

    vec_val.s0 = tab[mask.s0].val;
    vec_val.s1 = tab[mask.s1].val;
    vec_val.s2 = tab[mask.s2].val;
    vec_val.s3 = tab[mask.s3].val;
    vec_val.s4 = tab[mask.s4].val;
    vec_val.s5 = tab[mask.s5].val;
    vec_val.s6 = tab[mask.s6].val;
    vec_val.s7 = tab[mask.s7].val;

    sum += vec_val + vec_key;

    // change mask
    mask += change;
    mask = mask & ccount;

    pos += 8;
  }

  output[0].key = sum.s0;
  output[1].key = sum.s1;
  output[2].key = sum.s2;
  output[3].key = sum.s3;
  output[4].key = sum.s4;
  output[5].key = sum.s5;
  output[6].key = sum.s6;
  output[7].key = sum.s7;
}

__kernel __attribute__((vec_type_hint(uint8))) kernel void
scatter(global Bucket *output, const uint count, const ulong bound) {

  uint div = count / 32;
  uint min = count - 1;
  uint8 change = (uint8)(div, div, div, div, div, div, div, div);
  uint8 ccount = (uint8)(min, min, min, min, min, min, min, min);

  uint8 mask = (uint8)(1, 2, 3, 4, 5, 6, 7, 8) *
               (uint8)(3, 3, 3, 3, 3, 3, 3, 3) * change;

  ulong pos = 0;
  const uint8 vec_key = (uint8)(0, 1, 2, 3, 4, 5, 6, 7);
  const uint8 vec_val = (uint8)(0, 1, 2, 3, 4, 5, 6, 7);

  while (pos < bound) {

    // scatter
    output[mask.s0].key = vec_key.s0;
    output[mask.s1].key = vec_key.s1;
    output[mask.s2].key = vec_key.s2;
    output[mask.s3].key = vec_key.s3;
    output[mask.s4].key = vec_key.s4;
    output[mask.s5].key = vec_key.s5;
    output[mask.s6].key = vec_key.s6;
    output[mask.s7].key = vec_key.s7;

    output[mask.s0].val = vec_val.s0;
    output[mask.s1].val = vec_val.s1;
    output[mask.s2].val = vec_val.s2;
    output[mask.s3].val = vec_val.s3;
    output[mask.s4].val = vec_val.s4;
    output[mask.s5].val = vec_val.s5;
    output[mask.s6].val = vec_val.s6;
    output[mask.s7].val = vec_val.s7;

    // change mask
    mask += change;
    mask = mask & ccount;

    pos += 8;
  }
}
