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

// From Knuths multplicative method
// (https://stackoverflow.com/a/665545)
#define MULTIPLY_CONSTANT 2654435761

#ifndef KERNEL_LOOPS
#define KERNEL_LOOPS 1
#endif

struct Bucket {
  uint key;
  uint val;
};
typedef struct Bucket Bucket;

kernel __attribute__((vec_type_hint(uint8))) void
build(global uint *keys, global uint *vals, const ulong key_count,
      global Bucket *hash_table, const uint buckets, const uint empty) {

  ulong i;
  ulong fudge_buckets = buckets * 1.2;
  Bucket empty_bucket = {empty, empty};

  // set correct pointers for this kernel
  global uint *key = &keys[get_global_id(0) * key_count];
  global uint *val = &vals[get_global_id(0) * key_count];
  global Bucket *table = &hash_table[get_global_id(0) * fudge_buckets];

  for (int loops = 0; loops < KERNEL_LOOPS; ++loops) {
    // clear hash table
    for (i = 0; i != fudge_buckets; ++i) {
      table[i] = empty_bucket;
    }

    for (i = 0; i != key_count; ++i) {

      // read key and payload
      Bucket write_bucket = {key[i], val[i]};

      // determine hash value
      ulong h = (uint)(write_bucket.key * MULTIPLY_CONSTANT);
      h = (h * buckets) >> 32;

      // linear probing
      while (empty != table[h].key) {
        h++;
      }

      // write bucket to hash table
      table[h] = write_bucket;
    }
  }
}
