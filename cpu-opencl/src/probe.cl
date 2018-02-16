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

struct Bucket {
  uint key;
  uint val;
};
typedef struct Bucket Bucket;
int log_2(size_t n);

kernel __attribute__((vec_type_hint(uint8))) void
probe(global uint *outer, const ulong outer_count, global Bucket *hash_table,
      const uint bucket_count, global uint *output, const uint empty) {

  // partition of work
  const uint gid = get_global_id(0);
  const uint gs = get_global_size(0);
  const ulong bound = (outer_count / gs) * (gid + 1);
  global uint *out_tab = &output[(outer_count / gs) * gid];

  // shift constant
  const uchar shift = 32 - log_2(bucket_count);

  // output / input counter
  ulong counter = 0;
  ulong i = ((outer_count / gs) * gid);

  for (; i != bound; ++i) {

    // read key
    uint key = outer[i];

    // determine hash value and read from hash table
    ulong h = ((uint)(key * MULTIPLY_CONSTANT)) >> shift;
    Bucket read_bucket = hash_table[h];

    // check key of read bucket
    if (key == read_bucket.key) {
      out_tab[counter++] = read_bucket.val;
    } else {
      // do linear probing
      while (empty != read_bucket.key) {
        h = (h + 1) & (bucket_count - 1);
        read_bucket = hash_table[h];
        if (read_bucket.key == key) {
          out_tab[counter++] = read_bucket.val;
          break;
        }
      }
    }
  }
}

// function from Polychroniou
int log_2(size_t n) {
  size_t b = 1;
  int p = 0;
  while (b < n) {
    b += b;
    p++;
  }
  // assert(b == n);
  return p;
}
