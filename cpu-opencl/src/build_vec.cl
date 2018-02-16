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

// From Knuths mcplicative method
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

constant uchar8 perm_rd[256] = { // 8*256 = 2048 byte
    (uchar8)(0, 1, 2, 3, 4, 5, 6, 7), (uchar8)(1, 2, 3, 4, 5, 6, 7, 0),
    (uchar8)(1, 2, 3, 4, 5, 6, 0, 7), (uchar8)(2, 3, 4, 5, 6, 7, 0, 1),
    (uchar8)(1, 2, 3, 4, 5, 0, 6, 7), (uchar8)(2, 3, 4, 5, 6, 0, 7, 1),
    (uchar8)(2, 3, 4, 5, 6, 0, 1, 7), (uchar8)(3, 4, 5, 6, 7, 0, 1, 2),
    (uchar8)(1, 2, 3, 4, 0, 5, 6, 7), (uchar8)(2, 3, 4, 5, 0, 6, 7, 1),
    (uchar8)(2, 3, 4, 5, 0, 6, 1, 7), (uchar8)(3, 4, 5, 6, 0, 7, 1, 2),
    (uchar8)(2, 3, 4, 5, 0, 1, 6, 7), (uchar8)(3, 4, 5, 6, 0, 1, 7, 2),
    (uchar8)(3, 4, 5, 6, 0, 1, 2, 7), (uchar8)(4, 5, 6, 7, 0, 1, 2, 3),
    (uchar8)(1, 2, 3, 0, 4, 5, 6, 7), (uchar8)(2, 3, 4, 0, 5, 6, 7, 1),
    (uchar8)(2, 3, 4, 0, 5, 6, 1, 7), (uchar8)(3, 4, 5, 0, 6, 7, 1, 2),
    (uchar8)(2, 3, 4, 0, 5, 1, 6, 7), (uchar8)(3, 4, 5, 0, 6, 1, 7, 2),
    (uchar8)(3, 4, 5, 0, 6, 1, 2, 7), (uchar8)(4, 5, 6, 0, 7, 1, 2, 3),
    (uchar8)(2, 3, 4, 0, 1, 5, 6, 7), (uchar8)(3, 4, 5, 0, 1, 6, 7, 2),
    (uchar8)(3, 4, 5, 0, 1, 6, 2, 7), (uchar8)(4, 5, 6, 0, 1, 7, 2, 3),
    (uchar8)(3, 4, 5, 0, 1, 2, 6, 7), (uchar8)(4, 5, 6, 0, 1, 2, 7, 3),
    (uchar8)(4, 5, 6, 0, 1, 2, 3, 7), (uchar8)(5, 6, 7, 0, 1, 2, 3, 4),
    (uchar8)(1, 2, 0, 3, 4, 5, 6, 7), (uchar8)(2, 3, 0, 4, 5, 6, 7, 1),
    (uchar8)(2, 3, 0, 4, 5, 6, 1, 7), (uchar8)(3, 4, 0, 5, 6, 7, 1, 2),
    (uchar8)(2, 3, 0, 4, 5, 1, 6, 7), (uchar8)(3, 4, 0, 5, 6, 1, 7, 2),
    (uchar8)(3, 4, 0, 5, 6, 1, 2, 7), (uchar8)(4, 5, 0, 6, 7, 1, 2, 3),
    (uchar8)(2, 3, 0, 4, 1, 5, 6, 7), (uchar8)(3, 4, 0, 5, 1, 6, 7, 2),
    (uchar8)(3, 4, 0, 5, 1, 6, 2, 7), (uchar8)(4, 5, 0, 6, 1, 7, 2, 3),
    (uchar8)(3, 4, 0, 5, 1, 2, 6, 7), (uchar8)(4, 5, 0, 6, 1, 2, 7, 3),
    (uchar8)(4, 5, 0, 6, 1, 2, 3, 7), (uchar8)(5, 6, 0, 7, 1, 2, 3, 4),
    (uchar8)(2, 3, 0, 1, 4, 5, 6, 7), (uchar8)(3, 4, 0, 1, 5, 6, 7, 2),
    (uchar8)(3, 4, 0, 1, 5, 6, 2, 7), (uchar8)(4, 5, 0, 1, 6, 7, 2, 3),
    (uchar8)(3, 4, 0, 1, 5, 2, 6, 7), (uchar8)(4, 5, 0, 1, 6, 2, 7, 3),
    (uchar8)(4, 5, 0, 1, 6, 2, 3, 7), (uchar8)(5, 6, 0, 1, 7, 2, 3, 4),
    (uchar8)(3, 4, 0, 1, 2, 5, 6, 7), (uchar8)(4, 5, 0, 1, 2, 6, 7, 3),
    (uchar8)(4, 5, 0, 1, 2, 6, 3, 7), (uchar8)(5, 6, 0, 1, 2, 7, 3, 4),
    (uchar8)(4, 5, 0, 1, 2, 3, 6, 7), (uchar8)(5, 6, 0, 1, 2, 3, 7, 4),
    (uchar8)(5, 6, 0, 1, 2, 3, 4, 7), (uchar8)(6, 7, 0, 1, 2, 3, 4, 5),
    (uchar8)(1, 0, 2, 3, 4, 5, 6, 7), (uchar8)(2, 0, 3, 4, 5, 6, 7, 1),
    (uchar8)(2, 0, 3, 4, 5, 6, 1, 7), (uchar8)(3, 0, 4, 5, 6, 7, 1, 2),
    (uchar8)(2, 0, 3, 4, 5, 1, 6, 7), (uchar8)(3, 0, 4, 5, 6, 1, 7, 2),
    (uchar8)(3, 0, 4, 5, 6, 1, 2, 7), (uchar8)(4, 0, 5, 6, 7, 1, 2, 3),
    (uchar8)(2, 0, 3, 4, 1, 5, 6, 7), (uchar8)(3, 0, 4, 5, 1, 6, 7, 2),
    (uchar8)(3, 0, 4, 5, 1, 6, 2, 7), (uchar8)(4, 0, 5, 6, 1, 7, 2, 3),
    (uchar8)(3, 0, 4, 5, 1, 2, 6, 7), (uchar8)(4, 0, 5, 6, 1, 2, 7, 3),
    (uchar8)(4, 0, 5, 6, 1, 2, 3, 7), (uchar8)(5, 0, 6, 7, 1, 2, 3, 4),
    (uchar8)(2, 0, 3, 1, 4, 5, 6, 7), (uchar8)(3, 0, 4, 1, 5, 6, 7, 2),
    (uchar8)(3, 0, 4, 1, 5, 6, 2, 7), (uchar8)(4, 0, 5, 1, 6, 7, 2, 3),
    (uchar8)(3, 0, 4, 1, 5, 2, 6, 7), (uchar8)(4, 0, 5, 1, 6, 2, 7, 3),
    (uchar8)(4, 0, 5, 1, 6, 2, 3, 7), (uchar8)(5, 0, 6, 1, 7, 2, 3, 4),
    (uchar8)(3, 0, 4, 1, 2, 5, 6, 7), (uchar8)(4, 0, 5, 1, 2, 6, 7, 3),
    (uchar8)(4, 0, 5, 1, 2, 6, 3, 7), (uchar8)(5, 0, 6, 1, 2, 7, 3, 4),
    (uchar8)(4, 0, 5, 1, 2, 3, 6, 7), (uchar8)(5, 0, 6, 1, 2, 3, 7, 4),
    (uchar8)(5, 0, 6, 1, 2, 3, 4, 7), (uchar8)(6, 0, 7, 1, 2, 3, 4, 5),
    (uchar8)(2, 0, 1, 3, 4, 5, 6, 7), (uchar8)(3, 0, 1, 4, 5, 6, 7, 2),
    (uchar8)(3, 0, 1, 4, 5, 6, 2, 7), (uchar8)(4, 0, 1, 5, 6, 7, 2, 3),
    (uchar8)(3, 0, 1, 4, 5, 2, 6, 7), (uchar8)(4, 0, 1, 5, 6, 2, 7, 3),
    (uchar8)(4, 0, 1, 5, 6, 2, 3, 7), (uchar8)(5, 0, 1, 6, 7, 2, 3, 4),
    (uchar8)(3, 0, 1, 4, 2, 5, 6, 7), (uchar8)(4, 0, 1, 5, 2, 6, 7, 3),
    (uchar8)(4, 0, 1, 5, 2, 6, 3, 7), (uchar8)(5, 0, 1, 6, 2, 7, 3, 4),
    (uchar8)(4, 0, 1, 5, 2, 3, 6, 7), (uchar8)(5, 0, 1, 6, 2, 3, 7, 4),
    (uchar8)(5, 0, 1, 6, 2, 3, 4, 7), (uchar8)(6, 0, 1, 7, 2, 3, 4, 5),
    (uchar8)(3, 0, 1, 2, 4, 5, 6, 7), (uchar8)(4, 0, 1, 2, 5, 6, 7, 3),
    (uchar8)(4, 0, 1, 2, 5, 6, 3, 7), (uchar8)(5, 0, 1, 2, 6, 7, 3, 4),
    (uchar8)(4, 0, 1, 2, 5, 3, 6, 7), (uchar8)(5, 0, 1, 2, 6, 3, 7, 4),
    (uchar8)(5, 0, 1, 2, 6, 3, 4, 7), (uchar8)(6, 0, 1, 2, 7, 3, 4, 5),
    (uchar8)(4, 0, 1, 2, 3, 5, 6, 7), (uchar8)(5, 0, 1, 2, 3, 6, 7, 4),
    (uchar8)(5, 0, 1, 2, 3, 6, 4, 7), (uchar8)(6, 0, 1, 2, 3, 7, 4, 5),
    (uchar8)(5, 0, 1, 2, 3, 4, 6, 7), (uchar8)(6, 0, 1, 2, 3, 4, 7, 5),
    (uchar8)(6, 0, 1, 2, 3, 4, 5, 7), (uchar8)(7, 0, 1, 2, 3, 4, 5, 6),
    (uchar8)(0, 1, 2, 3, 4, 5, 6, 7), (uchar8)(0, 2, 3, 4, 5, 6, 7, 1),
    (uchar8)(0, 2, 3, 4, 5, 6, 1, 7), (uchar8)(0, 3, 4, 5, 6, 7, 1, 2),
    (uchar8)(0, 2, 3, 4, 5, 1, 6, 7), (uchar8)(0, 3, 4, 5, 6, 1, 7, 2),
    (uchar8)(0, 3, 4, 5, 6, 1, 2, 7), (uchar8)(0, 4, 5, 6, 7, 1, 2, 3),
    (uchar8)(0, 2, 3, 4, 1, 5, 6, 7), (uchar8)(0, 3, 4, 5, 1, 6, 7, 2),
    (uchar8)(0, 3, 4, 5, 1, 6, 2, 7), (uchar8)(0, 4, 5, 6, 1, 7, 2, 3),
    (uchar8)(0, 3, 4, 5, 1, 2, 6, 7), (uchar8)(0, 4, 5, 6, 1, 2, 7, 3),
    (uchar8)(0, 4, 5, 6, 1, 2, 3, 7), (uchar8)(0, 5, 6, 7, 1, 2, 3, 4),
    (uchar8)(0, 2, 3, 1, 4, 5, 6, 7), (uchar8)(0, 3, 4, 1, 5, 6, 7, 2),
    (uchar8)(0, 3, 4, 1, 5, 6, 2, 7), (uchar8)(0, 4, 5, 1, 6, 7, 2, 3),
    (uchar8)(0, 3, 4, 1, 5, 2, 6, 7), (uchar8)(0, 4, 5, 1, 6, 2, 7, 3),
    (uchar8)(0, 4, 5, 1, 6, 2, 3, 7), (uchar8)(0, 5, 6, 1, 7, 2, 3, 4),
    (uchar8)(0, 3, 4, 1, 2, 5, 6, 7), (uchar8)(0, 4, 5, 1, 2, 6, 7, 3),
    (uchar8)(0, 4, 5, 1, 2, 6, 3, 7), (uchar8)(0, 5, 6, 1, 2, 7, 3, 4),
    (uchar8)(0, 4, 5, 1, 2, 3, 6, 7), (uchar8)(0, 5, 6, 1, 2, 3, 7, 4),
    (uchar8)(0, 5, 6, 1, 2, 3, 4, 7), (uchar8)(0, 6, 7, 1, 2, 3, 4, 5),
    (uchar8)(0, 2, 1, 3, 4, 5, 6, 7), (uchar8)(0, 3, 1, 4, 5, 6, 7, 2),
    (uchar8)(0, 3, 1, 4, 5, 6, 2, 7), (uchar8)(0, 4, 1, 5, 6, 7, 2, 3),
    (uchar8)(0, 3, 1, 4, 5, 2, 6, 7), (uchar8)(0, 4, 1, 5, 6, 2, 7, 3),
    (uchar8)(0, 4, 1, 5, 6, 2, 3, 7), (uchar8)(0, 5, 1, 6, 7, 2, 3, 4),
    (uchar8)(0, 3, 1, 4, 2, 5, 6, 7), (uchar8)(0, 4, 1, 5, 2, 6, 7, 3),
    (uchar8)(0, 4, 1, 5, 2, 6, 3, 7), (uchar8)(0, 5, 1, 6, 2, 7, 3, 4),
    (uchar8)(0, 4, 1, 5, 2, 3, 6, 7), (uchar8)(0, 5, 1, 6, 2, 3, 7, 4),
    (uchar8)(0, 5, 1, 6, 2, 3, 4, 7), (uchar8)(0, 6, 1, 7, 2, 3, 4, 5),
    (uchar8)(0, 3, 1, 2, 4, 5, 6, 7), (uchar8)(0, 4, 1, 2, 5, 6, 7, 3),
    (uchar8)(0, 4, 1, 2, 5, 6, 3, 7), (uchar8)(0, 5, 1, 2, 6, 7, 3, 4),
    (uchar8)(0, 4, 1, 2, 5, 3, 6, 7), (uchar8)(0, 5, 1, 2, 6, 3, 7, 4),
    (uchar8)(0, 5, 1, 2, 6, 3, 4, 7), (uchar8)(0, 6, 1, 2, 7, 3, 4, 5),
    (uchar8)(0, 4, 1, 2, 3, 5, 6, 7), (uchar8)(0, 5, 1, 2, 3, 6, 7, 4),
    (uchar8)(0, 5, 1, 2, 3, 6, 4, 7), (uchar8)(0, 6, 1, 2, 3, 7, 4, 5),
    (uchar8)(0, 5, 1, 2, 3, 4, 6, 7), (uchar8)(0, 6, 1, 2, 3, 4, 7, 5),
    (uchar8)(0, 6, 1, 2, 3, 4, 5, 7), (uchar8)(0, 7, 1, 2, 3, 4, 5, 6),
    (uchar8)(0, 1, 2, 3, 4, 5, 6, 7), (uchar8)(0, 1, 3, 4, 5, 6, 7, 2),
    (uchar8)(0, 1, 3, 4, 5, 6, 2, 7), (uchar8)(0, 1, 4, 5, 6, 7, 2, 3),
    (uchar8)(0, 1, 3, 4, 5, 2, 6, 7), (uchar8)(0, 1, 4, 5, 6, 2, 7, 3),
    (uchar8)(0, 1, 4, 5, 6, 2, 3, 7), (uchar8)(0, 1, 5, 6, 7, 2, 3, 4),
    (uchar8)(0, 1, 3, 4, 2, 5, 6, 7), (uchar8)(0, 1, 4, 5, 2, 6, 7, 3),
    (uchar8)(0, 1, 4, 5, 2, 6, 3, 7), (uchar8)(0, 1, 5, 6, 2, 7, 3, 4),
    (uchar8)(0, 1, 4, 5, 2, 3, 6, 7), (uchar8)(0, 1, 5, 6, 2, 3, 7, 4),
    (uchar8)(0, 1, 5, 6, 2, 3, 4, 7), (uchar8)(0, 1, 6, 7, 2, 3, 4, 5),
    (uchar8)(0, 1, 3, 2, 4, 5, 6, 7), (uchar8)(0, 1, 4, 2, 5, 6, 7, 3),
    (uchar8)(0, 1, 4, 2, 5, 6, 3, 7), (uchar8)(0, 1, 5, 2, 6, 7, 3, 4),
    (uchar8)(0, 1, 4, 2, 5, 3, 6, 7), (uchar8)(0, 1, 5, 2, 6, 3, 7, 4),
    (uchar8)(0, 1, 5, 2, 6, 3, 4, 7), (uchar8)(0, 1, 6, 2, 7, 3, 4, 5),
    (uchar8)(0, 1, 4, 2, 3, 5, 6, 7), (uchar8)(0, 1, 5, 2, 3, 6, 7, 4),
    (uchar8)(0, 1, 5, 2, 3, 6, 4, 7), (uchar8)(0, 1, 6, 2, 3, 7, 4, 5),
    (uchar8)(0, 1, 5, 2, 3, 4, 6, 7), (uchar8)(0, 1, 6, 2, 3, 4, 7, 5),
    (uchar8)(0, 1, 6, 2, 3, 4, 5, 7), (uchar8)(0, 1, 7, 2, 3, 4, 5, 6),
    (uchar8)(0, 1, 2, 3, 4, 5, 6, 7), (uchar8)(0, 1, 2, 4, 5, 6, 7, 3),
    (uchar8)(0, 1, 2, 4, 5, 6, 3, 7), (uchar8)(0, 1, 2, 5, 6, 7, 3, 4),
    (uchar8)(0, 1, 2, 4, 5, 3, 6, 7), (uchar8)(0, 1, 2, 5, 6, 3, 7, 4),
    (uchar8)(0, 1, 2, 5, 6, 3, 4, 7), (uchar8)(0, 1, 2, 6, 7, 3, 4, 5),
    (uchar8)(0, 1, 2, 4, 3, 5, 6, 7), (uchar8)(0, 1, 2, 5, 3, 6, 7, 4),
    (uchar8)(0, 1, 2, 5, 3, 6, 4, 7), (uchar8)(0, 1, 2, 6, 3, 7, 4, 5),
    (uchar8)(0, 1, 2, 5, 3, 4, 6, 7), (uchar8)(0, 1, 2, 6, 3, 4, 7, 5),
    (uchar8)(0, 1, 2, 6, 3, 4, 5, 7), (uchar8)(0, 1, 2, 7, 3, 4, 5, 6),
    (uchar8)(0, 1, 2, 3, 4, 5, 6, 7), (uchar8)(0, 1, 2, 3, 5, 6, 7, 4),
    (uchar8)(0, 1, 2, 3, 5, 6, 4, 7), (uchar8)(0, 1, 2, 3, 6, 7, 4, 5),
    (uchar8)(0, 1, 2, 3, 5, 4, 6, 7), (uchar8)(0, 1, 2, 3, 6, 4, 7, 5),
    (uchar8)(0, 1, 2, 3, 6, 4, 5, 7), (uchar8)(0, 1, 2, 3, 7, 4, 5, 6),
    (uchar8)(0, 1, 2, 3, 4, 5, 6, 7), (uchar8)(0, 1, 2, 3, 4, 6, 7, 5),
    (uchar8)(0, 1, 2, 3, 4, 6, 5, 7), (uchar8)(0, 1, 2, 3, 4, 7, 5, 6),
    (uchar8)(0, 1, 2, 3, 4, 5, 6, 7), (uchar8)(0, 1, 2, 3, 4, 5, 7, 6),
    (uchar8)(0, 1, 2, 3, 4, 5, 6, 7), (uchar8)(0, 1, 2, 3, 4, 5, 6, 7)};

kernel __attribute__((vec_type_hint(uint8))) void
build_vec(global uint *keys, global uint *vals, const ulong key_count,
          global Bucket *hash_table, const uint buckets,
          const uint invalid_key) {

  ulong i = 0;
  ulong fudge_buckets = buckets * 1.2;
  Bucket empty_bucket = {invalid_key, invalid_key};

  // set correct pointers for this kernel
  global uint *key = &keys[get_global_id(0) * key_count];
  global uint *val = &vals[get_global_id(0) * key_count];
  global Bucket *table = &hash_table[get_global_id(0) * fudge_buckets];

  const uint8 mc =
      (uint8)(MULTIPLY_CONSTANT, MULTIPLY_CONSTANT, MULTIPLY_CONSTANT,
              MULTIPLY_CONSTANT, MULTIPLY_CONSTANT, MULTIPLY_CONSTANT,
              MULTIPLY_CONSTANT, MULTIPLY_CONSTANT);

  const uint8 empty =
      (uint8)(invalid_key, invalid_key, invalid_key, invalid_key, invalid_key,
              invalid_key, invalid_key, invalid_key);

  const ulong8 bu = (ulong8)(buckets, buckets, buckets, buckets, buckets,
                             buckets, buckets, buckets);

  const uint8 zeros = (uint8)(0, 0, 0, 0, 0, 0, 0, 0);
  const uint8 ones = (uint8)(1, 1, 1, 1, 1, 1, 1, 1);
  const uint8 check = (uint8)(0, 1, 2, 3, 4, 5, 6, 7);

  for (int loops = 0; loops < KERNEL_LOOPS; ++loops) {
    uint out_count = 8, mask_rd = 255;
    int8 out = (int8)(-1, -1, -1, -1, -1, -1, -1, -1);
    uint8 key_vec = zeros, val_vec = zeros, offset = zeros;

    // clear hash table
    for (i = 0; i != fudge_buckets; ++i) {
      table[i] = empty_bucket;
    }

    i = 0;
    while (i < key_count - 8) {

      // selective load
      uint8 perm_out = convert_uint8(perm_rd[mask_rd]);
      uint8 key_rd = vload8(0, &key[i]);
      uint8 val_rd = vload8(0, &val[i]);
      i += out_count;

      key_rd = shuffle(key_rd, perm_out);
      val_rd = shuffle(val_rd, perm_out);
      key_vec = select(key_vec, key_rd, out == -1);
      val_vec = select(val_vec, val_rd, out == -1);

      // hash keys
      ulong8 h = convert_ulong8(key_vec * mc);
      h = (h * bu) >> 32;
      h = h + convert_ulong8(offset);

      // gather already saved keys from tab
      uint8 tab_keys, tab_vals;
      tab_keys.s0 = table[h.s0].key;
      tab_keys.s1 = table[h.s1].key;
      tab_keys.s2 = table[h.s2].key;
      tab_keys.s3 = table[h.s3].key;
      tab_keys.s4 = table[h.s4].key;
      tab_keys.s5 = table[h.s5].key;
      tab_keys.s6 = table[h.s6].key;
      tab_keys.s7 = table[h.s7].key;
      tab_vals.s0 = table[h.s0].val;
      tab_vals.s1 = table[h.s1].val;
      tab_vals.s2 = table[h.s2].val;
      tab_vals.s3 = table[h.s3].val;
      tab_vals.s4 = table[h.s4].val;
      tab_vals.s5 = table[h.s5].val;
      tab_vals.s6 = table[h.s6].val;
      tab_vals.s7 = table[h.s7].val;

      // check if empty
      out = isequal(tab_keys, empty);

      // detect possible conflicts
      uint8 tab_conf;
      table[h.s0].key = check.s0;
      table[h.s1].key = check.s1;
      table[h.s2].key = check.s2;
      table[h.s3].key = check.s3;
      table[h.s4].key = check.s4;
      table[h.s5].key = check.s5;
      table[h.s6].key = check.s6;
      table[h.s7].key = check.s7;

      tab_conf.s0 = table[h.s0].key;
      tab_conf.s1 = table[h.s1].key;
      tab_conf.s2 = table[h.s2].key;
      tab_conf.s3 = table[h.s3].key;
      tab_conf.s4 = table[h.s4].key;
      tab_conf.s5 = table[h.s5].key;
      tab_conf.s6 = table[h.s6].key;
      tab_conf.s7 = table[h.s7].key;

      int8 no_conflict = isequal(tab_conf, check);
      out = select(convert_int8(zeros), out, no_conflict == -1);
      uint8 write_key = select(tab_keys, key_vec, out == -1);
      uint8 write_val = select(tab_vals, val_vec, out == -1);

      // scatter keys and values to tab
      table[h.s0].key = write_key.s0;
      table[h.s0].val = write_val.s0;
      table[h.s1].key = write_key.s1;
      table[h.s1].val = write_val.s1;
      table[h.s2].key = write_key.s2;
      table[h.s2].val = write_val.s2;
      table[h.s3].key = write_key.s3;
      table[h.s3].val = write_val.s3;
      table[h.s4].key = write_key.s4;
      table[h.s4].val = write_val.s4;
      table[h.s5].key = write_key.s5;
      table[h.s5].val = write_val.s5;
      table[h.s6].key = write_key.s6;
      table[h.s6].val = write_val.s6;
      table[h.s7].key = write_key.s7;
      table[h.s7].val = write_val.s7;

      // determine mask for next iteration
      mask_rd = -1 * out.s7 + -2 * out.s6 + -4 * out.s5 + -8 * out.s4 +
                -16 * out.s3 + -32 * out.s2 + -64 * out.s1 + -128 * out.s0;
      out_count = popcount(mask_rd);

      // adjust offset
      offset += ones;
      offset = select(offset, zeros, out == -1);
    }

    // finish unfinished keys from vector
    for (uchar old_key = out_count; old_key < 8; ++old_key) {

      // read key and payload
      uint key_scal, val_scal;
      if (out.s0 == 0) {
        out.s0 = -1;
        key_scal = key_vec.s0;
        val_scal = val_vec.s0;
      } else if (out.s1 == 0) {
        out.s1 = -1;
        key_scal = key_vec.s1;
        val_scal = val_vec.s1;
      } else if (out.s2 == 0) {
        out.s2 = -1;
        key_scal = key_vec.s2;
        val_scal = val_vec.s2;
      } else if (out.s3 == 0) {
        out.s3 = -1;
        key_scal = key_vec.s3;
        val_scal = val_vec.s3;
      } else if (out.s4 == 0) {
        out.s4 = -1;
        key_scal = key_vec.s4;
        val_scal = val_vec.s4;
      } else if (out.s5 == 0) {
        out.s5 = -1;
        key_scal = key_vec.s5;
        val_scal = val_vec.s5;
      } else if (out.s6 == 0) {
        out.s6 = -1;
        key_scal = key_vec.s6;
        val_scal = val_vec.s6;
      } else if (out.s7 == 0) {
        out.s7 = -1;
        key_scal = key_vec.s7;
        val_scal = val_vec.s7;
      }
      Bucket write_bucket = {key_scal, val_scal};

      // determine hash value
      ulong h = (uint)(write_bucket.key * MULTIPLY_CONSTANT);
      h = (h * buckets) >> 32;

      // linear probing
      while (invalid_key != table[h].key) {
        h++;
      }

      // write bucket to hash table
      table[h] = write_bucket;
    }

    // do the last 8 entries
    for (ulong old_key = i; old_key < key_count; ++old_key) {

      // read key and payload
      Bucket write_bucket = {key[old_key], val[old_key]};

      // determine hash value
      ulong h = (uint)(write_bucket.key * MULTIPLY_CONSTANT);
      h = (h * buckets) >> 32;

      // linear probing
      while (invalid_key != table[h].key) {
        h++;
      }

      // write bucket to hash table
      table[h] = write_bucket;
    }
  }
}
