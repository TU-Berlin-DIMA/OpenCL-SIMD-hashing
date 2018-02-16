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

struct Bucket {
  uint key;
  uint val;
};
typedef struct Bucket Bucket;
int log_2(size_t n);

constant uchar8 perm[256] = { // 8*256 = 2048 byte
    (uchar8)(0, 1, 2, 3, 4, 5, 6, 7), (uchar8)(7, 1, 2, 3, 4, 5, 6, 0),
    (uchar8)(6, 1, 2, 3, 4, 5, 0, 7), (uchar8)(6, 7, 2, 3, 4, 5, 0, 1),
    (uchar8)(5, 1, 2, 3, 4, 0, 6, 7), (uchar8)(5, 7, 2, 3, 4, 0, 6, 1),
    (uchar8)(5, 6, 2, 3, 4, 0, 1, 7), (uchar8)(5, 6, 7, 3, 4, 0, 1, 2),
    (uchar8)(4, 1, 2, 3, 0, 5, 6, 7), (uchar8)(4, 7, 2, 3, 0, 5, 6, 1),
    (uchar8)(4, 6, 2, 3, 0, 5, 1, 7), (uchar8)(4, 6, 7, 3, 0, 5, 1, 2),
    (uchar8)(4, 5, 2, 3, 0, 1, 6, 7), (uchar8)(4, 5, 7, 3, 0, 1, 6, 2),
    (uchar8)(4, 5, 6, 3, 0, 1, 2, 7), (uchar8)(4, 5, 6, 7, 0, 1, 2, 3),
    (uchar8)(3, 1, 2, 0, 4, 5, 6, 7), (uchar8)(3, 7, 2, 0, 4, 5, 6, 1),
    (uchar8)(3, 6, 2, 0, 4, 5, 1, 7), (uchar8)(3, 6, 7, 0, 4, 5, 1, 2),
    (uchar8)(3, 5, 2, 0, 4, 1, 6, 7), (uchar8)(3, 5, 7, 0, 4, 1, 6, 2),
    (uchar8)(3, 5, 6, 0, 4, 1, 2, 7), (uchar8)(3, 5, 6, 7, 4, 1, 2, 0),
    (uchar8)(3, 4, 2, 0, 1, 5, 6, 7), (uchar8)(3, 4, 7, 0, 1, 5, 6, 2),
    (uchar8)(3, 4, 6, 0, 1, 5, 2, 7), (uchar8)(3, 4, 6, 7, 1, 5, 2, 0),
    (uchar8)(3, 4, 5, 0, 1, 2, 6, 7), (uchar8)(3, 4, 5, 7, 1, 2, 6, 0),
    (uchar8)(3, 4, 5, 6, 1, 2, 0, 7), (uchar8)(3, 4, 5, 6, 7, 2, 0, 1),
    (uchar8)(2, 1, 0, 3, 4, 5, 6, 7), (uchar8)(2, 7, 0, 3, 4, 5, 6, 1),
    (uchar8)(2, 6, 0, 3, 4, 5, 1, 7), (uchar8)(2, 6, 7, 3, 4, 5, 1, 0),
    (uchar8)(2, 5, 0, 3, 4, 1, 6, 7), (uchar8)(2, 5, 7, 3, 4, 1, 6, 0),
    (uchar8)(2, 5, 6, 3, 4, 1, 0, 7), (uchar8)(2, 5, 6, 7, 4, 1, 0, 3),
    (uchar8)(2, 4, 0, 3, 1, 5, 6, 7), (uchar8)(2, 4, 7, 3, 1, 5, 6, 0),
    (uchar8)(2, 4, 6, 3, 1, 5, 0, 7), (uchar8)(2, 4, 6, 7, 1, 5, 0, 3),
    (uchar8)(2, 4, 5, 3, 1, 0, 6, 7), (uchar8)(2, 4, 5, 7, 1, 0, 6, 3),
    (uchar8)(2, 4, 5, 6, 1, 0, 3, 7), (uchar8)(2, 4, 5, 6, 7, 0, 3, 1),
    (uchar8)(2, 3, 0, 1, 4, 5, 6, 7), (uchar8)(2, 3, 7, 1, 4, 5, 6, 0),
    (uchar8)(2, 3, 6, 1, 4, 5, 0, 7), (uchar8)(2, 3, 6, 7, 4, 5, 0, 1),
    (uchar8)(2, 3, 5, 1, 4, 0, 6, 7), (uchar8)(2, 3, 5, 7, 4, 0, 6, 1),
    (uchar8)(2, 3, 5, 6, 4, 0, 1, 7), (uchar8)(2, 3, 5, 6, 7, 0, 1, 4),
    (uchar8)(2, 3, 4, 1, 0, 5, 6, 7), (uchar8)(2, 3, 4, 7, 0, 5, 6, 1),
    (uchar8)(2, 3, 4, 6, 0, 5, 1, 7), (uchar8)(2, 3, 4, 6, 7, 5, 1, 0),
    (uchar8)(2, 3, 4, 5, 0, 1, 6, 7), (uchar8)(2, 3, 4, 5, 7, 1, 6, 0),
    (uchar8)(2, 3, 4, 5, 6, 1, 0, 7), (uchar8)(2, 3, 4, 5, 6, 7, 0, 1),
    (uchar8)(1, 0, 2, 3, 4, 5, 6, 7), (uchar8)(1, 7, 2, 3, 4, 5, 6, 0),
    (uchar8)(1, 6, 2, 3, 4, 5, 0, 7), (uchar8)(1, 6, 7, 3, 4, 5, 0, 2),
    (uchar8)(1, 5, 2, 3, 4, 0, 6, 7), (uchar8)(1, 5, 7, 3, 4, 0, 6, 2),
    (uchar8)(1, 5, 6, 3, 4, 0, 2, 7), (uchar8)(1, 5, 6, 7, 4, 0, 2, 3),
    (uchar8)(1, 4, 2, 3, 0, 5, 6, 7), (uchar8)(1, 4, 7, 3, 0, 5, 6, 2),
    (uchar8)(1, 4, 6, 3, 0, 5, 2, 7), (uchar8)(1, 4, 6, 7, 0, 5, 2, 3),
    (uchar8)(1, 4, 5, 3, 0, 2, 6, 7), (uchar8)(1, 4, 5, 7, 0, 2, 6, 3),
    (uchar8)(1, 4, 5, 6, 0, 2, 3, 7), (uchar8)(1, 4, 5, 6, 7, 2, 3, 0),
    (uchar8)(1, 3, 2, 0, 4, 5, 6, 7), (uchar8)(1, 3, 7, 0, 4, 5, 6, 2),
    (uchar8)(1, 3, 6, 0, 4, 5, 2, 7), (uchar8)(1, 3, 6, 7, 4, 5, 2, 0),
    (uchar8)(1, 3, 5, 0, 4, 2, 6, 7), (uchar8)(1, 3, 5, 7, 4, 2, 6, 0),
    (uchar8)(1, 3, 5, 6, 4, 2, 0, 7), (uchar8)(1, 3, 5, 6, 7, 2, 0, 4),
    (uchar8)(1, 3, 4, 0, 2, 5, 6, 7), (uchar8)(1, 3, 4, 7, 2, 5, 6, 0),
    (uchar8)(1, 3, 4, 6, 2, 5, 0, 7), (uchar8)(1, 3, 4, 6, 7, 5, 0, 2),
    (uchar8)(1, 3, 4, 5, 2, 0, 6, 7), (uchar8)(1, 3, 4, 5, 7, 0, 6, 2),
    (uchar8)(1, 3, 4, 5, 6, 0, 2, 7), (uchar8)(1, 3, 4, 5, 6, 7, 2, 0),
    (uchar8)(1, 2, 0, 3, 4, 5, 6, 7), (uchar8)(1, 2, 7, 3, 4, 5, 6, 0),
    (uchar8)(1, 2, 6, 3, 4, 5, 0, 7), (uchar8)(1, 2, 6, 7, 4, 5, 0, 3),
    (uchar8)(1, 2, 5, 3, 4, 0, 6, 7), (uchar8)(1, 2, 5, 7, 4, 0, 6, 3),
    (uchar8)(1, 2, 5, 6, 4, 0, 3, 7), (uchar8)(1, 2, 5, 6, 7, 0, 3, 4),
    (uchar8)(1, 2, 4, 3, 0, 5, 6, 7), (uchar8)(1, 2, 4, 7, 0, 5, 6, 3),
    (uchar8)(1, 2, 4, 6, 0, 5, 3, 7), (uchar8)(1, 2, 4, 6, 7, 5, 3, 0),
    (uchar8)(1, 2, 4, 5, 0, 3, 6, 7), (uchar8)(1, 2, 4, 5, 7, 3, 6, 0),
    (uchar8)(1, 2, 4, 5, 6, 3, 0, 7), (uchar8)(1, 2, 4, 5, 6, 7, 0, 3),
    (uchar8)(1, 2, 3, 0, 4, 5, 6, 7), (uchar8)(1, 2, 3, 7, 4, 5, 6, 0),
    (uchar8)(1, 2, 3, 6, 4, 5, 0, 7), (uchar8)(1, 2, 3, 6, 7, 5, 0, 4),
    (uchar8)(1, 2, 3, 5, 4, 0, 6, 7), (uchar8)(1, 2, 3, 5, 7, 0, 6, 4),
    (uchar8)(1, 2, 3, 5, 6, 0, 4, 7), (uchar8)(1, 2, 3, 5, 6, 7, 4, 0),
    (uchar8)(1, 2, 3, 4, 0, 5, 6, 7), (uchar8)(1, 2, 3, 4, 7, 5, 6, 0),
    (uchar8)(1, 2, 3, 4, 6, 5, 0, 7), (uchar8)(1, 2, 3, 4, 6, 7, 0, 5),
    (uchar8)(1, 2, 3, 4, 5, 0, 6, 7), (uchar8)(1, 2, 3, 4, 5, 7, 6, 0),
    (uchar8)(1, 2, 3, 4, 5, 6, 0, 7), (uchar8)(1, 2, 3, 4, 5, 6, 7, 0),
    (uchar8)(0, 1, 2, 3, 4, 5, 6, 7), (uchar8)(0, 7, 2, 3, 4, 5, 6, 1),
    (uchar8)(0, 6, 2, 3, 4, 5, 1, 7), (uchar8)(0, 6, 7, 3, 4, 5, 1, 2),
    (uchar8)(0, 5, 2, 3, 4, 1, 6, 7), (uchar8)(0, 5, 7, 3, 4, 1, 6, 2),
    (uchar8)(0, 5, 6, 3, 4, 1, 2, 7), (uchar8)(0, 5, 6, 7, 4, 1, 2, 3),
    (uchar8)(0, 4, 2, 3, 1, 5, 6, 7), (uchar8)(0, 4, 7, 3, 1, 5, 6, 2),
    (uchar8)(0, 4, 6, 3, 1, 5, 2, 7), (uchar8)(0, 4, 6, 7, 1, 5, 2, 3),
    (uchar8)(0, 4, 5, 3, 1, 2, 6, 7), (uchar8)(0, 4, 5, 7, 1, 2, 6, 3),
    (uchar8)(0, 4, 5, 6, 1, 2, 3, 7), (uchar8)(0, 4, 5, 6, 7, 2, 3, 1),
    (uchar8)(0, 3, 2, 1, 4, 5, 6, 7), (uchar8)(0, 3, 7, 1, 4, 5, 6, 2),
    (uchar8)(0, 3, 6, 1, 4, 5, 2, 7), (uchar8)(0, 3, 6, 7, 4, 5, 2, 1),
    (uchar8)(0, 3, 5, 1, 4, 2, 6, 7), (uchar8)(0, 3, 5, 7, 4, 2, 6, 1),
    (uchar8)(0, 3, 5, 6, 4, 2, 1, 7), (uchar8)(0, 3, 5, 6, 7, 2, 1, 4),
    (uchar8)(0, 3, 4, 1, 2, 5, 6, 7), (uchar8)(0, 3, 4, 7, 2, 5, 6, 1),
    (uchar8)(0, 3, 4, 6, 2, 5, 1, 7), (uchar8)(0, 3, 4, 6, 7, 5, 1, 2),
    (uchar8)(0, 3, 4, 5, 2, 1, 6, 7), (uchar8)(0, 3, 4, 5, 7, 1, 6, 2),
    (uchar8)(0, 3, 4, 5, 6, 1, 2, 7), (uchar8)(0, 3, 4, 5, 6, 7, 2, 1),
    (uchar8)(0, 2, 1, 3, 4, 5, 6, 7), (uchar8)(0, 2, 7, 3, 4, 5, 6, 1),
    (uchar8)(0, 2, 6, 3, 4, 5, 1, 7), (uchar8)(0, 2, 6, 7, 4, 5, 1, 3),
    (uchar8)(0, 2, 5, 3, 4, 1, 6, 7), (uchar8)(0, 2, 5, 7, 4, 1, 6, 3),
    (uchar8)(0, 2, 5, 6, 4, 1, 3, 7), (uchar8)(0, 2, 5, 6, 7, 1, 3, 4),
    (uchar8)(0, 2, 4, 3, 1, 5, 6, 7), (uchar8)(0, 2, 4, 7, 1, 5, 6, 3),
    (uchar8)(0, 2, 4, 6, 1, 5, 3, 7), (uchar8)(0, 2, 4, 6, 7, 5, 3, 1),
    (uchar8)(0, 2, 4, 5, 1, 3, 6, 7), (uchar8)(0, 2, 4, 5, 7, 3, 6, 1),
    (uchar8)(0, 2, 4, 5, 6, 3, 1, 7), (uchar8)(0, 2, 4, 5, 6, 7, 1, 3),
    (uchar8)(0, 2, 3, 1, 4, 5, 6, 7), (uchar8)(0, 2, 3, 7, 4, 5, 6, 1),
    (uchar8)(0, 2, 3, 6, 4, 5, 1, 7), (uchar8)(0, 2, 3, 6, 7, 5, 1, 4),
    (uchar8)(0, 2, 3, 5, 4, 1, 6, 7), (uchar8)(0, 2, 3, 5, 7, 1, 6, 4),
    (uchar8)(0, 2, 3, 5, 6, 1, 4, 7), (uchar8)(0, 2, 3, 5, 6, 7, 4, 1),
    (uchar8)(0, 2, 3, 4, 1, 5, 6, 7), (uchar8)(0, 2, 3, 4, 7, 5, 6, 1),
    (uchar8)(0, 2, 3, 4, 6, 5, 1, 7), (uchar8)(0, 2, 3, 4, 6, 7, 1, 5),
    (uchar8)(0, 2, 3, 4, 5, 1, 6, 7), (uchar8)(0, 2, 3, 4, 5, 7, 6, 1),
    (uchar8)(0, 2, 3, 4, 5, 6, 1, 7), (uchar8)(0, 2, 3, 4, 5, 6, 7, 1),
    (uchar8)(0, 1, 2, 3, 4, 5, 6, 7), (uchar8)(0, 1, 7, 3, 4, 5, 6, 2),
    (uchar8)(0, 1, 6, 3, 4, 5, 2, 7), (uchar8)(0, 1, 6, 7, 4, 5, 2, 3),
    (uchar8)(0, 1, 5, 3, 4, 2, 6, 7), (uchar8)(0, 1, 5, 7, 4, 2, 6, 3),
    (uchar8)(0, 1, 5, 6, 4, 2, 3, 7), (uchar8)(0, 1, 5, 6, 7, 2, 3, 4),
    (uchar8)(0, 1, 4, 3, 2, 5, 6, 7), (uchar8)(0, 1, 4, 7, 2, 5, 6, 3),
    (uchar8)(0, 1, 4, 6, 2, 5, 3, 7), (uchar8)(0, 1, 4, 6, 7, 5, 3, 2),
    (uchar8)(0, 1, 4, 5, 2, 3, 6, 7), (uchar8)(0, 1, 4, 5, 7, 3, 6, 2),
    (uchar8)(0, 1, 4, 5, 6, 3, 2, 7), (uchar8)(0, 1, 4, 5, 6, 7, 2, 3),
    (uchar8)(0, 1, 3, 2, 4, 5, 6, 7), (uchar8)(0, 1, 3, 7, 4, 5, 6, 2),
    (uchar8)(0, 1, 3, 6, 4, 5, 2, 7), (uchar8)(0, 1, 3, 6, 7, 5, 2, 4),
    (uchar8)(0, 1, 3, 5, 4, 2, 6, 7), (uchar8)(0, 1, 3, 5, 7, 2, 6, 4),
    (uchar8)(0, 1, 3, 5, 6, 2, 4, 7), (uchar8)(0, 1, 3, 5, 6, 7, 4, 2),
    (uchar8)(0, 1, 3, 4, 2, 5, 6, 7), (uchar8)(0, 1, 3, 4, 7, 5, 6, 2),
    (uchar8)(0, 1, 3, 4, 6, 5, 2, 7), (uchar8)(0, 1, 3, 4, 6, 7, 2, 5),
    (uchar8)(0, 1, 3, 4, 5, 2, 6, 7), (uchar8)(0, 1, 3, 4, 5, 7, 6, 2),
    (uchar8)(0, 1, 3, 4, 5, 6, 2, 7), (uchar8)(0, 1, 3, 4, 5, 6, 7, 2),
    (uchar8)(0, 1, 2, 3, 4, 5, 6, 7), (uchar8)(0, 1, 2, 7, 4, 5, 6, 3),
    (uchar8)(0, 1, 2, 6, 4, 5, 3, 7), (uchar8)(0, 1, 2, 6, 7, 5, 3, 4),
    (uchar8)(0, 1, 2, 5, 4, 3, 6, 7), (uchar8)(0, 1, 2, 5, 7, 3, 6, 4),
    (uchar8)(0, 1, 2, 5, 6, 3, 4, 7), (uchar8)(0, 1, 2, 5, 6, 7, 4, 3),
    (uchar8)(0, 1, 2, 4, 3, 5, 6, 7), (uchar8)(0, 1, 2, 4, 7, 5, 6, 3),
    (uchar8)(0, 1, 2, 4, 6, 5, 3, 7), (uchar8)(0, 1, 2, 4, 6, 7, 3, 5),
    (uchar8)(0, 1, 2, 4, 5, 3, 6, 7), (uchar8)(0, 1, 2, 4, 5, 7, 6, 3),
    (uchar8)(0, 1, 2, 4, 5, 6, 3, 7), (uchar8)(0, 1, 2, 4, 5, 6, 7, 3),
    (uchar8)(0, 1, 2, 3, 4, 5, 6, 7), (uchar8)(0, 1, 2, 3, 7, 5, 6, 4),
    (uchar8)(0, 1, 2, 3, 6, 5, 4, 7), (uchar8)(0, 1, 2, 3, 6, 7, 4, 5),
    (uchar8)(0, 1, 2, 3, 5, 4, 6, 7), (uchar8)(0, 1, 2, 3, 5, 7, 6, 4),
    (uchar8)(0, 1, 2, 3, 5, 6, 4, 7), (uchar8)(0, 1, 2, 3, 5, 6, 7, 4),
    (uchar8)(0, 1, 2, 3, 4, 5, 6, 7), (uchar8)(0, 1, 2, 3, 4, 7, 6, 5),
    (uchar8)(0, 1, 2, 3, 4, 6, 5, 7), (uchar8)(0, 1, 2, 3, 4, 6, 7, 5),
    (uchar8)(0, 1, 2, 3, 4, 5, 6, 7), (uchar8)(0, 1, 2, 3, 4, 5, 7, 6),
    (uchar8)(0, 1, 2, 3, 4, 5, 6, 7), (uchar8)(0, 1, 2, 3, 4, 5, 6, 7)};

kernel __attribute__((vec_type_hint(uint8))) void
probe_vec(global uint *outer, const ulong outer_count,
          global Bucket *hash_table, const uint bucket_count,
          global uint *output, const uint invalid_key) {

  // partition of work
  const uint gid = get_global_id(0);
  const uint gs = get_global_size(0);
  const ulong bound = (outer_count / gs) * (gid + 1);
  global uint *out_tab = &output[(outer_count / gs) * gid];

  // constant definitions
  const uint shift_single = 32 - log_2(bucket_count);
  const uint8 shift =
      (uint8)(shift_single, shift_single, shift_single, shift_single,
              shift_single, shift_single, shift_single, shift_single);

  const uint8 mc =
      (uint8)(MULTIPLY_CONSTANT, MULTIPLY_CONSTANT, MULTIPLY_CONSTANT,
              MULTIPLY_CONSTANT, MULTIPLY_CONSTANT, MULTIPLY_CONSTANT,
              MULTIPLY_CONSTANT, MULTIPLY_CONSTANT);

  const uint8 empty =
      (uint8)(invalid_key, invalid_key, invalid_key, invalid_key, invalid_key,
              invalid_key, invalid_key, invalid_key);

  const uint max_h_single = bucket_count - 1;
  const uint8 max_h =
      (uint8)(max_h_single, max_h_single, max_h_single, max_h_single,
              max_h_single, max_h_single, max_h_single, max_h_single);

  const uint8 ones = (uint8)(1, 1, 1, 1, 1, 1, 1, 1);
  const uint8 zeros = (uint8)(0, 0, 0, 0, 0, 0, 0, 0);

  // variable declarations
  int8 inv, out;
  uint8 new_key, key, h_key, h_val;
  uint8 offset, h, perm_inv, perm_out;

  uchar mask_inv, mask_out;
  uchar inv_count, out_count;

  // output / input counter
  ulong counter = 0;
  ulong i = ((outer_count / gs) * gid);

  // init values
  inv = (int8)(-1, -1, -1, -1, -1, -1, -1, -1);
  inv_count = 8;

  while (i < bound - 8) {

    // load new keys from outer table
    new_key = vload8(0, &outer[i]);
    key = select(key, new_key, inv == -1);
    i += inv_count;

    // adjust offset
    offset += ones;
    offset = select(offset, zeros, inv == -1);

    // hash keys
    h = convert_uint8(key * mc);
    h >>= shift;
    h = h + offset; // add offset
    h = h & max_h;  // check if overflow and correct

    // gather buckets from hash table
    h_key.s0 = hash_table[h.s0].key;
    h_val.s0 = hash_table[h.s0].val;
    h_key.s1 = hash_table[h.s1].key;
    h_val.s1 = hash_table[h.s1].val;
    h_key.s2 = hash_table[h.s2].key;
    h_val.s2 = hash_table[h.s2].val;
    h_key.s3 = hash_table[h.s3].key;
    h_val.s3 = hash_table[h.s3].val;
    h_key.s4 = hash_table[h.s4].key;
    h_val.s4 = hash_table[h.s4].val;
    h_key.s5 = hash_table[h.s5].key;
    h_val.s5 = hash_table[h.s5].val;
    h_key.s6 = hash_table[h.s6].key;
    h_val.s6 = hash_table[h.s6].val;
    h_key.s7 = hash_table[h.s7].key;
    h_val.s7 = hash_table[h.s7].val;

    // compare
    inv = isequal(h_key, empty); // invalid entries
    out = isequal(h_key, key);   // matching entries
    inv = (inv || out);

    // load permutation masks
    mask_inv = -1 * inv.s7 + -2 * inv.s6 + -4 * inv.s5 + -8 * inv.s4 +
               -16 * inv.s3 + -32 * inv.s2 + -64 * inv.s1 + -128 * inv.s0;
    perm_inv = convert_uint8(perm[mask_inv]);
    inv_count = popcount(mask_inv);

    mask_out = -1 * out.s7 + -2 * out.s6 + -4 * out.s5 + -8 * out.s4 +
               -16 * out.s3 + -32 * out.s2 + -64 * out.s1 + -128 * out.s0;
    perm_out = convert_uint8(perm[mask_out]);
    out_count = popcount(mask_out);

    // permute invalid
    inv = shuffle(inv, perm_inv);
    key = shuffle(key, perm_inv);
    offset = shuffle(offset, perm_inv);

    // permute output and selective store
    h_val = select(zeros, h_val, out == -1);
    h_val = shuffle(h_val, perm_out);
    vstore8(h_val, 0, &out_tab[counter]);
    counter += out_count;
  }

  // finish unfinished keys from vector
  for (uchar old_key = inv_count; old_key < 8; ++old_key) {

    // read key
    uint key_scalar;
    if (old_key == 0) {
      key_scalar = key.s0;
    }
    if (old_key == 1) {
      key_scalar = key.s1;
    }
    if (old_key == 2) {
      key_scalar = key.s2;
    }
    if (old_key == 3) {
      key_scalar = key.s3;
    }
    if (old_key == 4) {
      key_scalar = key.s4;
    }
    if (old_key == 5) {
      key_scalar = key.s5;
    }
    if (old_key == 6) {
      key_scalar = key.s6;
    }
    if (old_key == 7) {
      key_scalar = key.s7;
    }

    // determine hash value and read from hash table
    ulong h_scalar = ((uint)(key_scalar * MULTIPLY_CONSTANT)) >> shift_single;
    Bucket read_bucket = hash_table[h_scalar];

    // check key of read bucket
    if (key_scalar == read_bucket.key) {
      out_tab[counter++] = read_bucket.val;
    } else {
      // do linear probing
      while (invalid_key != read_bucket.key) {
        h_scalar = (h_scalar + 1) & (bucket_count - 1);
        read_bucket = hash_table[h_scalar];
        if (key_scalar == read_bucket.key) {
          out_tab[counter++] = read_bucket.val;
          break;
        }
      }
    }
  }

  // do the last 8 entries
  for (ulong old_key = i; old_key < bound; ++old_key) {

    // read key
    uint key_scalar = outer[old_key];

    // determine hash value and read from hash table
    ulong h_scalar = ((uint)(key_scalar * MULTIPLY_CONSTANT)) >> shift_single;
    Bucket read_bucket = hash_table[h_scalar];

    // check key of read bucket
    if (key_scalar == read_bucket.key) {
      out_tab[counter++] = read_bucket.val;
    } else {
      // do linear probing
      while (invalid_key != read_bucket.key) {
        h_scalar = (h_scalar + 1) & (bucket_count - 1);
        read_bucket = hash_table[h_scalar];
        if (key_scalar == read_bucket.key) {
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
