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

#define BIT(var, pos) !!((var) & (1 << (pos)))

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

constant uchar8 fence[9] = { // 8*9 = 72 byte
    (uchar8)(0, 0, 0, 0, 0, 0, 0, 0), (uchar8)(1, 0, 0, 0, 0, 0, 0, 0),
    (uchar8)(1, 1, 0, 0, 0, 0, 0, 0), (uchar8)(1, 1, 1, 0, 0, 0, 0, 0),
    (uchar8)(1, 1, 1, 1, 0, 0, 0, 0), (uchar8)(1, 1, 1, 1, 1, 0, 0, 0),
    (uchar8)(1, 1, 1, 1, 1, 1, 0, 0), (uchar8)(1, 1, 1, 1, 1, 1, 1, 0),
    (uchar8)(1, 1, 1, 1, 1, 1, 1, 1)};

kernel __attribute__((vec_type_hint(uint8))) void
load(global uint *input, const ulong in_count, global uint *output) {

  uchar mask = 255;
  ulong pos_rd = 0;
  const ulong bound = in_count - 6;
  uint8 vec = convert_uint8(fence[0]);

  while (pos_rd < bound) {

    // load mask from lookup-table and permute vector
    uint8 perm_in = convert_uint8(perm[mask]);
    vec = shuffle(vec, perm_in);

    // mask-load new items from input
    uchar item_count = popcount(mask);
    uint8 new_vec = vload8(0, &input[pos_rd]);

    // combine vector with new items
    vec = select(vec, new_vec, convert_uint8(fence[item_count]) == 1);

    // increment reading position and change mask
    pos_rd += item_count;
    mask++;
  }
  // return -> function content will not be optimized away...
  vstore8(vec, 0, &output[0]);
}

kernel __attribute__((vec_type_hint(uint8))) void store(global uint *output,
                                                        const ulong out_count) {

  uchar mask = 255;
  ulong pos_wr = 0;
  const ulong bound = out_count - 6;
  uint8 vec = (uint8)(0, 1, 2, 3, 4, 5, 6, 7);

  while (pos_wr < bound) {

    // load mask from lookup-table and permute vector
    uint8 perm_out = convert_uint8(perm[mask]);
    vec = shuffle(vec, perm_out);

    // mask-store items from vector
    uchar item_count = popcount(mask);
    uint8 write_vec = select(convert_uint8(fence[0]), vec,
                             convert_uint8(fence[item_count]) == 1);
    vstore8(write_vec, 0, &output[pos_wr]);

    // increment writing position and change mask
    pos_wr += item_count;
    mask++;
  }
}
