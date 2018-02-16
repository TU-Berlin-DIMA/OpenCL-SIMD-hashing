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

#pragma once

#include <cassert>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <iostream>
#include <sstream>
#include <unistd.h>
#include <vector>

#include <CL/cl.h>
#include <boost/compute.hpp>
namespace bc = boost::compute;
#undef CL_VERSION_2_0 // we wanne use version 1.2
#define BOOST_COMPUTE_DEBUG_KERNEL_COMPILATION

#include <boost/align/aligned_allocator.hpp>

extern "C" {
#include "inner_outer.h"
}

// From Knuths multplicative method
// (https://stackoverflow.com/a/665545)
#define MULTIPLY_SHIFT_CONSTANT 2654435761

// Boost Aligned Vector
#ifndef ALIGNMENT
#define ALIGNMENT 8
#endif // ALIGNMENT

namespace ba = boost::alignment;
template <typename T>
using AlignedVector = std::vector<T, ba::aligned_allocator<T, ALIGNMENT>>;

// Table structures
struct Bucket {
  uint32_t key;
  uint32_t val;
};
typedef struct Bucket Bucket;

struct HashTable {
  uint64_t join_count;
  uint64_t checksum;
  uint32_t hash_bits;
  uint32_t size;
  uint32_t empty;
  Bucket *buckets;
  AlignedVector<Bucket> *buckets_vec;
};
typedef struct HashTable HashTable;

typedef uint32_t Item;
struct Table {
  size_t size;
  uint32_t empty;
  Item *items;
  AlignedVector<Item> *items_vec;
};
typedef struct Table Table;

// Benchmarking topics
void build(std::string opcl_kernel, size_t global, size_t local);
void probe(std::string opcl_kernel, size_t global, size_t local);

// Test data generation
void generate_tables(size_t probe_size, uint32_t hash_size, double selectivity,
                     double load_factor, Table *inner_tab, Table *outer_tab,
                     HashTable *h_tab);
void build_hash(Table *inner_tab, HashTable *h_tab, bool save_hash_key);
void join_size_checksum(const Table *outer, const HashTable *tab,
                        uint64_t *join_size, uint64_t *checksum);

const char *human_bytes(size_t items, size_t bytes);

// Time measuring
typedef uint64_t Timestamp;
using NanoSeconds = std::chrono::nanoseconds;
using Clock = std::chrono::high_resolution_clock;
Timestamp get_timestamp();

// OpenCL
cl_device_id get_intel_device_id();
const std::string deviceType(const cl_device_type &type);
