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
#include "opencl_avx.hpp"

#define VALUE_FACTOR 42
#define KERNEL_LOOPS 1

std::string cl_dir = _DIR;
uint32_t repetitions = 5;
uint64_t table_count = 1000 * 1000 * 100;
double table_loadfactor = 0.5;
double table_selectivity = 0.9;

const uint8_t hash_sizes = 15;
const uint32_t hash_size[] = {512,    1024,    2048,    4096,    8192,
                              16384,  32768,   65536,   131072,  262144,
                              524288, 1048576, 2097152, 4194304, 8388608};

int main() {

  std::cout << std::fixed;
  std::cerr << "Expecting OpenCL Kernels at: " << cl_dir << std::endl;

#ifdef _LS
#ifdef __AVX2__
  load_store_avx();
#endif // __AVX2__
  load_store();
#endif // _LS

#ifdef _GS
#ifdef __AVX2__
  gather_scatter_avx();
#endif // __AVX2__
  gather_scatter();
#endif // _GS

#ifdef _BUILD
  build("build", 8, 1);
  build("build_vec", 8, 1);
#endif // _BUILD

#ifdef _LP
  probe("probe", 8, 1);
  probe("probe_vec", 8, 1);
#endif // _LP

  return 0;
}

/**
 * Benchmark Operations - Load & Store
 */
void load_store() {

  std::cerr << "Load & Store --> start" << std::endl;

  // initiate OpenCL device, ...
  bc::device device = bc::device(get_intel_device_id());
  bc::context context = bc::context(device);

  bc::command_queue command =
      bc::command_queue(context, device, CL_QUEUE_PROFILING_ENABLE);

  // load and build program
  bc::program load_store_cl =
      bc::program::create_with_source_file((cl_dir + "load_store.cl"), context);
  try {
    load_store_cl.build();
  } catch (bc::opencl_error &e) {
    std::cerr << "FAILED " << e.error_string() << " (" << e.error_code() << ")"
              << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "FAILED " << e.what() << std::endl;
  }
  std::cerr << load_store_cl.build_log() << std::endl;
  bc::kernel load_kernel = load_store_cl.create_kernel("load");
  bc::kernel store_kernel = load_store_cl.create_kernel("store");

  for (uint16_t r = 0; r < repetitions * 4; ++r) {

    // generate test data
    Table *inner = new Table();
    Table *outer = new Table();
    HashTable *tab = new HashTable();

    generate_tables(table_count, hash_size[0], table_loadfactor,
                    table_selectivity, inner, outer, tab);

    // copy test data
    cl_long count = (cl_uint)(outer->size);
    bc::buffer b_in(context, count * sizeof(cl_uint));
    bc::buffer b_out(context, count * sizeof(cl_uint));
    command.enqueue_write_buffer(b_in, 0, count * sizeof(cl_uint),
                                 &outer->items[0]);

    // create kernels and set arguments
    load_kernel.set_arg(0, b_in);
    load_kernel.set_arg(1, count);
    load_kernel.set_arg(2, b_out);
    store_kernel.set_arg(0, b_out);
    store_kernel.set_arg(1, count);

    // run kernel
    bc::event load_event =
        command.enqueue_1d_range_kernel(load_kernel, 0, 1, 1);
    load_event.wait();

    // get and save running time
    Timestamp time_in_ns =
        load_event.duration<std::chrono::nanoseconds>().count();
    std::cout << "load;";
    std::cout << count * 1.0 / time_in_ns << std::endl;

    // run kernel
    bc::event store_event =
        command.enqueue_1d_range_kernel(store_kernel, 0, 1, 1);
    store_event.wait();

    // get and save running time
    time_in_ns = store_event.duration<std::chrono::nanoseconds>().count();
    std::cout << "store;";
    std::cout << count * 1.0 / time_in_ns << std::endl;

    delete inner->items_vec;
    delete outer->items_vec;
    delete tab->buckets_vec;

    delete inner;
    delete outer;
    delete tab;
  }

  std::cerr << "Load & Store --> end" << std::endl;
}

/**
 * Benchmark Operations - Load & Store AVX
 */
#ifdef __AVX2__
void load_store_avx() {

  std::cerr << "Load & Store AVX --> start" << std::endl;

  for (uint16_t r = 0; r < repetitions * 4; ++r) {

    // generate test data
    Table *inner = new Table();
    Table *outer = new Table();
    Table *output = new Table();
    HashTable *tab = new HashTable();

    generate_tables(table_count, hash_size[0], table_loadfactor,
                    table_selectivity, inner, outer, tab);

    AlignedVector<Item> *res_vec = new AlignedVector<Item>(outer->size);
    std::fill(res_vec->begin(), res_vec->end(), outer->empty);
    output->size = outer->size;
    output->empty = outer->empty;
    output->items_vec = res_vec;
    output->items = &res_vec->at(0);

    // run load
    Timestamp start = get_timestamp();
    avx::load(&outer->items[0], outer->size);
    Timestamp stop = get_timestamp();

    // get and save running time
    Timestamp time_in_ns = stop - start;
    std::cout << "load_avx"
              << ";";
    std::cout << outer->size * 1.0 / time_in_ns << std::endl;

    // run store
    start = get_timestamp();
    avx::store(&output->items[0], outer->size);
    stop = get_timestamp();

    // get and save running time
    time_in_ns = stop - start;
    std::cout << "store_avx"
              << ";";
    std::cout << outer->size * 1.0 / time_in_ns << std::endl;

    delete inner->items_vec;
    delete outer->items_vec;
    delete output->items_vec;
    delete tab->buckets_vec;

    delete inner;
    delete outer;
    delete output;
    delete tab;
  }

  std::cerr << "Load & Store AVX -- > End" << std::endl;
}
#endif // __AVX2__

/**
 * Benchmark Operations - Gather & Scatter
 */
void gather_scatter() {

  std::cerr << "Gather & Scatter --> start" << std::endl;

  // initiate OpenCL device, ...
  bc::device device = bc::device(get_intel_device_id());
  bc::context context = bc::context(device);

  bc::command_queue command =
      bc::command_queue(context, device, CL_QUEUE_PROFILING_ENABLE);

  // load and build program
  bc::program gather_scatter_cl = bc::program::create_with_source_file(
      (cl_dir + "gather_scatter.cl"), context);
  try {
    gather_scatter_cl.build();
  } catch (bc::opencl_error &e) {
    std::cerr << "FAILED " << e.error_string() << " (" << e.error_code() << ")"
              << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "FAILED " << e.what() << std::endl;
  }
  std::cerr << gather_scatter_cl.build_log() << std::endl;
  bc::kernel gather_kernel = gather_scatter_cl.create_kernel("gather");
  bc::kernel scatter_kernel = gather_scatter_cl.create_kernel("scatter");

  for (uint8_t s = 0; s < hash_sizes; ++s) {

    std::cerr << "Gather & Scatter --> Hash Size "
              << human_bytes(sizeof(uint64_t), hash_size[s]) << std::endl;

    for (uint16_t r = 0; r < repetitions; ++r) {

      // generate test data
      Table *inner = new Table();
      Table *outer = new Table();
      HashTable *tab = new HashTable();

      generate_tables(table_count, hash_size[s], table_loadfactor,
                      table_selectivity, inner, outer, tab);

      build_hash(inner, tab, false);

      // copy test data
      cl_uint tab_count = (cl_uint)(tab->size);
      bc::buffer b_tab(context, tab_count * sizeof(cl_ulong));

      cl_ulong out_count = (cl_ulong)(tab->size);
      bc::buffer b_out = bc::buffer(context, out_count * sizeof(cl_ulong));

      command.enqueue_write_buffer(b_tab, 0, tab_count * sizeof(cl_ulong),
                                   &tab->buckets[0]);

      // create kernel and set arguments
      gather_kernel.set_arg(0, b_tab);
      gather_kernel.set_arg(1, tab_count);
      gather_kernel.set_arg(2, table_count);
      gather_kernel.set_arg(3, b_out);

      scatter_kernel.set_arg(0, b_out);
      scatter_kernel.set_arg(1, tab_count);
      scatter_kernel.set_arg(2, table_count);

      // run kernel
      bc::event gather_event =
          command.enqueue_1d_range_kernel(gather_kernel, 0, 1, 1);
      gather_event.wait();
      // get and save running time
      Timestamp time_in_ns =
          gather_event.duration<std::chrono::nanoseconds>().count();
      std::cout << "gather_ocl"
                << ";";
      std::cout << human_bytes(sizeof(uint64_t), tab->size) << ";";
      std::cout << table_count * 1.0 / time_in_ns << std::endl;

      // run kernel
      bc::event scatter_event =
          command.enqueue_1d_range_kernel(scatter_kernel, 0, 1, 1);
      scatter_event.wait();
      // get and save running time
      time_in_ns = scatter_event.duration<std::chrono::nanoseconds>().count();
      std::cout << "scatter_ocl;";
      std::cout << human_bytes(sizeof(uint64_t), tab->size) << ";";
      std::cout << table_count * 1.0 / time_in_ns << std::endl;

      delete inner->items_vec;
      delete outer->items_vec;
      delete tab->buckets_vec;

      delete inner;
      delete outer;
      delete tab;
    }
  }

  std::cerr << "Gather & Scatter --> end" << std::endl;
}

/**
 * Benchmark Operations - Gather & Scatter AVX
 */
#ifdef __AVX2__
void gather_scatter_avx() {

  std::cerr << std::endl << "Gather & Scatter AVX --> start" << std::endl;

  for (uint8_t s = 0; s < hash_sizes; ++s) {

    std::cerr << "Gather & Scatter AVX --> Hash Size "
              << human_bytes(sizeof(uint64_t), hash_size[s]) << std::endl;

    for (uint16_t r = 0; r < repetitions; ++r) {
      // generate test data
      Table *inner = new Table();
      Table *outer = new Table();
      HashTable *output = new HashTable();
      HashTable *tab = new HashTable();

      generate_tables(table_count, hash_size[s], table_loadfactor,
                      table_selectivity, inner, outer, tab);

      build_hash(inner, tab, false);

      AlignedVector<Bucket> *res_vec = new AlignedVector<Bucket>(tab->size);
      Bucket empty{tab->empty, tab->empty};
      std::fill(res_vec->begin(), res_vec->end(), empty);
      output->size = tab->size;
      output->empty = tab->empty;
      output->buckets_vec = res_vec;
      output->buckets = &res_vec->at(0);

      // run avx
      Timestamp start = get_timestamp();
      avx::gather((uint64_t *)&tab->buckets[0], tab->size, table_count);
      Timestamp stop = get_timestamp();

      // get and save running time
      Timestamp time_in_ns = stop - start;
      std::cout << "gather_avx;";
      std::cout << human_bytes(sizeof(uint64_t), tab->size) << ";";
      std::cout << table_count * 1.0 / time_in_ns << std::endl;

      // run avx
      start = get_timestamp();
      avx::scatter((uint64_t *)&output->buckets[0], tab->size, table_count);
      stop = get_timestamp();

      // get and save running time
      time_in_ns = stop - start;
      std::cout << "scatter_avx;";
      std::cout << human_bytes(sizeof(uint64_t), tab->size) << ";";
      std::cout << table_count * 1.0 / time_in_ns << std::endl;

      delete inner->items_vec;
      delete outer->items_vec;
      delete tab->buckets_vec;

      delete inner;
      delete outer;
      delete tab;
    }
  }

  std::cerr << "Gather & Scatter --> end" << std::endl;
}
#endif // __AVX2__

/**
 * Benchmark build
 */
void build(std::string opcl_kernel, size_t global, size_t local) {

  std::cerr << "Build '" << opcl_kernel << "' --> start" << std::endl;

  // initiate OpenCL device, ...
  bc::device device = bc::device(get_intel_device_id());
  bc::context context = bc::context(device);
  bc::command_queue command =
      bc::command_queue(context, device, CL_QUEUE_PROFILING_ENABLE);

  // load and build program
  bc::program build_cl = bc::program::create_with_source_file(
      (cl_dir + opcl_kernel + ".cl"), context);
  try {
    std::stringstream ss;
    ss << "-DKERNEL_LOOPS=" << KERNEL_LOOPS;
    build_cl.build(ss.str());
  } catch (bc::opencl_error &e) {
    std::cerr << "FAILED " << e.error_string() << " (" << e.error_code() << ")"
              << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "FAILED " << e.what() << std::endl;
  }
  std::cerr << build_cl.build_log() << std::endl;
  bc::kernel kernel = build_cl.create_kernel(opcl_kernel);

  for (uint8_t s = 0; s < hash_sizes - 1; ++s) {

    std::cerr << "Build --> Hash Size "
              << human_bytes(sizeof(uint64_t), hash_size[s]) << std::endl;

    // do the build
    for (uint16_t r = 0; r < repetitions; ++r) {

      // generate test data
      Table *inner = new Table();
      Table *outer = new Table();
      HashTable *hash_tab = new HashTable();

      generate_tables(table_count, hash_size[s], table_loadfactor,
                      table_selectivity, inner, outer, hash_tab);

      // repeat keys and generate vals
      cl_ulong keys_count = (cl_ulong)(inner->size);
      AlignedVector<Item> *keys = new AlignedVector<Item>(global * keys_count);
      AlignedVector<Item> *vals = new AlignedVector<Item>(global * keys_count);
      for (uint16_t g = 0; g < global; ++g) {
        for (uint64_t t = 0; t < keys_count; ++t) {
          keys->at(g * keys_count + t) = inner->items[t];
          vals->at(g * keys_count + t) = inner->items[t] * VALUE_FACTOR;
        }
      }

      // copy keys and vals
      bc::buffer b_keys(context, global * keys_count * sizeof(cl_uint));
      bc::buffer b_vals(context, global * keys_count * sizeof(cl_uint));
      command.enqueue_write_buffer(
          b_keys, 0, global * keys_count * sizeof(cl_uint), &keys->at(0));
      command.enqueue_write_buffer(
          b_vals, 0, global * keys_count * sizeof(cl_uint), &vals->at(0));

      cl_ulong buckets_bytes = (cl_ulong)(hash_tab->size * sizeof(cl_ulong));
      bc::buffer b_tab(context, global * buckets_bytes * 1.2);

      // create kernel and set arguments
      kernel.set_arg(0, b_keys);
      kernel.set_arg(1, b_vals);
      kernel.set_arg(2, inner->size);
      kernel.set_arg(3, b_tab);
      kernel.set_arg(4, hash_tab->size);
      kernel.set_arg(5, inner->empty);

      // run kernel
      bc::event event =
          command.enqueue_1d_range_kernel(kernel, 0, global, local);
      event.wait();

      // get and save running time
      Timestamp time_in_ns = event.duration<std::chrono::nanoseconds>().count();
      std::cout << opcl_kernel << ";";
      std::cout << deviceType(device.type()) << ";";
      std::cout << human_bytes(sizeof(uint64_t), hash_tab->size) << ";";
      std::cout << hash_tab->size << ";";
      std::cout << "(" << global << "," << local << ")"
                << ";";
      std::cout << (global * inner->size * 1.0 * KERNEL_LOOPS) / time_in_ns;
      std::cout << ";" << time_in_ns << std::endl;

      delete inner->items_vec;
      delete outer->items_vec;

      delete vals;
      delete keys;
      delete inner;
      delete outer;
    }
  }

  std::cerr << "Build '" << opcl_kernel << "' --> end" << std::endl;
}

/**
 * Benchmark Linear Probing
 */
void probe(std::string opcl_kernel, size_t global, size_t local) {

  std::cerr << "Linear Probing '" << opcl_kernel << "' --> start" << std::endl;

  // initiate OpenCL device, ...
  bc::device device = bc::device(get_intel_device_id());
  bc::context context = bc::context(device);
  bc::command_queue command =
      bc::command_queue(context, device, CL_QUEUE_PROFILING_ENABLE);

  // load and build program
  bc::program probe_cl = bc::program::create_with_source_file(
      (cl_dir + opcl_kernel + ".cl"), context);
  try {
    probe_cl.build();
  } catch (bc::opencl_error &e) {
    std::cerr << "FAILED " << e.error_string() << " (" << e.error_code() << ")"
              << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "FAILED " << e.what() << std::endl;
  }
  std::cerr << probe_cl.build_log() << std::endl;
  bc::kernel kernel = probe_cl.create_kernel(opcl_kernel);

  // init kernel
  bc::program init_cl =
      bc::program::create_with_source_file((cl_dir + "init.cl"), context);
  init_cl.build();
  bc::kernel init_kernel = init_cl.create_kernel("init");

  // check kernel
  bc::program check_cl =
      bc::program::create_with_source_file((cl_dir + "checksum.cl"), context);
  check_cl.build();
  bc::kernel check_kernel = check_cl.create_kernel("checksum");
  bc::buffer b_check = bc::buffer(context, sizeof(cl_ulong));

  for (uint8_t s = 0; s < hash_sizes; ++s) {

    std::cerr << "Linear Probing --> Hash Size "
              << human_bytes(sizeof(uint64_t), hash_size[s]) << std::endl;

    // do the linear probing
    for (uint16_t r = 0; r < repetitions; ++r) {

      // generate test data
      Table *inner = new Table();
      Table *outer = new Table();
      HashTable *tab = new HashTable();

      generate_tables(table_count, hash_size[s], table_loadfactor,
                      table_selectivity, inner, outer, tab);
      build_hash(inner, tab, true);
      join_size_checksum(outer, tab, &tab->join_count, &tab->checksum);

      // copy test data
      cl_ulong outer_count = (cl_ulong)(outer->size);
      bc::buffer b_outer(context, outer_count * sizeof(cl_uint));
      command.enqueue_write_buffer(b_outer, 0, outer_count * sizeof(cl_uint),
                                   &outer->items[0]);

      cl_uint tab_count = (cl_uint)(tab->size);
      bc::buffer b_tab(context, tab_count * sizeof(cl_ulong));
      command.enqueue_write_buffer(b_tab, 0, tab_count * sizeof(cl_ulong),
                                   &tab->buckets[0]);

      cl_ulong out_count = (cl_ulong)(outer->size);
      bc::buffer b_out = bc::buffer(context, out_count * sizeof(cl_uint));

      // create kernel and set arguments
      kernel.set_arg(0, b_outer);
      kernel.set_arg(1, outer_count);
      kernel.set_arg(2, b_tab);
      kernel.set_arg(3, tab_count);
      kernel.set_arg(4, b_out);
      kernel.set_arg(5, tab->empty);

      // init output table
      init_kernel.set_arg(0, b_out);
      init_kernel.set_arg(1, out_count);
      init_kernel.set_arg(2, tab->empty);
      bc::event init_event =
          command.enqueue_1d_range_kernel(init_kernel, 0, 1, 1);
      init_event.wait();

      // run kernel
      bc::event event =
          command.enqueue_1d_range_kernel(kernel, 0, global, local);
      event.wait();

      // check result
      cl_ulong check_sum = 0;
      check_kernel.set_arg(0, b_out);
      check_kernel.set_arg(1, out_count);
      check_kernel.set_arg(2, tab->empty);
      check_kernel.set_arg(3, b_check);
      bc::event check_event =
          command.enqueue_1d_range_kernel(check_kernel, 0, 1, 1);
      check_event.wait();
      command.enqueue_read_buffer(b_check, 0, sizeof(cl_ulong), &check_sum);
      assert(check_sum == tab->checksum);

      // get and save running time
      Timestamp time_in_ns = event.duration<std::chrono::nanoseconds>().count();
      std::cout << opcl_kernel << ";";
      std::cout << deviceType(device.type()) << ";";
      std::cout << human_bytes(sizeof(uint64_t), tab->size) << ";";
      std::cout << tab->size << ";";
      std::cout << "(" << global << "," << local << ")";
      std::cout << ";" << outer->size * 1.0 / time_in_ns;
      std::cout << ";" << time_in_ns << std::endl;

      delete inner->items_vec;
      delete outer->items_vec;
      delete tab->buckets_vec;

      delete inner;
      delete outer;
      delete tab;
    }
  }

  std::cerr << "Linear Probing '" << opcl_kernel << "' --> end" << std::endl;
}

/**
 * Use the functions from Polychroniou to generate test data for given
 * parameters.
 */
void generate_tables(size_t probe_size, uint32_t hash_size, double load_factor,
                     double selectivity, Table *inner_tab, Table *outer_tab,
                     HashTable *h_tab) {

  assert(sizeof(uint32_t) == sizeof(Item));
  assert(sizeof(uint64_t) == sizeof(Bucket));

  // determine size of build table
  size_t build_size = load_factor * hash_size;

  // generate build and probe table by c-program from Orestis Polychroniou
  uint32_t *build_p = NULL;
  uint32_t *probe_p = NULL;
  inner_outer(build_size, probe_size, selectivity, &build_p, &probe_p);

  // set sizes and empty elements (invalid key)
  inner_tab->size = build_size;
  outer_tab->size = probe_size;
  inner_tab->empty = build_p[build_size];
  outer_tab->empty = build_p[build_size];

  // Assign tables to aligned vectors
  AlignedVector<Item> *inner_vec = new AlignedVector<Item>(build_size);
  AlignedVector<Item> *outer_vec = new AlignedVector<Item>(probe_size);

  for (size_t b = 0; b < build_size; ++b) {
    inner_vec->at(b) = build_p[b];
  }

  for (size_t p = 0; p < probe_size; ++p) {
    outer_vec->at(p) = probe_p[p];
  }

  // set vars of h_table
  h_tab->size = hash_size;

  inner_tab->items_vec = inner_vec;
  outer_tab->items_vec = outer_vec;
  inner_tab->items = &inner_vec->at(0);
  outer_tab->items = &outer_vec->at(0);

  free(build_p);
  free(probe_p);
}

/**
 * Build Hash Table *h_tab* for items in *inner_tab* using multiply shift hash.
 * The size set in HashTable structure is used as the number of buckets
 * for the new hash table. Empty element has to be set for inner_tab.
 *
 */
void build_hash(Table *inner_tab, HashTable *h_tab, bool save_hash_key) {

  assert(sizeof(uint32_t) == sizeof(Item));
  assert(sizeof(uint64_t) == sizeof(Bucket));

  // determine number of bits of hashed key
  h_tab->hash_bits = 0;
  uint32_t h_size = h_tab->size;
  while (h_size >>= 1) {
    ++h_tab->hash_bits;
  }

  // set all buckets to empty element
  AlignedVector<Bucket> *h_vec = new AlignedVector<Bucket>(h_tab->size);

  if (save_hash_key) {
    h_tab->empty = inner_tab->empty;
  } else {
    // for the operations benchmark we do not hash the key
    // --> just use the sufficient bits of generated key
    h_tab->empty = (inner_tab->empty & ((1 << h_tab->hash_bits) - 1));
  }

  Bucket empty_bucket;
  empty_bucket.key = h_tab->empty;
  empty_bucket.val = h_tab->empty;

  for (uint32_t h = 0; h < h_tab->size; ++h) {
    h_vec->at(h) = (empty_bucket);
  }

  // hash each key and save in h_tab
  for (size_t i = 0; i < inner_tab->size; ++i) {

    Item key = inner_tab->items[i];
    Item val = inner_tab->items[i] * VALUE_FACTOR;
    uint32_t h = (uint32_t)(key * MULTIPLY_SHIFT_CONSTANT);
    h >>= 32 - h_tab->hash_bits;

    while (h_vec->at(h).key != h_tab->empty) {
      h = (h + 1) & (h_tab->size - 1);
    }

    if (save_hash_key) {
      h_vec->at(h).key = key;
    } else {
      // for the operations benchmark we do not hash the key
      // --> just use the sufficient bits of generated key
      h_vec->at(h).key = (key & ((1 << h_tab->hash_bits) - 1));
    }
    h_vec->at(h).val = val;
  }

  h_tab->buckets_vec = h_vec;
  h_tab->buckets = &h_vec->at(0);
}

/**
 * Figure out join size and sum up checksum.
 */
void join_size_checksum(const Table *outer, const HashTable *tab,
                        uint64_t *join_size, uint64_t *checksum) {

  assert(sizeof(Item) == sizeof(int32_t));
  assert(sizeof(Bucket) == sizeof(uint64_t));

  uint64_t sum = 0;
  uint64_t count = 0;

  for (size_t i = 0; i < outer->size; ++i) {

    uint32_t key = outer->items[i];
    uint32_t h = (uint32_t)(key * MULTIPLY_SHIFT_CONSTANT);
    h >>= 32 - tab->hash_bits;

    while (tab->buckets[h].key != tab->empty) {
      if (tab->buckets[h].key == key) {
        sum += tab->buckets[h].val;
        count++;
        break;
      }
      h = (h + 1) & (tab->size - 1);
    }
  }

  *join_size = count;
  *checksum = sum;
}

/**
 * Get first Intel OpenCL Device ID.
 * If there is a Xeon Phi, choose this device.
 * Otherwise we choose the first Intel CPU.
 */
cl_device_id get_intel_device_id() {

  // get platforms
  cl_uint platform_count;
  clGetPlatformIDs(0, NULL, &platform_count);
  cl_platform_id platform_ids[platform_count];
  clGetPlatformIDs(platform_count, &platform_ids[0], NULL);

  // to check if platform is Intel
  // https://software.intel.com/en-us/node/540385
  char intel[16];
  strcpy(intel, "Intel(R) OpenCL");

  // iterate over platforms
  for (cl_uint p = 0; p < platform_count; ++p) {

    char buf[16];
    clGetPlatformInfo(platform_ids[p], CL_PLATFORM_NAME, sizeof(buf), buf,
                      NULL);
    if (strcmp(buf, intel) == 0) {

      cl_uint mic_count = 0;
      clGetDeviceIDs(platform_ids[p], CL_DEVICE_TYPE_ACCELERATOR, 0, NULL,
                     &mic_count);
      if (mic_count > 0 && 0) {
        cl_device_id mic;
        clGetDeviceIDs(platform_ids[p], CL_DEVICE_TYPE_ACCELERATOR, 1, &mic,
                       NULL);
        return mic;
      }

      cl_uint cpu_count = 0;
      clGetDeviceIDs(platform_ids[p], CL_DEVICE_TYPE_CPU, 0, NULL, &cpu_count);
      if (cpu_count > 0) {
        cl_device_id cpu;
        clGetDeviceIDs(platform_ids[p], CL_DEVICE_TYPE_CPU, 1, &cpu, NULL);
        return cpu;
      }
    }
  }
  return NULL;
}

/**
 * Get device type string.
 */
const std::string deviceType(const cl_device_type &type) {
  switch (type) {
  case CL_DEVICE_TYPE_CPU:
    return "CPU";
    break;
  case CL_DEVICE_TYPE_ACCELERATOR:
    return "Phi";
    break;
  default:
    return "Default";
  }
}

/**
 * Returns human readable string of size of *items* elements,
 * each of size *bytes* in a suitable unit.
 */
const char *human_bytes(size_t items, size_t bytes) {

  const char *suffix[4];
  suffix[0] = "B";
  suffix[1] = "KB";
  suffix[2] = "MB";
  suffix[3] = "GB";

  volatile uint8_t s;
  volatile size_t count = items * bytes;
  for (s = 0; s < 4; ++s) {
    if (count >= 1024) {
      count = (size_t)(count / 1024);
    } else {
      break;
    }
  }

  static char buf[16];
  sprintf(buf, "%d %s", (int)count, suffix[s]);
  return buf;
}

/**
 * Get current timestam prepresented in nanoseconds.
 */
Timestamp get_timestamp() {
  return std::chrono::duration_cast<NanoSeconds>(
             Clock::now().time_since_epoch())
      .count();
}
