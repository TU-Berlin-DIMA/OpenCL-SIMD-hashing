# Efficient SIMD Vectorization for Hashing in OpenCL
This repository contains our work on efficient SIMD vectorization methods for hashing in OpenCL.
It was first published at the [21th International Conference on Extending Database Technology (EDBT)](http://edbticdt2018.at/) in March 2018.  

**Abstract:** Hashing is at the core of many efficient database operators such as hash-based joins and aggregations. Vectorization is a technique that uses Single Instruction Multiple Data (SIMD) instructions to process multiple data elements at once. Applying vectorization to hash tables results in promising speedups for build and probe operations. However,  vectorization typically requires intrinsics – low-level APIs in which functions map to processorspecific SIMD instructions. Intrinsics are specific to a processor architecture and result in complex and difficult to maintain code.
OpenCL is a parallel programming framework which provides a higher abstraction level than intrinsics and is portable to different processors. Thus, OpenCL avoids processor dependencies, which results in improved code maintainability. In this paper, we add efficient, vectorized hashing primitives to OpenCL. Our results show that OpenCL-based vectorization is competitive to intrinsics on CPUs but not on Xeon Phi coprocessors.

**Publication:**
- Paper: [Efficient SIMD Vectorization for Hashing in OpenCL](https://github.com/TU-Berlin-DIMA/OpenCL-SIMD-hashing/blob/master/paper/Efficient-SIMD-Vectorization-for-Hashing-in-OpenCL.pdf)
- Poster: [EDBT 2018 Poster](https://github.com/TU-Berlin-DIMA/OpenCL-SIMD-hashing/blob/master/poster/EDBT-2018-Efficient-SIMD-Vectorization-for-Hashing-in-OpenCL.pdf)

- BibTeX citation:
```
@inproceedings{behrens2018efficient,
  title={Efficient SIMD Vectorization for Hashing in OpenCL},
  author={Behrens, Tobias and Rosenfeld, Viktor and Traub, Jonas and Bre{\ss}, Sebastian and Markl, Volker},
  booktitle={21th International Conference on Extending Database Technology (EDBT)},
  year={2018}
}
```
