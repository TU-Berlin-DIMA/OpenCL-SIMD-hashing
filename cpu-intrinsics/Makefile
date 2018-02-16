CC=gcc
CFLAGS=-O3 -march=core-avx2 -fno-tree-vectorize
CLIBS=-lpthread -lrt -lm

all:	avx_phi_probe_linear.c phi_build_probe_linear.c  \
	    inner_outer.c inner_outer.h rand.c rand.h
	$(CC) $(CFLAGS) $(AVX_CFLAGS) -o cpu_probe avx_phi_probe_linear.c inner_outer.c rand.c $(CLIBS)
	$(CC) $(CFLAGS) $(AVX_CFLAGS) -o cpu_build phi_build_probe_linear.c inner_outer.c rand.c $(CLIBS) -D _BUILD_ONLY

clean:
	rm -f xeon_probe xeon_build
