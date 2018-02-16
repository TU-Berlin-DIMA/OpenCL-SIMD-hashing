CC=gcc
CXX=g++

CFLAGS=-c -O3 -fno-tree-vectorize -I./include
CXXFLAGS=$(CFLAGS) -march=native -std=c++11

CLIBS=-lpthread -lrt -lm
CXXLIBS=$(CLIBS) -lOpenCL

METHOD=0
DIR=0

SRC=src
INC=include
OBJ=obj

all:	dir data-generation opencl_avx

dir:
	mkdir -p $(OBJ)

# OpenCL AVX
opencl_avx: data-generation $(OBJ)/opencl_avx.o $(OBJ)/avx.o
	$(CXX) -o opencl_avx \
	$(OBJ)/inner_outer.o $(OBJ)/rand.o $(OBJ)/opencl_avx.o $(OBJ)/avx.o $(CXXLIBS)

$(OBJ)/opencl_avx.o: $(SRC)/opencl_avx.cpp $(INC)/opencl_avx.hpp
	$(CXX) $(CXXFLAGS) -o $(OBJ)/opencl_avx.o $(SRC)/opencl_avx.cpp -D$(METHOD) -D_DIR=\"$(DIR)/src/\"

$(OBJ)/avx.o: $(SRC)/avx.cpp $(INC)/avx.hpp
	$(CXX) $(CXXFLAGS) -o $(OBJ)/avx.o $(SRC)/avx.cpp

# C data generation code by polychroniou
data-generation: $(OBJ)/rand.o $(OBJ)/inner_outer.o

$(OBJ)/rand.o: $(SRC)/rand.c $(INC)/rand.h
	$(CC) $(CFLAGS) -o $(OBJ)/rand.o $(SRC)/rand.c

$(OBJ)/inner_outer.o: $(SRC)/inner_outer.c $(INC)/inner_outer.h
	$(CC) $(CFLAGS) -o $(OBJ)/inner_outer.o $(SRC)/inner_outer.c

clean:
	rm -rf $(OBJ)
	rm -f opencl_avx
