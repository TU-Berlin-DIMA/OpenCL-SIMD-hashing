CC=gcc
CXX=g++

CFLAGS=-c -O3 -fno-tree-vectorize -I./include
CXXFLAGS=$(CFLAGS) -march=native -std=c++11

CLIBS=-lpthread -lrt -lm
CXXLIBS=$(CLIBS) -lOpenCL

DIR=0

SRC=src
INC=include
OBJ=obj

all:	dir data-generation opencl

dir:
	mkdir -p $(OBJ)

# OpenCL
opencl: data-generation $(OBJ)/opencl.o
	$(CXX) -o opencl \
	$(OBJ)/inner_outer.o $(OBJ)/rand.o $(OBJ)/opencl.o $(CXXLIBS)

$(OBJ)/opencl.o: $(SRC)/opencl.cpp $(INC)/opencl.hpp
	$(CXX) $(CXXFLAGS) -o $(OBJ)/opencl.o $(SRC)/opencl.cpp -D_DIR=\"$(DIR)/src/\"

# C data generation code by Polychroniou
data-generation: $(OBJ)/rand.o $(OBJ)/inner_outer.o

$(OBJ)/rand.o: $(SRC)/rand.c $(INC)/rand.h
	$(CC) $(CFLAGS) -o $(OBJ)/rand.o $(SRC)/rand.c

$(OBJ)/inner_outer.o: $(SRC)/inner_outer.c $(INC)/inner_outer.h
	$(CC) $(CFLAGS) -o $(OBJ)/inner_outer.o $(SRC)/inner_outer.c

clean:
	rm -rf $(OBJ)
	rm -f opencl
