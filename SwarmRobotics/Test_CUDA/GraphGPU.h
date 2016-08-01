#ifndef GRAPH_GPU_H
#define GRAPH_GPU_H

#include <iostream>
#include <fstream>
#include <vector>

#include "thrust/host_vector.h"
#include "thrust/device_vector.h"

void test_Thrust();

void test_Thrust_v1();
void test_Thrust_v2();
void test_Thrust_v3();

void test_Thrust_toCUDA();
void test_Thrust_fromCUDA();

#endif