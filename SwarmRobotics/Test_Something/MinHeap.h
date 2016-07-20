#pragma once

#include <iostream>
#include <cstdlib>
#include <vector>
#include <iterator>
#include "SearchNode.h"

class MinHeap
{
private:

	std::vector <SearchNode> heap;
	int left(int parent);
	int right(int parent);
	int parent(int child);
	void heapifyup(int index);
	void heapifydown(int index);

public:
	MinHeap() {};
	void insert(SearchNode element);
	void deleteMin();
	SearchNode extractMin();
	int size();
};

