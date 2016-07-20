//*************************************//
// Binary Heap for the open list
//*************************************//

#include "stdafx.h"
#include "MinHeap.h"

/*
 * Return Heap Size
 */
int MinHeap::size()
{
    return heap.size();
}

 
/*
 * Insert Element into a Heap
 */
void MinHeap::insert(SearchNode element)
{
    heap.push_back(element);
    heapifyup(heap.size() -1);
}

/*
 * Delete Minimum Element
 */
void MinHeap::deleteMin()
{
    if (heap.size() == 0)
    {
        std::cout << "Heap is Empty" << std::endl;
        return;
    }

    heap[0] = heap.at(heap.size() - 1);
    heap.pop_back();
    heapifydown(0);
}
 

/*
 * Extract Minimum Element
 */
SearchNode MinHeap::extractMin()
{
    if (heap.size() == 0)
    {
        return SearchNode(nullptr,0,0,nullptr);
    } else
        return heap.front();
}
 

/*
 * Return Left Child
 */
int MinHeap::left(int parent)
{
    int l = 2 * parent + 1;
    if (l < heap.size())
        return l;
    else
        return -1;
}
 

/*
 * Return Right Child
 */
int MinHeap::right(int parent)
{
    int r = 2 * parent + 2;
    if (r < heap.size())
        return r;
    else
        return -1;
}

 

/*
 * Return Parent
 */
int MinHeap::parent(int child)
{
    int p = (child - 1)/2;
    if (child == 0)
        return -1;
    else
        return p;
}

 

/*
 * Heapify- Maintain Heap Structure bottom up
 */
void MinHeap::heapifyup(int in)
{
    if (in >= 0 && parent(in) >= 0 && heap[parent(in)].cost > heap[in].cost)
    {
        SearchNode temp = heap[in];
        heap[in] = heap[parent(in)];
        heap[parent(in)] = temp;
        heapifyup(parent(in));
    }
}

 

/*
 * Heapify- Maintain Heap Structure top down
 */
void MinHeap::heapifydown(int in)
{
    int child = left(in);
    int child1 = right(in);

    if (child >= 0 && child1 >= 0 && heap[child].cost > heap[child1].cost)
    {
       child = child1;
    }

    if (child > 0)
    {
        SearchNode temp = heap[in];
        heap[in] = heap[child];
        heap[child] = temp;
        heapifydown(child);
    }
}