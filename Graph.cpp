#include "Graph.h"

Graph::Graph(bool type, int size) {
  m_Type = type;
  m_Size = size;
}

Graph::~Graph() {} // memory deallocation is handled by derived class

bool Graph::getType() { return m_Type; }
int Graph::getSize() { return m_Size; }