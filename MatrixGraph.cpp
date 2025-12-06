#include "MatrixGraph.h"
#include <iostream>
#include <string>
#include <vector>

// constructor - initialize adjacency matrix
MatrixGraph::MatrixGraph(bool type, int size) : Graph(type, size) {
  // allocate memory for adjacency matrix - array of row pointer
  m_Mat = new int *[size];

  // allocate each row and initialize all elements to 0
  for (int i = 0; i < size; i++) {
    m_Mat[i] = new int[size]; // allocate a row
    for (int j = 0; j < size; j++) {
      m_Mat[i][j] = 0; // initialize elements to 0
    }
  }
}

// destructor - free memory
MatrixGraph::~MatrixGraph() {
  // dellocate each row of adjacency matrix
  for (int i = 0; i < m_Size; i++) {
    delete[] m_Mat[i];
  }

  // deallocate array of row pointer
  delete[] m_Mat;
}

// get adjacent edges for undirected graph
// for undirected graph, return edges in both directions
void MatrixGraph::getAdjacentEdges(int vertex, map<int, int> *m) {
  // if invalid vertex index is given, terminate the function
  if (vertex < 0 || vertex >= m_Size) {
    return;
  }

  // find all non-zero edges from vertex (outgoing)
  for (int i = 0; i < m_Size; i++) {
    // if there is edge from vertex to i
    // store it in map
    if (m_Mat[vertex][i] != 0) {
      (*m)[i] = m_Mat[vertex][i];
    }
  }

  // for undirected graph, also check incoming edges (reverse direction)
  for (int i = 0; i < m_Size; i++) {
    // if there is edge from i to vertex
    if (m_Mat[i][vertex] != 0) {
      // only add if not already present
      if (m->find(i) == m->end()) {
        (*m)[i] = m_Mat[i][vertex];
      }
    }
  }
}

// get adjacent edges for directed graph
// for directed graph, return only outgoing edge
void MatrixGraph::getAdjacentEdgesDirect(int vertex, map<int, int> *m) {
  // if invalid vertex index is given, terminate the function
  if (vertex < 0 || vertex >= m_Size) {
    return;
  }

  // find all non-zero edges from vertex
  for (int i = 0; i < m_Size; i++) {
    // if there is edge from vertex to i
    // store it in map
    if (m_Mat[vertex][i] != 0) {
      (*m)[i] = m_Mat[vertex][i];
    }
  }
}

// insert edge into adjacency matrix
void MatrixGraph::insertEdge(int from, int to, int weight) {
  // if source or destination vertex index is out of range, terminate the
  // function
  if (from < 0 || from >= m_Size || to < 0 || to >= m_Size) {
    return;
  }

  // insert edge from -> to with weight
  m_Mat[from][to] = weight;
}

// print graph in adjacency matrix format
bool MatrixGraph::printGraph(ofstream *fout) {
  if (!fout) {
    return false;
  }

  // print header row with proper spacing
  (*fout) << "    ";
  for (int i = 0; i < m_Size; i++) {
    (*fout) << "[" << i << "] ";
  }
  (*fout) << endl;

  // print matrix rows
  for (int i = 0; i < m_Size; i++) {
    (*fout) << "[" << i << "] ";
    for (int j = 0; j < m_Size; j++) {
      (*fout) << m_Mat[i][j];
      if (j < m_Size - 1) {
        (*fout) << "   ";
      }
    }
    (*fout) << endl;
  }

  return true;
}