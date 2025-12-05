#include "ListGraph.h"
#include <iostream>
#include <utility>

// constructor - initialize adjacency list
ListGraph::ListGraph(bool type, int size) : Graph(type, size) {
  m_List = new map<int, int>[m_Size]; // create array where each vertex has its
                                      // own map
}

// destructor - free memory
ListGraph::~ListGraph() { delete[] m_List; }

// get adjacent edges for undirected graph
// for undirected graph, return all edges connected to vertex
void ListGraph::getAdjacentEdges(int vertex, map<int, int> *m) {
  // if invalid vertex index is given, terminate the function
  if (vertex < 0 || vertex >= m_Size) {
    return;
  }

  // copy all edges from vertex to m
  map<int, int>::iterator it;
  for (it = m_List[vertex].begin(); it != m_List[vertex].end(); it++) {
    (*m)[it->first] = it->second;
  }
}

// get adjacent edges for directed graph
// for directed graph, return only outgoing edges
void ListGraph::getAdjacentEdgesDirect(int vertex, map<int, int> *m) {
  // if invalid vertex index is given, terminate the function
  if (vertex < 0 || vertex >= m_Size) {
    return;
  }

  // copy all edges from vertex to m
  map<int, int>::iterator it;
  for (it = m_List[vertex].begin(); it != m_List[vertex].end(); it++) {
    (*m)[it->first] = it->second;
  }
}

// insert edge into adjacency list
void ListGraph::insertEdge(int from, int to, int weight) {
  // if source or destination vertex index is out of range, terminate the
  // function
  if (from < 0 || from >= m_Size || to < 0 || to >= m_Size) {
    return;
  }

  // insert edge from -> to with weight
  m_List[from][to] = weight;
}

// print graph in adjacency list format
bool ListGraph::printGraph(ofstream *fout) {
  if (!fout) {
    return false;
  }

  // print each vertex and its adjacent edges
  for (int i = 0; i < m_Size; i++) {
    (*fout) << "[" << i << "]";

    // print adjacent edges in ascending order by destination vertex
    map<int, int>::iterator it;
    for (it = m_List[i].begin(); it != m_List[i].end(); it++) {
      (*fout) << " -> (" << it->first << "," << it->second << ")";
    }
    (*fout) << endl;
  }
  return true;
}