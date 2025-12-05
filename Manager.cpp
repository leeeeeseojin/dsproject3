#include "Manager.h"
#include "GraphMethod.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// constructor - initialize manager
Manager::Manager() {
  graph = nullptr;
  fout.open("log.txt", ios::app);
  load = 0; // Anything is not loaded
}

// destructor - free memory and close file
Manager::~Manager() {
  if (load) // if graph is loaded, delete graph
    delete graph;
  if (fout.is_open()) // if fout is opened, close file
    fout.close();     // close log.txt File
}

// read commands from file and execute
void Manager::run(const char *command_txt) {
  ifstream fin;                        // Command File Input File Stream
  fin.open(command_txt, ios_base::in); // File open with read mode

  if (!fin) { // If command File cannot be read, Print error
    fout << "command file open error" << endl;
    return; // Return
  }

  string command;

  // read commands line by line
  // call appropriate function
  while (fin >> command) {
    if (command == "LOAD") {
      string filename;
      fin >> filename;
      LOAD(filename.c_str());
    }

    else if (command == "PRINT") {
      PRINT();
    }

    else if (command == "BFS") {
      char option;
      int vertex;
      fin >> option >> vertex;
      mBFS(option, vertex);
    }

    else if (command == "DFS") {
      char option;
      int vertex;
      fin >> option >> vertex;
      mDFS(option, vertex);
    }

    else if (command == "KRUSKAL") {
      mKRUSKAL();
    }

    else if (command == "DIJKSTRA") {
      char option;
      int vertex;
      fin >> option >> vertex;
      mDIJKSTRA(option, vertex);
    }

    else if (command == "BELLMANFORD") {
      char option;
      int s_vertex, e_vertex;
      fin >> option >> s_vertex >> e_vertex;
      mBELLMANFORD(option, s_vertex, e_vertex);
    }

    else if (command == "FLOYD") {
      char option;
      fin >> option;
      mFLOYD(option);
    }

    else if (command == "CENTRALITY") {
      mCentrality();
    }

    else if (command == "EXIT") {
      fout << "========EXIT========" << endl;
      fout << "Success" << endl;
      fout << "=======================" << endl;
      break;
    }
  }

  fin.close();
  return;
}

// load graph data from file
// read graph type, size, edge information
// create ListGraph or MatrixGraph based on type
bool Manager::LOAD(const char *filename) {
  ifstream fin;
  fin.open(filename);

  // if file cannot be opened, print error code 100
  if (!fin) {
    printErrorCode(100);
    return false;
  }

  // if graph already exists, delete old graph
  if (load) {
    delete graph;
    graph = nullptr;
    load = 0;
  }

  char type;
  int size;

  fin >> type >> size; // read graph type - L or M
                       // read graph size - number of vertices

  // create graph based on type
  if (type == 'L') {
    // create ListGraph for adjacency list
    graph = new ListGraph(true, size);

    int vertex = -1;
    string line;

    getline(fin, line); // skip remaining characters in size line

    // read adjacency list data
    // vertex number in one line -> edge info in next line
    while (getline(fin, line)) {
      // remove whitespace-only lines
      if (line.find_first_not_of(" \t\r\n") == string::npos) {
        continue;
      }

      stringstream ss(line);
      vector<int> nums;
      int value;

      // read all numbers in the current line
      while (ss >> value) {
        nums.push_back(value);
      }

      // if only one number exists -> vertex index line
      if (nums.size() == 1) {
        vertex = nums[0];
        continue;
      }

      if (vertex == -1) {
        continue;
      }

      // if line has pairs of numbers -> edges
      for (int i = 0; i + 1 < nums.size(); i += 2) {
        int dest = nums[i];
        int weight = nums[i + 1];
        graph->insertEdge(vertex, dest, weight);
      }
    }
  } else if (type == 'M') {
    // create MatrixGraph for adjacency matrix
    graph = new MatrixGraph(true, size);

    // read adjacency matrix data
    for (int i = 0; i < size; i++) {
      for (int j = 0; j < size; j++) {
        int weight;
        fin >> weight;

        // if weight is non-zero, insert edge
        if (weight != 0) {
          graph->insertEdge(i, j, weight);
        }
      }
    }
  } else {
    // invalid type
    fin.close();
    printErrorCode(100);
    return false;
  }

  fin.close();
  load = 1;

  fout << "========LOAD========" << endl;
  fout << "Success" << endl;
  fout << "=======================" << endl;

  return true;
}

// print graph information
// if graph is loaded, print adjacency list or matrix
bool Manager::PRINT() {
  // if graph is not loaded, print error code 200
  if (!load || !graph) {
    printErrorCode(200);
    return false;
  }

  fout << "========PRINT========" << endl;
  graph->printGraph(&fout);
  fout << "=======================" << endl;

  return true;
}

// functions that check graph state and forward the call to algorithm
// check if graph exists and call BFS algorithm
bool Manager::mBFS(char option, int vertex) {
  // if graph is not loaded or vertex is invalid, print error code 300
  if (!load || !graph || vertex < 0 || vertex >= graph->getSize()) {
    printErrorCode(300);
    return false;
  }

  // call BFS algorithm
  if (!BFS(graph, option, vertex)) {
    printErrorCode(300);
    return false;
  }

  return true;
}

// check if graph exists and call DFS algorithm
bool Manager::mDFS(char option, int vertex) {
  // if graph is not loaded or vertex is invalid, print error code 400
  if (!load || !graph || vertex < 0 || vertex >= graph->getSize()) {
    printErrorCode(400);
    return false;
  }

  // call DFS algorithm
  if (!DFS(graph, option, vertex)) {
    printErrorCode(400);
    return false;
  }

  return true;
}

// check if graph exists and call Dijkstra algorithm
bool Manager::mDIJKSTRA(char option, int vertex) {
  // if graph is not loaded or vertex is invalid, print error code 600
  if (!load || !graph || vertex < 0 || vertex >= graph->getSize()) {
    printErrorCode(600);
    return false;
  }

  // call Dijkstra algorithm
  if (!Dijkstra(graph, option, vertex)) {
    printErrorCode(600);
    return false;
  }

  return true;
}

// check if graph exists and call Kruskal algorithm
bool Manager::mKRUSKAL() {
  // if graph is not loaded, print error code 500
  if (!load || !graph) {
    printErrorCode(500);
    return false;
  }

  // call Kruskal algorithm
  if (!Kruskal(graph)) {
    printErrorCode(500);
    return false;
  }

  return true;
}

// check if graph exists and call Bellman-Ford algorithm
bool Manager::mBELLMANFORD(char option, int s_vertex, int e_vertex) {
  // if graph is not loaded or vertices are invalid, print error code 700
  if (!load || !graph || s_vertex < 0 || s_vertex >= graph->getSize() ||
      e_vertex < 0 || e_vertex >= graph->getSize()) {
    printErrorCode(700);
    return false;
  }

  // call Bellman-Ford algorithm
  if (!Bellmanford(graph, option, s_vertex, e_vertex)) {
    printErrorCode(700);
    return false;
  }

  return true;
}

// check if graph exists and call Floyd algorithm
bool Manager::mFLOYD(char option) {
  // if graph is not loaded, print error code 800
  if (!load || !graph) {
    printErrorCode(800);
    return false;
  }

  // call Floyd algorithm
  if (!FLOYD(graph, option)) {
    printErrorCode(800);
    return false;
  }

  return true;
}

// check if graph exists and call Centrality algorithm
bool Manager::mCentrality() {
  // if graph is not loaded, print error code 900
  if (!load || !graph) {
    printErrorCode(900);
    return false;
  }

  // call Centrality algorithm
  if (!Centrality(graph)) {
    printErrorCode(900);
    return false;
  }

  return true;
}

void Manager::printErrorCode(int n) {
  fout << "========ERROR=======" << endl;
  fout << n << endl;
  fout << "====================" << endl << endl;
}
