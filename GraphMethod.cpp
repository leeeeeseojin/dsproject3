#include "GraphMethod.h"
#include <fstream>
#include <iostream>
#include <list>
#include <map>
#include <queue>
#include <set>
#include <stack>
#include <utility>
#include <vector>

using namespace std;

// find operation
// find the root of the set containing x
int find(int *parent, int x) {
  if (parent[x] == x) {
    return x;
  }

  return parent[x] = find(
             parent,
             parent[x]); // recursively find root and update parent[x] to root
};

// union operation
// merge two sets containing x and y
void unite(int *parent, int x, int y) {
  int rootX = find(parent, x); // find root of the set containing x
  int rootY = find(parent, y); // find root of the set containing y

  // make rootY point to rootX
  if (rootX != rootY) {
    parent[rootY] = rootX;
  }
};

// visit start vertex and enqueue
// repeatedly remove a vertex from queue, visit its unvisited adjacent vertices
bool BFS(Graph *graph, char option, int vertex) {
  // if graph is null, return false
  if (!graph) {
    return false;
  }

  int size = graph->getSize();
  ofstream fout;
  fout.open("log.txt", ios::app);

  // initialize visited array
  bool *visited = new bool[size];
  for (int i = 0; i < size; i++) {
    visited[i] = false;
  }

  queue<int> q; // create queue for BFS

  // mark start vertex as visited and enqueue
  visited[vertex] = true;
  q.push(vertex);

  vector<int> order; // store traversal order

  // BFS main loop
  while (!q.empty()) {
    int current = q.front(); // dequeue a vertex from queue
    q.pop();
    order.push_back(current); // add to traversal order => record visit order

    // get adjacent edges based on graph type
    map<int, int> edges;
    if (option == 'Y' || option == 'O') {
      // directed graph - use direct edges only
      graph->getAdjacentEdgesDirect(current, &edges);
    } else {
      // undirected graph - use all edges
      graph->getAdjacentEdges(current, &edges);
    }

    // visit all unvisited adjacent vertices in ascending order
    map<int, int>::iterator it;
    for (it = edges.begin(); it != edges.end(); it++) {
      int next = it->first;

      // if not visited, mark and enqueue
      if (!visited[next]) {
        visited[next] = true;
        q.push(next);
      }
    }
  }

  // print BFS result
  fout << "========BFS========" << endl;
  if (option == 'Y' || option == 'O') {
    fout << "Directed Graph BFS" << endl;
  } else {
    fout << "Undirected Graph BFS" << endl;
  }
  fout << "Start: " << vertex << endl;

  // print traversal order
  for (int i = 0; i < order.size(); i++) {
    fout << order[i];
    if (i < order.size() - 1) {
      fout << " -> ";
    }
  }
  fout << endl;
  fout << "=======================" << endl;

  // free memory
  delete[] visited;
  fout.close();

  return true;
}

// visit start vertex and put into a stack
// repeatedly pop a vertex, mark as visited, push unvisited adjacent vertices
bool DFS(Graph *graph, char option, int vertex) {
  // if graph is null, return false
  if (!graph) {
    return false;
  }

  int size = graph->getSize();
  ofstream fout;
  fout.open("log.txt", ios::app);

  // initialize visited array
  bool *visited = new bool[size];
  for (int i = 0; i < size; i++) {
    visited[i] = false;
  }

  stack<int> s;      // create stack for DFS
  s.push(vertex);    // push start vertex to stack
  vector<int> order; // store traversal order

  // DFS main loop
  while (!s.empty()) {
    // pop a vertex from stack
    int current = s.top();
    s.pop();

    // if already visited, skip
    if (visited[current]) {
      continue;
    }

    // mark as visited and add to order
    visited[current] = true;
    order.push_back(current);

    map<int, int> edges;
    if (option == 'Y' || option == 'O') {
      // directed graph - use direct edges only
      graph->getAdjacentEdgesDirect(current, &edges);
    } else {
      // undirected graph - use all edges
      graph->getAdjacentEdges(current, &edges);
    }

    // push unvisited adjacent vertices to stack in descending order
    // => pop -> ascending order
    map<int, int>::reverse_iterator rit;
    for (rit = edges.rbegin(); rit != edges.rend(); rit++) {
      int next = rit->first;

      // if not visited, push to stack
      if (!visited[next]) {
        s.push(next);
      }
    }
  }

  // print DFS result
  fout << "========DFS========" << endl;
  if (option == 'Y' || option == 'O') {
    fout << "Directed Graph DFS" << endl;
  } else {
    fout << "Undirected Graph DFS" << endl;
  }
  fout << "Start: " << vertex << endl;

  // print traversal order
  for (int i = 0; i < order.size(); i++) {
    fout << order[i];
    if (i < order.size() - 1) {
      fout << " -> ";
    }
  }
  fout << endl;
  fout << "=======================" << endl;

  // free memory
  delete[] visited;
  fout.close();

  return true;
}

// use union-find to detect cycle
// sort edges by weight and add edges that don't form cycles
bool Kruskal(Graph *graph) {
  // if graph is null, return false
  if (!graph) {
    return false;
  }

  int size = graph->getSize();
  ofstream fout;
  fout.open("log.txt", ios::app);

  vector<pair<int, pair<int, int>>> edges; // collect all edges from graph

  for (int i = 0; i < size; i++) {
    map<int, int> adj;
    graph->getAdjacentEdgesDirect(i, &adj);

    map<int, int>::iterator it;
    for (it = adj.begin(); it != adj.end(); it++) {
      int to = it->first;
      int weight = it->second;

      edges.push_back(make_pair(
          weight, make_pair(i, to))); // store edge - (weight, (from, to))
    }
  }

  // sort edges by weight in ascending order
  sort(edges.begin(), edges.end());

  // initialize disjoint set
  int *parent = new int[size];
  for (int i = 0; i < size; i++) {
    parent[i] = i;
  }

  // store selected edges for MST
  vector<pair<int, pair<int, int>>> mst;
  int totalCost = 0;

  // start with lower weighted edges
  for (int i = 0; i < edges.size(); i++) {
    int weight = edges[i].first;
    int from = edges[i].second.first;
    int to = edges[i].second.second;

    // root node navigation of two vertices -> check cycle
    int rootFrom = find(parent, from);
    int rootTo = find(parent, to);

    // if different sets -> cycle x
    // add edge to MST
    if (rootFrom != rootTo) {
      mst.push_back(edges[i]);
      totalCost += weight;
      unite(parent, from, to); // merge two sets
    }
  }

  // print Kruskal result
  fout << "========KRUSKAL========" << endl;
  for (int i = 0; i < mst.size(); i++) {
    int from = mst[i].second.first;
    int to = mst[i].second.second;
    int weight = mst[i].first;
    fout << "(" << from << "," << to << ") -> " << weight << endl;
  }
  fout << "cost: " << totalCost << endl;
  fout << "=======================" << endl;

  // free memory
  delete[] parent;
  fout.close();

  return true;
}

// non-negative edge weights
// use priority queue to select minimum distance vertex
bool Dijkstra(Graph *graph, char option, int vertex) {
  // if graph is null, return false
  if (!graph) {
    return false;
  }

  int size = graph->getSize();
  ofstream fout;
  fout.open("log.txt", ios::app);

  // initialize distance array with -1 (= not visited)
  int *dist = new int[size];
  int *prev = new int[size];
  bool *visited = new bool[size];

  for (int i = 0; i < size; i++) {
    dist[i] = -1;
    prev[i] = -1;
    visited[i] = false;
  }

  // distance to start vertex is 0
  dist[vertex] = 0;

  // priority queue - (distance, vertex)
  priority_queue<pair<int, int>,          // (distance, vertex)
                 vector<pair<int, int>>,  // internal strorage structure
                 greater<pair<int, int>>> // small distance comes first
      pq;

  // push start node with distance 0 into priority queue
  pair<int, int> temp = pair<int, int>(0, vertex);
  pq.push(temp);

  // Dijkstra main loop
  while (!pq.empty()) {
    // get vertex with minimum distance
    int current = pq.top().second;
    pq.pop();

    // if already visited, skip
    if (visited[current]) {
      continue;
    }

    visited[current] = true; // mark as visited

    // get adjacent edges
    map<int, int> edges;
    graph->getAdjacentEdgesDirect(current, &edges);

    // update distance if a shorter path is found
    map<int, int>::iterator it;
    for (it = edges.begin(); it != edges.end(); it++) {
      int next = it->first;
      int weight = it->second;

      // if current vertex has valid distance
      if (dist[current] != -1) {
        // if next is not visited or found shorter path
        if (dist[next] == -1 || dist[current] + weight < dist[next]) {
          dist[next] = dist[current] + weight;
          prev[next] = current;
          pq.push(make_pair(dist[next], next));
        }
      }
    }
  }

  // print Dijkstra result
  fout << "========DIJKSTRA========" << endl;
  if (option == 'Y') {
    // print shortest path to all vertices
    for (int i = 0; i < size; i++) {
      if (i == vertex) {
        fout << "[" << i << "] " << i << " (0)" << endl;
        continue;
      }

      // if vertex is not reachable, skip
      if (dist[i] == -1) {
        continue;
      }

      // reconstruct path from vertex to i
      vector<int> path;
      int current = i;
      while (current != -1) {
        path.push_back(current);
        current = prev[current];
      }

      // print path in reverse order
      // start -> arrival
      fout << "[" << i << "] ";
      for (int j = path.size() - 1; j >= 0; j--) {
        fout << path[j];
        if (j > 0) {
          fout << " -> ";
        }
      }
      fout << " (" << dist[i] << ")" << endl;
    }
  } else {
    // print only distances
    for (int i = 0; i < size; i++) {
      if (dist[i] == -1) {
        fout << "[" << i << "] " << "x" << endl;
      } else {
        fout << "[" << i << "] " << dist[i] << endl;
      }
    }
  }
  fout << "=======================" << endl;

  // free memory
  delete[] dist;
  delete[] prev;
  delete[] visited;
  fout.close();

  return true;
}

// negative edge weights o
// update all edges in each of (V-1) times to progressively compute
bool Bellmanford(Graph *graph, char option, int s_vertex, int e_vertex) {
  // if graph is null, return false
  if (!graph) {
    return false;
  }

  int size = graph->getSize();
  ofstream fout;
  fout.open("log.txt", ios::app);

  // initialize distance array with -1 (= not visited)
  int *dist = new int[size];
  int *prev = new int[size];

  for (int i = 0; i < size; i++) {
    dist[i] = -1;
    prev[i] = -1;
  }

  dist[s_vertex] = 0; // distance to start vertex is 0

  // update all edges (V-1) times
  for (int k = 0; k < size - 1; k++) {
    // for each vertex
    for (int u = 0; u < size; u++) {
      // if u is not reachable, skip
      if (dist[u] == -1) {
        continue;
      }

      // get adjacent edges
      map<int, int> edges;
      if (option == 'Y' || option == 'O') {
        // directed graph
        graph->getAdjacentEdgesDirect(u, &edges);
      } else {
        // undirected graph
        graph->getAdjacentEdges(u, &edges);
      }

      // check all edges from u
      // update distances if a shorter path is found
      map<int, int>::iterator it;
      for (it = edges.begin(); it != edges.end(); it++) {
        int v = it->first;
        int weight = it->second;

        // update distance if shorter path found
        if (dist[v] == -1 || dist[u] + weight < dist[v]) {
          dist[v] = dist[u] + weight;
          prev[v] = u;
        }
      }
    }
  }

  // check for negative cycles
  bool hasNegativeCycle = false;
  for (int u = 0; u < size; u++) {
    // if u is not reachable, skip
    if (dist[u] == -1) {
      continue;
    }

    // get all adjacent vertices of u
    map<int, int> edges;
    if (option == 'Y' || option == 'O') {
      graph->getAdjacentEdgesDirect(u, &edges);
    } else {
      graph->getAdjacentEdges(u, &edges);
    }

    // iterate through each adjacent vertex
    map<int, int>::iterator it;
    for (it = edges.begin(); it != edges.end(); it++) {
      int v = it->first;
      int weight = it->second;

      // if can still improve distance, negative cycle exists
      if (dist[v] == -1 || dist[u] + weight < dist[v]) {
        hasNegativeCycle = true;
        break;
      }
    }

    if (hasNegativeCycle) {
      break;
    }
  }

  // print Bellman-Ford result
  fout << "========BELLMANFORD========" << endl;

  if (hasNegativeCycle) {
    // if negative cycle detected, print error
    fout << "Negative cycle detected" << endl;
  } else if (option == 'Y' || option == 'O') {
    fout << "Directed Graph Bellman-Ford" << endl;
  } else {
    fout << "Undirected Graph Bellman-Ford" << endl;
  }

  if (!hasNegativeCycle) {
    // print shortest path from s_vertex to e_vertex
    if (dist[e_vertex] == -1) {
      // if not reachable, print x
      fout << "x" << endl;
    } else {
      // reconstruct path
      vector<int> path;
      int current = e_vertex;
      while (current != -1) {
        path.push_back(current);
        current = prev[current];
      }

      // print path in reverse order
      for (int i = path.size() - 1; i >= 0; i--) {
        fout << path[i];
        if (i > 0) {
          fout << " -> ";
        }
      }
      fout << endl;
      fout << "Cost: " << dist[e_vertex] << endl;
    }
  }

  fout << "=======================" << endl;

  // free memory
  delete[] dist;
  delete[] prev;
  fout.close();

  return true;
}

// find shortest paths between all pairs of vertices
bool FLOYD(Graph *graph, char option) {
  // if graph is null, return false
  if (!graph) {
    return false;
  }

  int size = graph->getSize();
  ofstream fout;
  fout.open("log.txt", ios::app);

  // initialize distance matrix with -1 (= not reachable)
  int **dist = new int *[size];
  for (int i = 0; i < size; i++) {
    dist[i] = new int[size];
    for (int j = 0; j < size; j++) {
      if (i == j) {
        dist[i][j] = 0;
      } else {
        dist[i][j] = -1;
      }
    }
  }

  // fill initial distances from graph edges
  for (int i = 0; i < size; i++) {
    map<int, int> edges;
    if (option == 'Y' || option == 'O') {
      // directed graph
      graph->getAdjacentEdgesDirect(i, &edges);
    } else {
      // undirected graph
      graph->getAdjacentEdges(i, &edges);
    }

    // load direct edge weights into distance table
    map<int, int>::iterator it;
    for (it = edges.begin(); it != edges.end(); it++) {
      int j = it->first;       // adjacent vertex
      int weight = it->second; // edge weight
      dist[i][j] = weight;     // set initial distance
    }
  }

  // Floyd main loop
  // for each medium vertex k
  for (int k = 0; k < size; k++) {
    // for each source vertex i
    for (int i = 0; i < size; i++) {
      // for each destination vertex j
      for (int j = 0; j < size; j++) {
        // if path through k exists and is shorter => update distance
        // D(a,b) = min(D(a,b), D(a,k)+D(k,b))
        if (dist[i][k] != -1 && dist[k][j] != -1) {
          if (dist[i][j] == -1 || dist[i][k] + dist[k][j] < dist[i][j]) {
            dist[i][j] = dist[i][k] + dist[k][j];
          }
        }
      }
    }
  }

  // check for negative cycles
  bool hasNegativeCycle = false;
  for (int i = 0; i < size; i++) {
    if (dist[i][i] < 0) {
      hasNegativeCycle = true;
      break;
    }
  }

  // print Floyd result
  fout << "========FLOYD========" << endl;

  if (hasNegativeCycle) {
    // if negative cycle detected, return error
    fout.close();
    for (int i = 0; i < size; i++) {
      delete[] dist[i];
    }
    delete[] dist;
    return false;
  }

  // print graph type based on direction option
  if (option == 'Y' || option == 'O') {
    fout << "Directed Graph Floyd" << endl;
  } else {
    fout << "Undirected Graph Floyd" << endl;
  }

  // print distance matrix header
  fout << "  ";
  for (int i = 0; i < size; i++) {
    fout << "[" << i << "] ";
  }
  fout << endl;

  // print distance matrix
  for (int i = 0; i < size; i++) {
    fout << "[" << i << "] ";
    for (int j = 0; j < size; j++) {
      if (dist[i][j] == -1) {
        fout << "x";
      } else {
        fout << dist[i][j];
      }

      if (j < size - 1) {
        fout << " ";
      }
    }
    fout << endl;
  }

  fout << "=======================" << endl;

  // free memory
  for (int i = 0; i < size; i++) {
    delete[] dist[i];
  }
  delete[] dist;
  fout.close();

  return true;
}

// closeness centrality = (n-1) / sum of shortest distances to all other
// get all vertices pairs shortest paths -> Floyd
bool Centrality(Graph *graph) {
  // if graph is null, return false
  if (!graph) {
    return false;
  }

  int size = graph->getSize();
  ofstream fout;
  fout.open("log.txt", ios::app);

  // initialize distance matrix with -1 (not reachable)
  int **dist = new int *[size];
  for (int i = 0; i < size; i++) {
    dist[i] = new int[size];
    for (int j = 0; j < size; j++) {
      if (i == j) {
        dist[i][j] = 0;
      } else {
        dist[i][j] = -1;
      }
    }
  }

  // fill initial distances from graph edges (undirected)
  for (int i = 0; i < size; i++) {
    map<int, int> edges;
    graph->getAdjacentEdgesDirect(i, &edges);

    map<int, int>::iterator it;
    for (it = edges.begin(); it != edges.end(); it++) {
      int j = it->first;
      int weight = it->second;
      dist[i][j] = weight; // store weights for directly connected edges

      if (dist[j][i] == -1) {
        dist[j][i] = weight;
      }
    }
  }

  // run Floyd algorithm
  for (int k = 0; k < size; k++) {
    for (int i = 0; i < size; i++) {
      for (int j = 0; j < size; j++) {
        // if path through k exists and is shorter
        // => update distance
        if (dist[i][k] != -1 && dist[k][j] != -1) {
          if (dist[i][j] == -1 || dist[i][k] + dist[k][j] < dist[i][j]) {
            dist[i][j] = dist[i][k] + dist[k][j];
          }
        }
      }
    }
  }

  // check for negative cycles
  bool hasNegativeCycle = false;
  for (int i = 0; i < size; i++) {
    if (dist[i][i] < 0) {
      hasNegativeCycle = true;
      break;
    }
  }

  // if negative cycle detected, return error
  if (hasNegativeCycle) {
    fout.close();
    for (int i = 0; i < size; i++) {
      delete[] dist[i];
    }
    delete[] dist;
    return false;
  }

  // calculate closeness centrality for each vertex
  int *sum = new int[size];
  int minSum = -1;
  int mostCentral = -1;

  for (int i = 0; i < size; i++) {
    sum[i] = 0;
    bool reachable = true;

    for (int j = 0; j < size; j++) {
      if (i != j) {
        if (dist[i][j] == -1) {
          // if any vertex is not reachable => unreachable
          reachable = false;
          break;
        }
        sum[i] += dist[i][j]; // sum the shortest distance
      }
    }

    // if vertex is not reachable to all others => sum = -1
    if (!reachable) {
      sum[i] = -1;
    }

    // find vertex with minimum sum
    if (sum[i] != -1) {
      if (minSum == -1 || sum[i] < minSum) {
        minSum = sum[i];
        mostCentral = i;
      }
    }
  }

  // print centrality result
  fout << "========CENTRALITY========" << endl;
  for (int i = 0; i < size; i++) {
    fout << "[" << i << "] ";
    if (sum[i] == -1) {
      fout << "x";
    } else {
      fout << (size - 1) << "/" << sum[i];
    }

    if (i == mostCentral) {
      fout << " <- Most Central";
    }
    fout << endl;
  }
  fout << "=======================" << endl;

  // free memory
  for (int i = 0; i < size; i++) {
    delete[] dist[i];
  }
  delete[] dist;
  delete[] sum;
  fout.close();

  return true;
}