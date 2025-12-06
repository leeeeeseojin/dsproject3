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
      // undirected graph - use all edges (both directions)
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
      // undirected graph - use all edges (both directions)
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
// output MST as adjacency list format
bool Kruskal(Graph *graph) {
  // if graph is null, return false
  if (!graph) {
    return false;
  }

  int size = graph->getSize();
  ofstream fout;
  fout.open("log.txt", ios::app);

  vector<pair<int, pair<int, int>>> edges; // collect all edges from graph

  // collect edges treating graph as undirected
  for (int i = 0; i < size; i++) {
    map<int, int> adj;
    graph->getAdjacentEdgesDirect(i, &adj);

    map<int, int>::iterator it;
    for (it = adj.begin(); it != adj.end(); it++) {
      int to = it->first;
      int weight = it->second;

      // for undirected graph, add edge only once (from smaller to larger
      // vertex)
      if (i < to) {
        edges.push_back(make_pair(weight, make_pair(i, to)));
      } else {
        edges.push_back(make_pair(weight, make_pair(to, i)));
      }
    }
  }

  // remove duplicate edges
  sort(edges.begin(), edges.end());
  edges.erase(unique(edges.begin(), edges.end()), edges.end());

  // initialize disjoint set
  int *parent = new int[size];
  for (int i = 0; i < size; i++) {
    parent[i] = i;
  }

  // store MST as adjacency list
  // mstAdj[i] contains pairs of (adjacent vertex, weight)
  vector<pair<int, int>> *mstAdj = new vector<pair<int, int>>[size];
  int totalCost = 0;
  int edgeCount = 0;

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
      // add edge in both directions for undirected MST
      mstAdj[from].push_back(make_pair(to, weight));
      mstAdj[to].push_back(make_pair(from, weight));
      totalCost += weight;
      edgeCount++;
      unite(parent, from, to); // merge two sets
    }
  }

  // check if MST spans all vertices (size-1 edges needed)
  if (edgeCount < size - 1) {
    // graph is not connected, some vertices are unreachable
    delete[] parent;
    delete[] mstAdj;
    fout.close();
    return false;
  }

  // sort adjacency lists by vertex number
  for (int i = 0; i < size; i++) {
    sort(mstAdj[i].begin(), mstAdj[i].end());
  }

  // print Kruskal result in adjacency list format
  fout << "========KRUSKAL========" << endl;
  for (int i = 0; i < size; i++) {
    fout << "[" << i << "]";
    for (int j = 0; j < mstAdj[i].size(); j++) {
      fout << " " << mstAdj[i][j].first << "(" << mstAdj[i][j].second << ")";
    }
    fout << endl;
  }
  fout << "cost: " << totalCost << endl;
  fout << "=======================" << endl;

  // free memory
  delete[] parent;
  delete[] mstAdj;
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

  // check for negative weights in the graph
  for (int i = 0; i < size; i++) {
    map<int, int> edges;
    if (option == 'Y' || option == 'O') {
      graph->getAdjacentEdgesDirect(i, &edges);
    } else {
      graph->getAdjacentEdges(i, &edges);
    }

    map<int, int>::iterator it;
    for (it = edges.begin(); it != edges.end(); it++) {
      if (it->second < 0) {
        // negative weight found, return error
        return false;
      }
    }
  }

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

    // get adjacent edges based on direction option
    map<int, int> edges;
    if (option == 'Y' || option == 'O') {
      // directed graph
      graph->getAdjacentEdgesDirect(current, &edges);
    } else {
      // undirected graph
      graph->getAdjacentEdges(current, &edges);
    }

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
  if (option == 'Y' || option == 'O') {
    fout << "Directed Graph Dijkstra" << endl;
  } else {
    fout << "Undirected Graph Dijkstra" << endl;
  }
  fout << "Start: " << vertex << endl;

  // print shortest path to all vertices
  for (int i = 0; i < size; i++) {
    fout << "[" << i << "] ";

    // if vertex is not reachable
    if (dist[i] == -1) {
      fout << "x" << endl;
      continue;
    }

    // if same as start vertex
    if (i == vertex) {
      fout << i << " (0)" << endl;
      continue;
    }

    // reconstruct path from vertex to i
    vector<int> path;
    int current = i;
    while (current != -1) {
      path.push_back(current);
      current = prev[current];
    }

    // print path in reverse order (start -> arrival)
    for (int j = path.size() - 1; j >= 0; j--) {
      fout << path[j];
      if (j > 0) {
        fout << " -> ";
      }
    }
    fout << " (" << dist[i] << ")" << endl;
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

  // use large value instead of -1 for infinity
  const int INF = 1e9;

  // initialize distance array with INF
  int *dist = new int[size];
  int *prev = new int[size];

  for (int i = 0; i < size; i++) {
    dist[i] = INF;
    prev[i] = -1;
  }

  dist[s_vertex] = 0; // distance to start vertex is 0

  // update all edges (V-1) times
  for (int k = 0; k < size - 1; k++) {
    // for each vertex
    for (int u = 0; u < size; u++) {
      // if u is not reachable, skip
      if (dist[u] == INF) {
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
        if (dist[u] + weight < dist[v]) {
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
    if (dist[u] == INF) {
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
      if (dist[u] + weight < dist[v]) {
        hasNegativeCycle = true;
        break;
      }
    }

    if (hasNegativeCycle) {
      break;
    }
  }

  // if negative cycle detected, return error
  if (hasNegativeCycle) {
    delete[] dist;
    delete[] prev;
    return false;
  }

  // print Bellman-Ford result
  ofstream fout;
  fout.open("log.txt", ios::app);

  fout << "========BELLMANFORD========" << endl;
  if (option == 'Y' || option == 'O') {
    fout << "Directed Graph Bellman-Ford" << endl;
  } else {
    fout << "Undirected Graph Bellman-Ford" << endl;
  }

  // print shortest path from s_vertex to e_vertex
  if (dist[e_vertex] == INF) {
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
    fout << "cost: " << dist[e_vertex] << endl;
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

  // use large value for infinity
  const int INF = 1e9;

  // initialize distance matrix with INF
  int **dist = new int *[size];
  for (int i = 0; i < size; i++) {
    dist[i] = new int[size];
    for (int j = 0; j < size; j++) {
      if (i == j) {
        dist[i][j] = 0;
      } else {
        dist[i][j] = INF;
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
        if (dist[i][k] != INF && dist[k][j] != INF) {
          if (dist[i][k] + dist[k][j] < dist[i][j]) {
            dist[i][j] = dist[i][k] + dist[k][j];
          }
        }
      }
    }
  }

  // check for negative cycles (diagonal elements < 0)
  bool hasNegativeCycle = false;
  for (int i = 0; i < size; i++) {
    if (dist[i][i] < 0) {
      hasNegativeCycle = true;
      break;
    }
  }

  // if negative cycle detected, return error
  if (hasNegativeCycle) {
    for (int i = 0; i < size; i++) {
      delete[] dist[i];
    }
    delete[] dist;
    return false;
  }

  // print Floyd result
  ofstream fout;
  fout.open("log.txt", ios::app);

  fout << "========FLOYD========" << endl;

  // print graph type based on direction option
  if (option == 'Y' || option == 'O') {
    fout << "Directed Graph Floyd" << endl;
  } else {
    fout << "Undirected Graph Floyd" << endl;
  }

  // print distance matrix header
  fout << "    ";
  for (int i = 0; i < size; i++) {
    fout << "[" << i << "] ";
  }
  fout << endl;

  // print distance matrix
  for (int i = 0; i < size; i++) {
    fout << "[" << i << "] ";
    for (int j = 0; j < size; j++) {
      if (dist[i][j] == INF) {
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

  // use large value for infinity
  const int INF = 1e9;

  // initialize distance matrix with INF
  int **dist = new int *[size];
  for (int i = 0; i < size; i++) {
    dist[i] = new int[size];
    for (int j = 0; j < size; j++) {
      if (i == j) {
        dist[i][j] = 0;
      } else {
        dist[i][j] = INF;
      }
    }
  }

  // fill initial distances from graph edges (undirected)
  for (int i = 0; i < size; i++) {
    map<int, int> edges;
    graph->getAdjacentEdges(i, &edges); // use undirected edges

    map<int, int>::iterator it;
    for (it = edges.begin(); it != edges.end(); it++) {
      int j = it->first;
      int weight = it->second;
      dist[i][j] = weight;
    }
  }

  // run Floyd algorithm
  for (int k = 0; k < size; k++) {
    for (int i = 0; i < size; i++) {
      for (int j = 0; j < size; j++) {
        // if path through k exists and is shorter
        // => update distance
        if (dist[i][k] != INF && dist[k][j] != INF) {
          if (dist[i][k] + dist[k][j] < dist[i][j]) {
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
    for (int i = 0; i < size; i++) {
      delete[] dist[i];
    }
    delete[] dist;
    return false;
  }

  // calculate closeness centrality for each vertex
  // centrality = (n-1) / sum of shortest distances FROM this vertex TO all
  // others vertex is marked as 'x' only if it cannot reach ANY other vertex
  // (isolated)
  int *sum = new int[size];
  int minSum = -1;
  vector<int> mostCentral; // may have multiple vertices with same centrality

  for (int i = 0; i < size; i++) {
    sum[i] = 0;
    int reachableCount = 0;

    // sum distances from vertex i to all other vertices
    for (int j = 0; j < size; j++) {
      if (i != j) {
        if (dist[i][j] != INF) {
          sum[i] += dist[i][j];
          reachableCount++;
        }
      }
    }

    // if vertex cannot reach any other vertex => isolated => x
    if (reachableCount == 0) {
      sum[i] = -1;
    }

    // find vertex with minimum sum (highest centrality)
    if (sum[i] != -1) {
      if (minSum == -1 || sum[i] < minSum) {
        minSum = sum[i];
        mostCentral.clear();
        mostCentral.push_back(i);
      } else if (sum[i] == minSum) {
        mostCentral.push_back(i);
      }
    }
  }

  // print centrality result
  ofstream fout;
  fout.open("log.txt", ios::app);

  fout << "========CENTRALITY========" << endl;
  for (int i = 0; i < size; i++) {
    fout << "[" << i << "] ";
    if (sum[i] == -1) {
      fout << "x";
    } else {
      // print as fraction: (n-1)/sum
      fout << (size - 1) << "/" << sum[i];
    }

    // check if this vertex is one of the most central
    for (size_t j = 0; j < mostCentral.size(); j++) {
      if (i == mostCentral[j]) {
        fout << " <- Most Central";
        break;
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
  delete[] sum;
  fout.close();

  return true;
}