
#ifndef _GRAPH_HPP_
#define _GRAPH_HPP_

#include <list>
#include <ostream>
#include <map>
#include <iostream>
#include <cstdlib>
#include <sstream>
#include <iostream>
#include <fstream>
#include <stdint.h>
#include <list>
#include <queue>
#include <stack>
#include "UnionFind.hpp"
#include "NegativeGraphCycle.hpp"
using namespace std;

typedef pair<long, long> pair_long;
typedef priority_queue<pair_long,
                       vector<pair_long>,
                       greater<pair_long>>
    pr_queue_pair_long;

template <typename T>
struct Edge
{
  T from;
  T to;
  int fromOrder, toOrder;
  int dist;
  Edge(T f, T t, int d) : from(f), to(t), dist(d)
  {
  }
  bool operator<(const Edge<T> &e) const;
  bool operator>(const Edge<T> &e) const;
  template <typename U>
  friend std::ostream &operator<<(std::ostream &out, const Edge<U> &e);
};

template <typename T>
class cmpDistance
{
public:
  bool operator()(const T &e1, const T &e2) const
  {
    return e1 > e2;
  }
};

template <typename T>
bool Edge<T>::operator>(const Edge<T> &e) const
{
  if (this->dist > e.dist)
  {
    return true;
  }
  else
  {
    return false;
  }
}

template <typename T>
bool Edge<T>::operator<(const Edge<T> &e) const
{
  return !(this > e);
}

template <typename T>
std::ostream &operator<<(std::ostream &out, const Edge<T> &e)
{
  out << e.from << " -- " << e.to << " (" << e.dist << ")";
  return out;
}

template <typename T>
class Graph
{

public:
  Graph(bool isDirectedGraph = true, int capacity = 2);
  bool addVtx(const T &info);
  bool rmvVtx(const T &info);
  bool addEdg(const T &from, const T &to, int distance);
  bool rmvEdg(const T &from, const T &to);
  list<T> dfs(const T &info) const;
  list<T> bfs(const T &info) const;
  list<Edge<T>> mst();
  int getOrder(const T &vtx) const;

  void print2DotFile(const char *filename) const;
  list<T> dijkstra(const T &from, const T &to);
  list<T> bellman_ford(const T &from, const T &to);

private:
  bool contains(const T &info);

  int numberOfVertices;
  bool isDirectedGraph;

  vector<vector<long>> adjacencyMatrix;
  map<int, T> insertionOrder;
  list<Edge<T>> edges;
};

template <typename T>
Graph<T>::Graph(bool isDirectedGraph, int capacity)
{

  this->isDirectedGraph = isDirectedGraph;
  this->numberOfVertices = 0;

  for (int i = 0; i < capacity; i++)
  {
    adjacencyMatrix.push_back(vector<long>());

    for (int j = 0; j < capacity; j++)
    {
      adjacencyMatrix[i].push_back(INT64_MAX);
    }
  }
}

template <typename T>
void Graph<T>::print2DotFile(const char *filename) const
{
  std::ofstream file(filename);
  bool m[adjacencyMatrix.capacity()][adjacencyMatrix.capacity()];

  if (!file.is_open())
  {
    printf("dot %s NOK\n", filename);
  }

  if (!isDirectedGraph)
    file << "graph Tree {" << endl;
  else
    file << "digraph Tree {" << endl;

  for (int i = 0; i < numberOfVertices; i++)
  {
    int curr = i;

    T currS = insertionOrder.at(i);

    file << "\t" << curr + 1 << " "
         << "[label="
         << "\"" << currS << "\""
         << ",shape=circle,color=black]" << endl;
    for (int j = 0; j < numberOfVertices; j++)
    {
      if (adjacencyMatrix[curr][j] < INT64_MAX)
      {
        if (!m[curr][j] || !m[j][curr])
        {
          if (!isDirectedGraph)
            file << curr + 1 << " -- " << j + 1 << endl;
          else
            file << curr + 1 << " -> " << j + 1 << endl;

          m[curr][j] = true;
          if (!isDirectedGraph)
            m[j][curr] = true;
        }
      }
    }
  }

  file << "}";

  file.close();
  printf("dot %s OK\n", filename);
}

template <typename T>
int Graph<T>::getOrder(const T &vtx) const
{

  for (int i = 0; i < numberOfVertices; i++)
  {
    T v = insertionOrder.at(i);
    if (v == vtx)
    {
      return i;
    }
  }

  return -1;
}

template <typename T>
bool Graph<T>::rmvEdg(const T &from, const T &to)
{
  int fromPos = getOrder(from);
  int toPos = getOrder(to);

  if (fromPos == -1 || toPos == -1)
  {
    return false;
  }

  adjacencyMatrix[fromPos][toPos] = INT64_MAX;

  for (auto it = edges.begin(); it != edges.end(); it++)
  {
    if (it->from == from && it->to == to)
    {
      edges.erase(it);
      break;
    }
  }

  if (!isDirectedGraph)
  {
    adjacencyMatrix[toPos][fromPos] = INT64_MAX;

    for (auto it = edges.begin(); it != edges.end(); it++)
    {
      if (it->from == to && it->from == to)
      {
        edges.erase(it);
        break;
      }
    }
  }
  return true;
}
template <typename T>
bool Graph<T>::addEdg(const T &from, const T &to, int distance)
{
  int fromPos = getOrder(from);
  int toPos = getOrder(to);

  if (fromPos == -1 || toPos == -1)
  {
    return false;
  }

  if (adjacencyMatrix[fromPos][toPos] >= INT64_MAX)
  {
    adjacencyMatrix[fromPos][toPos] = distance;

    Edge<T> eFromTo(from, to, distance);

    edges.push_back(eFromTo);

    if (!isDirectedGraph)
    {
      Edge<T> eToFrom(to, from, distance);
      edges.push_back(eToFrom);

      adjacencyMatrix[toPos][fromPos] = distance;
    }
  }
  //If this edge already exists return false
  else
  {
    return false;
  }

  return true;
}
template <typename T>
bool Graph<T>::contains(const T &info)
{
  for (auto it = insertionOrder.begin(); it != insertionOrder.end(); it++)
  {
    if (it->second == info)
    {
      return true;
    }
  }
  return false;
}

template <typename T>
bool Graph<T>::addVtx(const T &info)
{

  if (contains(info))
    return false;

  vector<long>::iterator columnIterator;
  int i, j = 0;

  adjacencyMatrix.push_back(vector<long>());

  insertionOrder[numberOfVertices] = info;
  numberOfVertices++;

  for (i = 0; i < numberOfVertices; i++)
  {
    adjacencyMatrix[i].push_back(INT64_MAX);

    if (i == numberOfVertices - 1)
    {
      for (j = 0; j < numberOfVertices; j++)
      {
        adjacencyMatrix[numberOfVertices].push_back(INT64_MAX);
      }
    }
  }

  return true;
}

template <typename T>
bool Graph<T>::rmvVtx(const T &info)
{
  int order = -1;
  int i, j = 0;

  typename list<Edge<T>>::iterator listIt = edges.begin();

  vector<vector<long>>::iterator rowIterator = adjacencyMatrix.begin();

  vector<long>::iterator columnIterator;

  //Find insertion order of info
  order = getOrder(info);

  typename map<int, T>::iterator itCurrent = next(insertionOrder.begin(), order);
  typename map<int, T>::iterator itPrevious;

  if (order == -1)
  {
    return false;
  }

  if (order == numberOfVertices - 1)
  {
    insertionOrder.erase(numberOfVertices - 1);
  }
  else
  {
    itPrevious = itCurrent;
    itCurrent++;

    //Re-arrange map values
    while (itCurrent != insertionOrder.end())
    {

      itPrevious->second = itCurrent->second;

      itPrevious = itCurrent++;
    }

    insertionOrder.erase(numberOfVertices - 1);
  }

  //Remove all edges associated with info
  while (listIt != edges.end())
  {
    if (listIt->from == info || listIt->to == info)
    {
      listIt = edges.erase(listIt);
    }
    else
    {
      listIt++;
    }
  }

  while (rowIterator != adjacencyMatrix.end() && j < numberOfVertices)
  {
    if (j == order)
      break;
    rowIterator++;
    j++;
  }

  if (rowIterator != adjacencyMatrix.end())
  {
    rowIterator->clear();
    adjacencyMatrix.erase(rowIterator);
  }

  // //Walk through each row and delete the appropriate column
  for (i = 0, j = 0; i < numberOfVertices; ++i, j = 0)
  {
    columnIterator = adjacencyMatrix[i].begin();
    while (columnIterator != adjacencyMatrix[i].end() && j < numberOfVertices)
    {
      if (j == order)
        break;
      columnIterator++;
      j++;
    }

    if (columnIterator != adjacencyMatrix[i].end())
    {
      columnIterator = adjacencyMatrix[i].erase(columnIterator); // Remove that column entry
    }
  }

  numberOfVertices--;
  return true;
}

template <typename T>
list<T> Graph<T>::dijkstra(const T &from, const T &to)
{
  list<T> path;
  pr_queue_pair_long pr_queue;
  vector<long> cost(numberOfVertices, INT64_MAX);
  int parent[numberOfVertices];

  for (int i = 0; i < numberOfVertices; i++)
  {
    parent[i] = i;
  }

  int source = getOrder(from);
  int destination = getOrder(to);

  cost[source] = 0;
  parent[source] = -1;

  pr_queue.push(pair_long(cost[source], source));

  while (!pr_queue.empty())
  {
    int vtx = pr_queue.top().second;
    int weight = pr_queue.top().first;

    if (vtx == destination)
    {
      break;
    }

    pr_queue.pop();

    for (int i = 0; i < numberOfVertices; i++)
    {

      if (adjacencyMatrix[vtx][i] < INT64_MAX)
      {

        if (cost[i] > weight + adjacencyMatrix[vtx][i])
        {
          cost[i] = weight + adjacencyMatrix[vtx][i];
          parent[i] = vtx;

          // cout << "parent[" << insertionOrder[i] << "]=" << insertionOrder[vtx]
          //      << " cost[" << insertionOrder[i] << "]=" << cost[i] << endl;
          pr_queue.push(pair_long(cost[i], i));
        }
      }
    }
  }
  int i = destination;

  while (parent[i] != -1 && parent[i] != i)
  {
    path.push_front(insertionOrder[parent[i]]);
    i = parent[i];
  }

  if (path.empty())
  {
    return path;
  }

  path.push_back(insertionOrder[destination]);

  return path;
}
template <typename T>
list<T> Graph<T>::bellman_ford(const T &from, const T &to)
{
  list<T> path;
  vector<long> cost(numberOfVertices, INT64_MAX);
  typename list<Edge<T>>::iterator it;

  int parent[numberOfVertices];

  for (int i = 0; i < numberOfVertices; i++)
  {
    parent[i] = i;
  }

  int source = getOrder(from);
  int destination = getOrder(to);

  cost[source] = 0;

  for (int i = 0; i <= numberOfVertices - 1; i++)
  {
    for (it = edges.begin(); it != edges.end(); it++)
    {
      Edge<T> edge = *it;

      int u = getOrder(edge.from);
      int v = getOrder(edge.to);
      int weight = edge.dist;

      if (cost[u] != INT64_MAX && cost[u] + weight < cost[v])
      {
        cost[v] = cost[u] + weight;
        parent[v] = u;
      }
    }
  }

  for (it = edges.begin(); it != edges.end(); it++)
  {

    Edge<T> edge = *it;

    int u = getOrder(edge.from);
    int v = getOrder(edge.to);
    int weight = edge.dist;

    if (cost[u] != INT64_MAX && cost[u] + weight < cost[v])
    {
      throw NegativeGraphCycle();
      return path;
    }
  }

  int i = destination;

  while (parent[i] != -1 && parent[i] != i)
  {
    path.push_front(insertionOrder[parent[i]]);
    i = parent[i];
  }

  path.push_back(insertionOrder[destination]);

  return path;
}
template <typename T>
list<T> Graph<T>::dfs(const T &start) const
{
  list<T> trav;
  stack<int> st;

  bool visited[numberOfVertices];
  int startOrder = getOrder(start);

  if (startOrder == -1)
  {
    return trav;
  }

  st.push(startOrder);

  while (!st.empty())
  {
    int popped = st.top();
    st.pop();

    if (!visited[popped])
    {
      trav.push_back(insertionOrder.at(popped));
      visited[popped] = true;
    }

    for (int i = numberOfVertices - 1; i >= 0; i--)
    {
      if (adjacencyMatrix[popped][i] < INT64_MAX && !visited[i])
      {
        st.push(i);
      }
    }
  }

  return trav;
}

template <typename T>
list<T> Graph<T>::bfs(const T &start) const
{
  list<T> trav;
  queue<int> q;

  bool visited[numberOfVertices];
  int startOrder = getOrder(start);

  if (startOrder == -1)
  {
    return trav;
  }

  q.push(startOrder);
  visited[startOrder] = true;

  while (!q.empty())
  {
    int dequeued = q.front();
    q.pop();

    trav.push_back(insertionOrder.at(dequeued));

    for (int i = 0; i < numberOfVertices; i++)
    {
      if (adjacencyMatrix[dequeued][i] < INT64_MAX && !visited[i])
      {
        q.push(i);
        visited[i] = true;
      }
    }
  }

  return trav;
}

template <typename T>
list<Edge<T>> Graph<T>::mst()
{

  list<Edge<T>> spanningTree;
  priority_queue<Edge<T>, vector<Edge<T>>, cmpDistance<Edge<T>>> priority;
  int set[numberOfVertices];

  int fromIndex = -1;
  int toIndex = -1;

  typename list<Edge<T>>::iterator listIt = edges.begin();

  for (; listIt != edges.end(); listIt++)
  {
    priority.push(*listIt);
  }

  for (int i = 0; i < numberOfVertices; i++)
  {
    set[i] = -1;
  }

  while (!priority.empty())
  {
    Edge<T> minimum = priority.top();
    priority.pop();

    fromIndex = getOrder(minimum.from);
    toIndex = getOrder(minimum.to);

    if (find(fromIndex, set) != find(toIndex, set))
    {
      spanningTree.push_back(minimum);
      Union(find(fromIndex, set), find(toIndex, set), set);
    }
  }
  return spanningTree;
}
#endif
