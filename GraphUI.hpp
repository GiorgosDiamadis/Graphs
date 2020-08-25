
#ifndef _GRAPH_UI_
#define _GRAPH_UI_

#include <iostream>
#include <sstream>
#include "Graph.hpp"
#include "NegativeGraphCycle.hpp"
using namespace std;
template <typename T>
int graphUI()
{

  string option, line;
  // int distance;
  bool digraph = false;

  cin >> option;
  if (!option.compare("digraph"))
    digraph = true;
  Graph<T> g(digraph);

  while (true)
  {

    std::stringstream stream;
    cin >> option;

    if (!option.compare("av"))
    {
      getline(std::cin, line);
      stream << line;

      T vtx(stream);

      if (g.addVtx(vtx))
        cout << "av " << vtx << " OK\n";
      else
        cout << "av " << vtx << " NOK\n";
    }
    else if (!option.compare("rv"))
    {
      getline(std::cin, line);
      stream << line;

      T vtx(stream);

      if (g.rmvVtx(vtx))
        cout << "rv " << vtx << " OK\n";
      else
        cout << "rv " << vtx << " NOK\n";
    }
    else if (!option.compare("ae"))
    {

      getline(std::cin, line);
      stream << line;

      T from(stream);
      T to(stream);

      string weight;
      stream >> weight;

      if (g.addEdg(from, to, atoi(&weight[0])))
      {
        cout << "ae " << from << " " << to << " OK\n";
      }
      else
      {
        cout << "ae " << from << " " << to << " NOK\n";
      }
    }
    else if (!option.compare("re"))
    {
      getline(std::cin, line);
      stream << line;

      T from(stream);
      T to(stream);

      if (g.rmvEdg(from, to))
      {
        cout << "re " << from << " " << to << " OK\n";
      }
      else
      {
        cout << "re " << from << " " << to << " NOK\n";
      }
    }
    else if (!option.compare("dot"))
    {
      getline(std::cin, line);
      g.print2DotFile(&line[1]);
    }
    else if (!option.compare("bfs"))
    {
      getline(std::cin, line);
      stream << line;

      T vtx(stream);

      list<T> trav = g.bfs(vtx);

      cout << "\n----- BFS Traversal -----\n";
      for (auto it = trav.begin(); it != trav.end(); it++)
      {
        cout << *it;
        if (it != --(trav.end()))
          cout << " -> ";
      }
      cout << "\n-------------------------\n";
    }
    else if (!option.compare("dfs"))
    {
      getline(std::cin, line);
      stream << line;

      T vtx(stream);

      list<T> trav = g.dfs(vtx);

      cout << "\n----- DFS Traversal -----\n";
      for (auto it = trav.begin(); it != trav.end(); it++)
      {
        cout << *it;
        if (it != --(trav.end()))
          cout << " -> ";
      }
      cout << "\n-------------------------\n";
    }
    else if (!option.compare("dijkstra"))
    {
      getline(std::cin, line);
      stream << line;

      T from(stream);
      T to(stream);

      list<T> path = g.dijkstra(from, to);
      typename list<T>::iterator it;

      cout << "Dijkstra (" << from << " - " << to << "): ";

      unsigned int i;
      for (i = 0; i < path.size(); i++)
      {
        it = next(path.begin(), i);
        if (i < path.size() - 1)
        {

          cout << *it << ", ";
        }
        else
        {
          cout << *it << endl;
        }
      }

      if (i == 0)
      {
        cout << endl;
      }
    }
    else if (!option.compare("bellman-ford"))
    {
      getline(std::cin, line);
      stream << line;

      T from(stream);
      T to(stream);
      cout << "Bellman-Ford (" << from << " - " << to << "): ";

      try
      {
        list<T> path = g.bellman_ford(from, to);
        typename list<T>::iterator it;

        unsigned int i;
        for (i = 0; i < path.size(); i++)
        {
          it = next(path.begin(), i);
          if (i < path.size() - 1)
          {

            cout << *it << ", ";
          }
          else
          {
            cout << *it << endl;
          }
        }

        if (i == 0)
        {
          cout << endl;
        }
      }
      catch (NegativeGraphCycle &ex)
      {
        cout << ex.what() << endl;
      }
    }
    else if (!option.compare("mst"))
    {
      int sum = 0;
      list<Edge<T>> spanningTree;
      typename list<Edge<T>>::iterator listIt;

      spanningTree = g.mst();

      cout << "\n--- Min Spanning Tree ---\n";

      for (listIt = spanningTree.begin(); listIt != spanningTree.end(); listIt++)
      {

        Edge<T> edge = *listIt;

        if (g.getOrder(edge.from) < g.getOrder(edge.to))
        {
          cout << edge.from << " -- " << edge.to << " (" << edge.dist << ")" << endl;
        }
        else
        {
          cout << edge.to << " -- " << edge.from << " (" << edge.dist << ")" << endl;
        }
        sum += listIt->dist;
      }

      cout << "MST Cost: " << sum << endl;
    }
    else if (!option.compare("q"))
    {
      cout << "bye bye...\n";
      return 0;
    }
    else if (!option.compare("#"))
    {
      string line;
      getline(cin, line);
      // cout << "Skipping line: " << line << endl;
    }
    else
    {
      cout << "INPUT ERROR\n";
      return -1;
    }
  }
  return -1;
}

#endif