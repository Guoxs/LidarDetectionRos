/*
 * @Authors: Guojun Wang
 * @Date: 1970-01-01 08:00:00
 * @LastEditors: Guojun Wang
 * @LastEditTime: 2020-06-10 16:05:52
 */

#include <cstdio>
#include <queue>

#include "graph_util.h"


void ConnectedComponentAnalysis(const std::vector<std::vector<int>>& graph,
                                std::vector<std::vector<int>>* components) 
{
  int no_item = graph.size();
  std::vector<int> visited;
  visited.resize(no_item, 0);
  std::queue<int> que;
  std::vector<int> component;
  components->clear();

  for (int i = 0; i < no_item; ++i) 
  {
    if (visited[i]) 
    {
      continue;
    }
    component.push_back(i);
    que.push(i);
    visited[i] = 1;
    while (!que.empty()) 
    {
      int id = que.front();
      que.pop();
      for (size_t j = 0; j < graph[id].size(); ++j) 
      {
        int nb_id = graph[id][j];
        if (visited[nb_id] == 0) 
        {
          component.push_back(nb_id);
          que.push(nb_id);
          visited[nb_id] = 1;
        }
      }
    }
    components->push_back(component);
    component.clear();
  }
}

