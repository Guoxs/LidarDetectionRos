/*
 * @Authors: Guojun Wang
 * @Date: 1970-01-01 08:00:00
 * @LastEditors: Guojun Wang
 * @LastEditTime: 2020-06-10 16:05:41
 */


#ifndef GRAPH_UTIL_H_
#define GRAPH_UTIL_H_

#include <vector>


// bfs based component analysis
void ConnectedComponentAnalysis(const std::vector<std::vector<int>>& graph,
                                std::vector<std::vector<int>>* components);


#endif  // MODULES_PERCEPTION_OBSTACLE_COMMON_GRAPH_UTIL_H_
