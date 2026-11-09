#include <vector>
#include <stdexcept>
#include <iostream>
#include "nav2_navfn_planner/navfn.hpp"
#include "omove_description/trail_cost.hpp"
float route(std::vector<unsigned char> & map)
{
  nav2_navfn_planner::NavFn planner(100,100);
  planner.setCostmap(map.data(),true,false);
  int start[2]={10,50},goal[2]={90,50};
  planner.setStart(start);planner.setGoal(goal);
  if (!planner.calcNavFnDijkstra(true) || planner.calcPath(1000)==0) {
    throw std::runtime_error("No traversable route");
  }
  float sum=0;int count=0;
  for (int i=0;i<planner.getPathLen();++i) {
    if (planner.getPathX()[i]>40 && planner.getPathX()[i]<60) {
      sum+=planner.getPathY()[i];++count;
    }
  }
  if (!count) {throw std::runtime_error("Path did not cross test region");}
  return sum/count;
}
int main()
{
  std::vector<unsigned char> map(10000,0);
  for(int y=30;y<=70;++y) {for(int x=40;x<=60;++x) {map[y*100+x]=254;}}
  float baseline=route(map);
  for(int y=0;y<100;++y) {for(int x=0;x<100;++x) {
    if ((baseline<50 && y<50)||(baseline>=50 && y>=50)) {
      map[y*100+x]=omove_description::addTrailCost(map[y*100+x],45);
    }
  }}
  float preferred=route(map);
  if ((baseline-50)*(preferred-50)>=0) {
    throw std::runtime_error("Planner did not prefer the unvisited alternative");
  }
  std::fill(map.begin(),map.end(),254);
  for(int y=46;y<=54;++y) {for(int x=1;x<99;++x) {map[y*100+x]=45;}}
  route(map); // A sole visited corridor remains usable.
  std::cout << "NavFn avoided visited alternative and retained sole corridor\n";
}
