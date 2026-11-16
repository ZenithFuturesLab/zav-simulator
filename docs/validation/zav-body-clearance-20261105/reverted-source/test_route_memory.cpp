#include <path_search/route_memory.hpp>
#include <iostream>
#include <stdexcept>
void require(bool b,const char*m){if(!b)throw std::runtime_error(m);}
int main(){try{
 using Memory=path_search::RouteMemory;using Path=Memory::Path;using Point=Memory::Point;
 Memory memory;Path path;const Point goal(10,0,0);
 Path branch_a{{0,0,0},{1,1,0},{4,1,0},{7,0,0}};
 memory.remember(branch_a,goal);
 auto free=[](const Point&,const Point&){return true;};
 require(memory.reuse(Point(.1,.1,0),goal,6.,free,path),"safe branch must be reusable");
 require(path.size()>2&&path[1].x()>.1&&path[1].y()>.1,"must join ahead, not reverse to the old start");
 // Repeated small pose perturbations must keep the upper branch rather than
 // selecting an equally short lower route on alternating searches.
 for(int i=0;i<20;++i){
  require(memory.reuse(Point(.2,.18+(i%2)*.02,0),goal,5.,free,path),"small jitter lost the retained branch");
  require(path[1].y()>0.&&Memory::length(path)<=5.+1e-9,"route side or horizon changed");
 }
 auto blocked=[](const Point&a,const Point&b){return !(a.x()<3.&&b.x()>3.);};
 require(!memory.reuse(Point(.1,.1,0),goal,6.,blocked,path)&&path.empty(),"new obstacle must invalidate reuse");
 require(!memory.reuse(Point(.1,.1,0),Point(10,-5,0),6.,free,path),"changed goal must discard old route");
 require(!memory.reuse(Point(-3,0,0),goal,6.,free,path),"distant recovery must not drag back to old branch");
 memory.clear();require(!memory.reuse(Point(0,0,0),goal,6.,free,path),"failed optimizer cache must clear");
 std::cout<<"forward branch reuse, pose jitter, horizon, obstacle invalidation and goal reset passed\n";
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}return 0;}
