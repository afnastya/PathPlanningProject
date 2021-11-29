#ifndef SEARCH_H
#define SEARCH_H
#include "ilogger.h"
#include "searchresult.h"
#include "environmentoptions.h"
#include <list>
#include <functional>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <math.h>
#include <limits>
#include <chrono>

class Search
{
    public:
        Search();
        ~Search(void);
        SearchResult startSearch(ILogger *Logger, const Map &map, const EnvironmentOptions &options);

    protected:
        std::vector<Node*> getSuccessors(Node* node, const Map& map, const EnvironmentOptions &options);
        void generatePath(Node* goal);
        void setSresult();

        double getHeuristic(int i, int j, const Map& map, const EnvironmentOptions& options);

        //Hint 2. It's a good idea to define a heuristic calculation function, that will simply return 0
        //for non-heuristic search methods like Dijkstra

        //Hint 4. working with OPEN and CLOSE is the core
        //so think of the data structures that needed to be used, about the wrap-up classes (if needed)
        //Start with very simple (and ineffective) structures like list or vector and make it work first
        //and only then begin enhancement!

        SearchResult                                            sresult; //This will store the search result
        std::list<Node>                                         lppath, hppath; //
        std::unordered_map<int, Node*>                          points2nodes;
        std::set<Node*, std::function<bool(Node*, Node*)>>      OPEN;
        std::unordered_set<Node*>                               CLOSE;
};

#endif
