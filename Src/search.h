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
        std::vector<std::pair<Node*, double>> getSuccessors(Node* node, const Map& map, const EnvironmentOptions &options);
        bool checkDiagonalSuccessor(Node* node, int i, int j, const Map& map, const EnvironmentOptions &options);
        void generatePath(Node* goal);
        void setSearchResult(bool pathFound);
        double getHeuristic(int i, int j, const Map& map, const EnvironmentOptions& options);


        SearchResult                                            sresult; //This will store the search result
        std::list<Node>                                         lppath, hppath; //
        std::unordered_map<long long, Node*>                    cells2nodes;
        std::set<Node*, std::function<bool(Node*, Node*)>>      OPEN;
        std::unordered_set<Node*>                               CLOSE;
};

#endif
