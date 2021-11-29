#include "search.h"
#include <iostream>


Search::Search() : OPEN([](const Node* lhs, const Node* rhs) {
    return std::tie(lhs->F, lhs->i, lhs->j) < std::tie(rhs->F, rhs->i, rhs->j);
})
{
    sresult.numberofsteps = 0;
}

Search::~Search() {
    for (auto it = points2nodes.begin(); it != points2nodes.end(); ++it) {
        delete it->second;
    }
}


SearchResult Search::startSearch(ILogger *Logger, const Map &map, const EnvironmentOptions &options)
{
    std::pair<int, int> start = map.getStartPoint(), goal = map.getGoalPoint();
    points2nodes[start.first * map.getMapWidth() + start.second] = new Node(
        start.first,
        start.second,
        0,
        getHeuristic(start.first, start.second, map, options),
        nullptr
    );
    OPEN.insert(points2nodes[start.first * map.getMapWidth() + start.second]);

    while (!OPEN.empty()) {
        ++sresult.numberofsteps;
        Node* now = *OPEN.begin();
        OPEN.erase(OPEN.begin());
        CLOSE.insert(now);

        if (now->i == goal.first && now->j == goal.second) {
            generatePath(now);
            sresult.pathfound = true;
            setSresult();
            return sresult;
        }

        for (Node* successor : getSuccessors(now, map, options)) {
            if (CLOSE.contains(successor)) {
                continue;
            } else if (successor->g > now->g + 1) {
                successor->g = now->g + 1;
                successor->F = successor->g + successor->H;
                successor->parent = now;
                OPEN.erase(successor);
                OPEN.insert(successor);
            }
        }
    }
    sresult.pathfound = false;
    setSresult();
    return sresult;
}

std::vector<Node*> Search::getSuccessors(Node* node, const Map& map, const EnvironmentOptions &options) {
    std::vector<Node*> successors;

    for (int i = std::max(0, node->i - 1); i < std::min(node->i + 2, map.getMapHeight()); ++i) {
        for (int j = std::max(0, node->j - 1); j < std::min(node->j + 2, map.getMapWidth()); ++j) {
            if ((i + j) % 2 == (node->i + node->j) % 2) {
                continue; // diagonal successors
            } else if (map.CellIsTraversable(i, j)) {
                Node* successor = points2nodes[i * map.getMapWidth() + j];
                if (successor == nullptr) {
                    successor = new Node(i, j, node->g + 1, getHeuristic(i, j, map, options), node);
                    points2nodes[i * map.getMapWidth() + j] = successor;
                    OPEN.insert(successor);
                }
                successors.push_back(successor);
            }
        }
    }

    return successors;
}

void Search::generatePath(Node* goal) {
    Node* now = goal;
    while (now != nullptr) {
        lppath.push_front(*now);
        now = now->parent;
    }

    hppath = lppath;
}

void Search::setSresult() {
    sresult.nodescreated = OPEN.size() + CLOSE.size();
    sresult.pathlength = -1 + lppath.size();
    // sresult.time = ;
    sresult.hppath = &hppath; //Here is a constant pointer
    sresult.lppath = &lppath;
}

double Search::getHeuristic(int i, int j, const Map& map, const EnvironmentOptions& options) {
    std::pair<int, int> goal = map.getGoalPoint();
    int di = abs(i - goal.first), dj = abs(j - goal.second);

    if (options.metrictype == 0) {
        return sqrt(2) * std::min(di, dj) + abs(di - dj);
    } else if (options.metrictype == 2) {
        return sqrt(pow(di, 2) + pow(dj, 2));
    }

    return 0;
}

/*void Search::makePrimaryPath(Node curNode)
{
    //need to implement
}*/

/*void Search::makeSecondaryPath()
{
    //need to implement
}*/
