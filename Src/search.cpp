#include "search.h"
#include <iostream>

auto NodeCompare = [](const Node* lhs, const Node* rhs) {
    return std::tie(lhs->F, lhs->i, lhs->j) < std::tie(rhs->F, rhs->i, rhs->j);
};

Search::Search() : OPEN(NodeCompare) {
    sresult.numberofsteps = 0;
}

Search::~Search() {
    for (auto it = points2nodes.begin(); it != points2nodes.end(); ++it) {
        delete it->second;
    }
}


SearchResult Search::startSearch(ILogger* Logger, const Map& map, const EnvironmentOptions& options) {
    auto startTime = std::chrono::system_clock::now();

    std::pair<int, int> start = map.getStartPoint(), goal = map.getGoalPoint();
    points2nodes[1LL * start.first * map.getMapWidth() + start.second] = new Node(
        start.first,
        start.second,
        0,
        getHeuristic(start.first, start.second, map, options),
        nullptr
    );
    OPEN.insert(points2nodes[1LL * start.first * map.getMapWidth() + start.second]);

    while (!OPEN.empty()) {
        ++sresult.numberofsteps;
        Node* now = *OPEN.begin();
        OPEN.erase(OPEN.begin());
        CLOSE.insert(now);

        if (now->i == goal.first && now->j == goal.second) {
            generatePath(now);
            setSearchResult(true);
            auto endTime = std::chrono::system_clock::now();
            sresult.time = std::chrono::duration<double>(endTime - startTime).count();
            return sresult;
        }

        for (auto [successor, cost] : getSuccessors(now, map, options)) {
            if (CLOSE.contains(successor)) {
                continue;
            } else if (successor->g > now->g + cost) {
                successor->g = now->g + cost;
                successor->F = successor->g + successor->H;
                successor->parent = now;
                OPEN.erase(successor);
                OPEN.insert(successor);
            }
        }
    }

    setSearchResult(false);
    auto endTime = std::chrono::system_clock::now();
    sresult.time = std::chrono::duration<double>(endTime - startTime).count();
    return sresult;
}

std::vector<std::pair<Node*, double>> Search::getSuccessors(Node* node, const Map& map, const EnvironmentOptions& options) {
    std::vector<std::pair<Node*, double>> successors;

    for (int i = std::max(0, node->i - 1); i < std::min(node->i + 2, map.getMapHeight()); ++i) {
        for (int j = std::max(0, node->j - 1); j < std::min(node->j + 2, map.getMapWidth()); ++j) {
            if (map.CellIsTraversable(i, j)) {
                double cost = 1;
                if ((i + j) % 2 == (node->i + node->j) % 2) {
                    if (!checkDiagonalSuccessor(node, i, j, map, options)) {
                        continue;
                    }
                    cost = sqrt(2);
                }

                Node* successor = points2nodes[1LL * i * map.getMapWidth() + j];
                if (successor == nullptr) {
                    successor = new Node(i, j, node->g + cost, getHeuristic(i, j, map, options), node);
                    points2nodes[1LL * i * map.getMapWidth() + j] = successor;
                    OPEN.insert(successor);
                }
                successors.push_back({successor, cost});
            }
        }
    }

    return successors;
}

bool Search::checkDiagonalSuccessor(Node* node, int i, int j, const Map& map, const EnvironmentOptions &options) {
    if (!options.allowdiagonal || (node->i == i && node->j == j)) {
        return false;
    }

    bool nearCell1 = map.CellIsTraversable(node->i, j), nearCell2 = map.CellIsTraversable(i, node->j);
    return (nearCell1 && nearCell2) ||
           (options.cutcorners && (nearCell1 || nearCell2)) ||
           (options.allowsqueeze && !nearCell1 && !nearCell2);
}

void Search::generatePath(Node* goal) {
    Node* now = goal;
    while (now != nullptr) {
        lppath.push_front(*now);
        now = now->parent;
    }

    if (lppath.size() < 3) {
        hppath = lppath;
        return;
    }

    auto it1 = lppath.begin(), it3 = it1++, it2 = it1++;
    hppath.push_back(*it3);
    while (it1 != lppath.end()) {
        if ((it2->j - it3->j) * (it1->i - it3->i) - (it1->j - it3->j) * (it2->i - it3->i) != 0) {
            it2 = it1;
            it3 = --it1;
            ++it1;
            hppath.push_back(*it3);
        }

        ++it1;
    }

    hppath.push_back(lppath.back());
}

void Search::setSearchResult(bool pathFound) {
    sresult.pathfound = pathFound;
    sresult.pathlength = pathFound ? lppath.back().g : 0;
    sresult.lppath = &lppath;
    sresult.hppath = &hppath;
    sresult.nodescreated = OPEN.size() + CLOSE.size();
}

double Search::getHeuristic(int i, int j, const Map& map, const EnvironmentOptions& options) {
    auto [goal_i, goal_j] = map.getGoalPoint();
    int di = abs(i - goal_i), dj = abs(j - goal_j);

    if (options.metrictype == 0) {
        return sqrt(2) * std::min(di, dj) + abs(di - dj);
    } else if (options.metrictype == 1) {
        return di + dj;
    } else if (options.metrictype == 2) {
        return sqrt(pow(di, 2) + pow(dj, 2));
    } else if (options.metrictype == 3) {
        return std::max(di, dj);
    }

    return 0;
}