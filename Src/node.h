#ifndef NODE_H
#define NODE_H

//That's the data structure for storing a single search node.
//You MUST store all the intermediate computations occuring during the search
//incapsulated to Nodes (so NO separate arrays of g-values etc.)

struct Node
{
    int     i, j; //grid cell coordinates
    double  F, g, H; //f-, g- and h-values of the search node
    Node    *parent; //backpointer to the predecessor node (e.g. the node which g-value was used to set the g-value of the current node)

    Node(int _i, int _j, double _g, double _H, Node* _parent) : i(_i), j(_j), F(_g + _H), g(_g), H(_H), parent(_parent) {}
};
#endif
