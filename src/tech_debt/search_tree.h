#ifndef OPEN_LIST_H
#define OPEN_LIST_H

#include "node.h"
#include "map.h"
#include "tinyxml/tinyxml.h"
#include "tinyxml/tinystr.h"

#include <vector>
#include <queue>
#include <unordered_set>
#include <memory>

class SearchTree;
typedef std::weak_ptr<SearchTree> search_tree_weak_ptr;
typedef std::shared_ptr<SearchTree> search_tree_shared_ptr;

class SearchTree {
public:
    SearchTree();

    ~SearchTree();

    size_t size() const;
    bool empty() const;

    void add(const Node& new_node);
    std::optional<Node> getMin();
//    void pop(Node min);

    void writeToXml(TiXmlNode *child) const;

    // todo: make private?
    std::unordered_multimap<Node, Node> close_; // Close: list of nodes that were already expanded
private:
    std::vector<std::list<Node>> rows_;
    std::priority_queue<Node> open_;
};

#endif // OPEN_LIST_H
