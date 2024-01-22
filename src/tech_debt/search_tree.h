#ifndef OPEN_LIST_H
#define OPEN_LIST_H

#include "node.h"
#include "map.h"
#include "logger.h"
#include "tinyxml/tinyxml.h"
#include "tinyxml/tinystr.h"

#include <vector>
#include <queue>
#include <unordered_map>
#include <optional>
#include <memory>

typedef std::unordered_multimap<Node, Node>::iterator close_iterator;

class SearchTree {
public:
    SearchTree();

    ~SearchTree();

    size_t sizeOpen() const;
    size_t sizeClose() const;
    bool empty() const;

    void addOpen(const Node& new_node);
    std::optional<Node> getOpen();

    void addClose(const Node& new_node);
    Node* findClose(const Node &key);
    std::pair<close_iterator, close_iterator> findCloseRange(const Node &key);
//    void pop(Node min);

    void writeToXml(TiXmlNode *child) const;
    void saveToLogOpenAndClose(std::shared_ptr<Logger> &logger) const;
private:
    std::vector<std::list<Node>> rows_;
    std::priority_queue<Node> open_;
    std::unordered_multimap<Node, Node> close_; // Close: list of nodes that were already expanded
};

#endif // OPEN_LIST_H
