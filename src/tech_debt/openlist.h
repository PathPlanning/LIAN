#ifndef OPENLIST_H
#define OPENLIST_H

#include "node.h"
#include "tinyxml/tinyxml.h"
#include "tinyxml/tinystr.h"

#include <vector>

class OpenList {

public:
    OpenList();
    OpenList(int size_);

    ~OpenList();

    void resize(int size_);

    size_t get_size() const;
    bool is_empty() const;

    void add(Node new_node);
    Node getMin();
    void pop(Node min);

    void writeToXml(TiXmlNode *child) const;

private:
    std::list<Node> *elements;
    size_t size;
    size_t height;
};

#endif // OPENLIST_H
