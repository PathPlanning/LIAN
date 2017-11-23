#ifndef OPENLIST_H
#define OPENLIST_H

#include "node.h"
#include "tinyxml/tinyxml.h"
#include "tinyxml/tinystr.h"

#include <vector>

class OpenList {

public:
    OpenList();
    OpenList(int size_, int BT);

    ~OpenList();

    void resize(int size_, int BT);

    size_t get_size() const;
    bool is_empty() const;

    void add(Node new_node);
    Node getMin();
    void pop(Node min);

    TiXmlElement *writeToXml(TiXmlElement * element, TiXmlNode *child) const;

private:
    std::list<Node> *elements;
    size_t size;
    size_t height;
    int bt;
};

#endif // OPENLIST_H
