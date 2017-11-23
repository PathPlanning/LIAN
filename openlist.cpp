#include "openlist.h"

OpenList::OpenList() { size = 0; }

OpenList::OpenList(int size_, int BT) {
    elements = new std::list<Node>[size_];
    size = 0;
    height = size_;
    bt = BT;
}

void OpenList::resize(int size_, int BT) {
    elements = new std::list<Node>[size_];
    height = size_;
    size = 0;
    bt = BT;
}

OpenList::~OpenList() {
    delete [] elements;
}

size_t OpenList::get_size() const {
    return size;
}

bool OpenList::is_empty() const {
    if (size == 0) return true;
    return false;
}

void OpenList::add(Node new_node) {
    if (elements[new_node.i].empty()) {
        elements[new_node.i].push_back(new_node);
        ++size;
        return;
    }
    std::list<Node>::iterator pos = elements[new_node.i].end();
    bool pos_found = false;

    for(auto it = elements[new_node.i].begin(); it != elements[new_node.i].end(); ++it) {
        if ((it->F >= new_node.F) && (!pos_found)) {
            pos = it;
            pos_found = true;
        }
        if (*it == new_node) {
            if (new_node.F >= it->F) return;
            else {
                if(pos == it) {
                    it->g = new_node.g;
                    it->F = new_node.F;
                    it->c = new_node.c;
                    it->radius = new_node.radius;
                    return;
                }
                elements[new_node.i].erase(it);
                --size;
                break;
            }
        }
    }
    ++size;
    elements[new_node.i].insert(pos,new_node);
}

Node OpenList::getMin() {
    Node min;
    min.F = std::numeric_limits<float>::infinity();
    for(size_t i = 0; i < height; i++) {
        if(!elements[i].empty() && elements[i].begin()->F <= min.F) {
            if (elements[i].begin()->F == min.F) {
                if (elements[i].begin()->g >= min.g) {
                    min = *elements[i].begin();
                }
            } else {
                min = *elements[i].begin();
            }
        }
    }
    elements[min.i].pop_front();
    --size;
    return min;
}

TiXmlElement * OpenList::writeToXml(TiXmlElement * element, TiXmlNode * child) const {
    Node min;
    min.F = std::numeric_limits<float>::infinity();
    int exc = 0;
    for(size_t i = 0; i < height; ++i) {
        if(!elements[i].empty() && elements[i].front().lesser(min, bt)) {
            min = elements[i].front();
            exc = i;
        }
    }
    if(min.F != std::numeric_limits<float>::infinity()) {
        element = new TiXmlElement(CNS_TAG_NODE);
        element -> SetAttribute(CNS_TAG_ATTR_X, min.j);
        element -> SetAttribute(CNS_TAG_ATTR_Y, min.i);
        element -> SetDoubleAttribute(CNS_TAG_ATTR_F, min.F);
        element -> SetDoubleAttribute(CNS_TAG_ATTR_G, min.g);
        element -> SetAttribute(CNS_TAG_ATTR_PARX, min.parent->j);
        element -> SetAttribute(CNS_TAG_ATTR_PARY, min.parent->i);
        child -> InsertEndChild(*element);
    }
    for(size_t i = 0; i < height; ++i) {
        if(elements[i].empty()) {
            for (auto it = elements[i].begin(); it != elements[i].end(); ++it) {
                if(it != elements[exc].begin()) {
                    element -> Clear();
                    element -> SetAttribute(CNS_TAG_ATTR_X, it->j);
                    element -> SetAttribute(CNS_TAG_ATTR_Y, it->i);
                    element -> SetDoubleAttribute(CNS_TAG_ATTR_F, it->F);
                    element -> SetDoubleAttribute(CNS_TAG_ATTR_G, it->g);
                    if (it->g > 0){
                        element -> SetAttribute(CNS_TAG_ATTR_PARX, it->parent->j);
                        element -> SetAttribute(CNS_TAG_ATTR_PARY, it->parent->i);
                    }
                    child->InsertEndChild(*element);
                }
            }
        }
    }
    return element;
}
