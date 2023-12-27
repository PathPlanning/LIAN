#include "open_list.h"

OpenList::OpenList() { size_ = 0; }

OpenList::OpenList(int size) {
    resize(size);
    size_ = 0;
}

void OpenList::resize(int size) {
    rows_.resize(size);
}

OpenList::~OpenList() = default;

size_t OpenList::size() const {
    return size_;
}

bool OpenList::empty() const {
    return size_ == 0;
}

void OpenList::add(const Node &new_node) {
    if (rows_[new_node.i].empty()) {
        rows_[new_node.i].push_back(new_node);
        ++size_;
        return;
    }
    auto pos = rows_[new_node.i].end();
    bool pos_found = false;

    for (auto it = rows_[new_node.i].begin(); it != rows_[new_node.i].end(); ++it) {
        if ((it->F >= new_node.F) && (!pos_found)) {
            pos = it;
            pos_found = true;
        }
        if (*it == new_node) {
            if (new_node.F >= it->F) return;
            else {
                if (pos == it) {
                    it->g = new_node.g;
                    it->F = new_node.F;
                    it->radius = new_node.radius;
                    return;
                }
                rows_[new_node.i].erase(it);
                --size_;
                break;
            }
        }
    }
    ++size_;
    rows_[new_node.i].insert(pos, new_node);
}

// todo: rewrite to std::optional
Node OpenList::getMin() {
    Node min;
    min.F = std::numeric_limits<float>::infinity();
    for (auto & element : rows_) {
        if (element.empty() || element.begin()->F > min.F) continue;
        if (element.begin()->F != min.F || element.begin()->g >= min.g) {
            min = element.front();
        }
    }
    rows_[min.i].pop_front();
    --size_;
    return min;
}

void OpenList::writeToXml(TiXmlNode *child) const {
    Node min;
    min.F = std::numeric_limits<float>::infinity();
    for (const auto &element: rows_) {
        if (element.empty() || element.begin()->F > min.F) continue;
        if (element.begin()->F != min.F || element.begin()->g >= min.g) {
            min = element.front();
        }
    }
    if (min.F != std::numeric_limits<float>::infinity()) {
        TiXmlElement element(CNS_TAG_NODE);
        element.SetAttribute(CNS_TAG_ATTR_X, min.j);
        element.SetAttribute(CNS_TAG_ATTR_Y, min.i);
        element.SetDoubleAttribute(CNS_TAG_ATTR_F, min.F);
        element.SetDoubleAttribute(CNS_TAG_ATTR_G, min.g);
        element.SetAttribute(CNS_TAG_ATTR_PARX, min.parent->j);
        element.SetAttribute(CNS_TAG_ATTR_PARY, min.parent->i);
        child->InsertEndChild(element);
    }

    for (const auto &i: rows_) {
        if (i.empty()) continue;
        for (const auto &it: i) {
            if (it != min) {
                TiXmlElement element1(CNS_TAG_NODE);
                element1.SetAttribute(CNS_TAG_ATTR_X, it.j);
                element1.SetAttribute(CNS_TAG_ATTR_Y, it.i);
                element1.SetDoubleAttribute(CNS_TAG_ATTR_F, it.F);
                element1.SetDoubleAttribute(CNS_TAG_ATTR_G, it.g);
                if (it.g > 0) {
                    element1.SetAttribute(CNS_TAG_ATTR_PARX, it.parent->j);
                    element1.SetAttribute(CNS_TAG_ATTR_PARY, it.parent->i);
                }
                child->InsertEndChild(element1);
            }
        }
    }
}
