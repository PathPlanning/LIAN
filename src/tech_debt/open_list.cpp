#include "open_list.h"
#include "logger.h"

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
        TiXmlElement element(Logger::Tags::node);
        element.SetAttribute(Logger::Tags::x, min.j);
        element.SetAttribute(Logger::Tags::y, min.i);
        element.SetDoubleAttribute(Logger::Tags::F, min.F);
        element.SetDoubleAttribute(Logger::Tags::g, min.g);
        element.SetAttribute(Logger::Tags::parentX, min.parent->j);
        element.SetAttribute(Logger::Tags::parentY, min.parent->i);
        child->InsertEndChild(element);
    }

    for (const auto &i: rows_) {
        if (i.empty()) continue;
        for (const auto &it: i) {
            if (it != min) {
                TiXmlElement element1(Logger::Tags::node);
                element1.SetAttribute(Logger::Tags::x, it.j);
                element1.SetAttribute(Logger::Tags::y, it.i);
                element1.SetDoubleAttribute(Logger::Tags::F, it.F);
                element1.SetDoubleAttribute(Logger::Tags::g, it.g);
                if (it.g > 0) {
                    element1.SetAttribute(Logger::Tags::parentX, it.parent->j);
                    element1.SetAttribute(Logger::Tags::parentY, it.parent->i);
                }
                child->InsertEndChild(element1);
            }
        }
    }
}
