#include "search_tree.h"

SearchTree::SearchTree() {}

SearchTree::~SearchTree() = default;

size_t SearchTree::sizeOpen() const {
    return open_.size();
}

size_t SearchTree::sizeClose() const {
    return close_.size();
}

bool SearchTree::empty() const {
    return open_.empty();
}

void SearchTree::addOpen(const Node &new_node) {
    open_.push(new_node);
    return;
}

std::optional<Node> SearchTree::getOpen() {
    while (!open_.empty()) {
        Node min1 = open_.top();
        open_.pop();

        if (close_.find(min1) == close_.end()) {
            return std::make_optional(min1);
        }
    }
    return {};
}

void SearchTree::addClose(const Node &new_node) {
    close_.insert({new_node, new_node});
}

Node *SearchTree::findClose(const Node &key) {
    auto node_iter = close_.find(key);
    return (node_iter != close_.end()) ? &node_iter->second : nullptr;
}

std::pair<close_iterator, close_iterator> SearchTree::findCloseRange(const Node &key) {
    return close_.equal_range(key);
}

void SearchTree::writeToXml(TiXmlNode *child) const {
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
        element.SetAttribute(Logger::Tags::parentX, min.j);
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
                element1.SetAttribute(Logger::Tags::parentX, it.j);
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

void SearchTree::saveToLogOpenAndClose(std::shared_ptr<Logger> &logger) const {
    auto space = logger->logSpace<Logger::Levels::low>(Logger::Tags::lowLevel);
    if (!space) {
        return;
    }

    int iterate = 0;
    TiXmlNode *child = nullptr, *curNode = space;

    while ((child = curNode->IterateChildren(child))) {
        iterate++;
    }

    {
        TiXmlElement element(Logger::Tags::step);
        element.SetAttribute(Logger::Tags::number, iterate);
        curNode->InsertEndChild(element);
        curNode = curNode->LastChild();
    }

    {

        TiXmlElement element(Logger::Tags::open);
        curNode->InsertEndChild(element);
        child = curNode->LastChild();
    }

    writeToXml(child);

    {
        TiXmlElement element(Logger::Tags::close);
        curNode->InsertEndChild(element);
        child = curNode->LastChild();
    }

    for (const auto &it: close_) {
        TiXmlElement element(Logger::Tags::node);
        element.SetAttribute(Logger::Tags::parentX, it.second.j);
        element.SetAttribute(Logger::Tags::y, it.second.i);
        element.SetDoubleAttribute(Logger::Tags::F, it.second.F);
        element.SetDoubleAttribute(Logger::Tags::g, it.second.g);
        if (it.second.g > 0) {
            element.SetAttribute(Logger::Tags::parentX, it.second.parent->j);
            element.SetAttribute(Logger::Tags::parentY, it.second.parent->i);
        }
        child->InsertEndChild(element);
    }
}
