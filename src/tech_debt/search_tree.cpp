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

void SearchTree::saveToLogOpenAndClose(std::shared_ptr<Logger> &logger) const {
    auto space = logger->logSpace<CN_LOGLVL_LOW>(CNS_TAG_LOWLEVEL);
    if (!space) {
        return;
    }

    int iterate = 0;
    TiXmlNode *child = nullptr, *curNode = space;

    while ((child = curNode->IterateChildren(child))) {
        iterate++;
    }

    {
        TiXmlElement element(CNS_TAG_STEP);
        element.SetAttribute(CNS_TAG_ATTR_NUM, iterate);
        curNode->InsertEndChild(element);
        curNode = curNode->LastChild();
    }

    {

        TiXmlElement element(CNS_TAG_OPEN);
        curNode->InsertEndChild(element);
        child = curNode->LastChild();
    }

    writeToXml(child);

    {
        TiXmlElement element(CNS_TAG_CLOSE);
        curNode->InsertEndChild(element);
        child = curNode->LastChild();
    }

    for (const auto &it: close_) {
        TiXmlElement element(CNS_TAG_NODE);
        element.SetAttribute(CNS_TAG_ATTR_X, it.second.j);
        element.SetAttribute(CNS_TAG_ATTR_Y, it.second.i);
        element.SetDoubleAttribute(CNS_TAG_ATTR_F, it.second.F);
        element.SetDoubleAttribute(CNS_TAG_ATTR_G, it.second.g);
        if (it.second.g > 0) {
            element.SetAttribute(CNS_TAG_ATTR_PARX, it.second.parent->j);
            element.SetAttribute(CNS_TAG_ATTR_PARY, it.second.parent->i);
        }
        child->InsertEndChild(element);
    }
}
