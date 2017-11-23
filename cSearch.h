#ifndef CSEARCH_H
#define CSEARCH_H

#include "cList.h"
#include "cMap.h"
#include "cLogger.h"
#include "cXmlLogger.h"
#include "gl_const.h"
#include "searchresult.h"

class cSearch
{
public:
    cSearch(){};
    virtual ~cSearch(){};
    virtual void addOpen(Node& newNode) = 0;
    virtual SearchResult startSearch(cLogger *Log, const cMap &Map) = 0;

    SearchResult sresult;
};

#endif
