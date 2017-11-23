#ifndef CLIST_H
#define CLIST_H

#include<list>
#include"sNode.h"
using namespace std;

class cList
{
public:
    list<Node> List;

public:
    cList(){}
    ~cList()
    {
        List.clear();
    }

    bool find(int i, int j, int pi, int pj);

};

#endif

	
