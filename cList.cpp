#include"cList.h"
	
bool cList::find(int i, int j, int pi, int pj)
{
    list<Node>::iterator iter;

    for(iter = List.begin(); iter != List.end(); iter++)
        if ((iter->Parent != NULL)&&(iter->i == i)&&(iter->j == j))
            if ((iter->Parent->i == pi)&&(iter->Parent->j == pj))
                return true;
    return false;
}
