#ifndef _UNION_FIND_HPP_
#define _UNION_FIND_HPP_

// Optional implementation of Union-Find
// Submit blank if you don't implement a
// Union-Find class.

int find(int u, int set[])
{
    int x = u;
    while (set[x] > 0)
    {
        x = set[x];
    }
    return x;
}

void Union(int u, int v, int set[])
{
    if (set[u] < set[v])
    {
        set[u] = set[u] + set[v];
        set[v] = u;
    }
    else
    {
        set[v] += set[u];
        set[u] = v;
    }
}

#endif
