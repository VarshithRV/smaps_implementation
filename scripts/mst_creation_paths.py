import heapq

def generate_mst(graph):
    mst = {}
    visited = set()
    start_node = list(graph.keys())[0]
    heap = [(0, start_node, None)]  # (weight, current_node, parent_node)

    while heap:
        weight, current_node, parent_node = heapq.heappop(heap)

        if current_node not in visited:
            visited.add(current_node)

            if parent_node is not None:
                mst.setdefault(parent_node, []).append(current_node)

            for neighbor in graph[current_node]:
                if neighbor not in visited:
                    heapq.heappush(heap, (0, neighbor, current_node))

    return mst

def find_all_paths(graph, start, end, path=[]):
    path = path + [start]
    if start == end:
        return [path]
    paths = []
    for node in graph[start]:
        if node not in path:
            newpaths = find_all_paths(graph, node, end, path)
            for p in newpaths:
                paths.append(p)
    return paths

def find_leafs(mst):
    leafs = []
    for values in mst.values():
        for value in values:
            if value not in mst.keys():
                leafs.append(value)
    return leafs

def add_leafs(mst,leafs):
    for leaf in leafs:
        mst[leaf] = []    
    return mst

def find_paths(graph,root):
    mst = generate_mst(graph)
    leafs = find_leafs(mst)
    mst = add_leafs(mst,leafs)
    paths = []
    for leaf in leafs:
        path = find_all_paths(mst, root, leaf)
        paths.append(path)
    return paths


def main():
    graph = {'n1': ['n2', 'n3'], 'n2': ['n1', 'n3', 'n4'], 'n3': ['n1', 'n2'], 'n4': ['n2', 'n5'], 'n5': ['n4']}
    paths = find_paths(graph,'n1')
    print(paths)

if __name__ == "__main__":
    main()
