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

def main():
    graph = {'n1': ['n2', 'n3'], 'n2': ['n1', 'n3', 'n4','n5'], 'n3': ['n1', 'n2'], 'n4': ['n2', 'n5'], 'n5': ['n4','n2']}

    mst = generate_mst(graph)
    print(mst)

    
if __name__ == "__main__":
    main()
