import time
def create_spanning_tree(graph):
    """
    Create a spanning tree starting from the first node in the graph.
    """
    # Assuming the first node is the root
    root = list(graph.keys())[0]
    paths = []

    def dfs(node, path):
        """
        Depth-first search to find all paths from the root to the leaf nodes.
        """
        path.append(node)
        if not graph[node]: # If the node has no children, it's a leaf node
            paths.append(path.copy())
        else:
            for neighbor in graph[node]:
                dfs(neighbor, path)
        path.pop() # Backtrack

    dfs(root, [])
    return paths

# Example graph
graph5 = {
    'A': ['B', 'C'],
    'B': ['D', 'E'],
    'C': ['F'],
    'D': [],
    'E': ['F'],
    'F': []
}

graph10 = {
    '1':['2','3','4','5'],
    '2':['3','4','7','8','9'],
    '3':['6'],
    '4':['10','5'],
    '5':[],
    '6':[],
    '7':[],
    '8':[],
    '9':[],
    '10':[]
}

graph15 = {
    '1':['2','3','4','5'],
    '2':['3','4','7','8','9'],
    '3':['6'],
    '4':['10','5'],
    '5':[],
    '6':['11','12'],
    '7':[],
    '8':[],
    '9':[],
    '10':[],
    '11':['12','13','14','15'],
    '12':[],
    '13':[],
    '14':[],
    '15':[]
}

graph20 = {
    '1':['2','3','4','5'],
    '2':['3','4','7','8','9'],
    '3':['6'],
    '4':['10','5'],
    '5':[],
    '6':['11','12'],
    '7':['16'],
    '8':['17'],
    '9':['18'],
    '10':['19'],
    '11':['12','13','14','15'],
    '12':[],
    '13':[],
    '14':[],
    '15':[],
    '16':['20'],
    '17':[],
    '18':[],
    '19':[],
    '20':[]
}

graph25 = {
    '1':['2','3','4','5'],
    '2':['3','4','7','8','9'],
    '3':['6'],
    '4':['10','5'],
    '5':[],
    '6':['11','12'],
    '7':['16'],
    '8':['17','16'],
    '9':['18'],
    '10':['19'],
    '11':['12','13','14','15'],
    '12':[],
    '13':[],
    '14':[],
    '15':[],
    '16':['20'],
    '17':['20'],
    '18':['21','22','23'],
    '19':[],
    '20':[],
    '21':['24','22'],
    '22':['24','25'],
    '23':[],
    '24':[],
    '25':[]

}

graph30 = {
    '1':['2','3','4','5'],
    '2':['3','4','7','8','9'],
    '3':['6'],
    '4':['10','5'],
    '5':[],
    '6':['11','12'],
    '7':['16'],
    '8':['17','16'],
    '9':['18'],
    '10':['19'],
    '11':['12','13','14','15'],
    '12':[],
    '13':[],
    '14':[],
    '15':[],
    '16':['20'],
    '17':['20'],
    '18':['21','22','23'],
    '19':['26','28','27'],
    '20':[],
    '21':['24','22'],
    '22':['24','25'],
    '23':[],
    '24':[],
    '25':[],
    '26':['27','29'],
    '27':[],
    '28':['29'],
    '29':['30'],
    '30':[]

}

# Create the spanning tree
def find_duration(graph):
    spanning_tree_duration = time.time()
    spanning_tree = create_spanning_tree(graph)
    spanning_tree_duration = time.time() - spanning_tree_duration
    print(spanning_tree)
    # print(spanning_tree_duration)
    return spanning_tree_duration

duration5=find_duration(graph5)
duration10=find_duration(graph10)
duration15=find_duration(graph15)
duration20=find_duration(graph20)
duration25=find_duration(graph25)
duration30=find_duration(graph30)
print(duration5,duration10,duration15,duration20,duration25,duration30)