#STUDENT NAME: Diana Miranda
#STUDENT NUMBER: 107457

#DISCUSSED TPI-1 WITH: (names and numbers):


from tree_search import *
import math

class OrderDelivery(SearchDomain):

    def __init__(self,connections, coordinates):
        self.connections = connections
        self.coordinates = coordinates
        # ANY NEEDED CODE CAN BE ADDED HERE     
        # 
    def actions(self,state):
        city = state[0]
        actlist = []
        for (C1,C2,D) in self.connections:
            if (C1==city):
                actlist += [(C1,C2)]
            elif (C2==city):
               actlist += [(C2,C1)]
        return actlist 

    def result(self, state, action):
        (C1, C2) = action
        citys = [city for city in state[1] if city != C1]
        if C1 == state[0]:
            return (C2, citys)

    def satisfies(self, state, goal):
        return state[0] == goal and not state[1]

    def cost(self, state, action):
        for (C1, C2, D) in self.connections:
            if (C1, C2) == action or (C2, C1) == action:
                return D

    def heuristic(self, state, goal):
        cost = 0
        for city in state[1]:
            for (C1, C2, D) in self.connections:
                if (C1, C2) == (state[0], city) or (C2, C1) == (state[0], city):
                    cost += D
                    break
            for (C1, C2, D) in self.connections:
                if (C1, C2) == (city, goal) or (C2, C1) == (city, goal):
                    cost += D
                    break
        return cost

    
class MyNode(SearchNode):

    def __init__(self,state,parent,arg3=None,arg4=None,arg5=None,arg6=None):
        super().__init__(state,parent)
        #ADD HERE ANY CODE YOU NEED
        self.depth = arg3
        self.cost = arg4
        self.heuristic = arg5
        self.eval = arg6
        self.markedForDel = False
    
    def in_parent(self, newstate):
        if self.parent == None:
            return False
        if self.parent.state == newstate:
            return True
        return self.parent.in_parent(newstate)
        


class MyTree(SearchTree):

    def __init__(self,problem, strategy='breadth',maxsize=None):
        super().__init__(problem,strategy)
        #ADD HERE ANY CODE YOU NEED
        root = MyNode(problem.initial, None, 0, 0, 0)
        self.open_nodes = [root]
        self.terminals = 0
        self.maxsize = maxsize

    def astar_add_to_open(self,lnewnodes):
        for node in lnewnodes:
            self.open_nodes.append(node)
            self.open_nodes.sort(key= lambda node: (node.cost + node.heuristic, node.state))


    def search2(self):
        while self.open_nodes != []:
            self.terminals = len(self.open_nodes)
            node = self.open_nodes.pop(0)
            if self.problem.goal_test(node.state):
                self.solution = node
                return self.get_path(node)
            
            lnewnodes = []
            self.non_terminals +=1
            for a in self.problem.domain.actions(node.state):
                newstate = self.problem.domain.result(node.state,a)
                if not node.in_parent(newstate):
                    newnode = MyNode(newstate,node, node.depth + 1, node.cost + self.problem.domain.cost(node.state, a), self.problem.domain.heuristic(newstate, self.problem.goal), node.cost + self.problem.domain.cost(node.state, a) + self.problem.domain.heuristic(newstate, self.problem.goal))
                    lnewnodes.append(newnode)
                    
            self.manage_memory()
            self.add_to_open(lnewnodes)

        return None

    def manage_memory(self):
        if self.strategy != 'A*' or self.maxsize is None:
            return  
                
        while self.terminals + self.non_terminals > self.maxsize:
            for node in reversed(self.open_nodes):
                if node.markedForDel == False:
                    node.markedForDel = True
                    nodeToDel = node
                    break

            if nodeToDel.parent == None:
                return

            brothers = []
            for n in self.open_nodes:
                if n.parent == nodeToDel.parent:
                    brothers.append(n)

            if all(node.markedForDel for node in brothers):
                for node in brothers:
                    self.open_nodes.remove(node)
                nodeToDel.parent.eval = min(node.eval for node in brothers)
                self.non_terminals -= 1
                
            
def orderdelivery_search(domain, city, targetcities, strategy='breadth', maxsize=None):
    p = SearchProblem(domain, (city, targetcities), city)
    t = MyTree(p, strategy, maxsize)
    path = t.search2()
    return t, [city for city, _ in path]
