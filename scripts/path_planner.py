from __future__ import print_function

import numpy as np



class node(object):
  def __init__(self,pos,prev):
    self.prev=prev
    self.pos=pos


class planner_algorithm(object):
    def __init__(self,graph):
        self.graph=graph

    def find_childs(self,parent_pos):
        aux_list=[]
        if self.graph[parent_pos[0],parent_pos[1]] & (1 << 0):
            aux_list.append((np.uint(parent_pos[0]-1),parent_pos[1]))
        if self.graph[parent_pos[0],parent_pos[1]] & (1 << 2):
            aux_list.append((parent_pos[0],np.uint(parent_pos[1]-1)))
        if self.graph[parent_pos[0],parent_pos[1]] & (1 << 1):
            aux_list.append((np.uint(parent_pos[0]+1),parent_pos[1]))
        if self.graph[parent_pos[0],parent_pos[1]] & (1 << 3):
            aux_list.append((parent_pos[0],np.uint(parent_pos[1]+1)))
        return aux_list

    def path_find(self,origin, final):
        shape= self.graph.shape
        state=-np.ones(shape,dtype=np.int8)
        cost=-np.ones(shape,dtype=np.int)
        cola=[node(origin,None)]
        state[origin[0],origin[1]]=0
        cost[origin[0],origin[1]]=0
        final_node=None
        while final_node==None and len(cola)>0:
            aux_node=cola.pop(0)
            aux_node_position=aux_node.pos
            childs=self.find_childs(aux_node_position)
            for i in childs:
                if final==i:
                    final_node=node(i,aux_node)
                    cost[i[0],i[1]]=cost[aux_node_position[0],aux_node_position[1]]+1
                    state[i[0],i[1]]=0
                    break
                if state[i[0],i[1]]==-1:
                    cost[i[0],i[1]]=cost[aux_node_position[0],aux_node_position[1]]+1
                    state[i[0],i[1]]=0
                    cola.append(node(i,aux_node))
        return final_node,cost