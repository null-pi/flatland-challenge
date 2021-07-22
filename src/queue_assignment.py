# class for creating priority queue       
class Queue:
    
    """
    function to initialise priority queue
    """
    def __init__(self):
        self.queue = []
    
    """
    function to add nodes to queue
    
    parameters:
    element : element to add in the queue, a node
    
    time complexity: O(N)
    N is the length of the queue
    """
    def push(self, element):
        new = True          # value for checking whether the element is new
        changed = True      # value for checking whether the element has changed
        
        # iterating over the whole queue to find matching node
        # if matching node is found, swap the matched node with the last node
        # based on the f value or h value if f values are similar
        i = 0
        while i < len(self.queue):
            if self.queue[i] == element:
                if self.queue[i].f > element.f:
                    self.queue[i] = element
                    self.queue[i], self.queue[-1] = self.queue[-1], self.queue[i]
                elif self.queue[i].f == element.f and self.queue[i].h > element.h:
                    self.queue[i] = element
                    self.queue[i], self.queue[-1] = self.queue[-1], self.queue[i]
                else:
                    changed = False
                    
                i = len(self.queue)
                new = False
                
            i = i + 1
        
        # append element to queue since it is new
        if new:
            self.queue.append(element)
        
        # element is a changed value, restructure the priority queue
        # based on f value or h value, if f values are similar
        if changed:
            i = len(self.queue) - 1
            while i != 0:
                if self.queue[i].f < self.queue[i // 2].f:
                    self.queue[i], self.queue[i // 2] = self.queue[i // 2], self.queue[i]
                elif self.queue[i].f == self.queue[i // 2].f and self.queue[i].h < self.queue[i // 2].h:
                    self.queue[i], self.queue[i // 2] = self.queue[i // 2], self.queue[i]
                    
                i = i // 2
    
    """
    function for returning the first node in the priority queue
    
    returns:
    node having the highest priority
    
    time complexity: O(logN)
    N is the length of the queue
    """
    def pop(self):
        # if there are elements in the queue, return the first element or none
        if len(self.queue) >= 1:
            value = self.queue[0]
            
            # iterate over specific elements of the queue to restructure the queue
            # based on f value or h value if f values are similar
            i = 1
            while i * 2 < len(self.queue):
                if self.queue[i].f > self.queue[i * 2].f:
                    self.queue[i], self.queue[i * 2] = self.queue[i * 2], self.queue[i]
                elif self.queue[i].f == self.queue[i * 2].f and self.queue[i].h > self.queue[i * 2].h:
                    self.queue[i], self.queue[i * 2] = self.queue[i * 2], self.queue[i]
                    
                i = i * 2
             
            # exclude the 1st element of the queue   
            self.queue = self.queue[1:]
            
            return value
        else:
            return None