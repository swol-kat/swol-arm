import numpy as np
import kinematics as km

class Leg():
    def __init__(self):
        #Constants 
        self.l1 = 2
        self.l2 = 8 
        self.l3 = 8 

        #initializing thetas resplace with a config function
        self.q=(0,0,0)
        self.p=(-2,0,16)
        

    #forward kinematics 
    def fwkin(self, q=None,joint=4, transform = None):
        if not q:
            q=self.q
        
        T01 = km.htm(0,0,0,0)
        T12 = km.htm(q[0]+(np.pi/2),0,0,-np.pi/2)
        T23 = km.htm(q[1]-(np.pi/2),self.l1,self.l2,0)
        T34 = km.htm(q[2],0,self.l3,0)
        
        transforms = [T01,T12,T23,T34]
        
        if transform:
            return transforms[transform-1]
        
        result = km.htm(0,0,0,0)
        for i in range(joint):
            result = result @ transforms[i]
        
        return result
            
    #inverse kinmatics
    def ikin(self, p=None):
        if not p:
            p=self.p
        return None
    
    #jabobian
    def jacobian(self, q=None):
        if not q:
            q=self.q
        
        T = self.fwkin(q)
        pe = T[0:3,3]
        
        J = np.zeros((6,1))
        for i in range(T.shape[-1]):
            transform = self.fwkin(q,i)
            z = transform[0:3,2]
            pi = transform[0:3,3]
            Jp = np.cross(z,pe-pi).reshape((3,1))
            Jo = z.reshape((3,1))
            Ji = np.concatenate((Jp,Jo),axis=0)
            J = np.concatenate((J,Ji),axis=1)
            
        return J[:,1:]
            
        