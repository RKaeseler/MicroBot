"""
    Author: Rasmus Leck Kæseler 
    Date: 2025-03-12
    Version: 1.0
    Description: A forward kinematic model of the micro ABB robot. 
"""

import numpy as np
import json 
import matplotlib.pyplot as plt

class robot_arm: 
    L = json.load(open('robot/lengths.json'))
    
    def __init__(self, ax):
        
        self.q = np.array([0,20,-90,0,0,0])*np.pi/180
        self.forward_kinematics()
        
        [self.robot_lines] = ax.plot([0,self.pA[0], self.pB[0], self.pC[0], self.pD[0], self.pE[0], self.pT[0]], [0, self.pA[1], self.pB[1], self.pC[1], self.pD[1], self.pE[1], self.pT[1]], [0,self.pA[2], self.pB[2], self.pC[2], self.pD[2], self.pE[2], self.pT[2]], 'b') 
        [self.extra_lines] = ax.plot([self.pB[0], self.pS1[0], self.pS2[0], self.pC[0]], [self.pB[1], self.pS1[1], self.pS2[1], self.pC[1]], [self.pB[2], self.pS1[2], self.pS2[2], self.pC[2]], 'ro-')
        
        self.Caxis = []
        for i, c in enumerate(['r', 'b', 'g']) : 
            cax = ax.quiver(self.pT[0], self.pT[1], self.pT[2], self.T[0,i], self.T[1,i], self.T[2,i] , length=20, color=c)
            self.Caxis.append(cax)
        self.ax = ax
        
    def forward_kinematics(self):
        """
            q (_type_): angles of the (actuation) joints, i.e. where motors are attached. These are NOT equal to the DH joints, i.e. the ones used for the kinematic analysis 
        """
        
        """ROBOT:
                    
                  |_| <- End-effektor
                   |
        (q5)  F->  =
        (q4)  E->  O------||-##  <-- D (q3)
                  C -->      O###o               
                            /   / 
                           /   /
          (q1, q2) B-->   O---o   
                          ##   
                          ###
                           ====  <- A (q0)
                          ######
        """        
        self.limit_q()
        q = self.q
        a = self.find_joint3(q)        
        
        self.T0 = self._Rz(q[0]) @ self._T( 0, 0, self.L['world']['l']) 
        self.pA = self.T0[:3,3]
        
        self.T1 = self._T(self.L['base']['l'],0,0) @ self._Ry(q[1])
        Tb = self.T0@self.T1
        self.pB = Tb[:3,3]

        Ts1 = self.T0 @ self._T(self.L['base']['l'],0,0)  @ self._Ry(q[2])@self._T (0, 0, self.L['parallel_arm']['s1'])
        self.pS1 = Ts1[:3, 3]
        
        self.T2 =  self._T (0, 0, self.L['lower_arm']['l']) @ self._Ry(a) 
        Tc = Tb@self.T2
        self.pC = Tc[:3,3]
        
        Ts2 = Tb@self._T (0, 0, self.L['lower_arm']['l']) @ self._Ry(a) @ self._T (0, 0, -self.L['parallel_arm']['s2']) 
        self.pS2 = Ts2[:3, 3]
               

        self.T3 = self._T(-self.L['elbow']['l'], 0, 0) @ self._Rz(q[3])
        Td = Tc@self.T3
        self.pD = Td[:3,3]
        
        
                                
        self.T4 = self._T(0, 0, self.L['upper_arm']['l']) @ self._Ry(q[4])
        self.T5 = self._Rz(q[5])#
        self.T = Td @ self.T4 @ self.T5
        self.pE = self.T[:3,3]
        self.Tool = self.T @ self._T(0,0, self.L['wrist']['l'])
        self.pT = self.Tool[:3,3]

    def limit_q(self):
        Q0_MIN = -np.pi/2; Q0_MAX = np.pi/2;  
        self.q[0] = max(Q0_MIN, min(self.q[0], Q0_MAX))
        
        Q1_MIN = -20*np.pi/180; Q1_MAX = 160*np.pi/180;  
        self.q[1] = max(Q1_MIN, min(self.q[1], Q1_MAX))
        
        Q2_MIN = max(self.q[1]-180*np.pi/180, -135*np.pi/2); 
        Q2_MAX = min(self.q[1]-70*np.pi/180, (180-135)*np.pi/180); # limit it to be 5 degree before q1  
        self.q[2] = max(Q2_MIN, min(self.q[2], Q2_MAX))
        
        
        Q3_MIN = -np.pi/2; Q3_MAX = np.pi/2;  
        Q4_MIN = -np.pi/2; Q4_MAX = np.pi/2;  
        Q5_MIN = -np.pi/2; Q5_MAX = np.pi/2;  

        self.q[3] = max(Q3_MIN, min(self.q[3], Q3_MAX))
        self.q[4] = max(Q4_MIN, min(self.q[4], Q4_MAX))
        self.q[5] = max(Q5_MIN, min(self.q[5], Q5_MAX))

    
    def find_joint3(self, q):
        # Find angle for joint C, given the actuation joints q1 and a2. 

        theta_1 = q[1] - q[2]
        l1 = self.L['lower_arm']['l']
        d1 = self.L['parallel_arm']['s1']
        l2 = self.L['parallel_arm']['l']
        d2 = self.L['parallel_arm']['s2']
        
        """         o
             d2  .' :  
              .'    :
            o       :
            .       :L2
            .       : 
            . L1    :
            .       o
            .     . 
            .   . d1
            . . 
            o
        """
        x = (d1**2 + l1**2 - 2*d1*l1*np.cos(theta_1))**0.5 # Cosine rule
        phi_2 = np.arccos((d2**2 + x**2 - l2**2)/(2*d2*x)) # Cosine rule
        phi_1 = np.arcsin(np.sin(theta_1)*d1/x) # Sine rule
        
        return phi_1+phi_2
        
    def update_robot(self ):
        self.forward_kinematics()
        self.robot_lines.set_data([0,self.pA[0], self.pB[0], self.pC[0], self.pD[0], self.pE[0], self.pT[0]], [0, self.pA[1], self.pB[1], self.pC[1], self.pD[1], self.pE[1], self.pT[1]])
        self.robot_lines.set_3d_properties([0,self.pA[2], self.pB[2], self.pC[2], self.pD[2], self.pE[2], self.pT[2]])
        self.extra_lines.set_data([self.pB[0], self.pS1[0], self.pS2[0], self.pC[0]], [self.pB[1], self.pS1[1], self.pS2[1], self.pC[1]])
        self.extra_lines.set_3d_properties([self.pB[2], self.pS1[2], self.pS2[2], self.pC[2]])
        
        for i, c in enumerate(['r', 'b', 'g']) : 
            self.Caxis[i].remove()
            self.Caxis[i] = self.ax.quiver(self.pT[0], self.pT[1], self.pT[2], self.T[0,i], self.T[1,i], self.T[2,i] , length=20, color=c)
        
        PITCH = np.atan2(-self.T[2,0], np.sqrt(self.T[0,0]**2 + self.T[1,0]**2))
        if np.isclose(np.abs(PITCH), np.pi/2):
            # Gimbal lock case
            ROLL = 0 # Or define one of the angles
            YAW = np.atan2(self.T[0,1], self.T[1,1])
        else:
            ROLL = np.atan2(self.T[2,1], self.T[2,2])
            YAW = np.atan2(self.T[1,0], self.T[0,0])
        
        print("Angles: [{:.2f} , {:.2f} , {:.2f}, {:.2f} , {:.2f} , {:.2f}] degrees ".format(self.q[0], self.q[1], self.q[2], self.q[3], self.q[4], self.q[5]) )
        print("Position: [{:.2f} , {:.2f} , {:.2f}] mm ".format(self.pT[0], self.pT[1], self.pT[2]) )
        print("RPY: [{:.2f} , {:.2f} , {:.2f}] ".format(ROLL, PITCH, YAW) )

                
    def _Rx(self, theta):
        return np.array([[1, 0, 0, 0], [0, np.cos(theta), -np.sin(theta), 0], [0, np.sin(theta), np.cos(theta), 0], [0,0,0,1]])
    def _Ry(self, theta):
        return np.array([[np.cos(theta), 0, np.sin(theta), 0], [0, 1, 0, 0], [-np.sin(theta), 0, np.cos(theta), 0], [0,0,0,1]])
    def _Rz(self, theta):
        return np.array([[np.cos(theta), -np.sin(theta), 0, 0], [np.sin(theta), np.cos(theta), 0, 0], [0, 0, 1, 0], [0,0,0,1]])
    def _T(self, x, y, z):
        return np.array([[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0,0,0,1]])

if __name__ == '__main__':
    robot = robot_arm()
    T = robot.forward_kinematics(np.array([0,20,-90,0,0,0])*np.pi/180)
    robot.plot_robot()
    
