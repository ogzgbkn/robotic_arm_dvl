import numpy as np

#all angles in degrees
t1=0 
t2=90
t3=-90
t4=0
t5=90
t6=0

#converting to radians
t1=(t1/180.0)*np.pi
t2=(t2/180.0)*np.pi
t3=(t3/180.0)*np.pi
t4=(t4/180.0)*np.pi
t5=(t5/180.0)*np.pi
t6=(t6/180.0)*np.pi

#rotation matrices for each one
R0_1=[[np.cos(t1),0,np.sin(t1)],[np.sin(t1),0,-np.cos(t1)],[0,1,0]]
R1_2=[[np.cos(t2),-np.sin(t2),0],[np.sin(t2),np.cos(t2),0],[0,0,1]]
R2_3=[[np.cos(t3),0,np.sin(t3)],[np.sin(t3),0,-np.cos(t3)],[0,1,0]]
R3_4=[[np.cos(t4),0,-np.sin(t4)],[np.sin(t4),0,np.cos(t4)],[0,-1,0]]
R4_5=[[-np.sin(t5),0,np.cos(t5)],[np.cos(t5),0,np.sin(t5)],[0,1,0]]
R5_6=[[0,np.cos(t6),-np.sin(t6)],[0,np.sin(t6),np.cos(t6)],[1,0,0]]

R0_2=np.dot(R0_1,R1_2)
R0_3=np.dot(R0_2,R2_3)
R0_4=np.dot(R0_3,R3_4)
R0_5=np.dot(R0_4,R4_5)
R0_6=np.dot(R0_5,R5_6)

print(R0_6)