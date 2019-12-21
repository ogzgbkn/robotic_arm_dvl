import numpy as np
from random import randint
from sympy import *



class robotic_arm():

	t0 =Symbol('t0', real=True)	
	t1 =Symbol('t1', real=True)
	t2 =Symbol('t2', real=True)
	t3 =Symbol('t3', real=True)
	t4 =Symbol('t4', real=True)
	t5 =Symbol('t5', real=True)
  	
  	
	# the matrix we need to use 	
	def genT(theta, a, d, alpha):
		T =  np.array([[cos(theta), (-1*sin(theta)*cos(alpha)),sin(theta)*sin(alpha) , cos(theta)*a],
    	[sin(theta), (cos(theta)*cos(alpha)), -1*sin(alpha)*cos(theta), a*sin(theta)],
    	[0, sin(alpha), cos(alpha), d],
    	[0, 0, 0, 1]])

		return T

	"""t=[t1,t2+(pi/2),t3+(3*pi/4),t4,t5+(pi/2),t6]
	a=[0,400,0,150,0,0]
	d = [0,0,300,0,100,0]
	alpha=[3*pi/4,0,3*pi/4,pi/2,3*pi/4,0]"""
	
	# Lengths for the links are estimated values.
	t=[t0,(pi/2)+t1,(-pi/2)+t2,t3,(pi/2)+t4,(-pi/2)+t5]
	a =[0,400,300,0,0,0]
	d = [1,0,0,150,0,150]
	alpha=[pi/2,0,pi/2,pi/2,pi/2,0]

	

	T0_1=genT(t[0],a[0],d[0],alpha[0])
	T1_2=genT(t[1],a[1],d[1],alpha[1])
	T2_3=genT(t[2],a[2],d[2],alpha[2])
	T3_4=genT(t[3],a[3],d[3],alpha[3])
	T4_5=genT(t[4],a[4],d[4],alpha[4])

	T0_2=np.dot(T0_1,T1_2)
	T0_3=np.dot(T0_2,T2_3)

	z=T0_3[1][3]

	print(z)

	#x# -300*sin(t1)*sin(t2)*cos(t0) - 400*sin(t1)*cos(t0) + 300*cos(t0)*cos(t1)*cos(t2)


	#y# -300*sin(t0)*sin(t1)*sin(t2) - 400*sin(t0)*sin(t1) + 300*sin(t0)*cos(t1)*cos(t2)

	#z# 300*sin(t1)*cos(t2) + 300*sin(t2)*cos(t1) + 400*cos(t1) + 1
