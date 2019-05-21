#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Apr 08 09:12:00 2019
@author: Assis,Arthur,Isaac and Iago based on the file made for jean mario m lima
"""

from ufrn_al5d import RoboticArmAL5D
import time
import pyxhook
import math


###################################
######    CANAIS DOS SERVOS  ###### 
###### E LIMITES DE OPERAÇÃO ######
###################################
#0. BASE
BAS_SERVO = 0
#LIMITES
BAS_MIN = 500
BAS_MAX = 2400

#1. SHOULDER
SHL_SERVO = 1
#LIMITES
SHL_MIN = 1200
SHL_MAX = 2000

#2. ELBOW
ELB_SERVO = 2
#LIMITES
ELB_MIN = 1100
ELB_MAX = 2000

#3. WRIST
WRI_SERVO = 3
#LIMITES
WRI_MIN = 600
WRI_MAX = 2500

#4. GRIPPER
GRI_SERVO = 4
#LIMITES
GRI_MIN = 1300
GRI_MAX = 2400

#PROPRIEDADES DO BRAÇO: SERVOS E LIMITES DE OPERACAO
properties = [BAS_SERVO, BAS_MIN, BAS_MAX,
              SHL_SERVO, SHL_MIN, SHL_MAX,
              ELB_SERVO, ELB_MIN, ELB_MAX,
              WRI_SERVO, WRI_MIN, WRI_MAX,
              GRI_SERVO, GRI_MIN, GRI_MAX]
##################################
#comprimentos de elo
L1 = 28+45
L2 = 145
L3 = 186
L4 = 87
##################################
#posições iniciais
t_0 = 1500
t_1 = 1500
t_2 = 1500
t_3 = 600
t_4 = 1500 


#POSICAO INICIAL PARA TODOS OS SERVOS
HOME_POS = '#0P1500#1P1500#2P1500#3P600#4P1500T1500'
q1 = 0.0
q2 = 90.0
q3 = -90.0
q4 = -90.0

#INICIALIZACAO DO BRACO PASSANDO AS PROPRIEDADES COMO PARAMETRO
braco = RoboticArmAL5D(properties)

#CONFIGURACAO DA PORTA
braco.setup()

#ABRINDO A PORTA
if(braco.abre_porta() == -1):
    print ('Erro abrindo a porta serial /dev/ttyS0\nAbortando o programa...\n')
else: 
    print('PROGRAMA DEMONSTRACAO INICIADO\n\n');
    print ('Porta serial /dev/ttyS0 aberta com sucesso\n')


    print('\nPRIMEIRO COMANDO - POSICAL INICIAL\n')
    try:
        braco.envia_comando(HOME_POS)
        print(' Envio de comando com teste de envio: %s \n' % (HOME_POS))
    except:
        print('Problema no envio do comando\nAbortando o programa...')


###############################
######## NEW FUNCTIONS ########
###############################

#Inverse Cynematics function
def inversa(x,y,z,phi):
  '''
  input: cartesian position in {0}, and angle (in degrees) with the horizontal plane (x0Oy0)
  output: a tuple, containing joint angles (in degrees)
  '''
  phi = math.radians(phi) # The code needs a value in radians
  
  if (x*x+y*y) == 0:
    theta_1 = math.nan
  else:
    theta_1 = math.atan2(y,x)
    theta_1 = math.degrees(theta_1)
  
  '''
  No plano definido por x1Oz1, a dimensão horizontal do triângulo é igual a
   math.sqrt(x*x+y*y) - L4*math.cos(phi), e a dimensão vertical é dada por
   z - L4*math.sin(phi) - L1
  '''
  R1 = math.sqrt ( math.pow(z-L4*math.sin(phi)-L1, 2) + math.pow(math.sqrt(x*x+y*y) - L4*math.cos(phi) , 2) )
  cos_a1 =((L2*L2)+(R1*R1)-(L3*L3))/(2*L2*R1) #não esquecer parênteses no denominador
  a1 = math.degrees(math.atan2(math.sqrt(1-cos_a1*cos_a1),cos_a1)) # alpha 1 in degrees
  alpha = math.degrees( math.atan2( z - L4*math.sin(phi) - L1, math.sqrt(x*x+y*y) - L4*math.cos(phi) ) ) # alpha, in degrees.
  
  theta_2 = alpha - a1 # theta_2, in degrees
  
  theta_3 = 90 + a1 - math.degrees( math.acos( L2*math.sin( math.radians(a1) )/L3 ) ) # theta_3, in degrees
  
  phi = math.degrees( phi )
  
  theta_4 = phi - theta_2 - theta_3 # theta_4, in degrees

  return theta_1,theta_2,theta_3,theta_4


def show_clue():
    print('Waiting instructions...')
    print('Available commands: \n o - ABRE_GARRA() \n c - FECHA_GARRA() \n m - MOVE(X,Y,Z) \n r - REPOUSO()')


def ABRE_GARRA():
    print('abrindo garra...')
    try:
        pos = braco.trava(4,1400) # pretty open claw
	braco.envia_comando('#%dP%dT%d' % (4,1400,500))
        print('Envio de comando com teste de envio e de travas: %s \n' % ('#1%sT1500' % (pos)))
    except:
	print('Problema no envio do comando\nAbortando o programa...')

def FECHA_GARRA():
    print('fechando garra...')
    try:
	pos = braco.trava(4,2200) # pretty close claw
        braco.envia_comando('#%dP%dT%d' % (4,2200,500))
    except:
	print('Problema no envio do comando\nAbortando o programa...')

def MOVE(x,y,z):
    Q1,Q2,Q3,Q4 = inversa(x,y,z,0) # all Qi joint variables in degrees
    print('q1 = ',Q1, '\nq2 = ',Q2,'\nq3 = ',Q3,'\nq4 = ',Q4)

    #pos is defined by a single pulse plus the initial position of each joint.

    pos1=((Q1+90)/0.09)+1500
    pos2=((Q2+90)/0.09)+1500
    pos3=((Q3+90)/0.09)+1500
    pos4=((Q4+90)/0.09)+1500

    if ((pos1>=BAS_MIN)&(pos1<=BAS_MAX)):
		try:
      		  	pos = braco.trava(0,pos1) 
		 	braco.envia_comando('#%dP%dT%d' % (0,pos1,500))
        
    		except:
			print('Problema no envio do comando\nAbortando o programa...')


    if ((pos2>=SHL_MIN)&(pos2<=SHL_MAX)):
		try:
      		  	pos = braco.trava(1,pos2) 
		 	braco.envia_comando('#%dP%dT%d' % (1,pos2,500))
        
    		except:
			print('Problema no envio do comando\nAbortando o programa...')

    if ((pos3>=WRI_MIN)&(pos3<=WRI_MAX)):
		try:
      		  	pos = braco.trava(2,pos3) 
		 	braco.envia_comando('#%dP%dT%d' % (2,pos3,500))
        
    		except:
			print('Problema no envio do comando\nAbortando o programa...')

    if ((pos4>=GRI_MIN)&(pos4<=GRI_MAX)):
		try:
      		  	pos = braco.trava(3,pos4) 
		 	braco.envia_comando('#%dP%dT%d' % (3,pos4,500))
        
    		except:
			print('Problema no envio do comando\nAbortando o programa...')




def REPOUSO():
    print('\n INDO PARA POSICAO DE REPOUSO... \n')
    try:
        braco.envia_comando(HOME_POS)
        print(' Envio de comando com teste de envio: %s \n' % (HOME_POS))
    except:
        print('Problema no envio do comando\nAbortando o programa...')

###############################
######## NEW FUNCTIONS ########
###############################

print('antes do move')
MOVE(L3+L4,0,L1+L2)


print("FIM DO PROGRAMA")

