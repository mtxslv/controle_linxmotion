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
WRI_MIN = 500
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


#Inverse Cynematics function
def func_ic(x,y,z,phi):
    '''
	   input: position and angle
	   output: Tuple containing angular positions, in radians
    '''
    theta_1 = math.atan2(y,x) # relation just valid for math.sqrt(y*y+x*x) != 0
    R1 = math.sqrt(math.pow(z-L4*math.sin(phi)-L1,2)+math.pow(x-L4*math.cos(phi),2))
    cos_a1 = (L2*L2+R1*R1-L3*L3)/(2*L2*R1) # a1 = math.atan2( math.sqrt( 1-cos_a1*cos_a1) , cos_a1 )
    theta_3 = math.pi/2 + math.atan2( math.sqrt( 1-cos_a1*cos_a1) , cos_a1 ) - math.atan2( math.sqrt(1- (L2*cos_a1/L3)*(L2*cos_a1/L3)) , L2*cos_a1/L3 ) 
    theta_2 = math.atan2(z-L4*math.sin(phi)-L1,x-L4*math.cos(phi))-theta_3
    theta_4 = phi-theta_2-theta_3
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
    print('nao esta movendo ainda')

def REPOUSO():
    print('\n INDO PARA POSICAO DE REPOUSO... \n')
    try:
        braco.envia_comando(HOME_POS)
        print(' Envio de comando com teste de envio: %s \n' % (HOME_POS))
    except:
        print('Problema no envio do comando\nAbortando o programa...')

# This function is called every time a key is presssed
def kbevent(event, params):
    #if the ascii value matches o, open claw
    if event.Ascii == 111:
        ABRE_GARRA()
    #if the ascii value matches i, close claw 
    if event.Ascii == 105:
        FECHA_GARRA()
    # print key info
    print(event)
    # If the ascii value matches spacebar, terminate the while loop
    if event.Ascii == 32:
        params['running'] = False

parameters={'running':True}
# Create hookmanager
hookman = pyxhook.HookManager(parameters=True)
# Define our callback to fire when a key is pressed down
hookman.KeyDown = kbevent
# Define our parameters for callback function
hookman.KeyDownParameters = parameters
# Hook the keyboard
hookman.HookKeyboard()
# Start our listener
hookman.start()

# Create a loop to keep the application running
while parameters['running']:
    time.sleep(0.1)
# Close the listener when we are done
hookman.cancel()

print("FIM DO PROGRAMA")




