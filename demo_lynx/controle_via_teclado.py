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

#posições iniciais
t_0 = 1500
t_1 = 1500
t_2 = 1500
t_3 = 600
t_4 = 1500 

##################################
#######    PROGRAMA DEMO   #######
##################################

#POSICAO INICIAL PARA TODOS OS SERVOS
HOME_POS = '#0P1500#1P1500#2P1500#3P600#4P1500T1500'
q1 = 0.0
q2 = 0.0
q3 = 0.0
q4 = 0.0

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

#Direct Cynematics function
def func_cd(v1,v2,v3,v4):
    '''
	input: joint variables
	return: nothing, so far. Later, it gonna be list of lists (position and orientation)
    '''
    cd = list()
    cd = [[math.cos(v1)*math.cos(v2+v3+v4), -math.sin(v1), math.cos(v1)*math.sin(v2+v3+v4), math.cos(v1)*(87*math.sin(v2+v3+v4)+math.cos(v2)*(186*math.cos(v3)+145)) ],
	  [math.sin(v1)*math.cos(v2+v3+v4), -math.cos(v1), math.sin(v1)*math.sin(v2+v3+v4), math.sin(v1)*(87*math.sin(v2+v3+v4)+math.cos(v2)*(186*math.cos(v3)+145))], 
	  [math.sin(v2+v3+v4), 0,-math.cos(v2+v3+v4), 73-87*math.cos(v2+v3+v4)+math.sin(v2)*(186*math.cos(v3)+145)], 
	  [0, 0, 0, 1]]
    print cd
    return;

# This function is called every time a key is presssed
def kbevent(event):

    #estamos usando as variáveis lá de cima (globais)
    global t_0
    global t_1
    global t_2
    global t_3
    global t_4

    global q1
    global q2
    global q3
    global q4

    global running


    # If the ascii value matches spacebar, terminate the while loop
    if event.Ascii == 32:
	running = False

    # If the ascii value matches e, move the wrist up
    if event.Ascii == 101:
	t_3 += 11
	q4 += 1.0
	try:
		#FUNCAO TRAVA (trava) RECEBE COMO PARAMETROS
		#O SERVO E O VALOR DA POSICAO DESEJADA E
		#RETORNA A POSICAO CORRIGIDA DE ACORDO COM OS LIMITES MAX E MIN
		#ANTERIORMENTE ESTABELECIDOS
		
		pos = braco.trava(3,t_3)
		braco.envia_comando('#%dP%dT%d' % (3,t_3 ,500))
		print('Envio de comando com teste de envio e de travas: %s \n' % ('#0P%sT1500' % (pos)))
	except:
		print('Problema no envio do comando\nAbortando o programa...')
	func_cd(q1,q2,q3,q4)
 
    # If the ascii value matches d, MOVE THE wrist DOWN
    if event.Ascii == 100:
	t_3 -=11
	q4 -= 1.0
	try:
		#FUNCAO TRAVA (trava) RECEBE COMO PARAMETROS
		#O SERVO E O VALOR DA POSICAO DESEJADA E
		#RETORNA A POSICAO CORRIGIDA DE ACORDO COM OS LIMITES MAX E MIN
		#ANTERIORMENTE ESTABELECIDOS
		
		pos = braco.trava(3,t_3)
		braco.envia_comando('#%dP%dT%d' % (3,t_3,500))
		print('Envio de comando com teste de envio e de travas: %s \n' % ('#3%sT1500' % (pos)))
	except:
		print('Problema no envwio do comando\nAbortando o programa...')
	func_cd(q1,q2,q3,q4)

     # If the ascii value matches w, move elbow up
    if event.Ascii == 119:
	t_2 -= 11
	q3 += 1.0
	try:
		#FUNCAO TRAVA (trava) RECEBE COMO PARAMETROS
		#O SERVO E O VALOR DA POSICAO DESEJADA E
		#RETORNA A POSICAO CORRIGIDA DE ACORDO COM OS LIMITES MAX E MIN
		#ANTERIORMENTE ESTABELECIDOS
		pos = braco.trava(t_2,1300)
		braco.envia_comando('#%dP%dT%d' % (2,t_2,500))
		print('Envio de comando com teste de envio e de travas: %s \n' % ('#2%sT1500' % (pos)))
	except:
		#t_2 -= 500
		print('Problema no envio do comando\nAbortando o programa...')
	func_cd(q1,q2,q3,q4)

     # If the ascii value matches s, move elbow down
    if event.Ascii == 115:
	t_2 += 11
	q3 -= 1.0
	print("valor de t_2 = ",t_2)
	try:
		#FUNCAO TRAVA (trava) RECEBE COMO PARAMETROS
		#O SERVO E O VALOR DA POSICAO DESEJADA E
		#RETORNA A POSICAO CORRIGIDA DE ACORDO COM OS LIMITES MAX E MIN
		#ANTERIORMENTE ESTABELECIDOS
		
		pos = braco.trava(2,t_2)
		braco.envia_comando('#%dP%dT%d' % (2,t_2,500))
		print('Envio de comando com teste de envio e de travas: %s \n' % ('#2%sT1500' % (pos)))
	except:
		#t_2 +=500
		print('Problema no envio do comando\nAbortando o programa...')
	func_cd(q1,q2,q3,q4)

      # If the ascii value matches q, move shoulder up
    if event.Ascii == 113:
	t_1 += 11
	q2 +=1.0
	try:
 		pos = braco.trava(1,t_1)
		braco.envia_comando('#%dP%dT%d' % (1,t_1,500))
		print('Envio de comando com teste de envio e de travas: %s \n' % ('#1%sT1500' % (pos)))
	except:
		print('Problema no envio do comando\nAbortando o programa...')
	func_cd(q1,q2,q3,q4)


	# if the ascii value matches a, move shoulder down
    if event.Ascii == 97:
	t_1 -= 11
	q2 -= 1.0
	try:
 		pos = braco.trava(1,t_1)
		braco.envia_comando('#%dP%dT%d' % (1,t_1,500))
		print('Envio de comando com teste de envio e de travas: %s \n' % ('#1%sT1500' % (pos)))
	except:
		print('Problema no envio do comando\nAbortando o programa...')
	func_cd(q1,q2,q3,q4)

        #if the ascii value matches o, open claw
    if event.Ascii == 111:
	t_4 -= 77
	try:
		pos = braco.trava(4,t_4)
		braco.envia_comando('#%dP%dT%d' % (4,t_4,500))
		print('Envio de comando com teste de envio e de travas: %s \n' % ('#1%sT1500' % (pos)))
	except:
		print('Problema no envio do comando\nAbortando o programa...')
	func_cd(q1,q2,q3,q4)

      #if the ascii value matches i , close claw
    if event.Ascii == 105:
	t_4 += 77
	try:
		pos = braco.trava(4,t_4)
		braco.envia_comando('#%dP%dT%d' % (4,t_4,500))
	except:
		print('Problema no envio do comando\nAbortando o programa...')
	func_cd(q1,q2,q3,q4)
      
     #if the ascii value matches z, move body anti-clockwise
    if event.Ascii == 122:
	t_0 += 11
	q1 -= 1.0
	try:
		pos = braco.trava(0,t_0)
		braco.envia_comando('#%dP%dT%d' % (0,t_0,500))
	except:
		print('Problema no envio do comando\nAbortando o programa...')
	func_cd(q1,q2,q3,q4)

     #if the ascii value c, move body clockwise
    if event.Ascii == 99:
	t_0 -= 11
	q1 += 1.0
	try:
		pos = braco.trava(0,t_0)
		braco.envia_comando('#%dP%dT%d' % (0,t_0,500))
	except:
		print('Problema no envio do comando\nAbortando o programa...')
	func_cd(q1,q2,q3,q4)
		



# Create hookmanager
hookman = pyxhook.HookManager()
# Define our callback to fire when a key is pressed down
hookman.KeyDown = kbevent
# Hook the keyboard
hookman.HookKeyboard()
# Start our listener
hookman.start()


# Create a loop to keep the application running
running = True
while running:
    time.sleep(0.5)

# Close the listener when we are done
hookman.cancel()

print("ainda tem mais alguycjkflkgjdsfjlgdsjkfghdsjfgkfslds")

'''
#testando cliques no teclado:
if(kbevent(a)):
'''


