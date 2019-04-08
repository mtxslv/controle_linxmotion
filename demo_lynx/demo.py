#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jul 16 16:22:48 2018

@author: jean mario m lima
"""

from ufrn_al5d import RoboticArmAL5D
import time


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

##################################
#######    PROGRAMA DEMO   #######
##################################

#POSICAO INICIAL PARA TODOS OS SERVOS
HOME_POS = '#0P1500#1P1500#2P1500#3P1500#4P1500T1500'

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
    
    #############################
    ##### PRIMEIRO COMANDO ######
    #############################
    
    print('\nPRIMEIRO COMANDO - POSICAL INICIAL\n')
    try:
        braco.envia_comando(HOME_POS)
        print(' Envio de comando com teste de envio: %s \n' % (HOME_POS))
    except:
        print('Problema no envio do comando\nAbortando o programa...')
        
    raw_input('Pressione ENTER para continuar...')
    
    #############################
    ##### SEGUNDO COMANDO #######
    #############################
    
    print('\nSEGUNDO COMANDO - MOVER O PUNHO\n')
    print('Espere 2 segundos...\n')
    time.sleep(2)
    print('Envio de comando SEM teste de envio: %s \n' % ('#3P1900T1500'))
    braco.envia_comando('#3P1900T1500')
        
    raw_input("Pressione ENTER para continuar...")
    
    #############################
    ##### TERCEIRO COMANDO ######
    #############################
    
    print('\nTERCEIRO COMANDO - MOVER A GARRA\n')
    print('Espere 2 segundos...\n')
    time.sleep(2)
    try:
        braco.envia_comando('#%dP%dT%d' % (4,2400,1500))
        print('Envio de comando com teste de envio: %s \n' % ('#4P2500T1500'))
    except:
        print('Problema no envio do comando\nAbortando o programa...')
    
    raw_input("Pressione ENTER para continuar...")    
    #############################
    ###### QUARTO COMANDO  ######
    ###### TESTE DE TRAVAS ######
    #############################
    
    print('\nQUARTO COMANDO - MOVER A BASE TESTANDO TRAVAS\n')
    print('Espere 2 segundos...\n')
    time.sleep(2)
    try:
        #FUNCAO TRAVA (trava) RECEBE COMO PARAMETROS
        #O SERVO E O VALOR DA POSICAO DESEJADA E
        #RETORNA A POSICAO CORRIGIDA DE ACORDO COM OS LIMITES MAX E MIN
        #ANTERIORMENTE ESTABELECIDOS
        pos = braco.trava(BAS_SERVO,99999)
        braco.envia_comando('#%dP%dT%d' % (BAS_SERVO,pos,1500))
        print('Envio de comando com teste de envio e de travas: %s \n' % ('#0P%sT1500' % (pos)))
    except:
        print('Problema no envio do comando\nAbortando o programa...')
        
    ##FIM DO PROGRAMA DEMO##
    braco.fecha_porta()
    print('\nAcesso a porta serial /dev/ttyS0 finalizado\n')
    
print('\nPROGRAMA DEMONSTRACAO FINALIZADO\n\n')
    
    
    
    

