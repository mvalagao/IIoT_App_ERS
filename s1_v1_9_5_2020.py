#!/usr/bin/env python
# scripts/examples/simple_tcp_client.py
import socket
import time
import numpy as np

from opcua import Server
from random import randint
import datetime
import time
import threading
from pymodbus.client.sync import ModbusSerialClient
from pymodbus.client.sync import ModbusTcpClient
from struct import *

#Variables: registros que se van a leer del medidor

IEM_reg1=3059	#Potencia activa total
IEM_reg2=3067	#Potencia Reactiva total
IEM_reg3=3083	#Factor de potencia
IEM_reg4=3109	#Frecuencia
IEM_reg5=45101	#Energia activa total exportada
IEM_reg6=45105	#Energia reactiva total exportada
IEM_registros = [IEM_reg1, IEM_reg2, IEM_reg3, IEM_reg4] #Aca va [reg1, reg2, reg3, reg4, reg5, reg6]


#Variables del PLC (todas tienen un offset de 1, en realidad empieza en 40001


'''
PLC_reg1=40000  #Output pressure
PLC_reg2=40001	#Speed [RPM]
PLC_reg3=40002	#flow [m3/h]
PLC_reg4=40003	#Input pressure [kg/cm2]
PLC_registros=[PLC_reg1, PLC_reg2, PLC_reg3, PLC_reg4]
'''


PLC_reg1=0  #Output pressure [kg/cm2]
PLC_reg2=1	#Speed [RPM]
PLC_reg3=2	#flow [m3/h]
PLC_reg4=3	#Input pressure [kg/cm2]
PLC_registros=[PLC_reg1, PLC_reg2, PLC_reg3, PLC_reg4]



#Vectores para guardar luego en TXT
#IEM
IEM_v1_vector=[]
IEM_v2_vector=[]
IEM_v3_vector=[]
IEM_v4_vector=[]
IEM_v5_vector=[]
IEM_v6_vector=[]

#PLC
PLC_v7_vector=[]
PLC_v8_vector=[]
PLC_v9_vector=[]
PLC_v10_vector=[]
PLC_v11_vector=[]





#Definicion de cliente Modbus RTU con libreria Pymodbus
client = ModbusSerialClient(
    method='rtu',
    port='/dev/ttyS1',
    baudrate=9600,
    timeout=1,
    parity='N',
    stopbits=1,
    bytesize=8
)

#Conexion modbus TCP con PLC/modsim
	


server = Server()

url="opc.tcp://192.168.0.103:4840" #ip de maquina virtual/linux/raspberry
server.set_endpoint(url)

name = "OPCUA_LCA_UNPSJB"
addspace = server.register_namespace(name)

node = server.get_objects_node()
#print(node)

Param = node.add_object(addspace, "Parameters")

IEM_v1= Param.add_variable(addspace,"IEM1_P_act",0)
IEM_v2= Param.add_variable(addspace,"IEM2_P_react",0)
IEM_v3= Param.add_variable(addspace,"IEM3_FP",0)
IEM_v4= Param.add_variable(addspace,"IEM4_Frec",0)
IEM_v5= Param.add_variable(addspace,"IEM5_Energ_act",0)
IEM_v6= Param.add_variable(addspace,"IEM6_Energ_react",0)
PLC_v7= Param.add_variable(addspace,"PLC7_out_press",0)
PLC_v8= Param.add_variable(addspace,"PLC8_speed",0)
PLC_v9= Param.add_variable(addspace,"PLC9_flow",0)
PLC_v10=Param.add_variable(addspace,"PLC10_in_press",0)
PLC_v11=Param.add_variable(addspace,"PLC11_hydraulic_power",0)

IEM_v1.set_writable()
IEM_v2.set_writable()
IEM_v3.set_writable()
IEM_v4.set_writable()
IEM_v5.set_writable()
IEM_v6.set_writable()
PLC_v7.set_writable()
PLC_v8.set_writable()
PLC_v9.set_writable()
PLC_v10.set_writable()
PLC_v11.set_writable()


server.start()
print("Servidor OPC-UA iniciado en:{}".format(url))
#LECTURA SERIE RS-485
if client.connect(): # Trying for connect to Modbus Server/Slave
#Aca lee el medidor de energia
	def contar():
		r=0 #variable para contar
		contador=0
		while contador<10000:
			for x in IEM_registros:
				contador+=1
				r+=1
				time.sleep(1)
				res=client.read_holding_registers(address=x, count=2, unit=1)
				vector=np.array(res.registers)		
				i1= int(vector[0])
				i2= int(vector[1])
				if r==1:
					f = unpack('>f',pack('>HH',i1,i2))[0]
					IEM_v1.set_value(f)
					IEM_v1_vector.append(f)
					print(f)
				if r==2:
					f = unpack('>f',pack('>HH',i1,i2))[0]
					IEM_v2.set_value(f)
					IEM_v2_vector.append(f)
					print(f)
				if r==3:
					f = unpack('>f',pack('>HH',i1,i2))[0]
					IEM_v3.set_value(f)
					IEM_v3_vector.append(f)
					print(f)
				if r==4:
					f = unpack('>f',pack('>HH',i1,i2))[0]
					IEM_v4.set_value(f)
					IEM_v4_vector.append(f)
					print(f)				  
					r=0
	
				np.savetxt('IEM_data_v1.dat',IEM_v1_vector)
				np.savetxt('IEM_data_v2.dat',IEM_v2_vector)
				np.savetxt('IEM_data_v3.dat',IEM_v3_vector)
				np.savetxt('IEM_data_v4.dat',IEM_v4_vector)
				#np.savetxt('IEM_data_v5.dat',IEM_v5_vector)
				#np.savetxt('IEM_data_v6.dat',IEM_v6_vector)
				
				
				
				
				
	hilo1 = threading.Thread(target=contar)
	hilo1.start()
	print("Hilo iniciado")
	#LECTURA MODBUS TCP
#Aca lee el PLC porque es mas rapido. 
	clienttcp = ModbusTcpClient('192.168.0.101') #Ip del plc/modsim para la conexion por tcp
	connection = clienttcp.connect()
	i=1
	while i<100000:
                res = clienttcp.read_holding_registers(address=0, count=5, unit=1)
                print(res.registers)
                PLC_v7.set_value(res.registers[0])			#se mueven los valores para que salgan en variables OPC UA
                PLC_v7_vector.append(res.registers[0]/1000)
                np.savetxt('PLC_data_v7.dat',PLC_v7_vector)
                PLC_v8.set_value(res.registers[1])
                PLC_v8_vector.append(res.registers[1]/1000)
                np.savetxt('PLC_data_v8.dat',PLC_v8_vector)
                PLC_v9.set_value(res.registers[2])
                PLC_v9_vector.append(res.registers[2]/1000)
                np.savetxt('PLC_data_v9.dat',PLC_v9_vector)
                PLC_v10.set_value(res.registers[3])
                PLC_v10_vector.append(res.registers[3]/1000)
                np.savetxt('PLC_data_v10.dat',PLC_v10_vector)
                xpresion=0
                xcaudal=3
                presion=res.registers[xpresion]				#calculo de presion hidraulica
                caudal=res.registers[xcaudal]
                potencia=round(((presion*caudal*98.0665)/3600),2)
                #print(potencia)
                PLC_v11.set_value(potencia)
                i+=1
                #np.savetxt('PLC_data_v11.dat',PLC_v11_vector)
                time.sleep(0.1)

                
                
                
	# np.savetxt('PLC_data_v7.dat', PLC_v7_vector)
	# np.savetxt('PLC_data_v8.dat', PLC_v8_vector)
	# np.savetxt('PLC_data_v9.dat', PLC_v9_vector)
	# np.savetxt('PLC_data_v10.dat', PLC_v10_vector)
	# np.savetxt('PLC_data_v11.dat', PLC_v11_vector)
                
	    
server.stop()
	#sock.close()
	
