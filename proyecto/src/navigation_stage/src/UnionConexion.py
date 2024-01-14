# -*- coding: utf-8 -*-
# from __future__ import print_function

import rospy
import smach_ros
import tf2_ros
import tf2_geometry_msgs
import math
from smach import State,StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist
import actionlib
from actionlib_msgs.msg import GoalStatus
import sys
from geometry_msgs.msg import PoseStamped
import socket
import time # para contar el tiempo que tarda mientras hace el movimiento de emergencia
from kobuki_msgs.msg import Sound
import numpy as np

q_0 = np.array([1.0, 0.0, 0.0, 0.0])
q_90 = np.array([1/np.sqrt(2), 1/np.sqrt(2), 0.0, 0.0])
q_180 = np.array([0.0, 1.0, 0.0, 0.0])
q_270 = np.array([-1/np.sqrt(2), 1/np.sqrt(2), 0.0, 0.0])

PUERTA_BANO_X = 92.2
PUERTA_BANO_Y = 24
PUERTA_BANO_ORIENT = q_90

PUERTA_CUARTO_X = 64
PUERTA_CUARTO_Y = 25
PUERTA_CUARTO_ORIENT = q_90

PUERTA_HABITACION1_X = 32
PUERTA_HABITACION1_Y = 22
PUERTA_HABITACION1_ORIENT = q_0

PUERTA_SALON_X = 38
PUERTA_SALON_Y = 38
PUERTA_SALON_ORIENT = q_270

PUERTA_COCINA_X = 65
PUERTA_COCINA_Y = 36
PUERTA_COCINA_ORIENT = q_270

DESTINO_X_COCINA = 72.9
DESTINO_Y_COCINA = 44.2
DESTINO_X_SALON = 27.2
DESTINO_Y_SALON = 44.4
DESTINO_X_HABITACION1 = 24.9
DESTINO_Y_HABITACION1 = 23.9
DESTINO_X_CUARTO = 77.1
DESTINO_Y_CUARTO = 16.2
DESTINO_X_BANO = 92.2
DESTINO_Y_BANO = 11.7

TOPIC_VEL = "/cmd_vel"
TOPIC_SCAN = '/base_scan'
TOPIC_COLOR = '/color_detected'
TOPIC_CENTROIDE = '/centroide_x'
HABITACION = 0

SIM = True

HOST = "172.20.10.6"  # Dirección loopback // propio equipo
PORT = 8003        # > 1023 (Puerto de escucha abierto)

# Instanciamos un objeto socket que automáticamente se libera/cierra al acabar el bloque with
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind((HOST, PORT))
server.listen(5)  # Escucha
client_socket, client_address = server.accept()
print("Conexion establecida desde {client_address}")

class move_turtlebot(State):
    def __init__(self):
        State.__init__(self, outcomes=['success','end'])
        self.success = False
        self.end = False
        #Para mover al robot, estos valores son "move_base" y MoveBaseAction
        self.client =  actionlib.SimpleActionClient('move_base',MoveBaseAction)
        #esperamos hasta que el nodo 'move_base' este activo`
        self.client.wait_for_server()

    def execute(self, userdata):
        #un MoveBaseGoal es un punto objetivo al que nos queremos mover
        goal = MoveBaseGoal()
        #sistema de referencia que estamos usando
        goal.target_pose.header.frame_id = "map"
        global HABITACION

        if HABITACION == 0:
            goal.target_pose.pose.position.x = DESTINO_X_COCINA
            goal.target_pose.pose.position.y = DESTINO_Y_COCINA
        elif HABITACION== 1:
            goal.target_pose.pose.position.x = DESTINO_X_SALON
            goal.target_pose.pose.position.y = DESTINO_Y_SALON
        elif HABITACION== 2:
            goal.target_pose.pose.position.x = DESTINO_X_BANO
            goal.target_pose.pose.position.y = DESTINO_Y_BANO
        elif HABITACION== 3:
            goal.target_pose.pose.position.x = DESTINO_X_HABITACION1
            goal.target_pose.pose.position.y = DESTINO_Y_HABITACION1
        elif HABITACION== 4:
            goal.target_pose.pose.position.x = DESTINO_X_CUARTO
            goal.target_pose.pose.position.y = DESTINO_Y_CUARTO
        else:
            rospy.loginfo("No te he escuchado bien... ¿Puedes repetir la HABITACION?")
            return 'end'

        #La orientacion es un quaternion. Tenemos que fijar alguno de sus componentes
        goal.target_pose.pose.orientation.w = 1.0
        print("hab... ", HABITACION)
        rospy.loginfo("Sending goal...")
        #enviamos el goal 
        self.client.send_goal(goal)
        #vamos a comprobar cada cierto tiempo si se ha cumplido el goal
        #get_state obtiene el resultado de la accion 
        state = self.client.get_state()
        #ACTIVE es que esta en ejecucion, PENDING que todavia no ha empezado
        while state==GoalStatus.ACTIVE or state==GoalStatus.PENDING:
            rospy.Rate(10)   #nos da la oportunidad de escuchar mensajes de ROS
            state = self.client.get_state()
            
        if self.client.get_state() == GoalStatus.SUCCEEDED:    #si todo ha ido bien -> objetivo conseguido 
            return 'success'
        else:
            return 'end'    #si no se consigue ir a la posicion indicada volveremos a iniciar la busqueda de algo rojo 

class move_turtlebot_house(State):
    def __init__(self):
        State.__init__(self, outcomes=['success','end'])
        self.success = False
        self.end = False
        #Para mover al robot, estos valores son "move_base" y MoveBaseAction
        self.client =  actionlib.SimpleActionClient('move_base',MoveBaseAction)
        #esperamos hasta que el nodo 'move_base' este activo`
        self.client.wait_for_server()

    def execute(self, userdata):
        #un MoveBaseGoal es un punto objetivo al que nos queremos mover
        goal = MoveBaseGoal()
        #sistema de referencia que estamos usando
        goal.target_pose.header.frame_id = "map"

        ####################################################################################
        if HABITACION == 0:
            goal.target_pose.pose.position.x = PUERTA_COCINA_X
            goal.target_pose.pose.position.y = PUERTA_COCINA_Y
            puerta_q = PUERTA_COCINA_ORIENT
        elif HABITACION== 1:
            goal.target_pose.pose.position.x = PUERTA_SALON_X
            goal.target_pose.pose.position.y = PUERTA_SALON_Y
            puerta_q = PUERTA_SALON_ORIENT
        elif HABITACION== 2:
            goal.target_pose.pose.position.x = PUERTA_BANO_X
            goal.target_pose.pose.position.y = PUERTA_BANO_Y
            puerta_q = PUERTA_BANO_ORIENT
        elif HABITACION== 3:
            goal.target_pose.pose.position.x = PUERTA_HABITACION1_X
            goal.target_pose.pose.position.y = PUERTA_HABITACION1_Y
            puerta_q = PUERTA_HABITACION1_ORIENT
        elif HABITACION== 4:
            goal.target_pose.pose.position.x = PUERTA_CUARTO_X
            goal.target_pose.pose.position.y = PUERTA_CUARTO_Y
            puerta_q = PUERTA_CUARTO_ORIENT

        goal.target_pose.pose.orientation.x = puerta_q[3]
        goal.target_pose.pose.orientation.y = puerta_q[2]
        goal.target_pose.pose.orientation.z = puerta_q[1]
        goal.target_pose.pose.orientation.w = puerta_q[0]

        #enviamos el goal 
        self.client.send_goal(goal)
        #vamos a comprobar cada cierto tiempo si se ha cumplido el goal
        #get_state obtiene el resultado de la accion 
        state = self.client.get_state()
        #ACTIVE es que esta en ejecucion, PENDING que todavia no ha empezado
        while state==GoalStatus.ACTIVE or state==GoalStatus.PENDING:
            rospy.Rate(10)   #esto nos da la oportunidad de escuchar mensajes de ROS
            state = self.client.get_state()
        ####################################################################################

        goal.target_pose.pose.position.x = 55.7   
        goal.target_pose.pose.position.y = 31.5
        #La orientacion es un quaternion. Tenemos que fijar alguno de sus componentes
        goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo("Sending goal...")
        #enviamos el goal 
        self.client.send_goal(goal)
        #vamos a comprobar cada cierto tiempo si se ha cumplido el goal
        #get_state obtiene el resultado de la accion 
        state = self.client.get_state()
        #ACTIVE es que esta en ejecucion, PENDING que todavia no ha empezado
        while state==GoalStatus.ACTIVE or state==GoalStatus.PENDING:
            rospy.Rate(10)   #esto nos da la oportunidad de escuchar mensajes de ROS
            state = self.client.get_state()
            
        if self.client.get_state() == GoalStatus.SUCCEEDED:    #si todo ha ido bien -> objetivo conseguido -> acabamos
            return 'success'
        else:
            return 'end'    #si no se consigue ir a la posicion indicada volveremos a iniciar la busqueda de algo rojo 

class buscarPersona(State):
    def __init__(self):
        #inicializamos el estado, cuya condición de salida es orientado
        State.__init__(self, outcomes=['orientado'])    
        self.color_detected = False
        self.pub_vel = rospy.Publisher(TOPIC_VEL, Twist, queue_size=5)  #creamos el publicador con el que publicaremos la velocidad
        self.zona1 = False
        self.zona2 = False
        self.zona3 = False
        self.zona4 = False
        self.zona5 = False
        self.zona6 = False
        self.DISTANCIA = 5
        self.orientado = False
        self.centro_imagen = 480/2
        self.cx = -10000

    def execute(self, userdata):
        self.color_detected = False
        #nos suscribimos a los topics del láser y del color
        self.subScan = rospy.Subscriber(TOPIC_SCAN, LaserScan, self.laser_callback)
        self.subColor = rospy.Subscriber(TOPIC_COLOR, Int32 , self.color_detected_calback)
        self.subCentroide = rospy.Subscriber(TOPIC_CENTROIDE, Int32, self.valor_centroide_calback)
        rate = rospy.Rate(10)   #establecemos la frecuencia del bucle a 10 Hz

        #nos quedamos en el bucle mientras que no se detecte el color 
        while not rospy.is_shutdown() and not self.color_detected:
            rate.sleep()

        #normalmente en un nodo de ROS convencional no nos desuscribimos
        #porque se hace automáticamente al acabar el nodo, pero esto es un estado
        #de la máquina de estados y mejor "limpiar" todo antes de saltar a otro estado
        if self.color_detected: #si se detecta el color 
            #nos desuscribimos
            self.subScan.unregister()   
            self.subColor.unregister()
        
        if self.orientado : #si se encuentra orientado
            self.subCentroide.unregister()
        return "orientado"
        
    
    def laser_callback(self, data):
        #Procesamiento de los datos del láser
        if (data.ranges[1] < self.DISTANCIA or data.ranges[75] < self.DISTANCIA or data.ranges[150] < self.DISTANCIA):
            self.zona1 = True
        else:
            self.zona1 = False
        if (data.ranges[151] < self.DISTANCIA or data.ranges[225] < self.DISTANCIA or data.ranges[300] < self.DISTANCIA):
            self.zona2 = True
        else:
            self.zona2 = False
        if (data.ranges[301] < self.DISTANCIA or data.ranges[375] < self.DISTANCIA or data.ranges[450] < self.DISTANCIA):
            self.zona3 = True
        else:
            
            self.zona3 = False
        if (data.ranges[451] < self.DISTANCIA or data.ranges[525] < self.DISTANCIA or data.ranges[600] < self.DISTANCIA):
            self.zona4 = True
        else:
            self.zona4 = False
        if (data.ranges[601] < self.DISTANCIA or data.ranges[675] < self.DISTANCIA or data.ranges[750] < self.DISTANCIA):
            self.zona5 = True
        else:
            self.zona5 = False
        if (data.ranges[751] < self.DISTANCIA or data.ranges[825] < self.DISTANCIA or data.ranges[899] < self.DISTANCIA):
            self.zona6 = True
        else:
            self.zona6 = False

        # Generación de comandos de velocidad
        velocidad = Twist()
        if self.color_detected == False :
            if self.zona6 and self.zona5 and not self.zona4:
                #self.situacion = "recto"
                velocidad.linear.x = 5
                velocidad.angular.z = 0
                print("recto")
            elif self.zona3 or self.zona4:
                #self.situacion = "girando izquierda"
                velocidad.linear.x = 5
                velocidad.angular.z = 0.45
                print("izq")
            elif self.zona1 or self.zona2 or (not self.zona5 and not self.zona6):
                #self.situacion = "girando derecha"
                velocidad.linear.x = 5
                velocidad.angular.z = -0.45
                print("derecha")
            else:
                #self.situacion = "recto"
                velocidad.linear.x = 5
                velocidad.angular.z = 0
                print("recto2")  
        else :
            velocidad.linear.x = 0
            if self.centro_imagen != self.cx :
                velocidad.angular.z = 0.2
            else :
                velocidad.angular.z = 0
                self.orientado = True
        self.pub_vel.publish(velocidad)

    def color_detected_calback(self, msg):
        self.color_detected = True

    def valor_centroide_calback(self, msg):
        self.cx = msg.data 

class aproximarse(State):
    def __init__(self):
        State.__init__(self, outcomes=['encontrado'])    

        self.pub_vel = rospy.Publisher(TOPIC_VEL, Twist, queue_size=5)  #creamos el publicador con el que publicaremos la velocidad
        self.dist_persona = 6.5
        self.encontrado = False

    def execute(self, userdata):
        self.encontrado = False
        #nos suscribimos a los topics del láser y del color
        self.subScan = rospy.Subscriber(TOPIC_SCAN, LaserScan, self.laser_callback)
        rate = rospy.Rate(10)   #establecemos la frecuencia del bucle a 10 Hz

       #nos quedamos en el bucle 
        while not rospy.is_shutdown() and not self.encontrado:
            rate.sleep()
        #normalmente en un nodo de ROS convencional no nos desuscribimos
        #porque se hace automáticamente al acabar el nodo, pero esto es un estado
        #de la máquina de estados y mejor "limpiar" todo antes de saltar a otro estado
        if self.encontrado: #si ya estamos al lado de la persona 
            #nos desuscribimos
            self.subScan.unregister()   
            
        return "encontrado"
        
    
    def laser_callback(self, data):
        # Generación de comandos de velocidad
        velocidad = Twist()
        print(data.ranges[450])
        if data.ranges[450] > self.dist_persona+2 :
            print("recto1")
            velocidad.linear.x = 4
        elif data.ranges[450] > self.dist_persona :
            velocidad.linear.x = 1
            print("recto2")
        elif data.ranges[450] <= self.dist_persona:
            print("paro")
            velocidad.linear.x = 0
            self.encontrado = True

        self.pub_vel.publish(velocidad)

class escucha(State):
    def __init__(self):
        State.__init__(self, outcomes=['success','end'])
        self.success = False
        self.end = False

    def execute(self, userdata):
        global HABITACION
        funcion = 1
        client_socket.send(str(funcion).encode())
        HABITACION = client_socket.recv(1024)  # Función blocking => espera a recibir 
        HABITACION = int(HABITACION.decode())

        if HABITACION==0 or HABITACION==1 or HABITACION==2 or HABITACION==3 or HABITACION==4 :
            self.success = True
            return 'success'
        else :
            self.end = True
            return 'end'

class completado(State):
    def __init__(self):
        State.__init__(self, outcomes=['success','emer'])
        self.success = False
        self.end = False

    def execute(self, userdata):
        estado=0
        funcion = 2
        client_socket.send(str(funcion).encode())
        estado = client_socket.recv(1024)  # Función blocking => espera a recibir 
        estado = int(estado.decode())
        if estado==1:
            return 'success'
        elif estado==2:
            return 'emer'

class Emergency(State):
    def __init__(self):
        State.__init__(self, outcomes=['timeOut'])
        self.initTime = 0
        
        if(SIM):
            TOPIC_SOUND_REAL = "/mobile_base/commands/sound"
            TOPIC_VEL = "/cmd_vel"
        else:
            TOPIC_VEL = "/mobile_base/commands/velocity"
            self.sound = rospy.Publisher(TOPIC_SOUND_REAL, Twist, queue_size=5)

        self.pub = rospy.Publisher(TOPIC_VEL, Twist, queue_size=5)

    def crearTwist(self, x, y, z, alpha, beta, gamma): # funcion que crea el mensaje que establece las velocidades del robto
        twist_msg = Twist()

        # LINEAL
        twist_msg.linear.x = x
        twist_msg.linear.y = y
        twist_msg.linear.z = z

        # ANGULAR
        twist_msg.angular.x = alpha
        twist_msg.angular.y = beta
        twist_msg.angular.z = gamma 

        return twist_msg

    def execute(self, userdata):
        
        # comienza a girar y emitir sonido
        cmd = self.crearTwist(0,0,0,0,0,5) # giro constante
        self.pub.publish(cmd)

        if(not SIM):
            s = Sound()
            s.value = 4
            self.sound.publish(s)

        # espera 10 secs
        rate = rospy.Rate(10)
        frecuency = 340
        volume = 1
        end_time= time.time() + 10

        while time.time() < end_time:
            self.pub.publish(cmd)
            rate.sleep() 

        # deja de sonar y se para
        cmd = self.crearTwist(0,0.0,0.0,0.0,0.0,0) # giro constante
        self.pub.publish(cmd)

        if(not SIM):
            s = Sound()
            s.value = 0
            self.sound.publish(s)

        return "timeOut"


if __name__ == '__main__':
    rospy.init_node("proyecto")
    sm = StateMachine(outcomes=['stop'])
    with sm:
        StateMachine.add('escucha', escucha(), transitions={'success':'irLugar', 'end': 'irCasa'})
        StateMachine.add('irLugar', move_turtlebot(), transitions={'success':'buscarPersona', 'end': 'stop'})
        StateMachine.add('buscarPersona', buscarPersona(),transitions={'orientado':'aproximar'})
        StateMachine.add('gestos', completado(), transitions={'success':'irCasa', 'emer':'emergencia'})
        StateMachine.add('irCasa', move_turtlebot_house(), transitions={'success':'escucha', 'end': 'stop'})
        StateMachine.add('aproximar', aproximarse(), transitions={'encontrado':'gestos'})
        StateMachine.add('emergencia', Emergency(), transitions={'timeOut':'gestos'})
        
    
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    sm.execute()
    rospy.spin()   
    sis.stop()