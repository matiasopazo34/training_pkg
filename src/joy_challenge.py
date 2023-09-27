#!/usr/bin/env python
import message_filters
import rospy #importar ros para python
from sensor_msgs.msg import Point, Joy
from std_msgs.msg import String, Int32 #importa mensajes de ROS tipo String y Int32
from sensor_msgs.msg import Joy # impor mensaje tipo Joy
from geometry_msgs.msg import Twist # importar mensajes de ROS tipo geometry / Twist
from duckietown_msgs.msg import Twist2DStamped 


class Template(object):
    def __init__(self, args):
        super(Template, self).__init__()
        self.args = args
        #sucribir a joy 
        # self.sub = rospy.Subscriber("/duckiebot/joy" , Joy, self.callback)
        #publicar la intrucciones del control en possible_cmd
        self.publi = rospy.Publisher("/duckiebot/wheels_driver_node/car_cmd", Twist2DStamped, queue_size = "x")
        self.twist = Twist2DStamped()

	mando = message_filters.Subscriber('/duckiebot/joy', Joy)
	distancia = message_filters.Subscriber('/duckiebot/camera_note/image/distancia', Point)
	ts = message_filters.TimeSynchronizer([mando, distancia], 10)
	ts.registerCallback(callback)
	rospy.spin()


    #def publicar(self, msg):
        #self.publi.publish(msg)

    def callback(self,msg):
        # funciones python 
        a = msg.buttons[0]
        freno = msg.buttons[1] #B
        retroceder = msg.axes[5] #L2
        avanzar = msg.axes[2] #r2
        giro = msg.axes [0] 
        # z = msg.axes[]

        #print(freno, y, x)
        self.twist.omega = 0
        self.twist.v = 0

        if freno == 1:
            self.twist.omega = 0
            self.twist.v = 0
        
        self.publi.publish(self.twist) #mensaje que se publica, luego el duckie lo recibe

        if a == 1:
            print(freno)
            self.twist.omega = giro*10
            self.twist.v = (avanzar-1)*5*-1 - (retroceder-1)*5*-1

        
        self.publi.publish(self.twist) #mensaje que se publica, luego el duckie lo recibe
        




def main():
    rospy.init_node('test') #creacion y registro del nodo!

    obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

    #objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

    rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
    main()
