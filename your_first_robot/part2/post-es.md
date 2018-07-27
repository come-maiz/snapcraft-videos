# Su primer robot: introducción al Robot Operating System [2/5]

![ Ruedas del CamJam EduKit #3](https://kyrofa.com/uploads/proclaim/image/image/44/edukit.jpg)

Esta es la segunda entrada del blog de
[esta serie sobre crear su primer robot con ROS y Ubuntu Core](https://kyrofa.com/posts/your-first-robot-a-beginner-s-guide-to-ros-and-ubuntu-core-1-5).
En la [entrada anterior](https://kyrofa.com/posts/your-first-robot-a-beginner-s-guide-to-ros-and-ubuntu-core-1-5)
recorrimos todo el hardware necesario para seguir esta serie, e introdujimos
Ubuntu Core, el sistema operativo para dispositivos de internet de las cosas.
Lo instalamos en nuestra Raspberry Pi, y lo usamos para recorrer las
[hojas de trabajo de CamJam](http://camjam.me/?page_id=1035). En esta entrada
voy a presentarles al [Robot Operating System (ROS)](http://www.ros.org/), y
lo usaremos para mover nuestro robot. Estaremos usando ROS a lo largo del resto
de la serie. Recuerden que esto también es una serie en vídeo, siéntanse libres
de ver la versión en vídeo de esta entrada:

[![Vídeo: Su primer robot, parte 2: introducción al Robot Operating System](https://kyrofa.com/uploads/proclaim/image/image/45/thumbnail.jpg)](https://www.youtube.com/watch?v=Sw33EbZHris&list=PL1LO5F1-Jh8JfpHpsKCtUSaaSxVQUUOYw)

## ¿Qué es el Robot Operating System?

Simplificándolo, ROS es un conjunto de bibliotecas y herramientas de código
abierto para simplificar el desarrollo de robots. También brinda
infraestructura para conectar distintos componentes robóticos juntos. Por
ejemplo, si recorrieron todas las hojas de trabajo de CamJam (en particular
la #9), han escrito un único script de Python que es responsable de un montón
de cosas: controlar los motores, leer del detector de linea, leer del sensor
ultrasónico, etc. ¿Qué pasa si agregamos un control inalámbrico a la mezcla?
Este script rápido se vuelve complicado, y si quisieran cambiar un componente
por otro, necesitarían reescribir todo; i.e. estos componentes lógicamente
diferentes están estrechamente acoplados juntos, ya que están en el mismo
script.

![Diagrama de comunicación de ROS](https://kyrofa.com/uploads/proclaim/image/image/43/ros.png)

ROS brinda infraestructura de comunicación que permite extraer diferentes
lógicas en sus propios módulos, y que se comuniquen entre ellos de una forma
estandarizada, como se muestra en la imagen de arriba. Por ejemplo, si
quisiera cambiar los sensores ultrasónicos, y reescribir el módulo de
«distancia», podría hacerlo sin tener que tocar ninguno de los otros módulos
siempre y cuando el nuevo módulo de distancia hable de la misma forma que el
viejo.

Esto tendrá más sentido cuando nos sumerjamos, entonces, ¿Empezamos?

## Paso 1: instalar ROS en la Raspberry Pi

ROS tiene
[tres versiones soportadas actualmente](https://wiki.ros.org/Distributions):
Indigo Igloo, Kinetic Kame, y Lunar Loggerhead. Ubuntu Core serie 16 (la
que estamos usando) es Ubuntu Xenial, lo que limita nuestras opciones a
Kinetic y Lunar. Lunar es técnicamente nueva y más brillante, pero como Ubuntu,
ROS tiene versiones de soporte a largo plazo que son soportadas por un periodo
de tiempo extendido, y Kinetic es la más reciente de estas. Como resultado,
usaremos Kinetic aquí.

Entren por SSH en sus Pi, y abran un shell clásico:

```
$ sudo classic
```

Sigamos la
[guía de instalación de ROS Kinetic](https://wiki.ros.org/kinetic/Installation/Ubuntu).
ROS mantiene su propio repositorio de paquetes Debian, que agregaremos a
nuestro sistema:

```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

Luego necesitamos agregar las claves de ese repositorio a la lista de claves
que aceptaremos (esto verifica que los paquetes en el repositorio realmente
provienen de ROS):

```
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
```

Ahora actualizaremos el índice de todos los repositorios que hemos configurado,
ya que acabamos de agregar uno:

```
$ sudo apt update
```

Ahora instalemos ROS. Como verán en la guía de instalación, hay un montón de
meta paquetes disponibles (paquetes que existen sólo para traer otros
paquetes). Instalemos el más pequeño y sin extras, `ros-kinetic-ros-base`,
que usará cerca de 700MB. También instalaremos g++, el compilador de C++ (es
requerido, aunque estamos escribiendo Python):

```
$ sudo apt install g++ ros-kinetic-ros-base
```

En este punto, ROS está instalado con éxito, pero ninguna de sus herramientas
están disponibles para ser ejecutadas. Esto es porque ROS se instala a sí
mismo en lo que se llama un «espacio de trabajo», y brinda un script de shell
que activa ese espacio. Podemos asegurarnos de que ese espacio se active al
iniciar sesión agregándolo al archivo `.bashrc` en nuestro directorio de
inicio.

```
$ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

Ahora deberían poder ejecutar `roscore` sin problemas:

```
$ roscore
<...>
SUMMARY
========
PARAMETERS
* /rosdistro: kinetic
* /rosversion: 1.12.12
NODES
auto-starting new master
process[master]: started with pid [4987]
ROS_MASTER_URI=http://localhost.localdomain:11311/
setting /run_id to 1db9f4c6-e044-11e7-9931-b827eba43643
process[rosout-1]: started with pid [5000]
started core service [/rosout]
```

Deténganlo presionando CTRL+C.

## Paso 2: conociendo ROS

Una de las razones por las que ROS me gusta tanto (y, creo, una de las razones
por las que es tan popular) es que su documentación introductoria es
fantástica. Tiene un
[fenomenal conjunto de tutoriales](https://wiki.ros.org/ROS/Tutorials) que les
lleva de no saber absolutamente nada a sentirse más o menos confortables con
todo el sistema. Cada uno es fácilmente digerible en unos pocos minutos. Como
son tan buenos, en lugar de tratar de duplicar su arduo trabajo aquí, ustedes
nada más deberían empezar en el inicio y seguirlos por lo menos hasta que
completen el #13, «Examinando un publicador y suscriptor simples». Noten que
hay dos trayectorias de tutoriales en paralelo, una usa C++ y la otra usa
Python. En esta serie vamos a estar usando Python, entonces no se preocupen por
las de C++ a menos que les interesen.

## Paso 3: configurar Python

Ahora que hemos ganado algo de familiaridad con ROS, ya casi es tiempo de hacer
que nuestro robot se mueva usándolo. Sin embargo, hay algo que tenemos que
hacer primero. De vuelta en la hoja de trabajo #1 de CamJam, ellas mencionan lo
siguiente:

> «Cuando la Raspberry Pi fue publicada por primera vez, algunas de las
> bibliotecas importantes de Python sólo estaban disponibles para Python 2.7
> Sin embargo, casi todas las bibliotecas y todas las que usamos en estas hojas
> de trabajo están disponibles para Python 3.2. Hemos decidido que todo el
> código de este EduKit será desarrollado para Python 3.2.»
>
> ~ Hoja de trabajo #1 de CamJam

Eso está muy fino y elegante, y estoy de acuerdo, pero desafortunadamente los
enlaces de Python para ROS sólo están soportados de forma oficial en Python 2,
entonces necesitamos usar Python 2 de ahora en adelante en lugar de Python 3.
No se preocupen, todo el código de las hojas de trabajo debería seguir
funcionando, pero esto quiere decir que necesitamos instalar la versión de
RPi.GPIO para Python 2 (sólo tenemos la versión de Python 3 en este momento):

```
$ sudo apt install python-dev python-pip python-setuptools
$ pip install RPi.GPIO
```

## Paso 4: crear un paquete de ROS para nuestro robot

Muy bien, ¡divirtámonos un poco! Vamos a reescribir el código que escribimos
para la hoja de trabajo #7 de CamJam usando ROS. Vamos a agregar un poco de
manejo de mensajes, de forma que, usando ROS, podemos dirigir el robot para
que se mueva hacia adelante, gire a la izquierda, gire a la derecha, etc.

El primer paso es crear un nuevo espacio de trabajo. Aprendieron cómo hacer
esto en el primer tutorial de ROS. Yo llamaré al mío «edukit_bot_ws», si llaman
al suyo de forma diferente recuerden cambiar las direcciones:

```
$ mkdir -p ~/edukit_bot_ws/src
$ cd ~/edukit_bot_ws/src
$ catkin_init_workspace
```

Ahora, creemos un nuevo paquete en este espacio de trabajo. Yo llamaré al mío
«edukit_bot», y tiene tres dependencias `rospy` (los enlaces de Python para
ROS), `std_msgs` (los mensajes estándar de ROS, e.g. números, cadenas de
caracteres, etc.), y `python-rpi.gpio` (RPi.GPIO, que usamos para acceso GPIO):

```
$ cd ~/edukit_bot_ws/src
$ catkin_create_pkg edukit_bot rospy std_msgs -s python-rpi.gpio
```

## Paso 5: escribir el nodo ROS

Es tiempo de escribir algo de código. Primero, creemos un nuevo script de
Python en el directorio `src/` del paquete ROS:

```
$ touch ~/edukit_bot_ws/src/edukit_bot/src/driver_node
```

Las hojas de trabajo de CamJam no discutieron esto, pero si hacemos que el
script sea ejecutable, podemos ejecutarlo directamente en lugar de llamarlo con
`python path/to/script.py`. Hagamos eso:

```
$ chmod a+x ~/edukit_bot_ws/src/edukit_bot/src/driver_node
```

Abran el script en un editor de textos, y hagan que se vea como este (noten
que el paquete entero usado en esta entrada está
[disponible para referencia](https://github.com/kyrofa/your-first-robot/tree/master/part_2/edukit_bot)):

```
#!/usr/bin/env python

import rospy
from std_msgs.msg import String

import RPi.GPIO as GPIO

# Set the GPIO modes
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Set variables for the GPIO motor pins
pinMotorAForwards = 10
pinMotorABackwards = 9
pinMotorBForwards = 8
pinMotorBBackwards = 7

# How many times to turn the pin on and off each second
Frequency = 20
# How long the pin stays on each cycle, as a percent (here, it's 30%)
DutyCycle = 30
# Setting the duty cycle to 0 means the motors will not turn
Stop = 0

# Set the GPIO Pin mode to be Output
GPIO.setup(pinMotorAForwards, GPIO.OUT)
GPIO.setup(pinMotorABackwards, GPIO.OUT)
GPIO.setup(pinMotorBForwards, GPIO.OUT)
GPIO.setup(pinMotorBBackwards, GPIO.OUT)

# Set the GPIO to software PWM at 'Frequency' Hertz
pwmMotorAForwards = GPIO.PWM(pinMotorAForwards, Frequency)
pwmMotorABackwards = GPIO.PWM(pinMotorABackwards, Frequency)
pwmMotorBForwards = GPIO.PWM(pinMotorBForwards, Frequency)
pwmMotorBBackwards = GPIO.PWM(pinMotorBBackwards, Frequency)

# Start the software PWM with a duty cycle of 0 (i.e. not moving)
pwmMotorAForwards.start(Stop)
pwmMotorABackwards.start(Stop)
pwmMotorBForwards.start(Stop)
pwmMotorBBackwards.start(Stop)

# Turn all motors off
def StopMotors():
    pwmMotorAForwards.ChangeDutyCycle(Stop)
    pwmMotorABackwards.ChangeDutyCycle(Stop)
    pwmMotorBForwards.ChangeDutyCycle(Stop)
    pwmMotorBBackwards.ChangeDutyCycle(Stop)

# Turn both motors forwards
def Forwards():
    pwmMotorAForwards.ChangeDutyCycle(DutyCycle)
    pwmMotorABackwards.ChangeDutyCycle(Stop)
    pwmMotorBForwards.ChangeDutyCycle(DutyCycle)
    pwmMotorBBackwards.ChangeDutyCycle(Stop)

# Turn both motors backwards
def Backwards():
    pwmMotorAForwards.ChangeDutyCycle(Stop)
    pwmMotorABackwards.ChangeDutyCycle(DutyCycle)
    pwmMotorBForwards.ChangeDutyCycle(Stop)
    pwmMotorBBackwards.ChangeDutyCycle(DutyCycle)

# Turn left
def Left():
    pwmMotorAForwards.ChangeDutyCycle(Stop)
    pwmMotorABackwards.ChangeDutyCycle(DutyCycle)
    pwmMotorBForwards.ChangeDutyCycle(DutyCycle)
    pwmMotorBBackwards.ChangeDutyCycle(Stop)

# Turn Right
def Right():
    pwmMotorAForwards.ChangeDutyCycle(DutyCycle)
    pwmMotorABackwards.ChangeDutyCycle(Stop)
    pwmMotorBForwards.ChangeDutyCycle(Stop)
    pwmMotorBBackwards.ChangeDutyCycle(DutyCycle)

# Message handler
def CommandCallback(commandMessage):
    command = commandMessage.data
    if command == 'forwards':
        print('Moving forwards')
        Forwards()
    elif command == 'backwards':
        print('Moving backwards')
        Backwards()
    elif command == 'left':
        print('Turning left')
        Left()
    elif command == 'right':
        print('Turning right')
        Right()
    elif command == 'stop':
        print('Stopping')
        StopMotors()
    else:
        print('Unknown command, stopping instead')
        StopMotors()

rospy.init_node('driver')

rospy.Subscriber('command', String, CommandCallback)

rospy.spin()
print('Shutting down: stopping motors')
StopMotors()
GPIO.cleanup()
```

Mucho de esto debería ser familiar, pero dividámoslo en pedazos.

```
#!/usr/bin/env python

import rospy
from std_msgs.msg import String
```

La primera línea de este archivo se llama
[shebang](https://es.wikipedia.org/wiki/Shebang). Ya que hemos marcado este
archivo como ejecutable por si mismo, esto define el intérprete que ejecutará
este programa. En este caso, le estamos diciendo que necesita el comando
`python`.

Luego importamos `rospy`, que incluye los enlaces de Python para ROS, e
importamos el mensaje `String` de `std_msgs` de ROS. Usaremos ambos un poco
más adelante en el programa.

```
import RPi.GPIO as GPIO

# Set the GPIO modes
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# <...no necesitan ver todo esto de nuevo...>

# Turn Right
def Right():
    pwmMotorAForwards.ChangeDutyCycle(DutyCycle)
    pwmMotorABackwards.ChangeDutyCycle(Stop)
    pwmMotorBForwards.ChangeDutyCycle(Stop)
    pwmMotorBBackwards.ChangeDutyCycle(DutyCycle)
```

Esta sección entera fue copiada literalmente de la hoja de trabajo #7 de
CamJam. Está explicada ahí, así que no repetiré la explicación aquí.

```
# Message handler
def CommandCallback(commandMessage):
    command = commandMessage.data
    if command == 'forwards':
        print('Moving forwards')
        Forwards()
    elif command == 'backwards':
        print('Moving backwards')
        Backwards()
    elif command == 'left':
        print('Turning left')
        Left()
    elif command == 'right':
        print('Turning right')
        Right()
    elif command == 'stop':
        print('Stopping')
        StopMotors()
    else:
        print('Unknown command, stopping instead')
        StopMotors()
```

Aquí hay una parte nueva específica de ROS. La función `CommandCallback` se
crea para manejar un mensaje `String`. Simplemente revisa los datos (i.e., la
cadena de caracteres misma) contenidos dentro del mensaje, y toma la acción
apropiada. Por ejemplo, si la cadena de caracteres es la palabra «forward»,
mueve el robot hacia adelante llamando a la función `Forwards` creada en la
hoja de trabajo. De forma similar, si la palabra es «left», gira el robot
hacia la izquierda llamando a la función `Left`. Si el comando no es ninguna
de las palabras reconocidas, la función hace la cosa más segura posible: se
detiene.

```
rospy.init_node('driver')

rospy.Subscriber('command', String, CommandCallback)

rospy.spin()
print('Shutting down: stopping motors')
StopMotors()
GPIO.cleanup()
```

Aquí está la parte principal del programa. Primero que todo, inicializamos
el nodo y le damos un nombre («driver»). Esto inicia la comunicación con el
nodo director de ROS. Luego nos subscribimos a un tema llamado «command» y
especificamos que esperamos que el tema sea un mensaje `String`. Luego
brindamos nuestra función `CommandCallback` para solicitar que sea llamada
cuando un nuevo mensaje entre en este tema.

Luego llamamos `rospy.spin()` el que bloquea y espera a que lleguen mensajes.
Una vez que al nodo se le pide terminar (digamos, con CTRL+C), esta función
terminará, y en ese momento nos aseguraremos que los motores se hayan detenido.
¡No queremos que le robot se aleje corriendo de nosotras!

Estamos listas con nuestro espacio de trabajo por ahora, así que
construyámoslo:

```
$ cd ~/edukit_bot_ws
$ catkin_make
```

## Paso 6: mover el robot usando ROS

En este punto, tenemos un nodo de ROS creado que controlará a nuestro robot
como sea solicitado en un mensaje «command». Sin embargo, usa GPIO que aún
requiere `sudo`. En lugar de intentar de que nuestro espacio de trabajo
funcione usando `sudo`, vamos a cambiar los permisos de GPIO de forma temporal
para que no necesitemos `sudo` (esto se restablecerá al reiniciar):

```
$ sudo chmod a+rw /dev/gpiomem
```

`/dev/gpiomem` es un dispositivo que representa la memoria dedicada a GPIO
(i.e. no otras, más importantes partes de la memoria). Como resultado, esta
operación es relativamente segura, en particular comparada con hacer lo mismo
con e.g. `/dev/mem`.

Muy bien, ¡probémoslo! Necesitarán abrir tres terminales para esto, cada una
usando el shell clásico (recuerden, ejecutar `sudo classic` para entrar al
shell clásico). Primero necesitamos el director de ROS, ya que sin este
publicadores y subscriptores no se pueden encontrar entre ellos. Entonces, en
una terminal, ejecutemos el director de ROS:

```
$ roscore
```

En otra terminal, asegúrense de activar su recién construido espacio de trabajo
y ejecuten nuestro nodo «driver»:

```
$ cd ~/edukit_bot_ws
$ source devel/setup.sh
$ rosrun edukit_bot driver_node
```

Finalmente, en la tercera terminal, vamos a empezar a dar nuestros comandos
para hacer que el robot se mueva. Antes que nada, tomen nota de los temas
que tienen tanto publicadores como subscriptores:

```
$ rostopic list
/command
/rosout
/rosout_agg
```

Noten el tema `/command`. Este es el tema en el que nuestro nodo «driver»
está escuchando, por como el `Subscriber` esta configurado. Está esperando
un mensaje `String` aquí, entonces enviémosle un comando, digamos, para
moverse hacia adelante:

```
$ rostopic pub -1 /command std_msgs/String "forwards"
publishing and latching message for 3.0 seconds
```

Debería notar que el nodo «driver» dice que se está moviendo hacia adelante,
¡y luego las ruedas de su robot deberían empezar a rotar hacia adelante!
Intenten enviar cualquiera de las cadenas de caracteres que manejamos en
`CommandCallbaack` («left», «ḃackwards», etc.), y comandos que saben que son
inválidos para asegurar que se detenga de forma segura.

Felicidades, ¡están aprendiendo ROS rápidamente! En la
[próxima entrada de esta serie](https://kyrofa.com/posts/your-first-robot-the-controller-3-5),
nos liberaremos de las hojas de trabajo de CamJam y correremos por nuestra
propia cuenta. Vamos a introducir el control inalámbrico, y empezaremos a
trabajar para hacer nuestro robot a control remoto usando ROS.
