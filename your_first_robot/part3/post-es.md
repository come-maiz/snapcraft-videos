# Su primer robot: el control [3/5]

![Control inalámbrico](https://kyrofa.com/uploads/proclaim/image/image/47/controller.jpg)

Esta es la tercera entrada del blog en
[esta serie acerca de crear su primer robot con ROS y Ubuntu Core](https://kyrofa.com/posts/your-first-robot-a-beginner-s-guide-to-ros-and-ubuntu-core-1-5).
En la entrada anterior les presenté el Robot Operating System (ROS), y logramos
que sus robots se movieran al ROSificar una de las hojas de trabajo de CamJam.
Hoy vamos a ir más allá de lo que está en las hojas de trabajo de CamJam, y
trabajaremos hacia tener nuestro robot controlado de forma remota enfocándonos
en nuestro control inalámbrico: obteniendo datos de este y llevándolos a
mensajes de ROS.

Como mencioné en la parte 1, estoy usando el
[control inalámbrico de Pi Hut](https://thepihut.com/collections/raspberry-pi-gaming/products/raspberry-pi-compatible-wireless-gamepad-controller?variant=38135423121),
pero en realidad el único requerimiento para seguir esta entrada de forma
exacta es que se muestre como `/dev/input/jsX` en la Pi (dónde `X` es algún
número, por lo general 0). Tengo un control cableado de Xbox 360 que funciona
de esta misma forma, por ejemplo.

Muy bien, ¡empecemos! Recuerden que esto también es una serie de vídeo,
siéntanse libres de ver la versión en vídeo de esta entrada:

[![Vídeo: su primer robot, parte 3: el control](https://kyrofa.com/uploads/proclaim/image/image/50/thumbnail.jpg)](https://www.youtube.com/watch?v=xRK-tOgzeUo&list=PL1LO5F1-Jh8JfpHpsKCtUSaaSxVQUUOYw)

## Paso 1: obteniendo datos del control

Uno de los beneficios de usar ROS es que muchos de los problemas ya están
resueltos para nosotras. En este caso, ya hay un nodo de ROS que existe
únicamente para leer datos de controles como el nuestro, llamado
[joy](https://wiki.ros.org/joy). Entren en su shell clásico e instálenlo:

```
$ sudo apt install ros-kinetic-joy
```

Inserten el receptor USB del controlador en la Pi, y deberían verlo aparecer
como `/dev/input/js0`. Abran dos terminales más, y entren en su shell clásico
en cada una (de forma que ahora tienen tres). En una terminal, ejecuten el
director:

```
$ roscore
```

En otra terminal, ejecuten el nodo joy:

```
$ rosrun joy joy_node
[ INFO] [1513274242.219112701]: Opened joystick: /dev/input/js0. deadzone_: 0.050000.
```

En la tercera terminal, vamos a ver qué está saliendo del nodo joy. Primero
veamos qué temas están disponibles:

```
$ rostopic list
/diagnostics
/joy
/rosout
/rosout_agg
```

Tomen nota del tema `/joy`. Ese está siendo publicado por el nodo joy. Vamos
a darle un vistazo. Ejecuten:

```
$ rostopic echo /joy
```

Nada debería haber pasado al inicio. Ahora enciendan sus controles, y empiecen
a mover las palancas y presionar botones. Deberían ver toda clase de cosas
empezar a ser escupidas, que se ven como esto:

```
header:
seq: 1
stamp:
secs: 1513274448
nsecs: 114444863
frame_id: ''
axes: [0.0, 0.08846031129360199, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
---
header:
seq: 2
stamp:
secs: 1513274448
nsecs: 130450677
frame_id: ''
axes: [0.0, 0.26209455728530884, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
---
```

Esto es un montón de mensajes `Joy` que representan el estado actual del
control: palancas, botones, todo. Cuando muevan una palanca o presionen un
botón, más mensajes son enviados mostrando el cambio de estado. ¡Eso nos
ahorró un montón de trabajo! Sigan y salgan de `rostopic`, el nodo `joy` y
`roscore` con `CTRL+C`.

## Paso 2: ¿cómo controlamos a nuestro robot?

Entonces ya podemos obtener datos del control, pero esos datos aún están algo
crudos. Recuerden que, al final del día, el robot se mueve y gira a través de
cambios en las velocidades de sus ruedas. ¿Cómo obtenemos velocidades de ruedas
de estos mensajes del control? Podríamos escribir un nuevo nodo que se suscriba
a los mensajes del control, saque los movimientos de la palanca, y los convierta
en velocidades de ruedas. Sin embargo, eso seria acoplar estrechamente el
controlador del robot al control, lo que sería miope. ¿No hay alguna forma más
genérica de solicitar que el robot se mueva y gire? En efecto, la hay.

La respuesta la podemos encontrar al ver cómo haríamos autónomo el robot. Esto
probablemente usaría la
[pila de navegación de ROS](https://wiki.ros.org/navigation), que fue escrita
para robots arbitrarios, y que subraya exactamente cómo deberíamos controlar a
nuestro robot con su primer requerimiento de hardware:

> «[Este software] está destinado tanto para robots de control diferencial y de
> ruedas holonómicas, únicamente. Esto asume que la base móvil es controlada
> enviando los comandos de velocidad deseados para lograr en la forma de:
> velocidad x, velocidad y, velocidad theta.»
>
> ~ La pila de navegación de ROS

Bueno, ¿entonces qué significa esto? El robot que tenemos es
[de control diferencial](https://en.wikipedia.org/wiki/Differential_wheeled_robot).
Esto significa que se puede mover hacia adelante, atrás, y girar hacia la
izquierda y la derecha, todo con ruedas que no se dirigen como en su carro
(lo que sería e.g. dirección
[Ackermann](https://es.wikipedia.org/wiki/Geometr%C3%ADa_de_Ackermann)).
Esto lo hace rotando las ruedas a diferentes velocidades y/o en diferentes
direcciones. No puede, sin embargo, moverse lado a lado. Los sistemas de ruedas
holonómicas son más mágicos, en que se pueden mover hacia adelante, hacia
atrás, girar a la izquierda y a la derecha, y moverse lado a lado. Estos por lo
general usan ya sea
[ruedas omni](https://en.wikipedia.org/wiki/Omni_wheel) o
[ruedas mecanum](https://en.wikipedia.org/wiki/Mecanum_wheel).

Podemos hacer esto más fácil de entender dibujando los movimientos posibles por
cada tipo de control:

![Sistemas de control diferencial y holonómico](https://kyrofa.com/uploads/proclaim/image/image/49/both_drives.png)

En la izquierda tenemos el sistema de control diferencial, como nuestro robot.
Este se puede mover de forma linear a lo largo de la línea roja que etiquetamos
como X (i.e. puede moverse hacia adelante y hacia atrás), y puede moverse de
forma angular (rotar) alrededor del círculo verde que etiquetamos θ (theta). En
la derecha tenemos el sistema de control holonómico, que puede hacer
exactamente igual que en el sistema diferencial, pero también se puede mover a
lo largo de la línea azul que etiquetamos como Y. (i.e. se puede mover lado a
lado)

De forma concreta, la pila de navegación de ROS soporta ambos sistemas
brindando comandos que son velocidades en cada una de estas direcciones
(X, Y y  θ). Entonces, ¡hagamos que nuestro robot soporte esos tipos de
comandos! Ya que nuestro sistema no soporta velocidades Y, sólo usaremos X y θ.
ROS tiene un mensaje estándar que se usa exactamente para este tipo de dato: el
[mensaje Twist](https://docs.ros.org/api/geometry_msgs/html/msg/Twist.html),
que consiste en un conjunto de velocidades lineares y angulares. Entonces
convirtamos nuestros mensajes Joy en mensajes Twist.

## Paso 3: convertir mensajes Joy en mensajes Twist

Podría sonar como un disco rayado, pero adivinen qué: ¡esto ya existe, también!
Se llama
[teleop_twist_joy](https://wiki.ros.org/teleop_twist_joy). Instalémoslo:

```
$ sudo apt install ros-kinetic-teleop-twist-joy
```

Ahora abran cuatro terminales ejecutando el shell clásico (ya lo sé, esto se
está volviendo ridículo). En una, ejecute `roscore` de nuevo:

```
$ roscore
```

En otra, ejecute el nodo joy de nuevo:

```
$ rosrun joy joy_node
[ INFO] [1513274242.219112701]: Opened joystick: /dev/input/js0. deadzone_: 0.050000.
```

En la tercera, ejecute el recientemente instalado nodo teleop:

```
$ rosrun teleop_twist_joy teleop_node
[ INFO] [1513286128.076671780]: Teleop enable button 0.
[ INFO] [1513286128.077269945]: Linear axis x on 1 at scale 0.500000.
[ INFO] [1513286128.077666187]: Angular axis yaw on 0 at scale 0.500000.
```

Y por último, en la cuarta vamos a tocarla. Primero, vean los temas
disponibles:

```
$ rostopic list
/cmd_vel
/diagnostics
/joy
/rosout
/rosout_agg
```

Sabemos que el tema `/joy` contiene los datos del control, pero `/cmd_vel` es
nuevo. Ese es el nombre convencional para comandos de velocidad basados en
Twist. Vamos a ver esos:

```
$ rostopic echo /cmd_vel
```

Como antes, nada debería ocurrir al inicio. Enciendan sus controles y empiecen
a mover la palanca izquierda. ¡Uf!, eso no hace nada tampoco. Como el nodo
teleop fue escrito con controles remotos en mente, implemente un botón de
seguridad como un tipo de «dispositivo de persona muerta» si así se quiere.
De forma predeterminada, este es el botón 0, que en mi control es el botón X.
Manténganlo presionado, y LUEGO muevan la palanca izquierda, y deberían empezar
a ver comandos de velocidad pasando por la terminal, como este:

```
linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
---
linear:
  x: 0.0225298888981
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
---
```

Notarán que aunque hay un montón de componentes en este mensaje, sólo dos de
los componentes realmente cambian dependiendo de cómo mueven la palanca: el
componente linear X, y el componente angular Z. Esto en realidad se relaciona
con la imagen en el paso 2: X corresponde a la línea roja X que dibujamos, y Z
corresponde al círculo verde θ. Entonces los valores X significan «mover hacia
adelante o atrás así de rápido» y los valores Z significan «rotar hacia la
izquierda o derecha así de rápido». ¡Justo lo que necesitamos!

En la
[siguiente entrada de esta serie](https://kyrofa.com/posts/your-first-robot-the-driver-4-5),
reescribirán el nodo «controlador» simple que creamos la vez anterior para
aceptar mensajes Twist y convertirlos en velocidades de ruedas.
