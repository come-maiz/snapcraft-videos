# Su primer robot: el controlador [4/5]

![CamJam EduKit #3 ensamblado](https://kyrofa.com/uploads/proclaim/image/image/66/edukit.jpg)

Este es la cuarta entrada de blog en
[esta serie acerca de crear su primer robot con ROS y Ubuntu Core](https://kyrofa.com/posts/your-first-robot-a-beginner-s-guide-to-ros-and-ubuntu-core-1-5).
En la
[entrada anterior](https://kyrofa.com/posts/your-first-robot-the-controller-3-5)
trabajamos para obtener datos del control inalámbrico y llevarlos a ROS en un
formato pensado para controlar robots de conducción diferencial como el
nuestro: el mensaje `Twist`.
Hoy vamos a crear un nodo ROS que toma ese mensaje `Twist` y lo convierte en
las velocidades de ruedas apropiadas para controlar nuestro robot. Normalmente
en este punto les enlazaría con el vídeo correspondiente, pero temo que cambios
recientes en el equipo me han impedido tener el tiempo para hacer uno. Una vez
que tenga algo de tiempo actualizaré esta entrada.

En todo caso, hay varias formas en las que podemos atacar este problema. Si
estuviéramos trabajando en hacer nuestro robot autónomo, podríamos usar
[ros_control](https://wiki.ros.org/ros_control). Sin embargo, agregaría
bastantes partes más a esta serie, y además no tenemos los sensores necesarios
para hacer que valga la pena. Déjenme explicarlo.

## Ciclo abierto versus ciclo cerrado

Controlar un robot siempre es un ciclo de tareas que se hacen una y otra vez.
Por ejemplo, una vez que terminemos con esta entrada de blog, nuestro ciclo se
verá algo como así:

1. Leer datos del control
2. Convertir los datos del control en datos de velocidad
3. Convertir datos de velocidad en velocidades de ruedas
4. Convertir velocidades de ruedas en ciclos de trabajo
5. Cambiar el ciclo de trabajo de la rueda
6. Volver al paso 1

Hay dos tipos de ciclos de control: *abierto* o *cerrado*. La diferencia es que
un ciclo *cerrado* tiene retroalimentación. Como un ejemplo simple, digamos que
ustedes quieren hacer chocolate caliente (¡es esa época del año!) en el horno
de microondas. Si tienen un microondas como el mio, ponen el chocolate
adentro, lo dejan funcionando por algunos minutos y luego vuelven cuando está
listo. El ciclo de control de ese microondas se ve algo como así:

1. Configure el nivel de potencia en 10
2. ¿Ya se acabó el tiempo? Si no, regrese al paso 1

No hay un mecanismo de retroalimentación aquí: este es un ejemplo de control de
ciclo *abierto*. El microondas está calentando el chocolate sólo por un
periodo establecido de tiempo, y eso es todo lo que le importa. De hecho, con
frecuencia encuentro que no está suficientemente caliente y termino poniéndolo
de vuelta.

Sin embargo, tal vez ustedes tienen un microondas fino. Tal vez incluye un
sensor térmico de algún tipo, tal vez un termómetro que se inserta en el
chocolate cuando lo introduce en el microondas (qué asco). Ahora el
microondas puede determinar la temperatura del chocolate, lo que significa
que ya no hay que poner el tiempo, sólo tienen que decirle al microondas qué
tan caliente quieren su chocolate. Pongan su temperatura deseada, y ahora el
ciclo de control del microondas podría verse algo como así:

1. Configure el nivel de potencia en 10
2. ¿Qué tan caliente está el chocolate? ¿Ya alcanzó la temperatura deseada? Si
   no, vuelva al paso 1

Este sensor térmico le da la microondas algo de datos para retroalimentar su
ciclo, lo que significa que este es un control de ciclo cerrado. Este es
probablemente el más simple que se puedan imaginar, pero hay un campo entero
dedicado al control de ciclo cerrado, llamado
[teoría del control](https://es.wikipedia.org/wiki/Teor%C3%ADa_del_control)

En nuestro robot no tenemos sensores que nos den una forma razonable de
determinar un cambio en la posición. Por ejemplo, si tuviéramos sensores en
las ruedas que nos dijeran qué tan rápido están girando, podríamos usar
nuestro conocimiento del tamaño de las ruedas para estimar qué tan rápido
vamos (esto se llama
[odometría](https://es.wikipedia.org/wiki/Odometr%C3%ADa). Ya que no tenemos
esos datos, no tenemos nada que retroalimentar: estamos limitadas a un control
de ciclo abierto en este momento.

Es por esto que usar `ros_control` no vale el esfuerzo: no tenemos los datos
necesarios para cerrar el ciclo, entonces mejor escribamos nuestro propio
controlador de ciclo abierto super sencillo. Entonces, ¿empezamos?

## Prerrequisitos

En realidad sólo hay un nuevo prerrequisito aquí: un poco más de conocimiento de
Python. Lo hemos mantenido tan sencillo como pudimos hasta ahora, pero es
tiempo de que nuestro código crezca un poco. Ya están familiarizadas con las
funciones luego de seguir las hojas de trabajo. Hoy vamos a usar clases.
Lean
[este tutorial sobre clases (en inglés)](https://docs.python.org/2/tutorial/classes.html),
si es necesario.

## Paso 1: calcular velocidades de las ruedas a partir de Twist

Gracias a la parte 3, tenemos el control generando mensajes Twist, los que
representan las velocidades linear y angular deseadas para el robot («ir así de
rápido hacia adelante/atrás», «gire así de rápido hacia la izquierda/derecha»).
Al final del día, la única forma en la que el robot se mueve es cambiando la
dirección y velocidad de las ruedas. Necesitamos desarrollar una forma de
convertir las velocidades ordenadas en velocidades de ruedas que puedan lograr
lo ordenado. Para hacer esto, necesitamos un poquito de matemáticas.

Hablemos primero de velocidad linear, ya que es bastante simple. Si queremos
ordenar al robot que se mueva hacia adelante 1 metro por segundo (m/s), ¿en qué
dirección (hacia adelante o atrás) y a qué velocidad (en m/s) necesitamos que
cada rueda se mueva? La respuesta debería ser intuitiva: ambas ruedas necesitan
girar hacia adelante a 1 m/s. Entonces la formula de la parte de velocidad
linear de la velocidad de las ruedas es simplemente:

![Fórmula de velocidad linear](https://kyrofa.com/uploads/proclaim/image/image/56/linear_velocity.png)

La velocidad angular es un poco más complicada.

Si ordenamos al robot girar hacia la izquierda 90 grados por segundo, ¿en qué
dirección (hacia adelante o atrás) necesitamos que cada rueda se mueva? La
respuesta es bastante intuitiva, pero depende de cómo queremos que gire nuestro
robot. Hay dos opciones: giros de una única rueda o giros de doble rueda.

![Diagramas de giros de una única rueda y de doble rueda](https://kyrofa.com/uploads/proclaim/image/image/51/both_rotations.png)

Como pueden ver en el dibujo de arriba, los giros de una única rueda involucran
dejar una rueda inmóvil, y girar la otra. Los giros de doble rueda involucran
rotar ambas ruedas en direcciones opuestas, y así compartir el trabajo del
giro. Si escogemos el giro de una única rueda, entonces la respuesta a la
pregunta sería «la rueda izquierda no gira del todo, y la derecha gira hacia
adelante». Sin embargo, yo prefiero el método de giro de doble rueda, entonces
mi respuesta es «la rueda izquierda gira hacia atrás, y la derecha gira hacia
adelante».

![Diagrama de giro de doble rueda](https://kyrofa.com/uploads/proclaim/image/image/52/double_rotations_circle.png)

Bueno, tenemos las direcciones de las ruedas. Ahora vamos por la velocidad: si
ordenamos al robot girar hacia la izquierda 90 grados por segundo, ¿a qué
velocidad (en metros por segundo) necesitamos que gire cada rueda? Noten que
este giro es una porción de un círculo, fuera del cual se traza la ruta deseada
de nuestras ruedas. Recuerden la fórmula de la circunferencia de un círculo:

![Fórmula de circunferencia](https://kyrofa.com/uploads/proclaim/image/image/57/circumference.png)

¿Qué tan grande es la porción del círculo que queremos girar? Bueno, sabemos
que 360 grados es un círculo entero, entonces podemos usar una proporción y
combinarla con la circunferencia para crear nuestra fórmula para la velocidad
de las ruedas (que en realidad sólo es la fórmula del largo de un arco):

![Fórmula de velocidad de ruedas](https://kyrofa.com/uploads/proclaim/image/image/59/angular_velocity.png)

Muy bien, sólo nos queda una incógnita más en esta fórmula: el radio. ¿Qué es
eso? Pueden ver que en realidad es la mitad de la distancia entre las dos
ruedas (llamada la «vía»). Entonces nuestra fórmula se convierte en esto:

![Fórmula de velocidad de ruedas con vía](https://kyrofa.com/uploads/proclaim/image/image/60/angular_velocity_track.png)

¡Increíble! Usando esto, ahora pueden responder la pregunta. Yo medí la vía de
mi robot y es 0.091 metros. Usando esto, calculé que la rueda izquierda debería
girar hacia atrás a 0.071 m/s, y la delantera debería girar hacia adelante a la
misma velocidad.

En realidad podemos simplificar esta fórmula, porque el mensaje `Twist`
especifica velocidades angulares en
[radianes](https://es.wikipedia.org/wiki/Radi%C3%A1n) por segundo, en lugar de
en grados. 360 grados = 2π radianes. Si cambiamos nuestra proporción para usar
radianes, las cosas se empiezan a cancelar de una forma bella:

![Fórmula de velocidad de ruedas en radianes](https://kyrofa.com/uploads/proclaim/image/image/61/angular_velocity_radians.png)

Además de esto,
[las convenciones de ROS](http://www.ros.org/reps/rep-0103.html)
siguen la
[regla de la mano derecha](https://es.wikipedia.org/wiki/Regla_de_la_mano_derecha),
que significa que una velocidad angular significa un giro contrario a las
manecillas del reloj, un una velocidad angular negativa significa un giro
siguiendo las manecillas del reloj. Decidamos ya que un valor positivo
significa que nuestras ruedas giren hacia adelante, y un valor negativo que
giren hacia atrás. Usando estos dos hechos junto a nuestras fórmulas, podemos
encontrar formulas para las velocidades de ambas de nuestras ruedas:

![Fórmula de velocidades de ambas ruedas](https://kyrofa.com/uploads/proclaim/image/image/63/wheel_velocities.png)

## Paso 2: convertir velocidades de ruedas en ciclos de trabajo

Ya casi salimos de la matemática, pero tenemos un pequeño problema que
requiere nuestra atención. Nuestras velocidades de ruedas están en metros por
segundo, pero como aprendieron en la hoja de trabajo de CamJam #7, la forma de
controlar los motores es aplicando un ciclo de trabajo entre 0 y 100 (detenidos
y a toda velocidad, respectivamente). ¿Cómo obtenemos ciclos de trabajo a
partir de metros por segundo? Aquí es donde el control de ciclo cerrado
vendría a ser útil: si tuviéramos sensores de velocidad de ruedas, podríamos
comparar qué tan rápido está yendo el robot con qué tan rápido debería ir, y
decir «necesita un ciclo de trabajo más alto», o «necesita un ciclo de trabajo
más pequeño». Sin embargo, ya discutimos que esto necesita ser un *control
de ciclo abierto*. Necesitamos obtener nuestro ciclo de trabajo haciendo
algunas suposiciones en lugar de usar datos de retroalimentación. Haremos
esto determinando la máxima velocidad posible de nuestro robot, y obteniendo el
ciclo de trabajo dividiendo la velocidad solicitada por la velocidad máxima.

¿Cómo determinamos la velocidad máxima de nuestro robot? La forma más precisa
sería midiéndola. En esencia, medir un metro, poner el vehículo al inicio,
usar un cronómetro y hacer una carrera.

Pero, sólo por simplicidad, podemos hacer un poco de trampa. Si notaron en la
[parte 3](https://kyrofa.com/posts/your-first-robot-the-controller-3-5), de
forma predeterminada los datos del control tienen un valor máximo de 0.5.
Entonces, si vemos un 0.5 sabemos que el control llegó al máximo. Entonces, si
sólo suponemos que la velocidad máxima de nuestro robot es 0.5 metros por
segundo, llevar el control al máximo también llevará a nuestro robot a la
máxima velocidad. Es un poco sucio, pero funciona para nuestro caso ya que sólo
vamos a usar el control de todas formas.

![Fórmula de ciclo de trabajo](https://kyrofa.com/uploads/proclaim/image/image/65/duty_cycle.png)

## Paso 3: agregar geometry_msgs como dependencia

Estamos a punto de reescribir el nodo «controlador» que empezamos en la
[parte 2](https://kyrofa.com/posts/your-first-robot-introduction-to-the-robot-operating-system-2-5)
para manejar mensajes `Twist` en lugar de mensajes `String`. Esto requiere
cambiar la dependencia de `std_msgs` en nuestro paquete edukit_bot a
`geometry_msgs`. Abran el archivo `package.xml` del paquete `edukit_bot` y
háganlo que se vea algo como esto:

```
<?xml version="1.0"?>
<package>
  <name>edukit_bot</name>
  <version>0.1.0</version>
  <description>The edukit_bot package</description>

  <maintainer email="you@you.com">You</maintainer>

  <license>GPLv3</license>

  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>rospy</build_depend>

  <exec_depend>rospy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>python-rpi.gpio</exec_depend>
</package>
```

Es decir, «Requiero `catkin y rospy` para construir, y requiero `rospy`,
`geometry_msgs`y `python-rpi.gpio` para ejecutar."

## Paso 4: escribir el controlador

Ahora que salimos de la matemática, vamos a escribir nuestro controlador ROS
que la usará. De hecho, vamos a reescribir el nodo «controlador» en el
paquete `edukit_bot` que empezamos en la
[parte 2](https://kyrofa.com/posts/your-first-robot-introduction-to-the-robot-operating-system-2-5).
Primero, activamos el espacio de trabajo:

```
$ cd ~/edukit_bot_ws
$ source devel/setup.sh
```

Ahora, hacemos que el archivo
`~/edukit_bot_ws/src/edukit_bot/src/driver_node` se vea así (note que el
paquete entero creado en esta parte está
[disponible para referencia](https://github.com/kyrofa/your-first-robot/tree/master/part_4/edukit_bot)):

```
#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

import RPi.GPIO as GPIO


# Set the GPIO modes
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

_FREQUENCY = 20


def _clip(value, minimum, maximum):
    """Ensure value is between minimum and maximum."""

    if value < minimum:
        return minimum
    elif value > maximum:
        return maximum
    return value

class Motor:
    def __init__(self, forward_pin, backward_pin):
        self._forward_pin = forward_pin
        self._backward_pin = backward_pin

        GPIO.setup(forward_pin, GPIO.OUT)
        GPIO.setup(backward_pin, GPIO.OUT)

        self._forward_pwm = GPIO.PWM(forward_pin, _FREQUENCY)
        self._backward_pwm = GPIO.PWM(backward_pin, _FREQUENCY)

    def move(self, speed_percent):
        speed = _clip(abs(speed_percent), 0, 100)

        # Positive speeds move wheels forward, negative speeds
        # move wheels backward
        if speed_percent < 0:
            self._backward_pwm.start(speed)
            self._forward_pwm.start(0)
        else:
            self._forward_pwm.start(speed)
            self._backward_pwm.start(0)

class Driver:
    def __init__(self):
        rospy.init_node('driver')

        self._last_received = rospy.get_time()
        self._timeout = rospy.get_param('~timeout', 2)
        self._rate = rospy.get_param('~rate', 10)
        self._max_speed = rospy.get_param('~max_speed', 0.5)
        self._wheel_base = rospy.get_param('~wheel_base', 0.091)

        # Assign pins to motors. These may be distributed
        # differently depending on how you've built your robot
        self._left_motor = Motor(10, 9)
        self._right_motor = Motor(8, 7)
        self._left_speed_percent = 0
        self._right_speed_percent = 0

        # Setup subscriber for velocity twist message
        rospy.Subscriber(
            'cmd_vel', Twist, self.velocity_received_callback)

    def velocity_received_callback(self, message):
        """Handle new velocity command message."""

        self._last_received = rospy.get_time()

        # Extract linear and angular velocities from the message
        linear = message.linear.x
        angular = message.angular.z

        # Calculate wheel speeds in m/s
        left_speed = linear - angular*self._wheel_base/2
        right_speed = linear + angular*self._wheel_base/2

        # Ideally we'd now use the desired wheel speeds along
        # with data from wheel speed sensors to come up with the
        # power we need to apply to the wheels, but we don't have
        # wheel speed sensors. Instead, we'll simply convert m/s
        # into percent of maximum wheel speed, which gives us a
        # duty cycle that we can apply to each motor.
        self._left_speed_percent = (100 * left_speed/self._max_speed)
        self._right_speed_percent = (100 * right_speed/self._max_speed)

    def run(self):
        """The control loop of the driver."""

        rate = rospy.Rate(self._rate)

        while not rospy.is_shutdown():
            # If we haven't received new commands for a while, we
            # may have lost contact with the commander-- stop
            # moving
            delay = rospy.get_time() - self._last_received
            if delay < self._timeout:
                self._left_motor.move(self._left_speed_percent)
                self._right_motor.move(self._right_speed_percent)
            else:
                self._left_motor.move(0)
                self._right_motor.move(0)

            rate.sleep()

def main():
    driver = Driver()

    # Run driver. This will block
    driver.run()

if __name__ == '__main__':
    main()
```

Bueno, eso probablemente se ve bastante nuevo. ¿Por qué no lo rompemos pieza
por pieza?

```
#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

import RPi.GPIO as GPIO


# Set the GPIO modes
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

_FREQUENCY = 20
```

Esto es más o menos sacado directo de la
[parte 2](https://kyrofa.com/posts/your-first-robot-introduction-to-the-robot-operating-system-2-5),
aunque ahora estamos importando el mensaje `Twist` en lugar de `String`.
También estamos guardando la frecuencia en una variable con un nombre más
convencional, `_FREQUENCY` (el guión bajo indica que es sólo para uso interno,
todo en mayúsculas indica que es constante).

```
def _clip(value, minimum, maximum):
    """Ensure value is between minimum and maximum."""

    if value < minimum:
        return minimum
    elif value > maximum:
        return maximum
    return value
```

La función `_clip` es bien sencilla: se asegura que los valores dados estén
entre un mínimo y máximo dados. La usaremos más adelante para asegurarnos de
no intentar de mover los motores un ciclo de trabajo menor a 0 o mayor a 100.

```
class Motor:
```

Estamos creando una nueva clase para representar un motor que podemos mover.

```
    def __init__(self, forward_pin, backward_pin):
        self._forward_pin = forward_pin
        self._backward_pin = backward_pin

        GPIO.setup(forward_pin, GPIO.OUT)
        GPIO.setup(backward_pin, GPIO.OUT)

        self._forward_pwm = GPIO.PWM(forward_pin, _FREQUENCY)
        self._backward_pwm = GPIO.PWM(backward_pin, _FREQUENCY)
```

Esta es la inicialización para la clase `Motor`. Se llama de forma automática
siempre que una nueva instancia es creada. Acepta dos parámetros, uno para cada
pin involucrado en mover el motor (uno adelante, uno atrás). Luego establece
los pines como salidas, y guarda los PWMs para usar en la función `move()`:

```
    def move(self, speed_percent):
        speed = _clip(abs(speed_percent), 0, 100)

        # Positive speeds move wheels forward, negative speeds
        # move wheels backward
        if speed_percent < 0:
            self._backward_pwm.start(speed)
            self._forward_pwm.start(0)
        else:
            self._forward_pwm.start(speed)
            self._backward_pwm.start(0)
```

Esta es la función trabajadora principal de la clase `Motor`: es como se mueve
el motor. Acepta un porcentaje positivo o negativo (entre 0-100), donde
valores positivos mueve hacia adelante, y valores negativos mueven hacia atrás.
Aquí es donde utilizamos la función `_clip()` que discutimos arriba para
asegurar que el porcentaje solicitado no se sale del rango válido de 0-100, que
lo convierte en un ciclo de trabajo válido.

```
class Driver:
```

Aquí hay una clase más, representando el controlador de ROS mismo.

```
    def __init__(self):
        rospy.init_node('driver')

        self._last_received = rospy.get_time()
        self._timeout = rospy.get_param('~timeout', 2)
        self._rate = rospy.get_param('~rate', 10)
        self._max_speed = rospy.get_param('~max_speed', 0.5)
        self._wheel_base = rospy.get_param('~wheel_base', 0.091)

        # Assign pins to motors. These may be distributed
        # differently depending on how you've built your robot
        self._left_motor = Motor(10, 9)
        self._right_motor = Motor(8, 7)
        self._left_speed_percent = 0
        self._right_speed_percent = 0

        # Setup subscriber for velocity twist message
        rospy.Subscriber(
            'cmd_vel', Twist, self.velocity_received_callback)
```

Aquí está el inicializador de la clase `Driver`, llamado de forma automática
cuando se crea una nueva instancia. No acepta ningún parámetro, ya que soporta
cambiar su comportamiento usando los parámetros del ROS Parameter Server (del
que aprendieron un poquito en el
[tutorial de ROS número 7](http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams)).
Empezamos inicializando el nodo ROS, que inicia la comunicación con el director
de ROS. Luego grabamos la hora actual (que usaremos luego) y obtenemos los
valores de todos los parámetros que soportamos. Después de eso, creamos dos
instancias de la clase `Motor` definida arriba para representar los motores
izquierdo y derecho, e inicializamos nuestras velocidades en cero.

Como una nota al margen rápida: estos pines corresponde con qué motor es el
motor A, y qué motor es el motor B, y la polaridad que usamos para conectarlos.
Si ustedes conectaron los suyos de forma distinta a la mía (lo que está
perfectamente bien), ustedes podrían necesitar cambiar estos pines un poco o
sino su robot se moverá ridículamente mal.

Finalmente, nos suscribimos a nuestro tema `Twist`, que se llama `cmd_vel`
(siguiendo la convención, significa «velocidad ordenada»), y solicitamos que la
función `velocity_received_callback()` sea llamada cada vez que un nuevo
comando sea recibido.

```
    def velocity_received_callback(self, message):
        """Handle new velocity command message."""

        self._last_received = rospy.get_time()

        # Extract linear and angular velocities from the message
        linear = message.linear.x
        angular = message.angular.z

        # Calculate wheel speeds in m/s
        left_speed = linear - angular*self._wheel_base/2
        right_speed = linear + angular*self._wheel_base/2

        # Ideally we'd now use the desired wheel speeds along
        # with data from wheel speed sensors to come up with the
        # power we need to apply to the wheels, but we don't have
        # wheel speed sensors. Instead, we'll simply convert m/s
        # into percent of maximum wheel speed, which gives us a
        # duty cycle that we can apply to each motor.
        self._left_speed_percent = (100 * left_speed/self._max_speed)
        self._right_speed_percent = (100 * right_speed/self._max_speed)
```

Esta es la función `velocity_received_callback()`, parte de la clase `Driver`.
Gracias a nuestra subscripción en `__init__()`, esta función es llamada cuando
un nuevo mensaje de comando entra. Aquí es en donde la matemática que hicimos
en los paso 1 y 2 entra en juego. Primero que todo, grabamos la hora en la que
recibimos el mensaje (que discutiremos en un momento). Luego extraemos los
componentes de velocidades linear y angular del mensaje. Usando esta
información, usamos la fórmula de velocidad de ruedas que derribamos en el paso
1 para calcular las velocidades de las ruedas izquierda y derecha (en metros
por segundo). Luego usamos la fórmula de ciclo de trabajo derivada en el paso
2 para convertir esas velocidades de ruedas en porcentajes izquierdo y derecho.
Ya que estos valores pueden ser negativo, estos representan tanto el ciclo de
trabajo deseado como la dirección de la rueda.

Noten que esta función en realidad no cambia la velocidad de las ruedas, sólo
calcula lo que deberían hacer. ¿Por qué? Eso lo discutiremos en un momento.

```
    def run(self):
        """The control loop of the driver."""

        rate = rospy.Rate(self._rate)

        while not rospy.is_shutdown():
            # If we haven't received new commands for a while, we
            # may have lost contact with the commander-- stop
            # moving
            delay = rospy.get_time() - self._last_received
            if delay < self._timeout:
                self._left_motor.move(self._left_speed_percent)
                self._right_motor.move(self._right_speed_percent)
            else:
                self._left_motor.move(0)
                self._right_motor.move(0)

            rate.sleep()
```

Este es el ciclo de control de nuestra clase `Driver`. Aquí es donde realmente
aplicamos las recién calculadas velocidades de ruedas, en un ciclo que ejecuta
a un ritmo específico (10 Hz de forma predeterminada). ¿Qué pasa si no
recibimos una nueva orden de velocidad cuando ejecutamos el ciclo de nuevo?
Sólo usamos la que fue anteriormente ordenada. Eso parece un poco raro-- ¿para
qué tenemos un ciclo si sólo usamos la velocidad ordenada?

Digamos que sacamos esta lógica fuera del ciclo y la ponemos en el manejador de
mensajes (la función `velocity_received_callback()`). ¿Qué pasa si recibimos
unas cuantas ordenes de velocidad, pero en ese momento la red se cayó, o de
alguna otra forma perdimos comunicación con el controlador? El robot sólo se
continuaría moviendo a las velocidades anteriormente ordenadas, y tendrían que
ir a perseguirlo. Eso me pasó con un robot que pesaba varios cientos de libras
-- se salió del rango de la unidad de control y siguió caminando. Vaya si
aprendí la lección.

Al poner esta lógica en un ciclo, y guardar cuándo llegan los mensajes de
órdenes, podemos implementar un límite de tiempo para asegurarnos que el robot
nunca se irá corriendo lejos de nosotras si algún problema ocurre. Pueden ver
esto en el ciclo: calculamos qué tanto tiempo ha pasado desde que recibimos la
última velocidad ordenada. Si es menos que el límite (2 segundos de forma
predeterminada), la velocidad de las ruedas calculada se usa. Si es mayor que
el límite de tiempo (i.e. no hemos recibido un mensaje en algún tiempo),
entonces detenemos el robot.

```
def main():
    driver = Driver()

    # Run driver. This will block
    driver.run()
```

Esta es la parte principal del programa, donde sencillamente creamos una nueva
instancia de la clase `Driver` e iniciamos su ciclo de control.

```
if __name__ == '__main__':
    main()
```

Este es el punto de entrada de todo este nodo-- simplemente ejecutamos la
función `main()`.

Muy bien, hemos terminado con este nodo. Vamos a construirlo antes de seguir:

```
$ cd ~/edukit_bot_ws
$ catkin_make
```

## Paso 5: probar el nodo de control

Como en la
[parte 2](https://kyrofa.com/posts/your-first-robot-introduction-to-the-robot-operating-system-2-5),
necesitamos asegurarnos de tener permisos para accesar GPIO como usuarias
(recuerden que esto se pierde la reiniciar):

```
$ sudo chmod a+rw /dev/gpiomem
```

También vamos a usar lo que aprendimos en la
[la parte 3](https://kyrofa.com/posts/your-first-robot-the-controller-3-5) para
que el control nos de mensajes `Twist`. Abran cuatro terminales ejecutando el
shell clásico. En la primera, ejecuten `roscore`:

```
$ roscore
```

En la segunda, ejecuten el nodo joy:

```
$ rosrun joy joy_node
[ INFO] [1515003691.568172834]: Opened joystick: /dev/input/js0. deadzone_: 0.050000.
```

En la tercera, ejecuten el nodo teleop:

```
$ rosrun teleop_twist_joy teleop_node
[ INFO] [1515003734.469189240]: Teleop enable button 0.
[ INFO] [1515003734.469664500]: Linear axis x on 1 at scale 0.500000.
[ INFO] [1515003734.469924707]: Angular axis yaw on 0 at scale 0.500000.
```

Y finalmente, en la cuarta terminal activen nuestro espacio de trabajo y
ejecuten nuestro nodo controlador:

```
$ cd ~/edukit_bot_ws
$ source devel/setup.sh
$ rosrun edukit_bot driver_node
```

¡Ahora enciendan su control y empiecen a mover su robot! Ah, esperen, ¿ya
trataron de encenderlo y nada pasó? Bien, déjenme explicarlo.

¿Recuerdan en el paso 2 como usamos el hecho de que los mensajes twist que
estábamos obteniendo con el controlador tenían su máximo en 0.5? Eso funciona
muy bien para velocidades lineares, pero el control también tiene un máximo de
0.5 radianes por segundo para su velocidad angular, lo que es tan lento que los
motores no lo logran (sólo dan un silencioso zumbido en lugar de moverse, como
si quisieran que ustedes sepan que están dando su mejor esfuerzo).

La solución es solicitar al nodo teleop que escale esos valores a una velocidad
de giro más razonable. Vayan a la terminal en la que ejecutaron el nodo teleop,
háganle ctrl+c y vuélvanlo a ejecutar con un factor de escala angular. Pueden
experimentar con esto para ver qué tan sensible/nervioso quieren a su robot,
pero yo me quedé con un valor de 4 (valores más altos lo hacen más sensible):

```
$ rosrun teleop_twist_joy teleop_node _scale_angular:=4
[ INFO] [1515005430.818803229]: Teleop enable button 0.
[ INFO] [1515005430.819282707]: Linear axis x on 1 at scale 0.500000.
[ INFO] [1515005430.819531978]: Angular axis yaw on 0 at scale 4.000000.
```

AHORA deberían poder manejarlo.

En la
[entrada siguiente (y final) de esta serie](https://kyrofa.com/posts/your-first-robot-sharing-with-others-5-5),
hablaremos de cómo lanzar los archivos ahorrándonos tener que abrir mil
millones de terminales sólo para ejecutar unos cuantos nodos ROS, ¡y convertir
nuestro paquete ROS en un snap que se inicia al encender y es fácilmente
instalable para sus amistades!
