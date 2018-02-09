# Su primer robot: una guía de ROS y Ubuntu Core para principiantes [1/5]

![CamJam EduKit #3 en el árbol de navidad](https://kyrofa.com/uploads/proclaim/image/image/40/BB1A6282_scaled.jpg)

Hace algún tiempo hice una [serie de blog/vídeos](https://kyrofa.com/posts/from-ros-prototype-to-production-on-ubuntu-core-1-5) que llevaba a las personas lectoras a traves de la creación de un prototipo usando el Robot Operating System (ROS) y llevándolo a producción usando Ubuntu Core. Sin embargo, esa serie era más para profesionales de la robótica. Asumía bastante conocimiento de ROS y requería un equipo costoso (el robot era de cerca de $1000). Bueno, Ubuntu también es para aficionadas (y niñas) quienes no quieren gastar $1000 para jugar con robots. Así nació esta serie: una que no asume ningún conocimiento de ROS y que usa hardware que es tan barato que puede ser un buen regalo de navidad. Le presento un robot que cuesta menos de $100: El [CamJam EduKit #3](https://thepihut.com/collections/camjam-edukit/products/camjam-edukit-3-robotics), un kit ensamblable de robótica sobre ruedas que se controla con un Raspberry Pi. Déjeme listar todo el hardware que necesitará para esta serie:

* El EduKit, £18, digamos, $25
* Un Raspberry Pi (yo estoy usando el [Raspberry Pi 2](https://www.raspberrypi.org/products/raspberry-pi-2-model-b/), usted puede usar el 2 o el 3), $35
* Batería para Raspberry Pi (yo usé [esta](https://www.amazon.com/gp/product/B00MWV1TJ6/ref=od_aui_detailpages00?ie=UTF8&psc=1)), $15
  * Esto es opcional, pero si no tiene uno, su Pi estará amarrado a su fuente de poder.
* [Control inalámbrico](https://thepihut.com/collections/raspberry-pi-gaming/products/raspberry-pi-compatible-wireless-gamepad-controller?variant=38135423121), £14, digamos, $20
* Total, incluyendo componentes opcionales: $95

En mi serie de robótica anterior, una de las razones por las que el Turtlebot era tan caro es porque tiene una enorme cantidad de software ya escrito, como controladores ROS para movimiento, leer de sensores, etc. Este pequeño robot es diferente: necesitaremos escribir todo por nosotras mismas. Por suerte, la gente de CamJam brinda unas [hojas de trabajo](http://camjam.me/?page_id=1035) para introducirle a la plataforma y ayudarle a empezar a escribir software para controlar las ruedas, leer el sensor ultrasónico, y más. Podemos aprovecharlas para hacer funcionar el robot de forma rápida. Entonces, a través de esta serie cubriremos los siguientes temas:

* [Su primer robot: introducción al Robot Operating System [2/5]](https://kyrofa.com/posts/your-first-robot-introduction-to-the-robot-operating-system-2-5)
  * ¿Qué es el Robot Operating System (ROS)? ¿Por qué vale la pena aprenderlo? ¿Cómo usarlo?
* Su primer robot: el control remoto [3/5]
  * Obteniendo datos del control inalámbrico y usándolos en ROS.
* Su primer robot: el controlador [4/5]
  * Aprender un poco de teoría de control, escribir el controlador para el robot.
* Su primer robot: compartiendo con otras [5/5]
  * ¿Cómo se arranca el sistema sin tener que entrar por SSH? ¿Cómo compartir su producto final con sus amistades?

Como mencioné, las hojas de trabajo de CamJam nos llevarán lejos hacia lo que necesitamos aprender para esta serie. Sin embargo, en lugar de usar Raspbian, vamos a usar Ubuntu Core. Las hojas de trabajo aún son perfectamente aplicables, y lo probaré guiándole a través de la configuración de Ubuntu Core en su Raspberry Pi, y luego seguiremos una de las hojas de trabajo de CamJam. Note que esta también es una serie de vídeos, siéntase libre de ver la versión en vídeo de este post (con subtítulos en Español):

[![Vídeo: Su primer robot, parte 1: una guía introductoria a ROS y Ubuntu Core](http://img.youtube.com/vi/KidVVqbsIHI/0.jpg)](http://www.youtube.com/watch?v=KidVVqbsIHI)

## Prerrequisitos

Aunque esto es una introducción, quiero ser claro sobre algunas de las cosas que estoy asumiendo. Esta serie asume que usted está familiarizada un poco con Ubuntu (o alguna otra distro basada en Debian, como Raspbian), en particular, usando la línea de comandos. No espero que sea una profesional escribiendo scripts, pero debería saber cómo navegar el sistema de archivos y usar editores de terminal (vi, nano, etc.)

## ¿Qué es Ubuntu Core?

Ubuntu Core es una distribución especializada de Ubuntu que está dedicada a dispositivos que entran en saco de «Internet de las cosas» (IoT, por sus siglas en inglés). Esto incluye enrutadores, termostatos y, por supuesto, robots. Espero que, al introducirle al sistema operativo (y herramientas como ROS) que utilizan las profesionales para hacer su robots, ¡el camino quede listo para que usted logre grandes cosas en este campo!

Ubuntu Core es un poco diferente a Raspbian. ¡Es incluso un poco diferente a Ubuntu clásico (o Ubuntu MATE)! Empecemos instalándolo, y explicaré a qué me refiero.

## Step 1: Instalar Ubuntu Core

Esto ya está bien documentado, pero quiero agregar unas cuantas notas antes de que siga los pasos. Antes que todo, se requiren un teclado y un monitor, pero solo para el primer arranque. A diferencia de Raspbian o Ubuntu MATE, no hay nombre de usuario ni contraseña predefinidas, o un proceso de instalación en el que se cree un usuario. Ni siquiera utiliza un ratón-- esto es más como Ubuntu Server. De hecho, no hay un inicio de sesión local: todo se hace a través de SSH (una sesión remota a través de la red), con claves criptográficas en lugar de contraseñas. Esto es parte de la filosofía de «seguridad predeterminada» de Ubuntu Core. Hay muchos ejemplos de credenciales predefinidas siendo abusadas (vea la [red de bots Mirai](https://en.wikipedia.org/wiki/Mirai_(malware))), y como un sistema operativo enfocado en IoT, ¡Ubuntu Core no puede tener eso! Esa es la razón por la que verá que una cuenta de Ubuntu SSO y claves SSH son prerrequisitos en la guía de instalación (no se preocupe, ambas son fáciles, ¡y su robot estará super seguro!).

Bueno, sigamos la [guía de instalación de Raspberry Pi 2 o 3](https://developer.ubuntu.com/core/get-started/raspberry-pi-2-3) (comente aquí si tiene alguna pregunta). Al final de este paso, usted tendrá Ubuntu Core instalado, y debería poder entrar al Pi a través de SSH.

## Step 2: Preparar el ambiente de desarrollo

Como un sistema operativo enfocado en IoT, Ubuntu Core usa un formato de empaquetamiento diferente al de Raspbian y Ubuntu clásico. En lugar de usar paquetes Debian (que involucran familiaridad con herramientas como **apt**), Ubuntu Core usa un formato de paquetes llamado **snaps**. Los snaps son mucho más robustos cuando se trata de actualizaciones y seguridad, ambas importantes para dispositivos IoT, incluyendo robots. No hay **apt** en este sistema. Para darle una prueba de cómo es usar los comandos «snap», asegúrese que su sistema está completamente actualizado corriendno **snap refresh**.

Entonces, ¿cómo instala una todas las herramientas de desarrollo a las que está acostumbrada? Hay un snap especial para eso, llamado **classic**, que le da acceso a todas su herramientas conocidas (incluyendo **apt**).

Instale el snap **classic** siguiendo la sección de «developing on target» (desarrollando en el mismo sistema objetivo) de la [guía de configuración de desarrollo](https://developer.ubuntu.com/core/get-started/developer-setup) (de nuevo, comente aquí si tiene preguntas). Al final de este paso, usted debe poder ejecutar **sudo classic** y obtener acceso a un shell en el que puede instalar debs, ¡el cual usaremos para hackear en nuestro robot! Luego crearemo su propio snap para controlar el robot, lo que hace super fácil.

## Step 3: Hora de CamJam

Muy bien, ¡ahora estamos en el punto en el que podemos empezar a seguir las hojas de trabajo de CamJam! ¿Por qué no empezamos con la [primera](https://github.com/CamJam-EduKit/EduKit3/raw/master/CamJam%20EduKit%203%20-%20Robotics%20Worksheet%201%20-%20Introduction.pdf)?. Esta en realidad asume que usted está ejecutando Raspbian, lo que no estamos haciendo, entonces aunque puede leerla toda, vamos a saltar directo al paso «Identifying your Version of Raspbian» (Identificando su versión de Raspbian). No, aún no estamos usando Raspbian, pero esta sección tiene un punto interesante que quiero asegurarme de que note:

Ubuntu Core, al igual que Debian Wheezy, considera que el acceso a GPIO a través de mapeo de memoria es una operación privilegiada. Como resultado, cualquier código a lo largo de estas hojas de trabajo que usa GPIO necesitará que lo ejecute con **sudo**.

Todo bien, empecemos. Asegúrese que está en su shell classic ejecutando **sudo classic**. vi está disponible aquí, pero puede **sudo apt install nano** si quiere. Siguiendo las hojas de trabajo, abra **1-helloworld.py** y llénelo con lo siguiente:

```
# Print Hello World!
print('Hello World!')
```

Luego guarde y salga. Ahora puede ejecutar el código en ese archivo con este comando:

```
$ python3 1-helloworld.py
Hello World!
```

Siga adelante y lea el resto de la hoja de trabajo, le será útil luego.

Espero que esto le sirva como un buen inicio para usar Ubuntu Core para hackear en sus proyectos con Raspberry Pi. Como tarea para el resto de la serie, complete también las hojas de trabajo 2-4, y 7. Por supuesto puede hacer las otras también, pero nosotras no usaremos el sensor ultrasónico o el detector de línea en esta serie, entonces no tiene que. Pero, antes de hacer las otras hojas de trabajo, necesitará instalar el paquete RPi.GPIO. En su shell classic, ejecute:

```
$ sudo apt install gcc python3-dev python3-pip python3-setuptools
$ pip3 install RPi.GPIO
```

También recuerde que necesitará usar sudo para cualquier script de Python que utilice GPIO. El [siguiente post en esta serie](https://kyrofa.com/posts/your-first-robot-introduction-to-the-robot-operating-system-2-5) sera una introducción a ROS, ¿qué es?, ¿por qué es útil?, ¿por qué necesita conocerlo? y ¿cómo obtenerlo en su Raspberry Pi con Ubuntu Core?.
