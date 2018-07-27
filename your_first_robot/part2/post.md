# Your first robot: Introduction to the Robot Operating System [2/5]

![CamJam EduKit #3 wheels](https://kyrofa.com/uploads/proclaim/image/image/44/edukit.jpg)

This is the second blog post in
[this series about creating your first robot with ROS and Ubuntu Core](https://kyrofa.com/posts/your-first-robot-a-beginner-s-guide-to-ros-and-ubuntu-core-1-5).
In the [previous post](https://kyrofa.com/posts/your-first-robot-a-beginner-s-guide-to-ros-and-ubuntu-core-1-5)
we walked through all the hardware necessary to follow this series, and
introduced Ubuntu Core, the operating system for IoT devices. We installed it
on our Raspberry Pi, and used it to go through the
[CamJam worksheets](http://camjam.me/?page_id=1035). In this post, I'm going to
introduce you to the [Robot Operating System (ROS)](http://www.ros.org/), and
we'll use it to move our robot. We'll be using ROS throughout the rest of the
series. Remember that this is also a video series, feel free to watch the video
version of this post:

[![Video: Your first robot, part 2: Introduction to the Robot Operating System](https://kyrofa.com/uploads/proclaim/image/image/45/thumbnail.jpg)](https://www.youtube.com/watch?v=Sw33EbZHris&list=PL1LO5F1-Jh8JfpHpsKCtUSaaSxVQUUOYw)

## What is the Robot Operating System?

At its simplest, ROS is a set of open-source libraries and tools meant to ease
development of robots. It also provides an infrastructure for connecting
various robotic components together. For example, if you happened to go through
all of the CamJam worksheets (particularly #9), you've written a single Python
script that's responsible for a bunch of things: controlling the motors,
reading from the line detector, reading from the ultrasonic sensor, etc. What
if we threw a wireless controller into the mix? That script quickly gets
complicated, and if you happen to want to swap one component out for another,
you need to rewrite the whole thing; i.e. these logically different components
are tightly coupled together since they're in the same script.

![ROS communication diagram](https://kyrofa.com/uploads/proclaim/image/image/43/ros.png)

ROS provides a communication infrastructure that allows you to extract different
logic into their own modules, and have them communicate with each other in a
standard way, as depicted in the picture above. For example, if you wanted to
switch ultrasonic sensors, and rewrite the "distance" module, you could do that
without having to touch any of the other modules as long as the new distance
module talked the same way as the old one.

This will all make more sense once we dive in, so let's get started, shall we?

## Step 1: Install ROS on the Raspberry Pi

ROS has [three currently-supported releases](https://wiki.ros.org/Distributions):
Indigo Igloo, Kinetic Kame, and Lunar Loggerhead. Ubuntu Core series 16 (the
one we're using) is Ubuntu Xenial, which limits our options to Kinetic and
Lunar. Lunar is technically newer and shinier, but like Ubuntu, ROS has
Long-Term Support (LTS) releases that are supported for an extended period of
time, and Kinetic is their most recent LTS. As a result, we'll use Kinetic
here.

SSH into your Pi, and get into your classic shell:

```
$ sudo classic
```

Let's follow [ROS Kinetic's install guide](https://wiki.ros.org/kinetic/Installation/Ubuntu).
ROS maintains its own repository of Debian packages, which we need to add to
our system:

```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

We then need to add that repository's keys to the list of keys we'll accept
(this verifies that packages in that repository really do come from ROS):

```
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
```

Now we'll re-index all the repositories we have configured, since we just
added one:

```
$ sudo apt update
```

Now let's install ROS. As you'll see on the install guide, there are a bunch
of meta packages available (packages that exist solely to pull in other
packages). Let's install the smallest, bare-bones one, `ros-kinetic-ros-base`,
which will take up around 700MB. We'll also install g++, the C++ compiler (it's
still required, even though we're writing Python):

```
$ sudo apt install g++ ros-kinetic-ros-base
```

At this point, ROS is successfully installed, but none of its tools are
available to run. That's because ROS installs itself into what it calls a
"workspace", and provides a shell script that activates that workspace. We can
make sure we activate that workspace upon login by adding it to the `.bashrc`
file in our home directory:

```
$ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

Now you should be able to run `roscore` without troubles:

```
$ roscore
<snip>
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

Go ahead and quit that by pressing CTRL+C.

## Step 2: Get to know ROS

One of the reasons I like ROS so much (and, I think, one of the reasons it's
so popular) is that their introductory documentation is fantastic. They have a
[phenomenal set of tutorials](https://wiki.ros.org/ROS/Tutorials) that take you
from knowing absolutely nothing to feeling more or less comfortable with the
entire system. Each one is easily digestible in a few minutes. Because they are
so good, rather than try and duplicate their hard work here, you should just
start at the beginning and go through them at least until you complete #13,
"Examining the Simple Publisher and Subscriber". Note that there are two
parallel tutorial tracks, one that uses C++ and one that uses Python. We'll be
using Python through this series, so you don't need to worry about the C++ ones
unless they interest you.

## Step 3: Setup Python 2

Now that we've gained some familiarity with ROS, it's almost time to make our
robot move using it. However, there's something we need to do first. Back in
CamJam worksheet #1, they mention the following:

> "When the Raspberry Pi was first released, some of the important Python
> libraries were only available for Python 2.7. However, almost every library,
> and all the ones used in these worksheets, are available for Python 3.2. It
> has been decided that all code for this EduKit will be developed for Python
> 3.2."
>
> ~ CamJam worksheet #1

That's all fine and dandy, and I agree with it, but unfortunately the Python
bindings for ROS are only officially supported on Python 2, so we need to use
Python 2 from now on instead of Python 3. Don't worry, all the code from the
worksheets should still work, but it means we need to install the Python 2
version of RPi.GPIO (we only have the Python 3 version right now):

```
$ sudo apt install python-dev python-pip python-setuptools
$ pip install RPi.GPIO
```

## Step 4: Create ROS package for our robot

Alright, let's have some fun! We're going to rewrite the code we wrote for the
CamJam Worksheet #7 using ROS. We'll add some message handling to it, so, using
ROS, we can command the robot to move forward, turn left, turn right, etc.

The first step is to create a new workspace. You learned how to do this in the
first ROS tutorial. I'm calling mine "edukit_bot_ws", if you call yours
something else remember to change the directions accordingly:

```
$ mkdir -p ~/edukit_bot_ws/src
$ cd ~/edukit_bot_ws/src
$ catkin_init_workspace
```

Now let's create a new package in that workspace. I'll call mine "edukit_bot",
and it has three dependencies: `rospy` (the Python bindings for ROS),
`std_msgs` (the standard ROS messages, e.g. numbers, strings, etc.), and
`python-rpi.gpio` (RPi.GPIO, which we use for GPIO access):

```
$ cd ~/edukit_bot_ws/src
$ catkin_create_pkg edukit_bot rospy std_msgs -s python-rpi.gpio
```

## Step 5: Write the ROS node

Time to write some code. First, create a new Python script in the ROS package's
`src/` directory:

```
$ touch ~/edukit_bot_ws/src/edukit_bot/src/driver_node
```

The CamJam worksheets don't discuss this, but if we set the script to be
executable, we can run it directly instead of calling it like
`python path/to/script.py`. Let's do that:

```
$ chmod a+x ~/edukit_bot_ws/src/edukit_bot/src/driver_node
```

Open that script in a text editor, and make it look like this (note that the
entire package used in this post is
[available for reference](https://github.com/kyrofa/your-first-robot/tree/master/part_2/edukit_bot)):

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

Lots of that should look familiar, but let's break it down into chunks.

```
#!/usr/bin/env python

import rospy
from std_msgs.msg import String
```

The very first line of this file is called a
[shebang](https://en.wikipedia.org/wiki/Shebang_(Unix)). Since we've marked
this file as executable on its own, this defines the interpreter that will
execute this program. In this case, we're telling it that it needs the `python`
command.

We then import `rospy`, which includes the ROS Python bindings, and we import
the `String` message from the ROS `std_msgs`. We'll use both of these a little
later in the program.

```
import RPi.GPIO as GPIO

# Set the GPIO modes
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# <snip you don't need to see all this again...>

# Turn Right
def Right():
    pwmMotorAForwards.ChangeDutyCycle(DutyCycle)
    pwmMotorABackwards.ChangeDutyCycle(Stop)
    pwmMotorBForwards.ChangeDutyCycle(Stop)
    pwmMotorBBackwards.ChangeDutyCycle(DutyCycle)
```

This entire section was lifted virtually verbatim from the CamJam Worksheet #7.
It's explained there, so I won't repeat the explanation here.

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

Here's a new, ROS-specific part. The `CommandCallback` function is created to
handle a `String` message. It simply takes a look at the data (i.e. the string
itself) contained within the message, and takes action appropriately. For
example, if the string is the word "forwards" it moves the robot forward by
calling the `Forwards` function created in the worksheet. Similarly, if the
word is "left" it turns the robot left by calling the `Left` function. If the
command isn't one of the recognized words, the function does the safest thing
it can: stop.

```
rospy.init_node('driver')

rospy.Subscriber('command', String, CommandCallback)

rospy.spin()
print('Shutting down: stopping motors')
StopMotors()
GPIO.cleanup()
```

Here's the main part of the program. First of all, we initialize the node and
give it a name ("driver"). This begins communication with the ROS master. We
then subscribe to a topic named "command", and we specify that we expect that
topic to be a `String` message. We then provide our `CommandCallback` function
to request that it be called when a new messages comes in on that topic.

Then we call `rospy.spin()` which blocks and waits for messages to come in.
Once the node is asked to quit (say, with a CTRL+C), that function will exit,
at which time we ensure that the motors have been stopped. We don't want the
robot running away from us!

We're done with our workspace for now, so let's build it:

```
$ cd ~/edukit_bot_ws
$ catkin_make
```

## Step 6: Move the robot using ROS

At this point, we have a ROS node created that will drive our robot as
requested in a "command" message. It uses GPIO though, which still requires
`sudo`. Instead of trying to get our workspace working using `sudo`, let's
temporarily change the permissions of GPIO so that we don't need `sudo` (this
will be reset upon reboot):

```
$ sudo chmod a+rw /dev/gpiomem
```

`/dev/gpiomem` is a device representing the memory dedicated to GPIO (i.e. not
other, more important/dangerous memory). As a result, this operation is
relatively safe, particularly compared to doing the same to e.g. `/dev/mem`.

Alright, let's test this out! You'll need to open three terminals for this,
each one using the classic shell (remember, run `sudo classic` to enter the
classic shell). We first need the ROS master, as without it publishers and
subscribers can't find one another. So in one terminal, run the ROS master:

```
$ roscore
```

In another terminal, make sure you activate our newly-built workspace, and run
our "driver" node:

```
$ cd ~/edukit_bot_ws
$ source devel/setup.sh
$ rosrun edukit_bot driver_node
```

Finally, in the third terminal, we'll start giving our commands to make the
robot move. First of all, take note of the topics that either have publishers
or subscribers:

```
$ rostopic list
/command
/rosout
/rosout_agg
```

Notice the `/command` topic. This is the topic on which our "driver" node is
listening, because of the `Subscriber` we configured. It's expecting a `String`
message there, so let's send it a command, say, to move forward:

```
$ rostopic pub -1 /command std_msgs/String "forwards"
publishing and latching message for 3.0 seconds
```

You should notice the "driver" node say that it's moving forward, and then your
robot's wheels should start rotating forward! Try sending any of the strings we
handled in `CommandCallback` ("left", "backwards", etc.), and commands that you
know are invalid to ensure that it stops safely.

Congratulations, you're quickly learning ROS! In the
[next post in this series](https://kyrofa.com/posts/your-first-robot-the-controller-3-5),
we'll break free of the CamJam worksheets and strike out on our own. We'll
introduce the wireless controller, and begin working on making our robot
remotely-controlled using ROS.
