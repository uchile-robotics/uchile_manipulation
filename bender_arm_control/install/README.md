Instalación
===========

Instalar *udev rules*
---------------------

Permite que los disositivos de control (USB2Dynamixel FTDI FT232RL) sea asignado a un puerto en especifico cada vez de se conecta.

~~~
$ cp left_arm.rules /etc/udev/rules.d/left_arm.rules
$ cp right_arm.rules /etc/udev/rules.d/right_arm.rules
~~~

Desinstalar drivers Dynamixel del PPA ROS
-----------------------------------------

Los drivers son instalados desde el código, para desinstalar completamente usar:

~~~
$ sudo apt-get purge ros-indigo-dynamixel-driver ros-indigo-dynamixel-motor ros-indigo-dynamixel-msgs ros-indigo-dynamixel-controllers
~~~
