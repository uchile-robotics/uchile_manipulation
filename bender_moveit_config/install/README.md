Instalación
===========

Instalar dependencias
---------------------

Instalación de MoveIt!

~~~
$ sudo apt-get install ros-indigo-moveit-full
~~~

Dependencias

~~~
$ cd catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=indigo -y
~~~