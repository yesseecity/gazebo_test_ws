## Environment Setting

### About ROS
1. Git clone this repo.
2. ```git submodule init```
3. ```git submodule update```
4. ```catkin_make```
5. ```source ./devel/setup.bash```

### About Dualshock 4 Controller (DS4)
If you went to used DS4 Controller to your simulation car, you can refrence this repo [https://github.com/chrippa/ds4drv](https://github.com/chrippa/ds4drv)


## Launch

* Start simulation world  
```roslaunch my_simulations start.launch```

* DS4 Controller  
``` roslaunch my_simulations ds4_twist.launch```
