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
```roslaunch simulations start.launch```

* DS4 Controller  
```roslaunch simulations ds4_twist.launch```

* Create map
```roslaunch simulations SLAM-gazebo.launch```

* Voice Command
```roslaunch voice_interaction_robot deploy_voice_interaction.launch```


## Add user on AWS IAM  
[https://console.aws.amazon.com/iam/home#/users](https://console.aws.amazon.com/iam/home#/users)  
 
Add user
![AWS Add user](/readme_images/iam_add_user_1.png)  

### Select policy **AmazonPollyFullAccess**
![AWS Add user](/readme_images/iam_add_user_2.png)

### Select policy **AmazonLexFullAccess**
![AWS Add user](/readme_images/iam_add_user_3.png)

### Check Permissions
![AWS Add user](/readme_images/iam_add_user_4.png)

### Take down **Access key ID** and **Secret access key**  
![AWS Add user](/readme_images/iam_add_user_5.png)  

### Back user list and click user voice_command_user
![AWS Add user](/readme_images/iam_add_user_6.png)  

### Take down **User ARN**  
![AWS Add user](/readme_images/iam_add_user_7.png)  

### Add user access info to your ```~/.bashrc``` or ```~/.zshrc```  
![AWS Add user](/readme_images/iam_add_user_8.png)  


### Resource your  ```~/.bashrc``` or ```~/.zshrc```  
```source ~/.bashrc```  
or  
```source ~/.zshrc```  
