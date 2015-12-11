* Update certificates
```
sudo apt-get install --reinstall ca-certificates
```

Setup .gitconfig
```
[http]
        sslVerify = true
        sslCAinfo = /etc/ssl/certs/ca-certificates.crt
[user]
        email = derek@derekmolloy.ie
        name = derekmolloy
```
* Update apt-get archives
```
$ sudo apt-get update
```
* Install ssh
```
$ sudo apt-get install openssh-server
```
* Setup ssh (first time)
```
$ ssh-keygen 
```
* Everytime?
```
$ ssh-agent /bin/bash
$ ssh-add ~/.ssh/id_rsa
```

* Install git
```
$ sudo apt-get install git
```

* Run the runmeonrpi2.sh

* Edit .bashrc file. Add this line
```
source /opt/ros/indigo/setup.bash
```

* Install packages
```
sudo apt-get install ros-indigo-tf-conversions
sudo apt-get install ros-indigo-cv-bridge
sudo apt-get install ros-indigo-compressed-image-transport 
sudo apt-get install ros-indigo-camera-info-manager
```

* Enable the camera
Just modify the file /boot/firmware/config.txt is enough.
Just add a line "start_x=1" at the bottom of the file config.txt, save it, and reboot the system. 

* Install picamera?
```
$ sudo apt-get install python-pip
$ sudo pip install picamera
```

* [Get picamera to work](http://raspberrypi.stackexchange.com/questions/37359/how-to-use-raspistill-on-ubuntu)
```
sudo base -c "echo 'start_x=1' >> /boot/config.txt"
sudo base -c "echo 'gpu_mem=128' >> /boot/config.txt"
```

```
git clone https://github.com/raspberrypi/userland.git
cd userland
./buildme
touch ~/.bash_aliases
echo 'PATH=$PATH:/opt/vc/bin\nexport' >> ~/.bash_aliases
echo 'LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/vc/lib\nexport' >> ~/.bash_aliases
source ~/.bashrc
ldconfig
```