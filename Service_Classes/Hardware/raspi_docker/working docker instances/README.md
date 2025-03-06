Instructions on how to use the swarm raspberry container

1. flash your raspberry pi with raspi os
2. set up keyboard and enable ssh
3. conenct to your raspi over lan cable or wifi via ssh
4. copy the docker image into the raspberry
4.1 you can connect your raspberry over remote-ssh extension in vsc and then drag and drop the dockerfile
4.2 use copy commands
5. install docker on raspberry pi
5.1 sudo apt update && sudo apt upgrade -y
5.2 curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
5.3 (verify) docker --version

6. docker load -i swarm-img-5.tar
7. now you can spin up the container with (and you should land inside the containers bash (type exit to exit container)): docker run --privileged -it <image-name>
8. start GPIO interaction deamon if you want to spin the motors: pigpiod
9. inside the scripts folder you will se the current version of the scripts.





!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
10. IMPORTANT: Always pull your code down from the container or and synch it with gitlab
-> inform nik about newly installed packages
WHEN THE CONTAINER IS TURNED OFF PROGRESS WILL BE LOST.


11. if progress should not be lost, make a synched directory. docker run -it --privileged -v /path/on/host:/path/in/container my-container-image

Note: newly installed packages wont be safed until a new image is created. for this contact nik

12. connect container over vsc

    how to connect to remote container

    1. go into raspberry over ssh as usual
    2. you need remote development and dev containers installed
    3. then Press Ctrl+Shift+P
    4. Remote-Containers: Attach to Running Container
    5. select the container
    6. make sure to attach / or /scripts