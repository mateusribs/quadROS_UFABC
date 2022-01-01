# Quadrirrotor UFABC - ROS/Gazebo

O principal objetivo deste repositório é perpetuar o trabalho que vem sido realizado durante meu projeto de mestrado no programa de pós-graduação em Engenharia Mecânica pela Universidade Federal do ABC. O tema do meu trabalho é "Estimação de Posição e Atitude de um Quadrirrotoro Utilizando Visão Computacional", e tem como principal objetivo realizar a navegação de um veículo quadrirrotor, em ambientes fechados que não possuem sinal de GPS, utilizando visão computacional. Para realizar a estimação dos estados foi utilizado um Filtro de Kalman dos Erros de Estado (Error State Kalman Filter [1]) e a partir deles um sistema de controle em cascata foi implementado para que o quadrirrotor seguisse uma trajetória pré-definida.

O ambiente de simulação é configurado no sistema Gazebo/ROS, visto que são ferramentas amplamente utilizadas pela comunidade robótica e, portanto, oferecem muitas aplicações que satisfazem a necessidade desse trabalho.

A seguir serão expostas algumas informações de como configurar o ROS/Gazebo e também instruções para realizar a simulação. Caso hajam dúvidas contatar os colaboradores desse projeto.

## Instalação e configuração do ambiente ROS/Gazebo

A versão do ROS utilizada nesse trabalho foi a Noetic. As instruções para instalação dos ROS/Gazebo estão disponíveis em: http://wiki.ros.org/noetic/Installation/Ubuntu. 

OBS: O link acima oferece as informações para instalação no sistema operacional Ubuntu 20.04 LS. Para outros sistemas operacionais consultar: http://wiki.ros.org/noetic/Installation.

Após a instalação do ROS/Gazebo, é necessário criar um workspace. No prompt de comando, siga as seguintes instruções:

Crie um workspace no diretório home/username/quad_ufabc_ws, e em ~/quad_ufabc_ws/src serão armazenados os pacotes utilizados:

```
~$ mkdir -p ~/quad_ufabc_ws/src
~$ cd ~/quad_ufabc_ws/src
~$ source /opt/ros/noetic/setup.bash
~$ catkin_init_workspace 
```
Compile o workspace:

```
~$ cd ..
~$ catkin_make
```
Configure o bash (Mude o username pelo seu nome de usuário utilizado no Ubuntu):

```
~$ echo "source /home/username/quad_ufabc_ws/devel/setup.bash" >> ~/.bashrc
~$ source ~/.bashrc
```

