# Quadrirrotor UFABC - ROS/Gazebo

O principal objetivo deste repositório é perpetuar o trabalho que vem sido realizado durante meu projeto de mestrado no programa de pós-graduação em Engenharia Mecânica pela Universidade Federal do ABC. O tema do meu trabalho é "Estimação de Posição e Atitude de um Quadrirrotoro Utilizando Visão Computacional", e tem como principal objetivo realizar a navegação de um veículo quadrirrotor, em ambientes fechados que não possuem sinal de GPS, utilizando visão computacional. Para realizar a estimação dos estados foi utilizado um Filtro de Kalman dos Erros de Estado (Error State Kalman Filter [1]) e a partir deles um sistema de controle em cascata foi implementado para que o quadrirrotor seguisse uma trajetória pré-definida. Um vídeo contendo algumas simulações pode ser visto em: https://youtu.be/_FUcOvvbstQ.

O ambiente de simulação é configurado no sistema Gazebo/ROS, visto que são ferramentas amplamente utilizadas pela comunidade robótica e, portanto, oferecem muitas aplicações que satisfazem a necessidade desse trabalho.

A seguir serão expostas algumas informações de como configurar o ROS/Gazebo e também instruções para realizar a simulação. Caso hajam dúvidas contatar os colaboradores desse projeto.

## Instalação e configuração do ambiente ROS/Gazebo

A versão do ROS utilizada nesse trabalho foi a Noetic. As instruções para instalação dos ROS/Gazebo estão disponíveis em: http://wiki.ros.org/noetic/Installation/Ubuntu. Recomenda-se que, no tópico 1.4 - Installation, execute a instalação Desktop-Full Install, pois já instala todos os pacotes necessários do ROS, além de instalar o simulador Gazebo.

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

- Após ter configurado o workspace, realize o download ZIP desse repositório; 
- Copie a pasta "/src";
- Cole a pasta "/src" em "/quad_ufabc_ws" substituindo a pasta "/quad_ufabc_ws/src" criada por padrão.

## Execução da simulação

Tendo configurado o ambiente ROS/Gazebo, já é possível executar a simulação. 

Para abrir o simulador Gazebo, dê o seguinte comando no terminal:

```
~$ roslaunch quad_ufabc quad.launch
```
Ao executar esse código, o simulador Gazebo será aberto. Na simulação serão encontrados o quadrirrotor, os marcadores artificiais, as câmeras e um cone.

Para executar os comandos de simulação utilizando o ROS, é necessário que os seguintes pacotes Python estejam instalados na sua máquina:

- Numpy v.1.19.5;
- Pickle v.4;
- Scipy v.1.5.0;
- Matplotlib v.3.2.2;
- OpenCV v.4.2.0;
- Statistics v.1.0.3.5;

Obs: Caso haja algum problema na execução, favor contatar os contribuidores desse repositório.

Tendo todos os pacotes instalados, execute o seguinte comando:

```
~$ rosrun quad_ufabc main.py
```

Ao executar esse código, o ROS irá inicializar os nós de visão computacional, estimação dos estados e controle do veículo. O quadrirrotor seguirá uma trajetória pré-definida e serão mostradas as visualização das duas câmeras.


## Referências

[1] SOLÀ, J. Quaternion kinematics for the error-state kalman filter. 2017.
