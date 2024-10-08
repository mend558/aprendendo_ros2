# aprendendo_ros2
## Os próximos comandos são executados fora do Docker

Rodando o ambiente ROS
```bash
docker compose up ros-master
```
---
## Os próximos comandos são executados dentro do Docker


Compilando o workspace:<br>
```bash
colcon build
source install/setup.bash
```

Criando um Pacote:<br>
ex:
```bash
cd src
ros2 pkg create <nome_do_pacote> --build-type ament_python
# ex: ros2 pkg create meu_primeiro_pacote --build-type ament_python
cd ..

```

Executando um nó:<br>
ex:
```bash
ros2 run <nome_do_pacote> <nome_do_nó>
# ex: ros2 run meu_primeiro_pacote meu_primeiro_no

```

Executando um launch:<br>
ex:
```bash
ros2 launch <nome_do_pacote> <nome_do_nó>
# ex: ros2 launch meu_primeiro_pacote meu_primeiro_launch.py

```

Rodar um robô (tres terminais diferentes):<br>
ex:
```bash

# Carregar o ros2
source install/setup.bash

# Abrir o Gazebo
ros2 launch pacote_de_exemplos simulation.launch.py world_path:=src/pacote_de_exemplos/simulation/worlds/simple_room_with_gaps.world

# Carregar o robô
ros2 launch pacote_de_exemplos load.launch.py

# Rodar o controlador
ros2 run projeto1 r2d2

# Rodar o mapeador
ros2 launch pacote_de_exemplos mapping_cartographer.launch.py

# Salvar um mapa
ros2 run nav2_map_server map_saver_cli -f src/mapa

# Rodar uma simulação
ros2 launch pacote_de_exemplos mapearluis.launch.py

#tres entregas do projeto 2
#1 mapeamento do cartographer e salvar o mapa.
#2 imagem do matplotlib no programa. (desenvolver o algoritmo de mapear enquanto anda, usar a .pose do robô)
#3 utilziar o robô real.

# Se as portas não aparecerem, pode encaminhá-las manualmente pelo Portas abrindo a porta 3000.
# Para criar um novo terminal, usar alt+seta.
# Para deletar um terminal digitar exit e apertar enter.