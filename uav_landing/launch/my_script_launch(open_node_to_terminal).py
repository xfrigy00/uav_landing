import launch
import launch_ros.actions
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Vytvorenie popisu launch, ktory bude spustat proces
    return launch.LaunchDescription([
        ExecuteProcess(
            # Definovanie príkazu na spustenie procesu v novom termináli
            cmd=[
                'gnome-terminal',  # Prikaz na otvorenie noveho terminalu
                '--',  # Oznamuje, ze dalsie argumenty su pre spustenie prikazu v terminali
                'bash',  # Spustenie bash shell v novom terminali
                '-c',  # Prikaz pre bash, aby vykonal dalsi prikaz v uvodzovkach
                'ros2 run uav_landing image_subscriber; echo "Node terminated. Press any key to close..."; read'  # Spustenie node, potom vypis a cakanie na stlacenie klavesy
            ],
            output='screen'  # Nastavenie, aby sa vsetok vypis zobrazil na obrazovke
        ),
    ])

