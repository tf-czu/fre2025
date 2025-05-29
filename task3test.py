R = 1.2  # poloměr kružnice v metrech
wheel_base = 0.25  # vzdálenost mezi koly robota (např. 50 cm), uprav podle robota
v = 0.3  # střední rychlost robota (m/s)

# Výpočet rychlostí pro jednotlivá kola
v_left = v * (R - wheel_base / 2) / R
v_right = v * (R + wheel_base / 2) / R

# Příklad posílání příkazů v Osgaru
from osgar.node import Node

class CircleDriver(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        self.sleep_time = 0.1  # 10 Hz

    def send_circle_command(self):
        # Například pro RawLog sim: [left_speed, right_speed] in m/s
        self.publish('desired_speed', [v_left, v_right])

    def run(self):
        while self.should_run():
            self.send_circle_command()
            self.sleep(self.sleep_time)
