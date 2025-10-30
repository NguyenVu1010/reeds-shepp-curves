import turtle
import random
from draw import vec, goto, draw_path, set_random_pencolor
import reeds_shepp as rs
import re

# -----------------------------
# Hàm parse string path sang list PathElement
def parse_path_string(path_str):
    pattern = r"\{Steering:\s*(\w+),\s*Gear:\s*(\w+),\s*dist:\s*([0-9\.\-e]+)\}"
    elements = []
    for match in re.finditer(pattern, path_str):
        steering_str, gear_str, dist_str = match.groups()
        steering = getattr(rs.Steering, steering_str)
        gear = getattr(rs.Gear, gear_str)
        dist = float(dist_str)
        elements.append(rs.PathElement(dist, steering, gear))
    return elements

# -----------------------------
# Danh sách các path string (dữ liệu bạn đưa)
path_strings = [
    "[{Steering: LEFT, Gear: FORWARD, dist: 1.0472}, {Steering: RIGHT, Gear: BACKWARD, dist: 1.0472}, {Steering: LEFT, Gear: FORWARD, dist: 1.0472}]",
    "[{Steering: RIGHT, Gear: FORWARD, dist: 0.289752}, {Steering: LEFT, Gear: BACKWARD, dist: 1.5708}, {Steering: STRAIGHT, Gear: BACKWARD, dist: 4.7082}, {Steering: RIGHT, Gear: BACKWARD, dist: 0.289752}]",
    "[{Steering: LEFT, Gear: BACKWARD, dist: 1.5708}, {Steering: RIGHT, Gear: FORWARD, dist: 1.5708}, {Steering: STRAIGHT, Gear: FORWARD, dist: 2}]",
    "[{Steering: LEFT, Gear: FORWARD, dist: 0.25268}, {Steering: RIGHT, Gear: BACKWARD, dist: 1.5708}, {Steering: STRAIGHT, Gear: BACKWARD, dist: 5.74597}, {Steering: LEFT, Gear: BACKWARD, dist: 0.25268}]",
    "[{Steering: LEFT, Gear: BACKWARD, dist: 0.990951}, {Steering: STRAIGHT, Gear: BACKWARD, dist: 0.427008}, {Steering: LEFT, Gear: BACKWARD, dist: 1.27798}]",
    "[{Steering: RIGHT, Gear: BACKWARD, dist: 0.243713}, {Steering: STRAIGHT, Gear: BACKWARD, dist: 5.90316}, {Steering: LEFT, Gear: BACKWARD, dist: 1.5708}, {Steering: RIGHT, Gear: FORWARD, dist: 0.0691803}]",
    "[{Steering: LEFT, Gear: FORWARD, dist: 0.123366}, {Steering: STRAIGHT, Gear: FORWARD, dist: 6.88358}, {Steering: RIGHT, Gear: FORWARD, dist: 1.51963}]",
    "[{Steering: RIGHT, Gear: FORWARD, dist: 1.14876}, {Steering: STRAIGHT, Gear: FORWARD, dist: 4.8997}, {Steering: RIGHT, Gear: FORWARD, dist: 0.247503}]"
]

# -----------------------------
# Khởi tạo turtle
screen = turtle.Screen()
screen.title("Multiple Reeds-Shepp Paths from String")
screen.setup(width=800, height=800)
screen.bgcolor("white")

bob = turtle.Turtle()
bob.speed(1)
bob.up()

# -----------------------------
# Vị trí xuất phát
start_pos = (0, 0, 0)
goto(bob, start_pos)

# -----------------------------
# Vẽ từng path với màu khác nhau
for path_str in path_strings:
    path = parse_path_string(path_str)
    set_random_pencolor(bob)
    draw_path(bob, path)
    vec(bob)  # vẽ mũi tên cuối path

# -----------------------------
# Giữ màn hình
turtle.done()
