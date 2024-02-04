# Firmware for Scara Robotic Arm

Team members:
- [Maximiliano Butlerov](https://github.com/mbutlerov)
- [Guillermo Ocampos](https://github.com/GOcampos91)
- [Caleb Trepowski](https://github.com/calebtrepowski)

## Interface

Serial communication takes G codes as listed below:

- `G0 X_ Y_ Z-_`: Rapid movement (may not result in a straight line), Z always negative. If any parameter is omitted, its previous value is maintained. Applies inverse kinematics.

- `G1 X_ Y_ Z-_`: Smooth movement (linear interpolation), Z always negative. If any parameter is omitted, its previous value is maintained. Applies inverse kinematics.

- `G10`: Send all motors to the endstop.

- `G11 H_ J_ K_ L_`: Go to joint coordinates (H, J, K, L). Joint 1: degrees, Joint 2: mm, Joint 3: degrees, Joint 4: degrees. If any parameter is omitted, its previous value is maintained. Applies direct kinematics.

- `G12 H_ J_ K_ L_`: Set joint coordinates. Ideally used after G11 with the same values following a reset, or adjustment by eye (not recommended).

- `G20 P_`: Close gripper. The parameter is the value to close. 0 → fully open. 255 → fully closed.

- `G21`: Open gripper.

Responses contain the following format:

```plaintext
$[STATUS_CODE]XYZ:(x_value,y_value,z_value);J1J2J3J4(joint1_value,joint2_value,joint3_value,joint4_value)&
```

so it can be parsed with a master program. Example for Python with `re` module:

```python
import re

def parse_robot_params(robot_response: string):
        pattern = r"\[(\d+)\]XYZ:\(([\d.-]+),([\d.-]+),([\d.-]+)\);J1J2J3J4:\(([\d.-]+),([\d.-]+),([\d.-]+),([\d.-]+)\)"
        match = re.match(pattern, robot_response)

        if match:
            status_number = int(match.group(1))
            x_coord, y_coord, z_coord = float(match.group(2)), float(
                match.group(3)), float(match.group(4))
            j1, j2, j3, j4 = float(match.group(5)), float(
                match.group(6)), float(match.group(7)), float(match.group(8))

        return status_number, x_coord, y_coord, z_coord, j1, j2, j3, j4
```

## Demo

The robot has been used to develop a [egg box filler using computer vision](https://github.com/calebtrepowski/egg-supplier-cv). Here's a link to a [demo video](https://drive.google.com/file/d/1SxGCYM9XJyWuuOgOleMaBsQY9g8OEJad/view?usp=drive_link) of that project.