import curses
import sys
from time import sleep
from os import path
from roboclaw_3 import Roboclaw
from adafruit_servokit import ServoKit

BAUD_RATE = 115200
MOTOR_ADDRESSES = [128, 129, 130, 128, 129, 130]  # Direcciones de los motores
FORWARD_SPEED = 35  # Velocidad para mover los motores hacia adelante
BACKWARD_SPEED = 35  # Velocidad para mover los motores hacia atrás
ACCEL_RATE = 0.5
ACCEL_MAX = 2**15 - 1
ACCEL = int(ACCEL_MAX * ACCEL_RATE)

# Configuración de servos
kit = ServoKit(channels=16)

# Necesitamos agregar roboclaw.py al path
sys.path.append(path.join(path.expanduser('~'), 'osr_ws/src/osr-rover-code/ROS/osr_control/osr_control'))

def test_connection(address):
    roboclaw0 = Roboclaw("/dev/serial0", BAUD_RATE)
    roboclaw1 = Roboclaw("/dev/serial1", BAUD_RATE)
    connected0 = roboclaw0.Open() == 1
    connected1 = roboclaw1.Open() == 1
    if connected0:
        print("Connected to /dev/serial0.")
        return roboclaw0
    elif connected1:
        print("Connected to /dev/serial1.")
        return roboclaw1
    else:
        print("Could not open comport /dev/serial0 or /dev/serial1.")
        return None

def move_all_motors(rc, direction):
    for address in MOTOR_ADDRESSES:
        if address == 128:
            if direction == "backward":
                rc.BackwardM1(address, FORWARD_SPEED)
                rc.BackwardM2(address, FORWARD_SPEED)
            elif direction == "forward":
                rc.ForwardM1(address, BACKWARD_SPEED)
                rc.ForwardM2(address, BACKWARD_SPEED)

        if address == 129:
            if direction == "backward":
                rc.ForwardM1(address, FORWARD_SPEED)
                rc.BackwardM2(address, FORWARD_SPEED)
            elif direction == "forward":
                rc.BackwardM1(address, BACKWARD_SPEED)
                rc.ForwardM2(address, BACKWARD_SPEED)        

        if address == 130:
            if direction == "backward":
                rc.ForwardM1(address, FORWARD_SPEED)
                rc.ForwardM2(address, FORWARD_SPEED)
            elif direction == "forward":
                rc.BackwardM1(address, BACKWARD_SPEED)
                rc.BackwardM2(address, BACKWARD_SPEED)

def stop_all_motors(rc):
    for address in MOTOR_ADDRESSES:
        rc.ForwardM1(address, 0)
        rc.ForwardM2(address, 0)

def move_servo(servo_index, target_angle):
    kit.servo[servo_index].actuation_range = 300
    kit.servo[servo_index].set_pulse_width_range(500, 2500)
    kit.servo[servo_index].angle = target_angle
    print(f"Servo motor at channel {servo_index} set to {target_angle}")

def main(stdscr):
    # Limpiar la pantalla
    curses.curs_set(0)  # Ocultar el cursor
    stdscr.nodelay(1)   # No bloquear la entrada del teclado
    stdscr.timeout(10)   # Configurar el timeout para la entrada de teclas (más rápido)

    rc = test_connection(MOTOR_ADDRESSES[0])

    if rc is None:
        print("No se pudo conectar a los RoboClaw.")
        sys.exit(1)

    stdscr.addstr(0, 0, "Presiona las teclas para mover servos y motores.")
    stdscr.refresh()

    # Teclas presionadas
    while True:
        key = stdscr.getch()  # Leer la tecla presionada

        if key == 27:  # Si se presiona la tecla ESC (código ASCII 27)
            stop_all_motors(rc)
            break

        # Movimiento de los motores
        if key == curses.KEY_UP:  # Flecha hacia arriba
            move_all_motors(rc, "forward")
            stdscr.addstr(1, 0, "Moviendo todos los motores hacia adelante.")
            stdscr.refresh()

        elif key == curses.KEY_DOWN:  # Flecha hacia abajo
            move_all_motors(rc, "backward")
            stdscr.addstr(1, 0, "Moviendo todos los motores hacia atrás.")
            stdscr.refresh()

        else:
            stop_all_motors(rc)  # Detener los motores cuando no se presiona ninguna flecha

        # Movimiento de los servos con teclas específicas
        if key == ord('w'):  # Mover servo 0 a 120 grados
            move_servo(0, 120)
            stdscr.addstr(2, 0, "Moviendo servo 0 a 120 grados.")
            stdscr.refresh()

        elif key == ord('s'):  # Mover servo 0 a 180 grados
            move_servo(0, 180)
            stdscr.addstr(2, 0, "Moviendo servo 0 a 180 grados.")
            stdscr.refresh()

        elif key == ord('e'):  # Mover servo 1 a 120 grados
            move_servo(1, 120)
            stdscr.addstr(3, 0, "Moviendo servo 1 a 120 grados.")
            stdscr.refresh()

        elif key == ord('d'):  # Mover servo 1 a 180 grados
            move_servo(1, 180)
            stdscr.addstr(3, 0, "Moviendo servo 1 a 180 grados.")
            stdscr.refresh()

        elif key == ord('r'):  # Mover servo 2 a 120 grados
            move_servo(2, 120)
            stdscr.addstr(4, 0, "Moviendo servo 2 a 120 grados.")
            stdscr.refresh()

        elif key == ord('f'):  # Mover servo 2 a 180 grados
            move_servo(2, 180)
            stdscr.addstr(4, 0, "Moviendo servo 2 a 180 grados.")
            stdscr.refresh()

        elif key == ord('t'):  # Mover servo 3 a 120 grados
            move_servo(3, 120)
            stdscr.addstr(5, 0, "Moviendo servo 3 a 120 grados.")
            stdscr.refresh()

        elif key == ord('g'):  # Mover servo 3 a 180 grados
            move_servo(3, 180)
            stdscr.addstr(5, 0, "Moviendo servo 3 a 180 grados.")
            stdscr.refresh()

        sleep(0.01)  # Reducción de retardo, mejora la respuesta

if __name__ == "__main__":
    curses.wrapper(main)
