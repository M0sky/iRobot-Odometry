#!/usr/bin/env python3

import sys

import os
os.environ['TCL_LIBRARY'] = 'C:/Users/34652/AppData/Local/Programs/Python/Python313/tcl/tcl8.6'
os.environ['TK_LIBRARY'] = 'C:/Users/34652/AppData/Local/Programs/Python/Python313/tcl/tk8.6'

import pycreate2
from pycreate2 import Create2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import tkinter as tk
import time
import math
import argparse
import heapq
import queue

DEFAULT_PORT = 'COM3'
DEFAULT_BAUD = {'default': 115200, 'alt': 19200}

class RobotController:
    def __init__(self, port=DEFAULT_PORT, baud=DEFAULT_BAUD['default']):
        self.port = port
        self.baud = baud
        self.bot = None
        self.odometer = None
        self.plotter = None
        self.queue = None
        self.running = None
        self.mode = None
        self.back = False

    def initialize_robot(self):
        while self.bot is None:
            try:
                print(f"Conectando al robot en el puerto {self.port} a {self.baud} baudios.")
                self.bot = Create2(port=self.port, baud=self.baud)
                self.bot.start()
                self.bot.safe()
                self.mode = 'manual'

                self.odometer = Odometer(self.bot)
                self.odometer.initialize_odometry()

                self.queue = queue.Queue()  # Inicializar la cola
                plt.ion()  # Modo interactivo
                self.fig, self.ax = plt.subplots(figsize=(8, 8))  # Crear un solo subplot

                # Cargar la imagen del robot
                self.irobot_png = mpimg.imread('C:\\Users\\34652\\OneDrive\\Escritorio\\iRobot.png')
                self.irobot_size = 0.1 

                # Configuración inicial del gráfico
                self.ax.set_xlim(-2, 2)
                self.ax.set_ylim(-2, 2)
                self.ax.set_title('iRobot Roomba 650 Pathing')
                self.ax.set_xlabel('X (m)')
                self.ax.set_ylabel('Y (m)')
                self.ax.grid()

                # Crear una lista vacía para almacenar las coordenadas de la trayectoria
                self.trajectory = []
                self.current_path = []
                # Añadir la posición inicial a la trayectoria
                self.trajectory.append((0, 0))  # Agregar el punto inicial a la trayectoria

                self.bot.digit_led_ascii('O  O')
                
                #print(dir(self.bot.SCI.ser))

                # Definición de constantes y variables iniciales para A*
                self.step_distance_cm = 30     # Tamaño de cada paso en la cuadrícula en cm
                self.start = (0, 0)            # Punto inicial en la cuadrícula
                self.goal = None               # Objetivo, a definir dinámicamente
                self.g_cost = {self.start: 0}  # Costo acumulado desde el inicio a cada nodo
                self.came_from = {self.start: None}  # Para reconstruir el camino desde el inicio

                
                # Encender el LED de encendido
                self.bot.led(
                    power_intensity=255,    # Configura la intensidad máxima (0-255)
                    power_color=25         # Verde 0 - Rojo 255
                )

                self.update_plot()

                # Musica encendido
                self.canta(3)
                self.odometer.get_battery_status()

                print("Conexión exitosa con el robot.")
                #print(dir(self.bot)) 
            except Exception as e:
                print(f"Error al conectar el robot: {e}")
                self.bot = None  # Asegúrate de que bot sea None si hay un error
                sys.exit(1)

    def close_robot(self):
        """ Cierra la conexión con el robot de forma segura """
        self.canta(4)

        plt.close(self.fig)

        if self.bot is not None:
            self.bot.drive_stop()  # Asegúrate de parar el robot antes de desconectar
            self.bot.close()  # Cierra la conexión
            self.bot = None  # Libera la referencia al bot

        # Desactiva el método __del__ de Create2 para evitar errores al cerrar
        pycreate2.Create2.__del__ = lambda self: None

        print("Conexión cerrada, hasta luego!")

    def run(self):
        parser = argparse.ArgumentParser(description="Control de robot con parámetros.")
        parser.add_argument('--port', type=str, default=DEFAULT_PORT, help='Ruta del puerto del robot.')
        parser.add_argument('--baud', type=int, default=DEFAULT_BAUD['default'], help='Velocidad de baudios para la comunicación con el robot.')
        parser.add_argument('--mode', type=str, default='manual', choices=['manual', 'auto'], help='Modo de ejecución (manual o automático).')

        args = parser.parse_args()

        self.port = args.port
        self.baud = args.baud
        self.initialize_robot()

        if args.mode == 'manual':
            self.mode = args.mode
            print("Modo manual activado.")
            try:
                while True:
                    self.process_queue()  # Procesa la cola sin bloquear

                    command = CommandInterface.read_command(args.mode)
                    if command == "q" or command == 'Q':
                        break
                    elif command == "h" or command == 'H':
                        CommandInterface.print_command_help()
                        continue
                    
                    # Validaciones
                    parts = command.split()
                    action = parts[0]

                    if len(parts) != 2:
                        print("Comando no válido.")
                        continue
                    
                    try:
                        value = float(parts[1])
                    except ValueError:
                        print("Debe ingresar un número válido (distancia o ángulo).")
                        continue

                    # Movimientos
                    if action == 'm' or action == 'M':
                        self.move(value)
                        self.queue.put("update")
                        self.update_plot()  # Llamar a update_plot aquí también para el primer movimiento
                    elif action == 'g' or action == 'G':
                        self.turn(value)
                        self.queue.put("update")
                        self.update_plot()  # Llamar a update_plot aquí también para el primer movimiento
                    elif action == 'c' or action == 'C':
                        self.canta(int(value))
                    else:
                        print("Comando no reconocido. Usa 'm' para mover, 'g' para girar, 'q' para salir, 'c' para cantar, 'h' para ayuda.")
                        

            except KeyboardInterrupt:
                print("\nInterrupción detectada.")
            finally:
                self.close_robot()
                print("Acabo de cerrar las conexiones de manera correcta")
        else:
            self.mode = args.mode
            print("Modo automático activado.")
            try:
                while True:

                    # Leer comando usando CommandInterface
                    command = CommandInterface.read_command(args.mode)
                    if command.lower() == "q":
                        break
                    elif command.lower() == "h":
                        CommandInterface.print_auto_command_help()
                        continue
                        
                    # Validar formato de comando
                    parts = command.split()
                    if len(parts) != 2:
                        print("Debe proporcionar las coordenadas [X, Y] en un solo comando (ejemplo: '4 -1').")
                        continue

                    try:
                        # Convertir las coordenadas X e Y
                        target_x = int(parts[0])
                        target_y = int(parts[1])
                            
                        self.set_goal(target_x, target_y)

                        # Ejecutar el algoritmo A*
                        path = self.a_star()

                        # Si path es None, significa que no se encontró un camino; si no, contiene la lista de nodos
                        if path:
                            print("Camino calculado:", path)
                            # Camino hacia goal
                            self.follow_path(path)

                            if self.back:
                                print("Se encontraron obstáculos en el camino")
                                # Orientación inicial
                                self.return_orientation()
                                return

                            # Ruta de regreso al punto de partida invirtiendo el camino
                            path_reverse = path[::-1]
                            print("Camino de regreso al origen:", path_reverse)
                            self.follow_path(path_reverse)

                            # Orientación inicial
                            self.return_orientation()

                        else:
                            print("No se encontró un camino al objetivo.")

                    except ValueError:
                        # En caso de que la conversión falle, el valor no era un número válido
                        print("Error: por favor ingrese valores numéricos válidos para X y Y.")
                    except Exception as e:
                        # Cualquier otro error imprevisto
                        print(f"Se ha producido un error: {e}")
            except KeyboardInterrupt:
                print("\nInterrupción detectada.")
            finally:
                self.close_robot()
                print("Acabo de cerrar las conexiones de manera correcta")

    def return_orientation(self):
        # Orientación inicial
        current_angle = self.odometer.current_angle
        current = math.degrees(current_angle)  # Convertir el ángulo a grados para imprimir
        initial = self.odometer.starting_angle
        if current != initial:
            angle_correction = initial - current
                                    
        print(f"Ajustando la orientación a {initial} grados.")
        print(f"Ángulo actual: {current} grados.")
                                    
        self.turn(angle_correction)

    def follow_path(self, path):
        """Recorre el path proporcionado moviéndose de nodo a nodo."""

        # Convertir cada punto del camino a posición física y hacer que el robot se mueva
        for node in path:
            physical_x, physical_y = self.grid_to_physical_position(*node)
            print(f"Moviendo a posición física: ({physical_x} cm, {physical_y} cm)")

            # Actualizar la odometría antes de calcular la distancia y el ángulo
            self.odometer.update_odometry()

            # Calcular la distancia desde la posición actual a la nueva posición
            current_x, current_y = self.odometer.get_current_position()
            distance_to_move = math.sqrt((physical_x - current_x)**2 + (physical_y - current_y)**2)
                                
            # Calcular el ángulo necesario para girar
            angle_to_turn = self.odometer.calculate_angle(physical_x, physical_y)
                                
            # Girar al nuevo ángulo
            self.turn(angle_to_turn)
            # Mover a la nueva posición
            self.move(distance_to_move)

            # Actualizar las coordenadas de la cuadrícula después del movimiento
            self.odometer.update_grid_coordinates()

            # Guardo el path actual para volver sobre mis pasos
            if not self.back:
                self.current_path.append((int(physical_x/30), int(physical_y/30)))

            # Actualiza plot en tiempo real
            self.queue.put("update")
            self.update_plot() 
            self.process_queue()  # Procesa la cola sin bloquear       

    def update_plot(self):
        # Obtener la posición actual en metros
        current_position = self.odometer.current_position

        odom_x = current_position[0] / 1000  # Convertir a metros
        odom_y = current_position[1] / 1000  # Convertir a metros

        # Actualizar la imagen del robot
        if hasattr(self, 'robot_patch'):
            # Actualiza la posición de la imagen del robot
            self.robot_patch.set_extent((
                odom_x - self.irobot_size / 2, 
                odom_x + self.irobot_size / 2,
                odom_y - self.irobot_size / 2, 
                odom_y + self.irobot_size / 2
            ))
        else:
            # Inicializa la imagen del robot
            self.robot_patch = self.ax.imshow(
                self.irobot_png,
                extent=(odom_x - self.irobot_size/2, odom_x + self.irobot_size/2,
                        odom_y - self.irobot_size/2, odom_y + self.irobot_size/2),
                animated=True
            )


        # Añadir la posición actual a la trayectoria
        self.trajectory.append((odom_x, odom_y))

        # Dibujar la trayectoria
        if len(self.trajectory) > 1:  # Si hay más de un punto, dibujar la línea
            self.ax.plot(*zip(*self.trajectory), color='red')

        # Actualizar el gráfico
        self.fig.canvas.draw()
        plt.pause(0.01)

    def close(self):
        plt.close(self.fig)

    def process_queue(self):
        try:
            while True:
                message = self.queue.get_nowait()  # No bloquea
                if message == "update":
                    self.update_plot()  # Actualiza el gráfico
        except queue.Empty:
            pass

    def move(self, distance_cm):
        """ Mueve el robot hacia adelante o atrás usando la distancia deseada controlada por odometría """

        # Obtener la lectura inicial de los encoders
        sensor = self.odometer.get_sensor_data()

        if sensor is None:
            print("Error: No se pueden obtener datos de sensores - Move()")
            self.odometer.clear_serial_buffer()
            self.move(distance_cm)
            return
        
        if (int(sensor.encoder_counts_left) == 0 or int(sensor.encoder_counts_right) == 0):
            print("Error de lectura de encoders -> 000 - Move()")
            return

        # Convertir la distancia a milímetros
        target_distance_mm = distance_cm * 10  # Convertimos a milímetros (mm)
        
        # Inicializamos la distancia recorrida
        total_distance_mm = 0
        encoder_min_value = -32768
        encoder_max_value = 32767
        max_encoder_range = encoder_max_value - encoder_min_value

        # Velocidad de movimiento basada en la dirección (positivo para adelante, negativo para atrás)
        speed = 100 if distance_cm > 0 else -100
        print(f'Moviendo {"adelante" if distance_cm > 0 else "atrás"} {abs(distance_cm):.2f} cm')

        # Encoders inicialmente
        encoderIzqIni = int(sensor.encoder_counts_left)
        encoderDerIni = int(sensor.encoder_counts_right)

        # Comenzar el movimiento
        self.bot.drive_direct(speed, speed)

        # Seguimos actualizando la odometría y el avance hasta llegar a la distancia deseada
        while abs(total_distance_mm) < abs(target_distance_mm):

            # Obtener nuevas lecturas de los sensores
            sensor = self.odometer.get_sensor_data()
            
            if sensor is None:
                print("Error: No se pueden obtener datos de sensores - Move While()")
                self.bot.drive_stop()
                self.odometer.clear_serial_buffer()
                self.move(int(target_distance_mm - total_distance_mm) / 10)
                return

            if (int(sensor.encoder_counts_left) == 0 or int(sensor.encoder_counts_right) == 0):
                self.bot.drive_stop()
                print("Error de lectura de encoders -> 000 - Move While()")
                return
            
            # Comprobación de obstáculos
            if self.odometer.is_obstacle_near():
                print("Obstáculo detectado. Retrocediendo.")
                self.bot.drive_stop()

                if self.mode == 'auto':
                    # Logica para volver marcha atras hasta la ultima coordenada
                    reverse_distance_cm = -abs(target_distance_mm - total_distance_mm) / 10
                    print(f"Retrocediendo a la ultima coordenada: {reverse_distance_cm:.2f} cm")
                else:
                    reverse_distance_cm = -10  # Retroceder 10 cm
                
                reverse_encoderIzqIni = sensor.encoder_counts_left
                reverse_encoderDerIni = sensor.encoder_counts_right  
                reverse_speed = -100  # Velocidad negativa para retroceder
                reverse_total_distance_cm = 0    

                # Comenzar el retroceso
                self.bot.drive_direct(reverse_speed, reverse_speed)
                
                # Retroceder hasta la distancia deseada mediante odometría
                while abs(reverse_total_distance_cm) < abs(reverse_distance_cm):
                    sensor = self.odometer.get_sensor_data()
                    
                    if sensor is None:
                        print("Error: No se pueden obtener datos de sensores - Retroceso")
                        self.bot.drive_stop()
                        self.odometer.clear_serial_buffer()
                        self.move(int(reverse_distance_cm - reverse_total_distance_cm))
                        break

                    if (int(sensor.encoder_counts_left) == 0 or int(sensor.encoder_counts_right) == 0):
                        self.bot.drive_stop()
                        print("Error de lectura de encoders -> 000 - Retroceso While()")
                        return

                    # Nuevos encoders
                    reverse_encoderIzq = sensor.encoder_counts_left
                    reverse_encoderDer = sensor.encoder_counts_right

                    # Calcular la diferencia de los encoders
                    delta_encoder_izq = reverse_encoderIzq - reverse_encoderIzqIni
                    delta_encoder_der = reverse_encoderDer - reverse_encoderDerIni

                    # Manejo de desbordamiento de encoders
                    if delta_encoder_izq < -max_encoder_range // 2:
                        delta_encoder_izq += max_encoder_range
                    elif delta_encoder_izq > max_encoder_range // 2:
                        delta_encoder_izq -= max_encoder_range

                    if delta_encoder_der < -max_encoder_range // 2:
                        delta_encoder_der += max_encoder_range
                    elif delta_encoder_der > max_encoder_range // 2:
                        delta_encoder_der -= max_encoder_range

                    # Calcular distancia de retroceso en cm
                    deltaSIzq = (delta_encoder_izq * self.odometer.mm_per_tick) / 10  # mm a cm
                    deltaSDer = (delta_encoder_der * self.odometer.mm_per_tick) / 10  # mm a cm
                    delta_s_reverse = (deltaSIzq + deltaSDer) / 2
                    reverse_total_distance_cm += -abs(delta_s_reverse)  # Registrar negativo

                    # Actualizar la odometría
                    self.odometer.update_odometry()
                                    
                    print(f"Retroceso acumulado: {reverse_total_distance_cm:.2f} cm")

                    reverse_encoderIzqIni, reverse_encoderDerIni = reverse_encoderIzq, reverse_encoderDer
                    time.sleep(0.1)

                self.bot.drive_stop()
                print("Retroceso completado.")

                self.bot.led(
                    power_intensity=255,    # Configura la intensidad máxima (0-255)
                    power_color=25         # Verde 0 - Rojo 255
                )

                # Imprimir la odometría final (opcional)
                self.odometer.print_odometry()

                if self.mode == 'auto':
                    self.back = True
                    reversed_path = self.current_path[::-1]  # Crea una nueva lista que es la inversa de current_path
                    print("Camino de regreso al origen:", reversed_path)
                    self.follow_path(reversed_path)
                    return
                else:
                    return

            # Nuevos encoders        
            encoderIzq = sensor.encoder_counts_left
            encoderDer = sensor.encoder_counts_right    

            # Calcular los cambios en los encoders
            delta_encoder_izq = self.handle_encoder_overflow(encoderIzqIni, encoderIzq, max_encoder_range)
            delta_encoder_der = self.handle_encoder_overflow(encoderDerIni, encoderDer, max_encoder_range)

            # Corrección de posibles desbordamientos de los encoders
            if abs(delta_encoder_der) > 1000:
                print("Corregir salto en encoder derecho")
                encoderDer = encoderDerIni  # Usar el valor anterior para evitar saltos
                encoderIzq = encoderIzqIni
                self.odometer.update_odometry
                self.queue.put("update")
                self.plotter.update_plot()
                self.move(int(target_distance_mm - total_distance_mm) / 10)


            if abs(delta_encoder_izq) > 1000:
                print("Corregir salto en encoder izquierdo")
                encoderDer = encoderDerIni  # Usar el valor anterior para evitar saltos
                encoderIzq = encoderIzqIni
                self.odometer.update_odometry
                self.queue.put("update")
                self.update_plot()
                self.move(int(target_distance_mm - total_distance_mm) / 10)

            
            # Calcular el avance en cada rueda
            deltaSIzq = delta_encoder_izq * self.odometer.mm_per_tick
            deltaSDer = delta_encoder_der * self.odometer.mm_per_tick

            # Cálculo del avance lineal (promedio de ambas ruedas)
            delta_s = (deltaSIzq + deltaSDer) / 2

            # Actualizamos la distancia total recorrida
            total_distance_mm += delta_s

            # Actualizamos la odometría
            self.odometer.update_odometry()
            

            # Mostrar progreso del avance
            #print(f"Avance actual: {total_distance_mm / 10:.2f} cm / {distance_cm} cm")
            #print(f"Encoder izquierdo: {encoderIzq}, Encoder derecho: {encoderDer}")

            # Actualizamos los encoders iniciales
            encoderIzqIni = encoderIzq
            encoderDerIni = encoderDer
            
            time.sleep(0.1)  # Pausa breve para evitar sobrecargar la CPU

        # Detener el robot cuando la distancia objetivo se haya alcanzado
        self.bot.drive_stop()
        print(f"Movimiento completado. Distancia final: {total_distance_mm / 10:.2f} cm")
        
        # Imprimir la odometría final (opcional)
        self.odometer.print_odometry()
    
    def turn(self, angle_deg):
        """Gira el robot con precisión basada en la odometría y los encoders"""

        # Obtener la lectura inicial de los encoders
        sensor = self.odometer.get_sensor_data()

        if sensor is None:
            print("Error: No se pueden obtener datos de los sensores - Turn()")
            self.odometer.clear_serial_buffer()
            self.turn(angle_deg)
            return

        if (int(sensor.encoder_counts_left) == 0 or int(sensor.encoder_counts_right) == 0):
            print("Error de lectura de encoders -> 000 - Turn()")
            return

        # Constantes
        wheel_base_mm = self.odometer.wheel_base  # Distancia entre las ruedas en mm
        
        # Factores de corrección
        velocity_correction_factor = 0.515

        # Variables de control
        total_turned_angle_deg = 0
        encoder_min_value = -32768
        encoder_max_value = 32767
        max_encoder_range = encoder_max_value - encoder_min_value

        # Establecer velocidades bajas para mejor control
        right_wheel_speed = 60 if angle_deg > 0 else -60  # Gira a la izquierda si el ángulo es positivo
        left_wheel_speed = -60 if angle_deg > 0 else 60   # Gira a la derecha si el ángulo es negativo
        print(f'Girando {"izquierda" if angle_deg > 0 else "derecha"} {abs(angle_deg)} cm')

        # Encoders inicialmente
        encoderIzqIni = int(sensor.encoder_counts_left)
        encoderDerIni = int(sensor.encoder_counts_right)

        # Comienza el giro
        self.bot.drive_direct(right_wheel_speed, left_wheel_speed)

        # Seguimos actualizando la odometría y el avance hasta llegar al ángulo deseado
        while abs(total_turned_angle_deg) < abs(angle_deg):
            
            # Obtener nuevas lecturas de los sensores
            sensor = self.odometer.get_sensor_data()

            if sensor is None:
                print("Error: No se pueden obtener datos de los sensores - Turn While()")
                self.bot.drive_stop()
                self.odometer.clear_serial_buffer()
                self.turn(angle_deg - total_turned_angle_deg)
                return

            if (int(sensor.encoder_counts_left) == 0 or int(sensor.encoder_counts_right) == 0):
                self.bot.drive_stop()
                print("Error de lectura de encoders -> 000 - Turn While()")
                return

            # Nuevos encoders               
            encoderIzq = sensor.encoder_counts_left
            encoderDer = sensor.encoder_counts_right

            # Calcular los cambios en los encoders
            delta_encoder_izq = self.handle_encoder_overflow(encoderIzqIni, encoderIzq, max_encoder_range)
            delta_encoder_der = self.handle_encoder_overflow(encoderDerIni, encoderDer, max_encoder_range)

            # Corrección de posibles desbordamientos de los encoders
            if abs(delta_encoder_der) > 1000:
                print("Corregir salto en encoder derecho")
                encoderDer = encoderDerIni  # Usar el valor anterior para evitar saltos
                encoderIzq = encoderIzqIni
                self.odometer.update_odometry
                self.queue.put("update")
                self.update_plot()  # Llamar a update_plot aquí también para el primer movimiento
                self.turn(angle_deg - total_turned_angle_deg)
                return
            
            if abs(delta_encoder_izq) > 1000:
                print("Corregir salto en encoder izquierdo")
                encoderDer = encoderDerIni  # Usar el valor anterior para evitar saltos
                encoderIzq = encoderIzqIni
                self.odometer.update_odometry
                self.queue.put("update")
                self.update_plot()  # Llamar a update_plot aquí también para el primer movimiento
                self.turn(angle_deg - total_turned_angle_deg)
                return

            # Calcular las distancias recorridas por cada rueda desde el inicio del giro
            deltaSIzq = delta_encoder_izq * self.odometer.mm_per_tick
            deltaSDer = delta_encoder_der * self.odometer.mm_per_tick

            # Calcular el avance angular en radianes basado en la diferencia de distancias
            delta_s = (deltaSDer - deltaSIzq) / wheel_base_mm
            delta_s *= velocity_correction_factor

            # Actualizar el ángulo total girado (en grados)
            total_turned_angle_deg += math.degrees(delta_s * 2)

            # Actualizamos la odometría
            self.odometer.update_odometry()

            # Mostrar progreso
            #print(f"Ángulo girado actual: {total_turned_angle_deg:.2f} grados / {angle_deg} grados")
            #print(f"Encoder izquierdo: {encoderIzq}, Encoder derecho: {encoderDer}")

            # Actualizar los encoders iniciales
            encoderIzqIni = encoderIzq
            encoderDerIni = encoderDer

            # Pausa para no sobrecargar la CPU
            time.sleep(0.05)  # Aumentar la pausa para leer más lentamente y evitar lecturas excesivas

        # Detener el robot una vez alcanzado el ángulo
        self.bot.drive_stop()
        print(f"Giro completado. Ángulo final: {total_turned_angle_deg:.2f} grados")

        # Imprimir la odometría final (opcional)
        self.odometer.print_odometry()

    def canta(self, seleccion):
        # Definimos las canciones con notas
        songs = {
            1: [59, 64, 62, 32, 69, 96, 67, 64, 62, 32, 60, 96, 59, 64, 59, 32, 59, 32, 60, 32, 62, 32, 64, 96, 62, 96],  # Canción 1
            2: [76, 16, 76, 16, 76, 32, 76, 16, 76, 16, 76, 32, 76, 16, 79, 16, 72, 16, 74, 16, 76, 32, 77, 16, 77, 16, 77, 16, 77, 32, 77, 16],  # Canción 2
            3: [76, 12, 76, 12, 20, 12, 76, 12, 20, 12, 72, 12, 76, 12, 20, 12, 79, 12, 20, 36],  # Canción 3 - START
            4: [72, 12, 20, 24, 67, 12, 20, 24, 64, 24, 69, 16, 71, 16, 69, 16, 68, 24, 70, 24, 68, 24, 67, 12, 65, 12, 67, 48]  # Canción 4 - END
        }

        # Verificar si la selección es válida
        if seleccion not in songs:
            print("Selección inválida. Elige (1 2 3 4).")
            return

        # Obtener la canción seleccionada
        song = songs[seleccion]
        #print(">> song len: ", len(song)//2)

        # Asignar un número de canción único
        song_num = int(seleccion) - 1  # Ajustar para que empiece desde 0

        # Crear la canción en el robot
        self.bot.createSong(song_num, song)
        time.sleep(0.1)

        # Reproducir la canción
        how_long = self.bot.playSong(song_num)

        # Esperar la duración total de la canción antes de continuar
        #print('Duración de la canción: ', how_long)
        time.sleep(how_long)

    def handle_encoder_overflow(self, previous_value, current_value, max_value):
        difference = current_value - previous_value

        # Manejar el desbordamiento positivo
        if difference < -max_value // 2:
            difference += max_value
        # Manejar el desbordamiento negativo
        elif difference > max_value // 2:
            difference -= max_value

        return difference

    def set_goal(self, goal_x, goal_y):
        """Define el objetivo en la cuadrícula."""
        self.goal = (goal_x, goal_y)

    def a_star(self):
        """Ejecuta el algoritmo A* desde el inicio al objetivo."""
        if self.goal is None:
            print("Error: No se ha definido un objetivo.")
            return None

        # Configuración inicial de cola de prioridad - open_set (lista)
        open_set = []
        # Conjunto para rastrear nodos ya añadidos a open_set
        open_set_set = set()
        # Agregamos el primer nodo (0 es el costo inicial de la posicion (0, 0))
        heapq.heappush(open_set, (0, self.start))  # Agregamos el nodo inicial

        # 8 direcciones posibles (arriba, abajo, izquierda, derecha y diagonales)
        neighbors = [
            (0, 1), (1, 0), (0, -1), (-1, 0),  # Movimientos cardinales
            (1, 1), (-1, -1), (1, -1), (-1, 1) # Movimientos diagonales
        ]

        while open_set:
            _, current = heapq.heappop(open_set)

            # Si alcanzamos el objetivo, reconstruimos el camino
            if current == self.goal:
                return self.reconstruct_path(current)

            for dx, dy in neighbors:
                neighbor = (current[0] + dx, current[1] + dy)

                # Costo del movimiento
                movement_cost = self.step_distance_cm * math.sqrt(dx**2 + dy**2)
                new_cost = self.g_cost[current] + movement_cost

                # Si el nodo vecino no está en g_cost o se encuentra un camino más corto
                if neighbor not in self.g_cost or new_cost < self.g_cost[neighbor]:
                    self.g_cost[neighbor] = new_cost
                    
                    # Llamada a la heurística según el tipo de movimiento
                    if abs(dx) + abs(dy) == 1:  # Movimiento cardinal
                        h_cost = self.manhattan_heuristic(neighbor)
                    else:  # Movimiento diagonal
                        h_cost = self.euclidean_heuristic(neighbor)

                    f_cost = new_cost + h_cost
                    
                    # Solo añadir al conjunto y la cola si no está ya presente
                    if neighbor not in open_set_set:
                        heapq.heappush(open_set, (f_cost, neighbor))
                        open_set_set.add(neighbor)  # Añadir el vecino al conjunto
                        self.came_from[neighbor] = current
                
        return None

    def manhattan_heuristic(self, current):
        """Calcula la heurística de Manhattan al objetivo."""
        dx = abs(self.goal[0] - current[0])
        dy = abs(self.goal[1] - current[1])
        return self.step_distance_cm * (dx + dy)

    def euclidean_heuristic(self, current):
        """Calcula la heurística Euclidiana al objetivo."""
        dx = self.goal[0] - current[0]
        dy = self.goal[1] - current[1]
        return self.step_distance_cm * math.sqrt(dx**2 + dy**2)

    def reconstruct_path(self, current):
        """Reconstruye el camino desde el objetivo al inicio."""
        path = []
        while current is not None:
            path.append(current)
            current = self.came_from[current]
        path.reverse()  # Invertir el camino para que vaya de inicio a fin
        return path

    def grid_to_physical_position(self, x, y):
        """Convierte coordenadas de cuadrícula (x, y) a posición física en cm."""
        physical_x = x * self.step_distance_cm
        physical_y = y * self.step_distance_cm
        return physical_x, physical_y

class Odometer:
    
    # Códigos de color ANSI
    RESET = "\033[0m"
    HEADER = "\033[95m"
    OKBLUE = "\033[94m"
    OKGREEN = "\033[92m"
    WARNING = "\033[93m"
    FAIL = "\033[91m"
    BOLD = "\033[1m"

    def __init__(self, bot):
        self.bot = bot  # Ahora bot es el objeto Create2, no el RobotController
        self.current_position = [0, 0]              # Representa la posición (x, y) en mm
        self.current_angle = 0.0                    # Ángulo actual en grados
        self.wheel_diameter = 72.0                  # Diámetro de las ruedas en mm
        self.wheel_base = 235.0                     # Diámetro de las ruedas en mm
        self.ticks_per_revolution = 508.8           # Número de ticks por revolución de rueda
        self.mm_per_tick = (math.pi * self.wheel_diameter) / self.ticks_per_revolution

        # Estado de los encoders para el cálculo de movimiento
        self.last_left_encoder = 0
        self.last_right_encoder = 0

        # Declaración de atributos de bumpers
        self.bumps_wheeldrops_left = 0
        self.bumps_wheeldrops_right = 0

        # Declaración de atributos de light bumpers
        self.light_bumper_left = 0
        self.light_bumper_front_left = 0
        self.light_bumper_center_left = 0
        self.light_bumper_center_right = 0
        self.light_bumper_front_right = 0
        self.light_bumper_right = 0

        # Posición inicial para establecer la coordenada (0, 0) A*
        self.starting_position = [0, 0]
        self.starting_angle = 0.0

        # Para almacenar coordenadas en la cuadrícula
        self.grid_coordinates = {}  # Clave: (x, y), Valor: coordenada correspondiente
        self.last_position_x = 0
        self.last_position_y = 0
        self.last_angle = 0.0

    def initialize_odometry(self):
        """Inicializa la odometría tomando la posición inicial como (0, 0)."""
        self.starting_position = self.read_position_sensors()                                       # Obtenemos la posición inicial de los sensores
        self.starting_angle = self.read_angle_sensor()                                              # Obtenemos el ángulo inicial
        
        self.current_position = [0, 0]                                                              # Establecer la posición relativa inicial en (0, 0)
        self.current_angle = 0.0                                                                    # Ángulo inicial en grados
        self.current_angle = 0.0                                                                    # Ángulo inicial en grados

        self.last_position_x = 0
        self.last_position_y = 0
        self.last_angle = 0.0
        
        self.update_grid_coordinates()                                                              # Actualiza las coordenadas de la cuadrícula al inicializar

    def read_position_sensors(self):
        """Lee los encoders y devuelve la posición inicial en mm."""
        sensors = self.get_sensor_data() 
        left_encoder = sensors.encoder_counts_left
        right_encoder = sensors.encoder_counts_right

        self.last_left_encoder = left_encoder
        self.last_right_encoder = right_encoder

        # Calcula la posición inicial usando los encoders
        x = (left_encoder + right_encoder) * 0.5 * self.mm_per_tick
        y = 0  # Si el robot está orientado en dirección (x, 0), la coordenada inicial en y es 0
        return [x, y]

    def read_angle_sensor(self):
        """Calcula el ángulo inicial en radianes basándose en la diferencia de los encoders."""
        sensors = self.get_sensor_data() 
        left_encoder = sensors.encoder_counts_left
        right_encoder = sensors.encoder_counts_right

        # Calcula el cambio de ángulo en radianes
        delta_left = left_encoder - self.last_left_encoder
        delta_right = right_encoder - self.last_right_encoder

        # Actualiza los encoders
        self.last_left_encoder = left_encoder
        self.last_right_encoder = right_encoder

        # Calcula el cambio de ángulo (radianes) utilizando la base de las ruedas
        delta_angle = (delta_right - delta_left) * (self.mm_per_tick / self.wheel_base)

        # Suma el cambio de ángulo al ángulo actual
        self.current_angle += delta_angle

        # Normaliza el ángulo a [0, 2π]
        self.current_angle = self.current_angle % (2 * math.pi)

        return self.current_angle

    def update_grid_coordinates(self):
        """Actualiza las coordenadas en la cuadrícula basándose en la posición actual."""
        grid_x = int(self.current_position[0] / 300)  # Supongamos que cada cuadrícula es de 300 mm
        grid_y = int(self.current_position[1] / 300)
        self.grid_coordinates = (grid_x, grid_y)

    def get_sensor_data(self):
        """Obtiene los datos de los sensores del objeto Create2, validación + intentos."""
        try:
            sensors = self.bot.get_sensors()
            if sensors and sensors.encoder_counts_left != 0 and sensors.encoder_counts_right != 0:
                return sensors  # Si los datos son válidos, regresamos los sensores
        except Exception as e:
            print(f"Error al obtener datos de sensores: {e}")
        
        return None

    def clear_serial_buffer(self):
        """Limpia el buffer de entrada del puerto serial."""
        try:
            # Limpia el buffer de entrada
            self.bot.SCI.ser.reset_input_buffer()
            print("Buffer de entrada limpiado.")
        except Exception as e:
            print(f"Error al limpiar el buffer: {e}")

    def update_odometry(self):
        sensors = self.get_sensor_data()
        if sensors is None:
            return  # Si no se pudieron obtener los datos de sensores, no se actualiza nada

        # Leer los encoders actuales
        current_left_encoder = sensors.encoder_counts_left
        current_right_encoder = sensors.encoder_counts_right

        # Calcular cambios en ticks
        delta_left_ticks = current_left_encoder - self.last_left_encoder
        delta_right_ticks = current_right_encoder - self.last_right_encoder
        
        # Actualizar valores de último encoder
        self.last_left_encoder = current_left_encoder
        self.last_right_encoder = current_right_encoder
        
        # Calcular cambios en distancia
        delta_left_distance = delta_left_ticks * self.mm_per_tick
        delta_right_distance = delta_right_ticks * self.mm_per_tick
        
        # Calcular el avance y cambio de ángulo del robot
        delta_distance = (delta_left_distance + delta_right_distance) / 2
        delta_angle = (delta_right_distance - delta_left_distance) / self.wheel_base
        
        # Actualizar el ángulo actual (normalizado)
        self.current_angle += delta_angle
        self.current_angle = (self.current_angle + math.pi) % (2 * math.pi) - math.pi
        
        # Calcular cambios en posición x, y        
        delta_x = delta_distance * math.cos(self.current_angle)
        delta_y = delta_distance * math.sin(self.current_angle)

        # Actualizar la posición
        self.current_position[0] += delta_x
        self.current_position[1] += delta_y

    def print_odometry(self):

        # Convertir a centímetros dividiendo por 10
        position_cm = (self.current_position[0] / 10, self.current_position[1] / 10)  # Asumiendo que current_position está en mm
        angle_deg = math.degrees(self.current_angle)  # Convertir el ángulo a grados para imprimir

        print(f"{self.HEADER}Odometría actual:{self.RESET}")    
        print(f"     {self.OKBLUE}Posición (x, y): ({position_cm[0]:.2f}, {position_cm[1]:.2f}) cm{self.RESET}")
        print(f"     {self.OKGREEN}Ángulo : {angle_deg:.2f} grados{self.RESET}")

    def get_battery_status(self):
        """Obtiene el estado de la batería del Roomba"""
        try:
            sensors = self.bot.get_sensors()  # Obtener datos del sensor
            if sensors:
                battery_charge = sensors.battery_charge
                battery_capacity = sensors.battery_capacity
                battery_percentage = (battery_charge / battery_capacity) * 100
                print(f"Estado de la batería: {battery_percentage:.2f}%")
                return battery_percentage
            else:
                print("No se pudieron obtener los datos de la batería.")
        except Exception as e:
            print(f"Error al obtener el estado de la batería: {e}")
            return None
    
    def is_obstacle_near(self, threshold=1000):
        """Verifica si hay un obstáculo cercano usando bumps y light bumpers."""
        sensors = self.get_sensor_data()  # Asegúrate de que esto devuelva el objeto de sensores
        
        if sensors is not None:
            # Comprobando bumps
            if sensors.bumps_wheeldrops.bump_left or sensors.bumps_wheeldrops.bump_right:
                print("¡Obstáculo detectado - Bumpers!!!")
                self.bot.led(
                    power_intensity=255,    # Configura la intensidad máxima (0-255)
                    power_color=255         # Verde 0 - Rojo 255
                )
                return True

            # Comprobando light bumpers
            # Extraemos los valores específicos de los sensores de light bumper
            left = sensors.light_bumper_left
            front_left = sensors.light_bumper_front_left
            center_left = sensors.light_bumper_center_left
            center_right = sensors.light_bumper_center_right
            front_right = sensors.light_bumper_front_right
            right = sensors.light_bumper_right

            # Imprimimos los valores para depuración
            # print(f"Valores de light bumpers: {left} {front_left} {center_left} {center_right} {front_right} {right}")

            # Verificamos si alguno supera el umbral para considerar un obstáculo cercano
            if (left > threshold or front_left > threshold or center_left > threshold or
                center_right > threshold or front_right > threshold or right > threshold):
                print("¡Obstáculo detectado - Light Bumpers!!!")
                self.bot.led(
                    power_intensity=255,    # Configura la intensidad máxima (0-255)
                    power_color=200         # Verde 0 - Rojo 255
                )
                return True
            
        return False
    
    def get_current_position(self):
        """Devuelve la posición actual del robot como una tupla (x, y) en cm."""
        return (self.current_position[0] / 10, self.current_position[1] / 10)

    def calculate_angle(self, target_x, target_y):
        """
        Calcula el ángulo de giro necesario para dirigir el robot hacia las coordenadas (target_x, target_y).
        Este ángulo se ajusta respecto al ángulo actual del robot.
        """
        
        delta_x = target_x - self.last_position_x
        delta_y = target_y - self.last_position_y

        self.last_position_x = target_x
        self.last_position_y = target_y
        self.last_angle = math.degrees(self.current_angle)

        # Calcular el ángulo en radianes y convertirlo a grados
        angle_rad = math.atan2(delta_y, delta_x)
        angle_deg = math.degrees(angle_rad)

        # Ajustar el ángulo relativo al ángulo actual del robot
        angle_to_turn = angle_deg - self.last_angle

        # Normalizar el ángulo para que esté en el rango [-180, 180]
        angle_to_turn = (angle_to_turn + 180) % 360 - 180

        return angle_to_turn

class CommandInterface:
    # Códigos de color ANSI
    RESET = "\033[0m"
    HEADER = "\033[95m"
    OKBLUE = "\033[94m"
    OKGREEN = "\033[92m"
    WARNING = "\033[93m"
    FAIL = "\033[91m"
    BOLD = "\033[1m"

    @staticmethod
    def read_command(mode='manual'):
        if mode == 'manual':
            return input(f"{CommandInterface.OKBLUE}Ingrese comando (m para mover, g para girar, q para salir, c para cantar, h para ayuda): {CommandInterface.RESET}").strip()
        elif mode == 'auto':
            print(f"{CommandInterface.HEADER}README - Recuerde que cada unidad corresponde a 30 cm en la realidad.{CommandInterface.RESET}")
            return input(f"{CommandInterface.OKBLUE}Ingrese coordenadas [X, Y] objetivo, q para salir, h para ayuda: {CommandInterface.RESET}").strip()

    @staticmethod
    def print_command_help():
        print(f"{CommandInterface.OKGREEN}Comandos disponibles:{CommandInterface.RESET}")
        print(f"    m <distancia_cm>    : Mueve el robot hacia adelante (positivo) o hacia atrás (negativo) la distancia en cm.")
        print(f"    g <ángulo_grados>   : Gira el robot a la izquierda (positivo) o a la derecha (negativo) en grados.")
        print(f"    q                   : Sale del programa.")
        
        print(f"{CommandInterface.WARNING}Ejemplos:{CommandInterface.RESET}")
        print(f"    m 50      -> Mueve el robot 50 cm hacia adelante.")
        print(f"    m -30     -> Mueve el robot 30 cm hacia atrás.")
        print(f"    g 90      -> Gira el robot 90 grados hacia la izquierda.")
        print(f"    g -45     -> Gira el robot 45 grados hacia la derecha.")
        print(f"    q         -> Termina la ejecución.")

    @staticmethod
    def print_auto_command_help():
        print(f"{CommandInterface.OKGREEN}Comandos disponibles:{CommandInterface.RESET}")
        print(f"        <x> <y>            : Establece la coordenada objetivo (X, Y) para que el robot se mueva a esa posición.")
        print(f"        q                  : Sale del programa.")
        
        print(f"{CommandInterface.WARNING}Ejemplos:{CommandInterface.RESET}")
        print(f"        4 -1               -> Mueve el robot a la coordenada (4, -1).")
        print(f"        0 0                -> Mueve el robot de vuelta al origen (0, 0).")
        print(f"        q                  -> Termina la ejecución.")


if __name__ == "__main__":
    controller = RobotController(DEFAULT_PORT, DEFAULT_BAUD['default'])
    controller.run()