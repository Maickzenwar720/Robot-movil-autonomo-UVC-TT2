#! /usr/bin/env python3

import tkinter as tk
import rclpy
from rclpy.node import Node
import subprocess
from time import sleep
from signal import SIGINT

comando_uvcON = "source /opt/ros/humble/setup.bash && source /home/robot/diff_ws/install/setup.bash && ros2 run uvc_controller uvc_node"
comando_uvcOFF = "pkill -2 uvc_node"
comando_int_robot = "source /opt/ros/humble/setup.bash && source /home/robot/diff_ws/install/setup.bash && ros2 launch diff_pkg launch_robot.launch.py"
comando_rviz = ["ros2","launch","diff_pkg","rviz.launch.py"]
comando_mask = ["ros2","launch","diff_pkg","costmap_filter_info.launch.py"]
comando_nav = ["ros2","launch","diff_pkg","bringup.launch.py"]
comando_rutina = ["ros2","run","follow_waypoints","follow_waypoints_exe"]
comando_paro_rutina = ["pkill","-2","follow_waypoint"]
comando_paro_robot = "pkill -2 ros2_control_no && pkill -2 robot_state_pub && pkill -2 rplidar_composi && pkill -2 twist_mux"
comando_apagar_rasp = "sudo shutdown -h now"
lista = ["controller_manager","diff_cont","joint_broad","robot_state_publisher","rplidar_composition","twist_mux"]

class UiNode(Node):
    
    def __init__(self):
        super().__init__('ui_node')
        self.init_ui()


    def init_ui(self):
        self.get_logger().info("Iniciando nodo...")
        self.ventana = tk.Tk()
        self.ventana.geometry('400x300')
        self.ventana.title('Interfaz ROS 2')

        # Crear una etiqueta para mostrar mensajes
        self.mensaje_label = tk.Text(self.ventana, height=2, width=25)
        self.mensaje_label.configure(state='normal')
        self.mensaje_label.place(x=100, y=50)

        self.boton_iniciar_robot = tk.Button(self.ventana, text='Iniciar Robot', command=self.iniciar_robot)
        self.boton_iniciar_robot.place(x=50, y=150)

        self.boton_iniciar_sanitizacion = tk.Button(self.ventana, text="Encender luz UVC", command=self.iniciar_sanitizacion)
        self.boton_iniciar_sanitizacion.place(x=200, y=150)

        self.boton_paro_sanitizacion = tk.Button(self.ventana, text="Apagar luz UVC", command=self.paro_sanitizacion)

        self.boton_iniciar_rutina = tk.Button(self.ventana, text="Iniciar Rutina", command=self.iniciar_rutina)        

        self.boton_paro_rutina = tk.Button(self.ventana, text="Paro Rutina", command=self.paro_rutina)  

        self.boton_exit = tk.Button(self.ventana, text="Salir", command=self.salir)
        self.boton_exit.place(x=285, y=220)

        self.ventana.mainloop()


    def iniciar_robot(self):
        self.mensaje_label.configure(state='normal')
        self.mensaje_label.delete("1.0", "end")
        self.mensaje_label.insert(tk.END,'Encendiendo robot...')
        self.mensaje_label.configure(state='disabled')
        
        self.proceso_robot = subprocess.Popen(["ssh", "robot@192.168.43.230", comando_int_robot], stdin=subprocess.PIPE)
        sleep(20.0)
        # names = self.get_node_names()
        # while not all(nodos in names for nodos in lista):
        #     self.proceso_robot = subprocess.Popen(["ssh", "robot@192.168.43.230", comando_paro_robot], stdin=subprocess.PIPE)
        #     sleep(10.0)
        #     self.proceso_robot.wait()
        #     self.proceso_robot.terminate()
        #     self.proceso_robot = subprocess.Popen(["ssh", "robot@192.168.43.230", comando_int_robot], stdin=subprocess.PIPE)
        #     sleep(20.0)
        # self.proceso_rviz = subprocess.Popen(comando_rviz)
        self.proceso_nav = subprocess.Popen(comando_nav)
        sleep(10.0)
        self.proceso_mask = subprocess.Popen(comando_mask)
        sleep(10.0)
        self.boton_iniciar_rutina.place(x=48, y=220)
        self.boton_iniciar_robot.place_forget()     

    def iniciar_sanitizacion(self):
        self.proceso_uvc = subprocess.Popen(["ssh", "robot@192.168.43.230", comando_uvcON], stdin=subprocess.PIPE)
        sleep(2.0)
        self.mensaje_label.configure(state='normal')
        self.mensaje_label.delete("1.0", "end")
        self.mensaje_label.insert(tk.END, 'Encendiendo luz UVC...')
        self.mensaje_label.configure(state='disabled')
        self.boton_iniciar_sanitizacion.place_forget()
        self.boton_paro_sanitizacion.place(x=208, y=150)
        self.boton_iniciar_sanitizacion.config(state=tk.DISABLED)
        self.boton_paro_sanitizacion.config(state=tk.ACTIVE) 

    def paro_sanitizacion(self):
        self.proceso_uvc = subprocess.Popen(["ssh", "robot@192.168.43.230", comando_uvcOFF], stdin=subprocess.PIPE)
        sleep(2.0)
        self.proceso_uvc.terminate()
        self.mensaje_label.configure(state='normal')
        self.mensaje_label.delete("1.0", "end")
        self.mensaje_label.insert(tk.END, 'Apagando luz UVC...')
        self.mensaje_label.configure(state='disabled')
        self.boton_paro_sanitizacion.place_forget()
        self.boton_iniciar_sanitizacion.place(x=200, y=150)
        self.boton_iniciar_sanitizacion.config(state=tk.ACTIVE)
        self.boton_paro_sanitizacion.config(state=tk.DISABLED)

    def iniciar_rutina(self):
        self.proceso_rutina = subprocess.Popen(comando_rutina)
        self.mensaje_label.configure(state='normal')
        self.mensaje_label.delete("1.0", "end")
        self.mensaje_label.insert(tk.END, 'Iniciando rutina...')
        self.mensaje_label.configure(state='disabled')
        self.boton_iniciar_rutina.place_forget()
        self.boton_paro_rutina.place(x=54, y=220)   
        # self.proceso_rutina.wait()

    def paro_rutina(self):
        self.proceso_rutina = subprocess.Popen(comando_paro_rutina)
        self.proceso_uvc = subprocess.Popen(["ssh", "robot@192.168.43.230", comando_uvcOFF], stdin=subprocess.PIPE)
        sleep(2.0)
        self.proceso_uvc.terminate()
        self.mensaje_label.configure(state='normal')
        self.mensaje_label.delete("1.0", "end")
        self.mensaje_label.insert(tk.END, 'Deteniendo rutina...')
        self.mensaje_label.configure(state='disabled')
        self.boton_paro_rutina.place_forget()
        self.boton_iniciar_rutina.place(x=48, y=220)
        self.boton_paro_sanitizacion.place_forget()
        self.boton_iniciar_sanitizacion.place(x=200, y=150)
        self.boton_iniciar_sanitizacion.config(state=tk.ACTIVE)
        self.boton_paro_sanitizacion.config(state=tk.DISABLED)
        

    def salir(self):
        
        if(hasattr(self,'proceso_uvc')):
            self.proceso_uvc = subprocess.Popen(["ssh", "robot@192.168.43.230", comando_uvcOFF], stdin=subprocess.PIPE)
            sleep(2.0)
            self.proceso_uvc.terminate()
        if(hasattr(self,'proceso_rutina')):
            self.proceso_rutina = subprocess.Popen(comando_paro_rutina)
            self.proceso_rutina.wait()
            self.proceso_rutina.terminate()
        if (hasattr(self, 'proceso_rviz')):
            self.proceso_rviz.send_signal(SIGINT)
            self.proceso_rviz.wait()
            self.proceso_rviz.terminate()
        if (hasattr(self, 'proceso_mask')):
            self.proceso_mask.send_signal(SIGINT)
            self.proceso_mask.wait()
            self.proceso_mask.terminate()
        if (hasattr(self,'proceso_nav')):
            self.proceso_nav.send_signal(SIGINT)
            self.proceso_nav.wait()
            self.proceso_nav.terminate()
        if(hasattr(self,'proceso_robot')):
            self.proceso_robot = subprocess.Popen(["ssh", "robot@192.168.43.230", comando_paro_robot], stdin=subprocess.PIPE)
            self.proceso_robot.wait()
            self.proceso_robot.terminate()
            sleep(2.0)
        # self.proceso_apagado = subprocess.Popen(["ssh", "robot@192.168.43.230", comando_apagar_rasp], stdin=subprocess.PIPE)
        # self.proceso_apagado.wait()
        rclpy.shutdown()
        exit(0)


def main(args=None):
    rclpy.init(args=args)
    ui_node = UiNode()
    while rclpy.ok():
        rclpy.spin_once(ui_node)
    
    ui_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


#pkill -2 ros2_control_no
#pkill -2 robot_state_pub
#pkill -2 rplidar_composi
#pkill -2 twist_mux