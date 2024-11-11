#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from sympy import *
#from sympy import solve, Symbol, Matrix, cos, sin, simplify, pi, diff, symbols

import matplotlib
import matplotlib.pyplot as plt
import numpy

class TrayectoryGenerator():
  def __init__(self, dim = (0.3, 0.3, 0.3), base_dim = (0, 0, 0.1), base_rot = (pi/2, 0, 0)):
    #Parámetros de dimensiones
    self.dim = dim
    self.base_dim = base_dim
    self.base_rot = base_rot
    #Variables para cinemática directa
    self.theta_O_1 = Symbol('theta_O_1')
    self.theta_1_2 = Symbol('theta_1_2')
    self.theta_2_3 = Symbol('theta_2_3')
    #Grados de libertad
    self.x_O_P = Symbol('x_O_P')
    self.z_O_P = Symbol('y_O_P')
    self.theta_O_P = Symbol('theta_O_P')
    #Velocidades
    self.x_O_P_dot = Symbol('x_O_P_dot')
    self.z_O_P_dot = Symbol('y_O_P_dot')
    self.theta_O_P_dot = Symbol('theta_O_P_dot')
    #Transformaciones homogéneas
    T_O_O = self.trans_homo(self.base_dim[0], self.base_dim[1], self.base_dim[2], 
                            self.base_rot[0], self.base_rot[1], self.base_rot[2])
    
    T_O_1 = self.trans_homo(0, 0, 0, 
                            0, 0, self.theta_O_1)
    T_1_2 = self.trans_homo(self.dim[0], 0, 0, 
                            0, 0, self.theta_1_2)
    T_2_3 = self.trans_homo(self.dim[1], 0, 0, 
                            0, 0, self.theta_2_3)
    T_3_P = self.trans_homo(self.dim[2], 0, 0, 
                            0, 0, 0)
    T_O_P = simplify(T_O_O*T_O_1 * T_1_2 * T_2_3 * T_3_P)
    print("GDL del robot")
    print(T_O_P[0,3])
    print(T_O_P[1,3])
    print(T_O_P[2,3])
    self.xi_O_P = Matrix([T_O_P[0, 3], T_O_P[2, 3], 
                          self.theta_O_1 + self.theta_1_2 + self.theta_2_3])
    self.J = Matrix.hstack(diff(self.xi_O_P, self.theta_O_1), 
                           diff(self.xi_O_P, self.theta_1_2), 
                           diff(self.xi_O_P, self.theta_2_3))
    self.J_inv = self.J.inv()
    #print(self.J)
  def trans_homo(self, x, y, z, gamma, beta, alpha):
    T = Matrix([[cos(alpha)*cos(beta), -sin(alpha)*cos(gamma)+sin(beta)*sin(gamma)*cos(alpha), sin(alpha)*sin(gamma)+sin(beta)*cos(alpha)*cos(gamma), x],
         [sin(alpha)*cos(beta), sin(alpha)*sin(beta)*sin(gamma)+cos(alpha)*cos(gamma), sin(alpha)*sin(beta)*cos(gamma)-sin(gamma)*cos(alpha), y],
        [-sin(beta), sin(gamma)*cos(beta), cos(beta)*cos(gamma), z],[0, 0, 0, 1]])
    return T 
  def polinomial_trayectory_generator(self, frec = 20, time = (0, 3), gdl_in = (0.8, 0.1, 0), gdl_fn = (0.4, 0.4, 0)):
    self.frec = frec
    a_0, a_1, a_2, a_3, a_4, a_5, t = symbols('a_0 a_1 a_2 a_3 a_4 a_5 t')
    lam = a_0 + a_1*t + a_2*(t**2) + a_3*(t**3) + a_4*(t**4) + a_5*(t**5)
    lam_dot = diff(lam, t)
    lam_dot_dot = diff(lam_dot, t)
    terms = solve([
      lam.subs(t, time[0]) - 0,
      lam.subs(t, time[1]) - 1,
      lam_dot.subs(t, time[0]) - 0,
      lam_dot.subs(t, time[1]) - 0,
      lam_dot_dot.subs(t, time[0]) - 0,
      lam_dot_dot.subs(t, time[1]) - 0
    ], [a_0, a_1, a_2, a_3, a_4, a_5], dict = True)
    print(terms)
    #Lambda con valores sustituidos
    lam_s = lam.subs(terms[0])
    lam_s_dot = lam_dot.subs(terms[0])
    lam_s_dot_dot = lam_dot_dot.subs(terms[0])
    samples = frec * (time[1] - time[0]) + 1
    dt = 1.0 / frec

    #Creando arreglos para guardar las posiciones de los GDL
    gdl = []
    gdl_dot = []
    gdl_dot_dot = []
    for a in range(samples):
      gdl.append((gdl_in[0] + lam_s.subs(t, time[0] + float(a)/frec) * (gdl_fn[0] - gdl_in[0]),
                  gdl_in[1] + lam_s.subs(t, time[0] + float(a)/frec) * (gdl_fn[1] - gdl_in[1]), 
                  gdl_in[2] + lam_s.subs(t, time[0] + float(a)/frec) * (gdl_fn[2] - gdl_in[2]) ))
      gdl_dot.append((lam_s_dot.subs(t, time[0] + float(a)/frec) * (gdl_fn[0] - gdl_in[0]),
                      lam_s_dot.subs(t, time[0] + float(a)/frec) * (gdl_fn[1] - gdl_in[1]), 
                      lam_s_dot.subs(t, time[0] + float(a)/frec) * (gdl_fn[2] - gdl_in[2]) ))
      gdl_dot_dot.append((lam_s_dot_dot.subs(t, time[0] + float(a)/frec) * (gdl_fn[0] - gdl_in[0]),
                      lam_s_dot_dot.subs(t, time[0] + float(a)/frec) * (gdl_fn[1] - gdl_in[1]), 
                      lam_s_dot_dot.subs(t, time[0] + float(a)/frec) * (gdl_fn[2] - gdl_in[2]) ))
    #Obteniendo posición inicial del ws
    self.xi_desp = self.xi_O_P - Matrix([gdl_in[0], 
                                        gdl_in[1], 
                                        gdl_in[2]])
    """print("Calculando posiciones iniciales")
    q_in = solve([self.xi_desp],
                  [self.theta_O_1, 
                   self.theta_1_2, 
                   self.theta_2_3])
    print("Posiciones iniciales:")
    print(q_in)
    q_in_def = 0
    for i in q_in:
      if i[0]>0:
        q_in_def = i
        break
    """
    q_in_def = (0.585685543457151, 
                -1.17137108691430, 
                0.585685543457151)
    #Calculando velocidad inicial
    self.xi_O_P_dot = Matrix([self.x_O_P_dot,
                              self.z_O_P_dot,
                              self.theta_O_P_dot])
    self.q_O_P_dot = self.J_inv * self.xi_O_P_dot
    #Arreglos para guardar las posiciones del ws
    ws = []
    ws_dot = []
    ws_dot_dot = []
    #Agregando posición inicial
    ws.append(q_in_def)
    #Agregando velocidad inicial
    q_dot_in = self.q_O_P_dot.subs({
      self.x_O_P_dot:     gdl_dot[0][0],
      self.z_O_P_dot:     gdl_dot[0][1],
      self.theta_O_P_dot: gdl_dot[0][2],
      self.theta_O_1: ws[0][0],
      self.theta_1_2: ws[0][1],
      self.theta_2_3: ws[0][2]})
    ws_dot.append(q_dot_in)
    #Calculando todos los valores por cinemática inversa
    for a in range(samples - 1):
      #Posición 
      ws.append((ws[a][0] + ws_dot[a][0] * dt,
                 ws[a][1] + ws_dot[a][1] * dt,
                 ws[a][2] + ws_dot[a][2] * dt))
      #Velocidad
      q_dot_iter = self.q_O_P_dot.subs({
                                      self.x_O_P_dot:     gdl_dot[a+1][0],
                                      self.z_O_P_dot:     gdl_dot[a+1][1],
                                      self.theta_O_P_dot: gdl_dot[a+1][2],
                                      self.theta_O_1: ws[a+1][0],
                                      self.theta_1_2: ws[a+1][1],
                                      self.theta_2_3: ws[a+1][2]})
      ws_dot.append(q_dot_iter)
      #Aceleración
      ws_dot_dot.append((
                    (ws_dot[a+1][0]-ws_dot[a][0]) / dt,
                    (ws_dot[a+1][1]-ws_dot[a][1]) / dt,
                    (ws_dot[a+1][2]-ws_dot[a][2]) / dt))
      print("Iteración: " + str(a))
    #Aceleración final
    ws_dot_dot.append((0,0,0))
    self.gdl = gdl
    self.gdl_dot = gdl_dot
    self.gdl_dot_dot = gdl_dot_dot
    self.ws = ws
    self.ws_dot = ws_dot
    self.ws_dot_dot = ws_dot_dot
  def trayectory_graphics(self):
    #Grados de libertad
    gdl_t = []
    gdl_p_0 = []
    gdl_p_1 = []
    gdl_p_2 = []
    gdl_v_0 = []
    gdl_v_1 = []
    gdl_v_2 = []
    gdl_a_0 = []
    gdl_a_1 = []
    gdl_a_2 = []
    #Espacio de trabajo
    ws_t = []
    ws_p_0 = []
    ws_p_1 = []
    ws_p_2 = []
    ws_v_0 = []
    ws_v_1 = []
    ws_v_2 = []
    ws_a_0 = []
    ws_a_1 = []
    ws_a_2 = []

    for i in range(len(self.gdl)):
      gdl_t.append(float(i) / self.frec)
      gdl_p_0.append(self.gdl[i][0])
      gdl_p_1.append(self.gdl[i][1])
      gdl_p_2.append(self.gdl[i][2])
      gdl_v_0.append(self.gdl_dot[i][0])
      gdl_v_1.append(self.gdl_dot[i][1])
      gdl_v_2.append(self.gdl_dot[i][2])
      gdl_a_0.append(self.gdl_dot_dot[i][0])
      gdl_a_1.append(self.gdl_dot_dot[i][1])
      gdl_a_2.append(self.gdl_dot_dot[i][2])
    for i in range(len(self.ws)):
      ws_t.append(float(i) / self.frec)
      ws_p_0.append(self.ws[i][0])
      ws_p_1.append(self.ws[i][1])
      ws_p_2.append(self.ws[i][2])
      ws_v_0.append(self.ws_dot[i][0])
      ws_v_1.append(self.ws_dot[i][1])
      ws_v_2.append(self.ws_dot[i][2])
      ws_a_0.append(self.ws_dot_dot[i][0])
      ws_a_1.append(self.ws_dot_dot[i][1])
      ws_a_2.append(self.ws_dot_dot[i][2])
    fig, ((gdl_plot, gdl_dot_plot, gdl_dot_dot_plot), 
          (ws_plot, ws_dot_plot, ws_dot_dot_plot),
          ) = plt.subplots(nrows = 2, ncols = 3)    
    #Gráficas GDL
    gdl_plot.set_title("Posición GDL")
    gdl_plot.axis((0, 3, -1.5, 1.5))
    gdl_plot.plot(gdl_t, gdl_p_0, color = "RED")
    gdl_plot.plot(gdl_t, gdl_p_1, color = "GREEN")
    gdl_plot.plot(gdl_t, gdl_p_2, color = "BLUE")
    gdl_dot_plot.set_title("Velocidad GDL") 
    gdl_dot_plot.axis((0, 3, -0.6, 0.6))
    gdl_dot_plot.plot(gdl_t, gdl_v_0, color = "RED")
    gdl_dot_plot.plot(gdl_t, gdl_v_1, color = "GREEN")
    gdl_dot_plot.plot(gdl_t, gdl_v_2, color = "BLUE")
    gdl_dot_dot_plot.set_title("Aceleración GDL")
    gdl_dot_dot_plot.axis((0, 3, -0.6, 0.6))
    gdl_dot_dot_plot.plot(gdl_t, gdl_a_0, color = "RED")
    gdl_dot_dot_plot.plot(gdl_t, gdl_a_1, color = "GREEN")
    gdl_dot_dot_plot.plot(gdl_t, gdl_a_2, color = "BLUE")
    #Gráficas WS
    ws_plot.set_title("Posición WS")
    ws_plot.axis((0, 3, -2.5, 2.5))
    ws_plot.plot(ws_t, ws_p_0, color = "RED")
    ws_plot.plot(ws_t, ws_p_1, color = "GREEN")
    ws_plot.plot(ws_t, ws_p_2, color = "BLUE")

    ws_dot_plot.set_title("Velocidad WS")
    ws_dot_plot.axis((0, 3, -1.5, 1.5))
    ws_dot_plot.plot(ws_t, ws_v_0, color = "RED")
    ws_dot_plot.plot(ws_t, ws_v_1, color = "GREEN")
    ws_dot_plot.plot(ws_t, ws_v_2, color = "BLUE")
    ws_dot_dot_plot.set_title("Aceleración WS")
    ws_dot_plot.axis((0, 3, -1.5, 1.5))
    ws_dot_dot_plot.plot(ws_t, ws_a_0, color = "RED")
    ws_dot_dot_plot.plot(ws_t, ws_a_1, color = "GREEN")
    ws_dot_dot_plot.plot(ws_t, ws_a_2, color = "BLUE")
    plt.show()

class TrayectoryPublisherNode(Node):
  def __init__(self):
    super().__init__("trayectory_publiser_node")
    self.get_logger().info("Nodo publicador de trayectoria")
    self.joint_pub = self.create_publisher(JointState, 
                                           "/joint_states",
                                           10)
  def publisher_init(self, ws, ws_dot, 
                     ws_dot_dot, frec):
    self.ws = ws
    self.ws_dot = ws_dot
    self.ws_dot_dot = ws_dot_dot
    self.joint_msg = JointState()
    self.joint_msg.name = ["shoulder_joint", 
                           "arm_joint",
                           "forearm_joint"]
    self.count = 0
    self.create_timer(1.0 / frec, 
                      self.send_joint_state)
  def send_joint_state(self):
    t_s = self.get_clock().now().seconds_nanoseconds()
    self.joint_msg.header.stamp.sec = t_s[0]
    self.joint_msg.header.stamp.nanosec = t_s[1]

    self.joint_msg.position = [float(self.ws[self.count][0]),
                               float(self.ws[self.count][1]),
                               float(self.ws[self.count][2])]
    self.joint_msg.velocity = [float(self.ws_dot[self.count][0]),
                               float(self.ws_dot[self.count][1]),
                               float(self.ws_dot[self.count][2])]
    self.joint_pub.publish(self.joint_msg)
    self.count += 1
    if self.count >= len(self.ws):
      self.count = 0

def main():
  trayectory_gen = TrayectoryGenerator()
  trayectory_gen.polinomial_trayectory_generator()
  #trayectory_gen.trayectory_graphics()
  rclpy.init()
  node = TrayectoryPublisherNode()
  node.publisher_init(trayectory_gen.ws,
                      trayectory_gen.ws_dot,
                      trayectory_gen.ws_dot_dot,
                      trayectory_gen.frec)
  rclpy.spin(node)
  rclpy.shutdown()

if __name__ == "__main__":
  main()