import numpy as np
import matplotlib.pyplot as plt


def homogeneousTransformationMatrix(x, y, theta):
    """
    Retorna a matriz de transformação homogênea 4x4 (plana SE(2) estendida para 3D)
    para uma translação (x, y) e rotação theta (em radianos) em torno do eixo Z.
    """
    return np.array([
        [np.cos(theta), -np.sin(theta), 0.0, float(x)],
        [np.sin(theta),  np.cos(theta), 0.0, float(y)],
        [0.0,            0.0,           1.0, 0.0     ],
        [0.0,            0.0,           0.0, 1.0     ]
    ])


def inverseHomogeneousMatrix(T):
    """
    Calcula a inversa de uma matriz de transformação homogênea rígida.
    Para T = [R | p], T^{-1} = [R^T | -R^T * p].
    """
    R = T[0:3, 0:3]
    p = T[0:3, 3]
    
    T_inv = np.eye(4)
    T_inv[0:3, 0:3] = R.T
    T_inv[0:3, 3] = -R.T @ p
    return T_inv


def transformToRobotFrame(T_world_robot, T_world_object):
    """
    Calcula a matriz de transformação homogênea do objeto no referencial local do robô:
    ^R T_obj = (^W T_R)^{-1} * ^W T_obj
    """
    T_robot_world = inverseHomogeneousMatrix(T_world_robot)
    return T_robot_world @ T_world_object


def plot_frame(origin, orientation_rad, ax=None, axis_size=0.4, label=None, 
               colors=('red', 'green'), text_offset=(0.0, 0.15)):
    """
    Plota um referencial 2D centrado em `origin` com orientação `orientation_rad`.
    Eixo X: Vermelho, Eixo Y: Verde.
    """
    if ax is None:
        ax = plt.gca()
    
    x0, y0 = origin[0], origin[1]
    
    # Vetores diretores dos eixos X e Y
    dx_x = axis_size * np.cos(orientation_rad)
    dx_y = axis_size * np.sin(orientation_rad)
    dy_x = -axis_size * np.sin(orientation_rad)
    dy_y = axis_size * np.cos(orientation_rad)
    
    # Plot dos eixos como setas (quiver)
    ax.quiver(x0, y0, dx_x, dx_y, angles='xy', scale_units='xy', scale=1, 
              color=colors[0], width=0.005, headwidth=4, headlength=5, zorder=5)
    ax.quiver(x0, y0, dy_x, dy_y, angles='xy', scale_units='xy', scale=1, 
              color=colors[1], width=0.005, headwidth=4, headlength=5, zorder=5)
    
    if label:
        ax.text(x0 + text_offset[0], y0 + text_offset[1], label, 
                fontsize=9, fontweight='bold', ha='center', va='bottom', zorder=6,
                bbox=dict(boxstyle='round,pad=0.25', facecolor='#FFFDD0', edgecolor='#888888', alpha=0.9))