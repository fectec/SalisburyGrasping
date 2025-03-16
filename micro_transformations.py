import numpy as np
from scipy.linalg import logm, expm

def interpolate_transformations(T0, Tf, num_steps=10):
    """
    Interpola una serie de transformaciones homogéneas absolutas entre T0 y Tf.

    Parámetros:
      T0: np.array de forma (4,4), transformación homogénea inicial.
      Tf: np.array de forma (4,4), transformación homogénea final.
      num_steps: Número de pasos intermedios (por defecto 10).

    Retorna:
      Una lista de matrices (4x4) que representan la trayectoria desde T0 hasta Tf.
    """
    # Calcula la transformación relativa de T0 a Tf.
    T_relative = np.linalg.inv(T0) @ Tf
    # Calcula el logaritmo matricial de la transformación relativa.
    log_T = logm(T_relative)
    
    transforms = []
    for i in range(num_steps + 1):
        t = i / num_steps  # parámetro de interpolación entre 0 y 1
        # Calcula la transformación incremental usando la exponencial.
        T_inc = expm(t * log_T)
        # La transformación actual es T0 multiplicado por el incremento.
        T_current = T0 @ T_inc
        transforms.append(T_current)
    return transforms

def incremental_transformations(T0, Tf, num_steps=10):
    """
    Genera la secuencia de transformaciones incrementales (deltas) entre T0 y Tf.
    Cada delta es la transformación que lleva la pose actual a la siguiente.

    Parámetros:
      T0: np.array de forma (4,4), transformación homogénea inicial.
      Tf: np.array de forma (4,4), transformación homogénea final.
      num_steps: Número de pasos intermedios (por defecto 10).

    Retorna:
      Una lista de matrices (4x4) representando las transformaciones incrementales.
    """
    # Primero generamos la trayectoria de poses absolutas.
    abs_poses = interpolate_transformations(T0, Tf, num_steps)
    
    increments = []
    # Para cada par consecutivo, calculamos la transformación incremental.
    for i in range(len(abs_poses) - 1):
        # Delta que lleva de la pose i a la pose i+1.
        delta = np.linalg.inv(abs_poses[i]) @ abs_poses[i+1]
        increments.append(delta)
    return increments

# Ejemplo de uso:
if __name__ == "__main__":
    # Definición de la transformación inicial (identidad) y final (rotación y traslación)
    T0 = np.eye(4)
    
    # Definición de una transformación final: rotación de 45° en Z y traslación en X y Y.
    angle = np.deg2rad(45)
    Rz = np.array([[np.cos(angle), -np.sin(angle), 0],
                   [np.sin(angle),  np.cos(angle), 0],
                   [0,              0,             1]])
    Tf = np.eye(4)
    Tf[:3, :3] = Rz
    Tf[:3, 3] = np.array([1.0, 1.0, 0.0])
    
    # Genera 20 transformaciones incrementales
    increments = incremental_transformations(T0, Tf, num_steps=20)

    # # Muestra cada transformación incremental
    # for i, delta in enumerate(increments):
    #     print(f"Incremento {i} (de T_{i} a T_{i+1}):\n{delta}\n")

    product = np.eye(4)
    for i, delta in enumerate(increments):
        product = product @ delta
    
    # Muestra la transformación final
    print(f"Transformación final:\n{product}\n")
    print(f"Transformación final (Tf):\n{Tf}\n")

