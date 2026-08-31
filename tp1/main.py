import argparse
import time
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

import utils


def get_global_pose(sim, handle):
    """
    Recupera a pose global 2D (x, y, theta) de um objeto na cena via ZeroMQ API.
    """
    pos = sim.getObjectPosition(handle, sim.handle_world)
    ori = sim.getObjectOrientation(handle, sim.handle_world)
    return pos[0], pos[1], ori[2]


class HokuyoSensorSim:
    """Leitor do sensor fastHokuyo usado pelo modelo PioneerP3DX."""

    _angle_min = -120 * np.pi / 180
    _angle_increment = (240 / 684) * np.pi / 180

    def __init__(self, sim, base_path):
        self.sim = sim
        self.base = sim.getObject(base_path)
        self.sensors = [
            sim.getObject(f"{base_path}/sensor1"),
            sim.getObject(f"{base_path}/sensor2"),
        ]

    def get_points(self):
        """Retorna pontos [x, y, z] no referencial local do laser."""
        points = []
        angle = self._angle_min

        for sensor in self.sensors:
            _, _, packed = self.sim.readVisionSensor(sensor)
            if not packed:
                continue

            # Cada vision sensor devolve os pontos no proprio referencial.
            # O exemplo do Hokuyo primeiro os leva para o referencial do laser.
            T_laser_sensor = matriz_sim_para_numpy(
                self.sim.getObjectMatrix(sensor, self.base)
            )
            width, height = int(packed[0]), int(packed[1])
            for j in range(height):
                for k in range(width):
                    index = 2 + 4 * (j * width + k)
                    point = packed[index:index + 3]
                    angle += self._angle_increment
                    if len(point) == 3:
                        points.append(transform_points([point], T_laser_sensor)[0])

        return np.asarray(points, dtype=float).reshape((-1, 3))


def transform_points(points, transform):
    """Aplica uma transformacao homogenea 4x4 a pontos 3D."""
    points = np.asarray(points, dtype=float).reshape((-1, 3))
    if points.size == 0:
        return np.empty((0, 3))

    points_h = np.column_stack((points, np.ones(len(points))))
    return (np.asarray(transform, dtype=float) @ points_h.T).T[:, :3]


def transform_laser_points_to_world(points_laser, T_R_L, T_W_R):
    """Aplica a cadeia laser -> robô -> mundo aos pontos do laser."""
    points_laser = np.asarray(points_laser, dtype=float)
    if points_laser.size == 0:
        return np.empty((0, 2))

    points_h = np.column_stack((points_laser[:, :3], np.ones(len(points_laser))))
    points_world = (T_W_R @ T_R_L @ points_h.T).T
    return points_world[:, :2]


def calculate_global_plot_limits(points, margin=0.6):
    """Calcula os limites 2D mantendo a extensao completa do mapa global."""
    points = np.asarray(points, dtype=float).reshape((-1, 2))
    if points.size == 0:
        points = np.zeros((1, 2), dtype=float)

    minimum = points.min(axis=0) - margin
    maximum = points.max(axis=0) + margin
    return (float(minimum[0]), float(maximum[0]),
            float(minimum[1]), float(maximum[1]))


def salvar_figura(output):
    """Salva o PNG em ``tp1/images`` usando um arquivo já aberto."""
    output_path = Path(output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with output_path.open("w+b") as output_file:
        plt.savefig(output_file, format="png", dpi=200, bbox_inches="tight")

    if output_path.stat().st_size == 0:
        raise RuntimeError(f"Matplotlib criou uma imagem vazia: {output_path}")

    return output_path


def run_question_5(sim, output):
    """Plota a leitura do laser no referencial global e salva a imagem."""
    robot = sim.getObject("/PioneerP3DX")
    laser = sim.getObject("/PioneerP3DX/fastHokuyo")

    sensor = HokuyoSensorSim(sim, "/PioneerP3DX/fastHokuyo")
    sim.startSimulation()
    sim.step()

    try:
        # A pose é lida depois do primeiro passo para que as matrizes e os
        # pontos correspondam ao mesmo estado atual da simulação.
        # getObjectMatrix(objeto, referencial) retorna ^referencial T_objeto.
        # Portanto: ^R T_L é laser -> robô e ^W T_R é robô -> mundo.
        T_R_L_raw = sim.getObjectMatrix(laser, robot)
        T_W_R_raw = sim.getObjectMatrix(robot, sim.handle_world)
        T_R_L = matriz_sim_para_numpy(T_R_L_raw)
        T_W_R = matriz_sim_para_numpy(T_W_R_raw)

        points_laser = sensor.get_points()
        max_sensor_range = 5.0
        if len(points_laser):
            detected = np.linalg.norm(points_laser, axis=1) < max_sensor_range - 0.1
            points_laser = points_laser[detected]
        points_world = transform_laser_points_to_world(
            points_laser, T_R_L, T_W_R
        )

        robot_position = sim.getObjectPosition(robot, sim.handle_world)
        robot_orientation = sim.getObjectOrientation(robot, sim.handle_world)
        laser_position = sim.getObjectPosition(laser, sim.handle_world)

        scene_specs = [
            ("Bill", ["/Bill"]),
            ("Cadeira", ["/diningChair"]),
            ("Mesa", ["/customizableTable"]),
            ("Planta 1", ["/indoorPlant", "/indoorPlant[0]"]),
            ("Planta 2", ["/indoorPlant[1]"]),
            ("Quadcopter 1", ["/Quadcopter[0]", "/Quadcopter"]),
            ("Quadcopter 2", ["/Quadcopter[1]"]),
            ("Robo NAO", ["/NAO"]),
        ]
        scene_objects = []
        for name, paths in scene_specs:
            for path in paths:
                try:
                    handle = sim.getObject(path)
                    position = sim.getObjectPosition(handle, sim.handle_world)
                    scene_objects.append((name, position))
                    break
                except Exception:
                    continue

        print("\nQuestão 5 - matrizes de transformação")
        print("^R T_L (laser -> robô):\n", np.round(T_R_L, 4))
        print("^W T_R (robô -> mundo):\n", np.round(T_W_R, 4))
        print(f"Pose global do robô: {robot_position[:2]}, "
              f"theta={robot_orientation[2]:+.4f} rad")
        print(f"Pontos válidos: {len(points_world)}")

        fig, ax = plt.subplots(figsize=(7, 7), dpi=120)
        scene_xy = [[0.0, 0.0], robot_position[:2], laser_position[:2]]
        scene_xy.extend(position[:2] for _, position in scene_objects)
        scene_xy.extend(points_world.tolist())
        x_min, x_max, y_min, y_max = calculate_global_plot_limits(scene_xy)

        if len(points_world):
            ax.scatter(points_world[:, 0], points_world[:, 1], s=5,
                       color="darkorange", label="Leitura do laser")
        ax.scatter(*robot_position[:2], marker="s", s=100, color="blue",
                   label="Robô")
        ax.scatter(*laser_position[:2], marker="^", s=70, color="black",
                   label="Laser")
        utils.plot_frame(robot_position, robot_orientation[2], ax=ax,
                         axis_size=0.45, label="{R}", fontsize=9)
        ax.set_title("Questão 5 — leitura do laser no mapa global")
        ax.set_xlabel("X mundo [m]")
        ax.set_ylabel("Y mundo [m]")
        ax.grid(True, linestyle=":")
        ax.set_aspect("equal")
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        ax.legend()
        # O arquivo aberto explicitamente evita a falha do Pillow no Windows.
        output_path = normalizar_caminho_saida(output)
        salvar_figura(output_path)
        print(f"[INFO] Gravando gráfico em: {output_path}")
        plt.close(fig)
        print(f"[SUCESSO] Gráfico da questão 5 salvo em '{output_path}'")
    finally:
        sim.stopSimulation()


def matriz_sim_para_numpy(matrix):
    """Converte a matriz 3x4 da RemoteAPI em uma matriz homogênea 4x4."""
    matrix = np.asarray(matrix, dtype=float).reshape(3, 4)
    result = np.eye(4)
    result[:3, :4] = matrix
    return result


def normalizar_caminho_saida(output):
    """Retorna somente um arquivo dentro da pasta ``tp1/images``."""
    output = str(output)
    while r"\_" in output:
        output = output.replace(r"\_", "_")

    filename = Path(output).name
    if not filename or filename in {".", ".."}:
        raise ValueError("O arquivo de saída precisa ter um nome válido")

    images_dir = Path(__file__).resolve().parent / "images"
    return (images_dir / filename).resolve()


def main():
    parser = argparse.ArgumentParser(description="Transformações Homogêneas - TP1 Robótica Móvel")
    parser.add_argument("--output", "-o", default="images/questao3.png", help="Caminho do arquivo de saída da imagem")
    parser.add_argument("--question", choices=("3", "5"), default="3",
                        help="Questão a executar (padrão: 3)")
    args = parser.parse_args()

    print(f"[INFO] Script em execução: {Path(__file__).resolve()}")

    output_path = normalizar_caminho_saida(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    # Conexão com o CoppeliaSim via ZeroMQ Remote API
    client = RemoteAPIClient()
    sim = client.require('sim')

    initial_sim_state = sim.getSimulationState()
    if initial_sim_state != 0:
        sim.stopSimulation()
        time.sleep(1)

    if args.question == "5":
        run_question_5(sim, output_path)
        return

    # Lista de elementos a serem mapeados na cena
    object_specs = [
        ("Pioneer 3DX (Robô)", ["/PioneerP3DX"]),
        ("Bill", ["/Bill"]),
        ("Cadeira", ["/diningChair"]),
        ("Mesa", ["/customizableTable"]),
        ("Planta 1", ["/indoorPlant", "/indoorPlant[0]"]),
        ("Planta 2", ["/indoorPlant[1]"]),
        ("Quadcóptero 1", ["/Quadcopter[0]", "/Quadcopter"]),
        ("Quadcóptero 2", ["/Quadcopter[1]"]),
        ("Robô NAO", ["/NAO"])
    ]

    handles = {}
    for name, paths in object_specs:
        for path in paths:
            try:
                h = sim.getObject(path)
                handles[name] = h
                break
            except Exception:
                continue

    if "Pioneer 3DX (Robô)" not in handles:
        raise RuntimeError("Robô PioneerP3DX não foi encontrado na cena!")

    pioneer_handle = handles["Pioneer 3DX (Robô)"]

    sim.startSimulation()
    time.sleep(0.5)

    # 1. Configuração global e matrizes do robô
    xr, yr, thetar = get_global_pose(sim, pioneer_handle)
    T_world_robot = utils.homogeneousTransformationMatrix(xr, yr, thetar)
    T_robot_world = utils.inverseHomogeneousMatrix(T_world_robot)

    print(f"\nConfiguração Global do Robô q = [x, y, theta]:")
    print(f"  x     = {xr:+.4f} m")
    print(f"  y     = {yr:+.4f} m")
    print(f"  theta = {thetar:+.4f} rad ({np.rad2deg(thetar):+.2f}°)")
    print(f"\nMatriz de Transformação Homogênea Global do Robô (^W T_R):")
    print(np.round(T_world_robot, 4))
    print(f"\nMatriz Inversa (^R T_W = (^W T_R)^-1):")
    print(np.round(T_robot_world, 4))
    print("\n" + "-" * 80)
    print(f"{'Elemento':<22} | {'Posição Global (x, y, theta)':<30} | {'Posição Local no Robô (^R x, ^R y, ^R theta)'}")
    print("-" * 80)

    # 2. Matrizes homogêneas dos outros elementos no referencial do robô
    elements_data = []
    for name, handle in handles.items():
        if name == "Pioneer 3DX (Robô)":
            continue

        xi, yi, thetai = get_global_pose(sim, handle)
        T_world_elem = utils.homogeneousTransformationMatrix(xi, yi, thetai)
        T_robot_elem = utils.transformToRobotFrame(T_world_robot, T_world_elem)

        x_local = T_robot_elem[0, 3]
        y_local = T_robot_elem[1, 3]
        theta_local = np.arctan2(T_robot_elem[1, 0], T_robot_elem[0, 0])

        elements_data.append({
            "name": name,
            "global_pos": (xi, yi, thetai),
            "local_pos": (x_local, y_local, theta_local)
        })

        global_str = f"({xi:+.2f}, {yi:+.2f}, {np.rad2deg(thetai):+.1f}°)"
        local_str = f"({x_local:+.2f}, {y_local:+.2f}, {np.rad2deg(theta_local):+.1f}°)"
        print(f"{name:<22} | {global_str:<30} | {local_str}")

    print("=" * 80)

    # 3. Plotagem dos referenciais e vetores de relacionamento
    fig, (ax_global, ax_local) = plt.subplots(1, 2, figsize=(17, 8))

    # Limites simétricos centrados na origem (evita overflow e mantém o robô centralizado em {R})
    global_points = [(0, 0), (xr, yr)] + [item["global_pos"][:2] for item in elements_data]
    local_points = [(0, 0)] + [item["local_pos"][:2] for item in elements_data]

    L_global = max(3.0, float(np.max(np.abs(global_points))) + 0.6)
    L_local = max(3.5, float(np.max(np.abs(local_points))) + 0.8)

    # --- (A) Referencial Global {W} ---
    utils.plot_frame([0, 0], 0, ax=ax_global, axis_size=0.5, label="{W}\n(Mundo)", colors=('red', 'green'), text_offset=(0.0, 0.28))
    utils.plot_frame([xr, yr], thetar, ax=ax_global, axis_size=0.45, label="{R}\n(Robô)", colors=('red', 'green'), text_offset=(0.0, 0.28))
    ax_global.scatter([xr], [yr], color='blue', s=60, zorder=5)

    for item in elements_data:
        xi, yi, thetai = item["global_pos"]
        name = item["name"]
        offset = utils.get_label_offset(xi, yi, x_ref=xr, y_ref=yr, name=name)
        utils.plot_frame([xi, yi], thetai, ax=ax_global, axis_size=0.35, label=f"{{{name}}}", colors=('red', 'green'), text_offset=offset)
        ax_global.annotate('', xy=(xi, yi), xytext=(xr, yr), arrowprops=dict(arrowstyle='->', color='#7B1FA2', lw=1.5, ls='--'))

    ax_global.set_title("Referencial Global {W}", fontsize=13, fontweight='bold', pad=10)
    ax_global.set_xlabel("X_mundo [m]", fontsize=11)
    ax_global.set_ylabel("Y_mundo [m]", fontsize=11)
    ax_global.grid(True, linestyle=':', alpha=0.7)
    ax_global.set_aspect('equal')
    ax_global.set_xlim(-L_global, L_global)
    ax_global.set_ylim(-L_global, L_global)

    # --- (B) Referencial Local do Robô {R} (Centrado no Robô) ---
    utils.plot_frame([0, 0], 0, ax=ax_local, axis_size=0.5, label="{R}\n(Robô)", colors=('red', 'green'), text_offset=(0.0, 0.28))
    ax_local.scatter([0], [0], color='blue', s=60, zorder=5)

    for item in elements_data:
        x_loc, y_loc, theta_loc = item["local_pos"]
        name = item["name"]
        offset = utils.get_label_offset(x_loc, y_loc, x_ref=0, y_ref=0, name=name)
        label_text = f"{{{name}}}\n({x_loc:+.2f}, {y_loc:+.2f})"
        utils.plot_frame([x_loc, y_loc], theta_loc, ax=ax_local, axis_size=0.35, label=label_text, colors=('red', 'green'), text_offset=offset, fontsize=8.5)
        ax_local.annotate('', xy=(x_loc, y_loc), xytext=(0, 0), arrowprops=dict(arrowstyle='->', color='#7B1FA2', lw=1.5, ls='--'))

    ax_local.set_title("Referencial Local do Robô {R}", fontsize=13, fontweight='bold', pad=10)
    ax_local.set_xlabel("X_robô [m] (Frente)", fontsize=11)
    ax_local.set_ylabel("Y_robô [m] (Esquerda)", fontsize=11)
    ax_local.grid(True, linestyle=':', alpha=0.7)
    ax_local.set_aspect('equal')
    ax_local.set_xlim(-L_local, L_local)
    ax_local.set_ylim(-L_local, L_local)

    # Legenda centralizada
    legend_elements = [
        Line2D([0], [0], color='red', lw=2, label='Eixo X (Frente)'),
        Line2D([0], [0], color='green', lw=2, label='Eixo Y (Esquerda)'),
        Line2D([0], [0], color='#7B1FA2', lw=1.5, ls='--', label='Vetor de Posição Relativa (^R p_i)')
    ]
    fig.legend(handles=legend_elements, loc='lower center', ncol=3, fontsize=10.5, frameon=True, bbox_to_anchor=(0.5, 0.01))

    plt.subplots_adjust(bottom=0.14)
    salvar_figura(output_path)
    plt.close(fig)
    print(f"\n[SUCESSO] Gráfico salvo em '{output_path}'")

    sim.stopSimulation()


if __name__ == "__main__":
    main()


