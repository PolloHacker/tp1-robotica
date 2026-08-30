import argparse
import os
import time
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


def main():
    parser = argparse.ArgumentParser(description="Transformações Homogêneas - TP1 Robótica Móvel")
    parser.add_argument("--output", "-o", default="images/questao3.png", help="Caminho do arquivo de saída da imagem")
    args = parser.parse_args()

    os.makedirs(os.path.dirname(args.output) or ".", exist_ok=True)

    # Conexão com o CoppeliaSim via ZeroMQ Remote API
    client = RemoteAPIClient()
    sim = client.require('sim')

    initial_sim_state = sim.getSimulationState()
    if initial_sim_state != 0:
        sim.stopSimulation()
        time.sleep(1)

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
    plt.savefig(args.output, dpi=200, bbox_inches='tight')
    plt.close(fig)
    print(f"\n[SUCESSO] Gráfico salvo em '{args.output}'")

    sim.stopSimulation()


if __name__ == "__main__":
    main()


