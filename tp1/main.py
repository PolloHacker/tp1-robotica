import os
import time
import numpy as np
import matplotlib.pyplot as plt
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
    # Conexão com o CoppeliaSim via ZeroMQ Remote API
    client = RemoteAPIClient()
    sim = client.require('sim')

    # Garante que o diretório de imagens existe
    os.makedirs("images", exist_ok=True)

    # Parar simulação se estiver rodando para garantir estado consistente
    initial_sim_state = sim.getSimulationState()
    if initial_sim_state != 0:
        sim.stopSimulation()
        time.sleep(1)

    # Lista de elementos na cena para mapear
    # Suporta caminhos padrão e variações de indexação no CoppeliaSim
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

    # Inicia a simulação
    sim.startSimulation()
    time.sleep(0.5)

    # 1. Recuperar configuração global do robô q = [x, y, theta]
    xr, yr, thetar = get_global_pose(sim, pioneer_handle)
    q_robot = np.array([xr, yr, thetar])
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

    # 2. Calcular matrizes homogêneas para todos os outros elementos no referencial do robô
    elements_data = []

    for name, handle in handles.items():
        if name == "Pioneer 3DX (Robô)":
            continue

        # Recupera pose global do elemento via ZeroMQ API
        xi, yi, thetai = get_global_pose(sim, handle)
        T_world_elem = utils.homogeneousTransformationMatrix(xi, yi, thetai)

        # Matriz no referencial local do robô: ^R T_elem = (^W T_R)^-1 * ^W T_elem
        T_robot_elem = utils.transformToRobotFrame(T_world_robot, T_world_elem)

        # Extrair posição e orientação locais
        x_local = T_robot_elem[0, 3]
        y_local = T_robot_elem[1, 3]
        theta_local = np.arctan2(T_robot_elem[1, 0], T_robot_elem[0, 0])

        elements_data.append({
            "name": name,
            "global_pos": (xi, yi, thetai),
            "T_world": T_world_elem,
            "T_robot": T_robot_elem,
            "local_pos": (x_local, y_local, theta_local)
        })

        global_str = f"({xi:+.2f}, {yi:+.2f}, {np.rad2deg(thetai):+.1f}°)"
        local_str = f"({x_local:+.2f}, {y_local:+.2f}, {np.rad2deg(theta_local):+.1f}°)"
        print(f"{name:<22} | {global_str:<30} | {local_str}")

    print("=" * 80)

    # 3. Plotagem dos referenciais e vetores de relacionamento
    fig, (ax_global, ax_local) = plt.subplots(1, 2, figsize=(16, 7.5))

    # Dicionário de offsets para os rótulos de modo a evitar sobreposição visual
    custom_offsets = {
        "Mesa": (-0.20, 0.25),
        "Quadcóptero 1": (+0.20, -0.30),
        "Robô NAO": (-0.05, -0.25),
        "Quadcóptero 2": (-0.05, 0.25),
        "Bill": (0.0, 0.20),
        "Cadeira": (0.0, 0.20),
        "Planta 1": (0.0, 0.20),
        "Planta 2": (0.0, 0.20)
    }

    # --- (A) Gráfico no Referencial Global {W} ---
    utils.plot_frame([0, 0], 0, ax=ax_global, axis_size=0.5, label="{W}\n(Mundo)", colors=('red', 'green'))
    utils.plot_frame([xr, yr], thetar, ax=ax_global, axis_size=0.45, label="{R}\n(Robô)", colors=('red', 'green'))
    ax_global.scatter([xr], [yr], color='blue', s=60, zorder=5)

    for item in elements_data:
        xi, yi, thetai = item["global_pos"]
        name = item["name"]
        offset = custom_offsets.get(name, (0.0, 0.15))
        utils.plot_frame([xi, yi], thetai, ax=ax_global, axis_size=0.35, label=f"{{{name}}}", colors=('red', 'green'), text_offset=offset)
        # Vetor do Robô para o Elemento
        ax_global.annotate('', xy=(xi, yi), xytext=(xr, yr),
                           arrowprops=dict(arrowstyle='->', color='#7B1FA2', lw=1.5, ls='--'))

    ax_global.set_title("Referencial Global {W}", fontsize=12, fontweight='bold')
    ax_global.set_xlabel("X_mundo [m]", fontsize=11)
    ax_global.set_ylabel("Y_mundo [m]", fontsize=11)
    ax_global.grid(True, linestyle=':', alpha=0.7)
    ax_global.set_aspect('equal')
    ax_global.set_xlim(-3.0, 3.0)
    ax_global.set_ylim(-3.0, 3.0)

    # --- (B) Gráfico no Referencial Local do Robô {R} ---
    # No referencial local do robô, a origem do robô é (0, 0) e orientação é 0 rad
    utils.plot_frame([0, 0], 0, ax=ax_local, axis_size=0.5, label="{R}\n(Robô)", colors=('red', 'green'))
    ax_local.scatter([0], [0], color='blue', s=60, zorder=5)

    for item in elements_data:
        x_loc, y_loc, theta_loc = item["local_pos"]
        name = item["name"]
        offset = custom_offsets.get(name, (0.0, 0.15))
        utils.plot_frame([x_loc, y_loc], theta_loc, ax=ax_local, axis_size=0.35, label=f"{{{name}}}", colors=('red', 'green'), text_offset=offset)
        # Vetor da Origem Local do Robô para o Elemento
        ax_local.annotate('', xy=(x_loc, y_loc), xytext=(0, 0),
                          arrowprops=dict(arrowstyle='->', color='#7B1FA2', lw=1.5, ls='--'))
        
        # Posição personalizada para o texto com coordenadas locais
        text_dx, text_dy = 0.0, -0.22
        if "Mesa" in name:
            text_dx, text_dy = -0.25, 0.08
        elif "Quadcóptero 1" in name:
            text_dx, text_dy = +0.25, -0.45
        elif "Quadcóptero 2" in name:
            text_dx, text_dy = -0.45, 0.05
        elif "Robô NAO" in name:
            text_dx, text_dy = -0.05, -0.42
        elif "Planta 2" in name:
            text_dx, text_dy = -0.15, -0.22
        elif "Bill" in name:
            text_dx, text_dy = +0.20, -0.22

        ax_local.text(x_loc + text_dx, y_loc + text_dy, f"({x_loc:.2f}, {y_loc:.2f})",
                      fontsize=8, ha='center', color='#1A237E', fontweight='bold',
                      bbox=dict(boxstyle='square,pad=0.15', facecolor='white', edgecolor='none', alpha=0.85))

    ax_local.set_title("Referencial Local do Robô {R}", fontsize=12, fontweight='bold')
    ax_local.set_xlabel("X_robô [m] (Frente)", fontsize=11)
    ax_local.set_ylabel("Y_robô [m] (Esquerda)", fontsize=11)
    ax_local.grid(True, linestyle=':', alpha=0.7)
    ax_local.set_aspect('equal')
    ax_local.set_xlim(-3.5, 3.5)
    ax_local.set_ylim(-3.5, 3.5)

    # Adicionar legenda customizada
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], color='red', lw=2, label='Eixo X (Frente)'),
        Line2D([0], [0], color='green', lw=2, label='Eixo Y (Esquerda)'),
        Line2D([0], [0], color='#7B1FA2', lw=1.5, ls='--', label='Vetor de Posição Relativa (^R p_i)')
    ]
    fig.legend(handles=legend_elements, loc='lower center', ncol=3, fontsize=10, frameon=True, bbox_to_anchor=(0.5, -0.02))

    plt.tight_layout()
    plt.subplots_adjust(bottom=0.1)

    output_plot_path = "images/questao3.png"
    plt.savefig(output_plot_path, dpi=200, bbox_inches='tight')
    print(f"\n[SUCESSO] Gráfico salvo em '{output_plot_path}'")

    # Para a simulação
    sim.stopSimulation()


if __name__ == "__main__":
    main()

