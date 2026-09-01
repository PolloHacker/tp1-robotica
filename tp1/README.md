# Trabalho Prático 1 - Robótica Móvel (UFMG)
**Professor:** Douglas G. Macharet  

---

## Questão 3: Transformações Homogêneas e Referenciais

### 1. Definição Matemática das Transformações

Considere o robô móvel posicionado no espaço bidimensional com configuração global dada por:
$$q_R = [x_R, y_R, \theta_R]^T$$

A matriz de transformação homogênea que representa a pose do robô em relação ao referencial global $\{W\}$ (Mundo) é expressa como:
$$^W T_R = \begin{bmatrix} \cos\theta_R & -\sin\theta_R & 0 & x_R \\ \sin\theta_R & \cos\theta_R & 0 & y_R \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

Para qualquer elemento $i$ presente na cena, sua configuração global é dada por $q_i = [x_i, y_i, \theta_i]^T$, com matriz de transformação homogênea no referencial global dada por:
$$^W T_i = \begin{bmatrix} \cos\theta_i & -\sin\theta_i & 0 & x_i \\ \sin\theta_i & \cos\theta_i & 0 & y_i \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

Para determinar a posição e orientação de cada elemento $i$ no **referencial local do robô $\{R\}$**, utilizamos a composição de transformações homogêneas:
$$^R T_i = (^W T_R)^{-1} \cdot {}^W T_i = {}^R T_W \cdot {}^W T_i$$

A inversa de uma transformação rígida homogênea $T = \begin{bmatrix} R & p \\ \mathbf{0} & 1 \end{bmatrix}$ é calculada analiticamente por:
$$(^W T_R)^{-1} = \begin{bmatrix} R_R^T & -R_R^T p_R \\ \mathbf{0} & 1 \end{bmatrix} = \begin{bmatrix} \cos\theta_R & \sin\theta_R & 0 & -(x_R \cos\theta_R + y_R \sin\theta_R) \\ -\sin\theta_R & \cos\theta_R & 0 & x_R \sin\theta_R - y_R \cos\theta_R \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

Multiplicando-se $(^W T_R)^{-1}$ por $^W T_i$, obtém-se as coordenadas locais $(^R x_i, ^R y_i)$ e orientação local $^R\theta_i$:
$$^R x_i = (x_i - x_R)\cos\theta_R + (y_i - y_R)\sin\theta_R$$
$$^R y_i = -(x_i - x_R)\sin\theta_R + (y_i - y_R)\cos\theta_R$$
$$^R \theta_i = \theta_i - \theta_R$$

---

### 2. Execução do Script

Para executar a solução com o **CoppeliaSim** aberto:

```bash
cd tp1

# Questão 3 (Cena padrão)
uv run python main.py --output images/questao3.png

# Questão 4 (Com as respectivas cenas TP1-Cena-q4_p*.ttt abertas no CoppeliaSim)
uv run python main.py --output images/questao4_p1.png
uv run python main.py --output images/questao4_p2.png
uv run python main.py --output images/questao4_p3.png

# Questão 5 (abrir uma cena q5 por vez no CoppeliaSim)
uv run python main.py --question 5 --output images/questao5_p1_global.png
uv run python main.py --question 5 --output images/questao5_p2_global.png
uv run python main.py --question 5 --output images/questao5_p3_global.png

# Questão 6 (navegação básica, 40 s de simulação por padrão)
uv run python main.py --question 6

# Questão 6 com duração e nome de saída personalizados
uv run python main.py --question 6 --duration 5 --output images/questao6_navegacao_global.png
```

O script:

1. Conecta-se à cena do **CoppeliaSim** via **ZeroMQ Remote API** (`coppeliasim_zmqremoteapi_client`).
2. Recupera as posições e orientações globais de todos os elementos e do robô.
3. Computa analiticamente as matrizes de transformação homogêneas globais e locais $^R T_i$.
4. Calcula limites dinâmicos com proporção 1:1 e resolve automaticamente sobreposições entre caixas de texto.
5. Plota os referenciais de coordenadas (Eixo X em vermelho, Eixo Y em verde) e os vetores de posição relativa.
6. Salva a figura gerada no caminho especificado em `--output`.

### Questão 5: laser no referencial global

As cenas utilizadas são `TP1-Cena-q5_p1.ttt`, `TP1-Cena-q5_p2.ttt` e
`TP1-Cena-q5_p3.ttt`. O robô deve possuir o objeto
`/PioneerP3DX/fastHokuyo`, com os sensores filhos `sensor1` e `sensor2`.

Na execução da questão 5, o script recupera pela RemoteAPI as matrizes
`^R T_L = sim.getObjectMatrix(laser, robot)` e
`^W T_R = sim.getObjectMatrix(robot, sim.handle_world)`. Cada ponto é
transformado pela composição `^W p = ^W T_R ^R T_L ^L p` antes de ser plotado.

### Questão 6: navegação, trajetória e laser no referencial global

A questão 6 adapta o script básico de navegação da aula para o PioneerP3DX.
Abra no CoppeliaSim uma cena que contenha:

- o robô `/PioneerP3DX`;
- as juntas `/PioneerP3DX/leftMotor` e `/PioneerP3DX/rightMotor`;
- o laser `/PioneerP3DX/fastHokuyo`, com os sensores filhos `sensor1` e `sensor2`.

Execute o comando a partir de `robotica/tp1`, com o CoppeliaSim aberto:

```powershell
uv run python main.py --question 6
```

O padrão é executar 40 segundos de simulação e salvar o gráfico em
`images/questao6_navegacao_global.png`. Para uma duração diferente, use
`--duration`, por exemplo:

```powershell
uv run python main.py --question 6 --duration 5
```

Durante a navegação, o script registra a posição global do robô, transforma
cada leitura local do laser pela cadeia `^W T_R ^R T_L` e acumula todos os
pontos válidos. Ao final, uma única figura mostra:

1. a trajetória global do robô em linha azul tracejada;
2. todas as leituras acumuladas do laser em laranja;
3. os marcadores de início e fim do percurso.

Se a cena iniciar pausada, o script tenta retomá-la automaticamente. O
arquivo de saída também pode ser alterado com `--output`.

