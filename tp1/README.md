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

Para executar a solução da Questão 3 e gerar os gráficos:

```bash
cd tp1
uv run python main.py
```

O script:
1. Conecta-se à cena do **CoppeliaSim** via **ZeroMQ Remote API** (`coppeliasim_zmqremoteapi_client`).
2. Recupera as posições e orientações globais de todos os elementos e do robô.
3. Computa analiticamente as matrizes de transformação homogêneas globais e locais $^R T_i$.
4. Plota os referenciais de coordenadas (Eixo X em vermelho, Eixo Y em verde) e os vetores de posição relativa.
5. Salva a figura gerada em `images/questao3.png`.
