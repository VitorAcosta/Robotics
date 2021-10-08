# Robotics
Repositório com atividades da matéria de robótica e inteligência artificial.

# Atividade 1 - Trajetória robô
A atividade 1 tem como objetivo definir um algoritmo em que o robô seja capaz
de deslocar-se pelo cenário e encostar, pelo meno uma vez, em cada caixa disposta.

## Resumo do algoritmo
Foram consideradas algumas premissas iniciais para que o algoritmo fosse desenvolvido,
são elas:
 * A posição de toda caixa é conhecida previamente.
 * A posição do robô é conhecida e atualizada.
 * A velocidade rotacional e tangencial do robô é conhecida.
 * Não existem obstáculos no cenário além das próprias caixas.

O algoritmo baseia-se na ideia de calcular a distância à caixa não visitada mais
próxima e então calcular o ângulo theta entre o robô e o centro da caixa que será
utilizado para rotacionar a frente do robô em direção à caixa.

![Esquema](Atv1-TrajetoriaRobo/docs/definicao_theta.png "Ideia Base do algoritmo")

Obs: Note que os eixos dispostos na imagem seguem o padrão de crescimento do WeBots.


# Atividade 2 - Robô caixa bloco
A atividade 2 tem como objetivo definir um algoritmo o qual o robô seja capaz
de deslocar-se pelo cenário e encostar, pelo menos uma vez, em cada caixa disposta do cenário.
A diferença é que: algumas das caixas inseridas no cenário são leves, e portanto podem deslocar-se
ao toque do robô.

O objetivo então, além do mencionado, é também categorizar qual caixa é leve e qual não é.
Para essa categorização, basta acender por exemplo um LED do robô, indicando que a caixa
que foi encostada é leve.

## Resumo do algoritmo
As mesmas premissas da atividade 1 foram aqui consideradas, com a adição de uma quinta:
 * A posição de todas as caixas é monitorada, assim, qualquer deslocamento da caixa pelo toque do robô já é detectado.

O algoritmo ainda segue a mesma base abordada na atividade 1, com a diferença de que:
 * Para adquirir a posição do robô, utiliza-se o GPS.
 * Para verificar o norte do robô, utiliza-se a bússola (ou compass).
Essas adições foram imprescidíveis para a conclusão do laboratório, uma vez que os valores
adquiridos pelos módulos possibilita a redução de erro na tomada de decisão, quando comparado
com o método utilizado no laboratório 1, o supervisor.

