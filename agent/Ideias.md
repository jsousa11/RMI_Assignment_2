 No Movement model no N(1, o2) -> usar 1 ao inves de usar essa coisa

 Ver posiçao do robo:
 - a partir do ponto inicial;
 - das paredes que se situam num ponto x ou y em expecifico e fazer os calculos para o meio da celula (Foto no tele)?
 - dos beacons e as suas posições

 Aplicar filtros aos movimentos(Movement model)

 Aplicar filtros a bussola?

 Cada beacon que passa é +1 o ponto inicial é o beacon 0




1. Controle PID para Movimentação Suave
Seu código atual utiliza movimentações baseadas em valores fixos. A inclusão de um controlador PID permitirá:
Ajustes dinâmicos na velocidade e direção.
Redução de erros de posicionamento ao se mover de uma célula para outra.
Como integrar:
Adicione dois controladores PID (um para o eixo X e outro para o eixo Y).
Use os dados de posição do GPS para calcular o erro e ajustar a velocidade dos motores com base no feedback do controlador.


2. Mapeamento Incremental
Adote um sistema de mapeamento dinâmico similar ao do segundo algoritmo. Isso permite que o robô construa um mapa global enquanto explora.
Benefícios:
Identificação clara de células visitadas, paredes e áreas a explorar.
Facilita a replanejamento eficiente quando novas informações são descobertas.
Como integrar:
Crie uma matriz global GRID_MAP para representar o labirinto.
Atualize esta matriz dinamicamente com os dados dos sensores IR do robô durante a exploração.


4. Navegação Orientada e Rotação Dinâmica
Adicione funções para ajustar a orientação do robô com base na bússola.
Benefícios:
Reduz a complexidade ao lidar com ângulos.
Garante que o robô esteja sempre alinhado com a direção desejada antes de se mover.
Como integrar:
Implemente uma função para ajustar o ângulo para o múltiplo de 90 mais próximo antes de cada movimento.
Utilize rotação incremental com base na diferença angular calculada.


5. Exploração Guiada por Fronteiras
Utilize uma estratégia de exploração baseada em fronteiras, onde o robô prioriza a exploração de áreas desconhecidas próximas a células já visitadas.
Benefícios:
Maximiza a eficiência da exploração ao evitar caminhos redundantes.
Como integrar:
Identifique células desconhecidas adjacentes a células conhecidas.
Priorize essas células como objetivos na busca.


6. Salvar o Mapa em Tempo Real
Adicione uma funcionalidade para salvar o mapa em um arquivo de texto ou exibi-lo graficamente em tempo real.
Benefícios:
Facilita o acompanhamento da exploração.
Permite análise pós-execução.
Como integrar:
Adicione uma função que converta o GRID_MAP em um formato visual e o salve em um arquivo ou exiba no terminal.

