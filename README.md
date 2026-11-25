# 3ª avaliação de Robótica

## Abrir e testar

1. Abra `SonarRobotica/SonarRobotica.ino` no Arduino IDE com o pacote **esp32 by Espressif Systems 3.x**. Selecione **ESP32 Dev Module** para uma DevKit ESP32 clássica e a porta da sua placa.
2. Grave o sketch e conecte o computador ou celular à rede **Robo-Sonar**, senha de demonstração **sonar-2026**. Se o aparelho avisar que a rede não tem internet, mantenha a conexão.
3. Abra **http://192.168.4.1**. O Monitor Serial a **115200** também mostra o endereço e as leituras.
4. Com o robô parado, coloque uma caixa à frente e clique em **Iniciar varredura**. A IHM registra 19 posições entre 0° e 180° em passos de 10°. Mova a caixa e inicie outra varredura para comparar o mapa.
5. **Parar** interrompe a varredura e conserva os pontos já medidos. **Limpar mapa** remove os pontos. Uma nova varredura substitui a anterior.

## Usar o supervisório pelo USB no Windows

1. Grave o sketch atualizado, conecte o cabo USB e feche o Monitor Serial do Arduino.
2. Abra `Abrir_supervisorio.cmd`. A janela do supervisório abre diretamente a **COM3**, sem navegador. Se o Windows atribuir outra porta, ajuste `SerialBridge("COM3")` em `Supervisorio_Desktop.py`.
3. Aguarde **ESP32 conectada · USB COM3** e use **Iniciar varredura** para o primeiro teste com o robô parado.

## Avançar, detectar, parar e mapear

Clique em **Avançar e mapear** com a área e o cabo livres. O sensor vai para 90° e verifica a frente antes de acionar as rodas. O robô avança em malha aberta com PWM **175/180**, mede à frente aproximadamente a cada 70 ms e para diante de um eco válido a **45 cm ou menos**. Sem eco válido, ele deixa de avançar. Cada avanço dura no máximo **5 segundos** e tem corte independente se a atualização das leituras travar.

Após a parada, a ESP32 espera **750 ms** antes de varrer de 0° a 180°. A IHM mostra o motivo da parada e desenha os ecos medidos com o robô parado. **Parar** cancela o avanço, a espera ou a varredura. O mapa é local à posição da parada, e uma nova operação substitui as leituras anteriores.

Na bancada, teste primeiro com uma caixa grande em frente e com distância de frenagem disponível. Calibre a posição frontal e confira o sentido das rodas antes de avançar no chão. O limiar `OBSTACLE_CM` e o tempo de avanço `ADVANCE_LIMIT_MS` ficam no início do sketch.

## Ligações

| Componente | Sinal | ESP32 |
| --- | --- | --- |
| SG90 | PWM | D22 |
| HC-SR04 | TRIG | D18 |
| HC-SR04 | ECHO | D19 com divisor de tensão |
| Ponte H | Motor esquerdo ENA / IN1 / IN2 | D13 / D26 / D25 |
| Ponte H | Motor direito ENB / IN3 / IN4 | D4 / D33 / D32 |
| FC-03 | DO esquerdo / direito (montagem anterior) | D2 / D15 |

Alimente o SG90 e o HC-SR04 com 5 V regulados e compartilhe o GND com a ESP32 e a ponte H. No ECHO, use **1 kΩ do ECHO ao nó do D19** e **2 kΩ do nó ao GND**. Use a entrada de alimentação apropriada da sua DevKit e confira a ligação do driver conforme a placa. O PWM vai em ENA/ENB. Se a ponte H tiver jumpers que ligam ENA/ENB ao nível alto, retire-os antes de ligar D13/D4. O firmware do sonar não lê os encoders. Mantenha o GND comum e adeque o DO para 3,3 V se o encoder fornecer sinal de 5 V.

## Conferência de bancada

1. Com os motores parados, abra **Ajustar a montagem do sensor → Centralizar servo**. Alinhe a frente do HC-SR04 com a frente do robô nessa posição.
2. Use a fonte externa regulada de 5 V dimensionada para o conjunto. O enunciado recomenda 3–5 A e um cabo flexível de aproximadamente 1,5 m. Fixe o cabo fora das rodas e use GND comum.
3. Ponha uma caixa a uma distância que possa conferir com a trena. Rode a varredura e compare o valor na tabela **Leituras da varredura**. Em seguida, desloque a caixa lateralmente e repita. A posição estimada deve mudar no mapa.
4. Confira se a varredura percorre o curso físico esperado, se a posição de 90° aponta para a frente e se o sentido do mapa coincide com a montagem. Só ajuste os limites de pulso após conferir o curso do seu servo.

## Varredura e calibração

- Ao ligar, o SG90 vai para 90°. Cada varredura passa primeiro pelo extremo esquerdo (0°), mede até o extremo direito (180°) e volta lentamente a 90° sem apagar o mapa. Durante essa sequência o servo avança 1° a cada 45 ms e espera 300 ms em cada posição de medição. Parar interrompe a sequência e mantém os pontos já coletados.
- Se a câmera estiver montada no mesmo suporte, ela acompanha o movimento do SG90. Como o servo não informa a posição física à ESP32, alinhe o suporte na primeira montagem com o servo já na posição comandada.

- Monte o HC-SR04 de modo que **90° aponte para a frente**. O supervisório interpreta 0° à esquerda e 180° à direita. Se estiver espelhado, altere `REVERSE_SERVO` para `true`.
- O código começa com pulsos de **1.000 a 2.000 µs**. Confira o curso do SG90 e calibre `SERVO_MIN_US` e `SERVO_MAX_US` para obter os extremos físicos, sem pressionar os batentes. O ângulo exibido é o comando, pois o SG90 não fornece posição à ESP32.
- Em cada ângulo, o firmware faz cinco disparos espaçados em 70 ms. A distância registrada é a mediana, aceita quando pelo menos três ecos válidos ficam a até 5 cm ou 10% dela, o que for maior. A faixa nominal é de 2 a 400 cm. Sem concordância suficiente, a leitura fica fora do mapa.
- O controle **Testar motores** aplica **175 à esquerda e 180 à direita** por 1 segundo em malha aberta. A varredura mantém os motores parados. O teste dos motores é separado, apaga o mapa anterior e só pode rodar após a varredura parar. Esse PWM reproduz a referência informada, e o trajeto depende do piso e da alimentação.

## Exploração automática com um botão

1. Grave o firmware em `SonarRobotica/`. Feche o Monitor Serial e o supervisório manual para liberar a COM3.
2. Posicione o robô em uma área plana, com espaço para girar e o cabo fora das rodas. A posição de partida passa a ser **(0,0)**. A frente do robô é **+Y** e sua direita é **+X**. Confira primeiro no painel manual se a varredura física vai da esquerda para a direita e termina no centro.
3. Abra `Abrir_autonomia.cmd` e clique em **Iniciar**. O painel não exige medidas ou calibração para começar. Use **Parar** ou **Esc** para encerrar. O mapa é salvo em `mapa-sessao.json`. É possível reabrir mapas anteriores pelo botão **Abrir mapa JSON**.
4. O mapa cresce a partir dos ecos. Os contornos em laranja são obstáculos com localização estimada. A escala de **50 cm** aparece com **≈** quando a pose não foi confirmada. Uma passagem observada fica marcada mesmo depois da travessia, e a exploração continua.

### Movimento e segurança

- O perfil interno pressupõe um chassi de **25 × 30 cm**, velocidade de **25 cm/s** e giros de **100°/s**. São referências conservadoras de software e a escala é estimada. Se o chassi real for maior, ajuste o perfil em `Navegacao.py` antes de usar a autonomia.
- O robô avança em pulsos de até **350 ms** e confere a frente entre eles. Faz varredura completa ao ligar a exploração, ao bloquear e a cada **25 cm estimados**. Se a frente fechar, tenta esquerda e depois direita. Os giros no lugar usam PWM **175/180**, em sentidos opostos, e conferem a área varrida pelo retângulo do chassi. Ao avançar, a ESP32 corta a tração se detectar um obstáculo a **45 cm ou menos**, ou se o sonar deixar de receber eco. O corte de tempo e a perda do vínculo USB também são tratados pela placa.
- O planejador prioriza fronteiras e usa células livres para retornar a uma área com outras direções possíveis. Se cair numa situação sem passagem observável, mantém as rodas paradas, informa **Aguardando passagem segura** e volta a medir. Uma queda da COM3 encerra a sessão.
- Uma passagem provável precisa de bordas observadas e de folga medível dos dois lados. Sem delimitar o perímetro, o sonar não consegue garantir que a passagem é a saída da arena. A marca some após duas varreduras válidas além dela. Com o robô imóvel, vale a última posição confiável. Depois de mover as rodas, o sonar precisa reconhecer a posição novamente. Âmbar com **1/2** indica a primeira confirmação. Uma marca sem retorno correspondente continua aguardando confirmação.

### Recuperação por ré

Se o robô ficar bloqueado, o painel procura primeiro um giro com folga. A ré fica limitada a **120 ms** por pulso e só é autorizada sobre o último trecho reto percorrido, enquanto a posição for confiável e a faixa atrás do chassi continuar livre no mapa. O sonar mede à frente. Ao iniciar cercado, sem histórico traseiro, o robô mantém as rodas paradas. Pressione **Parar** ou **Esc** para cancelar.

O estado serial inclui `reverseMaxMs` e `rawPulses`. Os cinco pulsos brutos em microssegundos por ângulo aparecem na ordem de aquisição. Zero representa ausência de pulso capturado. Compare uma varredura com caixa e outra sem caixa para distinguir marca antiga de um eco que ainda chega ao sensor.

### Testes

Execute `python -B -m unittest discover -s testes -p "teste_*.py" -v`. Os testes reproduzem orientação, persistência, cancelamento, perda de comunicação, busca e remoção de obstáculos. No robô, valide também o curso do servo, a folga de giro e a distância de parada com rodas no piso. As medidas e o comportamento físico dependem da montagem.

As três leituras no topo indicam o motivo da decisão; o cartão verde marca a direção escolhida. O sonar frontal continua sendo o veto de avanço em tempo real.


### Decisão rápida e pontos incertos

O comando USB `probe` faz uma sondagem independente em 30°, 60°, 90°, 120° e 150°, com as rodas paradas. O servo avança em passos de até 3° a cada 45 ms, equivalentes a 1° por 15 ms, acomoda por 220 ms em cada posição e volta a 90°. São três disparos separados por 70 ms. A capacidade aparece em `quickProbe` e o resultado em `scanKind: quick`, sem misturar a leitura com uma varredura completa.

O painel mede a frente entre pulsos. Ao bloquear depois de mover, usa a leitura rápida antes de varrer tudo. Depois de dois giros ou 25 cm de deslocamento estimado, atualiza a referência com uma varredura completa. Só essa varredura altera o mapa persistente. O painel mostra a camada incerta mais recente; as marcas confirmadas são conservadas até duas varreduras completas confiáveis comprovarem espaço livre.

### Recuperação de bloqueios

O contorno violeta indica as células que impedem a manobra. A leitura curta (USB) pode retirar o veto de um eco provisório quando a pose é comparável e um retorno válido passa além da célula. Uma marca confirmada continua exigindo duas varreduras completas. Se a leitura curta não alcançar a margem, o painel volta à varredura completa. O controlador tenta giros de 25°, 15°, 10° e 5° conforme a folga e o tempo mínimo de pulso. A ré continua limitada ao último trecho conhecido. Se nenhuma opção tiver folga, o robô espera e mede de novo.
