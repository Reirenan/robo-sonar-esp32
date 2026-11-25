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

## Exploração autônoma pela COM3

A exploração usa somente o HC-SR04 para observar a arena, o SG90 para apontar o sonar e a ponte H para tracionar as rodas. O computador planeja sobre uma grade de 10 cm. A ESP32 mantém o corte de potência independente do computador.

1. Grave o sketch em `SonarRobotica/`. Feche o Monitor Serial, o supervisório manual e outras aplicações que possam usar a COM3.
2. No painel manual, verifique se o suporte do sonar aponta para a esquerda no comando lógico de 0°. Este firmware está configurado com `REVERSE_SERVO = true`, de acordo com o relato de montagem invertida. Inverta essa constante se o teste físico mostrar o contrário. Centralize em 90° para alinhar o sensor à frente e confira os dois limites antes de iniciar a navegação.
3. Coloque o robô na posição escolhida para **(0,0)**, com a frente perpendicular à parede traseira, apontando para **+Y**. O eixo **+X** aponta à direita. Meça largura, comprimento, o afastamento da origem até as paredes esquerda e traseira, e as dimensões do chassi. A posição de referência do mapa é o centro do chassi. Para reduzir erro geométrico, instale o sonar o mais perto possível do eixo de giro.
4. Abra `Abrir_autonomia.cmd`. Use os botões de calibração com área livre para aplicar um pulso por vez. Com o robô no piso, meça o avanço em cm ou o giro em graus. Divida a medida pelo tempo de tração mostrado em segundos e preencha o campo correspondente. Repita os três testes e use a mediana. O avanço usa os valores de referência 175 e 180, à esquerda e à direita.
5. Preencha as medidas e clique em **Iniciar nova exploração**. Use **Parar** ou **Esc** para cancelar. **Abrir mapa JSON** consulta uma sessão gravada; uma nova exploração parte de uma origem física novamente conhecida.

### Como os pontos são validados

- O mapa usa distância válida até um obstáculo para marcar o ponto final e indicar as células atravessadas pelo feixe. Só registra espaço livre de novas varreduras com pose corrigida pelo sonar. Pontos de uma pose incerta são mantidos à parte no mapa para inspeção. Se uma célula ocupada ficar livre, a retirada da marca exige dois lotes independentes de ecos válidos além dela.
- Em cada parada, o ajuste compara os retornos previstos com duas paredes ortogonais. Pouca geometria visível, rodas escorregando ou obstáculos junto às paredes podem impedir a localização. Nesse caso a exploração para e indica o motivo.
- Um pulso autônomo dura de 40 a 350 ms. Ao avançar, a ESP32 mede com o sonar apontado para a frente e interrompe a tração com um eco a até 45 cm ou sem eco válido. A ESP32 também cancela a sessão se não receber consultas USB por 2,5 segundos.
- Uma saída candidata exige bordas observadas e corredor medido além da parede. A confirmação exige que a posição corrigida e a margem geométrica indiquem que o chassi passou completamente. Uma abertura para uma região sem superfícies retornando eco pode permanecer sem confirmação. O painel não considera falta de eco uma passagem comprovada.

### Teste de bancada e de arena

- Apoie o robô com as rodas livres antes de conferir os sentidos dos motores. Para a calibração no piso, use espaço livre e mantenha a parada acessível.
- Mova a mesma caixa entre esquerda, frente e direita e compare o lado do eco e a escala de 50 cm. Depois inicie na arena, retire um obstáculo em uma área revisitada e confira que a marca só some após duas observações válidas.
- Durante os testes, valide parada por obstáculo, cancelamento manual, perda de eco e interrupção USB. Para a saída, meça a abertura e verifique visualmente se o chassi cruzou completamente.
- Os testes automatizados usam cenários sintéticos. Execute `python -B -m unittest discover -s testes -p "teste_*.py" -v`. A validação na arena exige a montagem física e o sonar respondendo.
