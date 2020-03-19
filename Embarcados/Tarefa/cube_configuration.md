# Configurando o Cube

Eai pessoal! Como vocês devem ter aprendido nas aulas de embarcados, é preciso configurar o arquivo do cube antes de programar seu microcontrolador. Para garantir que todos façam tudo da mesma forma e fique mais fácil de testar, preparamos esse pequeno guia para ajudar vocês nessa parte, ~~que pode ser bem chatinha no começo~~.

Antes de mais nada, recomendo **fortemente** consultar o nosso [STM32Guide](https://github.com/ThundeRatz/STM32Guide) enquanto você segue cada passo desse guia. Acredite, você vai precisar dele ;)

## Requisitos do projeto

A tarefa proposta necessita de apenas 3 elementos: um pino para leitura de ADC, um pino para mandar uma PWM e um pino de interrupção. Para isso, utilizaremos os seguintes GPIO's:

* ADC: PA0
* PWM: PA5
* Interrupção: PC13

O código será rodado em uma [NUCLEO-F303RE](https://os.mbed.com/platforms/ST-Nucleo-F303RE/), que posui um `STM32F303RE`, microcontrolador de arquitetura ARM 32 bits, muito usados em nossos projetos.

O arquivo `tarefa_embarcados.ioc`, disponibilizado para vocês, já possui algumas configurações básicas feitas, então não se assustem ao abrir o cube e ver várias coisas setadas, o que elas são vai ficar mais claro com o tempo.

## Configurando seu Hardware

Bom, finalmente vamos ao que importa!

### PWM

Para configurar a PWM enviada para o LED, precisarem de um timer. Por facilidade, utilizaremos o canal 1 do timer 2 (TIM2_CH1).

Abra a aba `Timers` a esquerda e selecione o **TIM2**. Uma outra aba chamada `TIM2 Mode and Configuration` aparecerá, e é aqui que iremos configurar nossa PWM.

Em `Mode`, precisamos, primeiramente, fornecer uma fonte de clock para nosso timer, então clique `Clock Source` e selecione a opção **Internal Clock**. Além disso, clique em `Channel 1` e selecione **PWM Generation CH1**.

Agora, precisamos escolher qual GPIO mandará nossa PWM para o mundo físico, pois nessa placa, temos duas opções para o TIM2_CH1. Por padrão, o Cube setará para você o pino PA0. Porém, utilizaremos o PA5, que também suporta essa funcionalidade. Para trocar, basta segurar `Ctrl` e apertar e segurar o botão esquerdo do mouse em cima do PA0. Quando fizer isso, o PA5 irá se iluminar, então basta arrastar o mouse até ele e soltar. Pronto, agora, o pino PA5 deve ter ficado verde e o PA0 voltará para a cor cinza.

Já temos uma fonte de clock e um pino, mas ainda falta setar algumas coisas e essa parte é com vocês! Todo o resto deve ser feito na aba `Configuration`: vocês devem configurar a PWM com frequencia de 1 kHz e o counter period escolhido deve permitir uma precisão de, no mínimo, 1% para definir o duty cycle. Se quiserem, podem usar o Fast Mode, como o STM32Guide sugere.

OBS: O clock interno do uC é de 72 MHz!

### Interrupção

O pino que será usado para ler a interrupção será o PC13. Para configurá-lo, clique com o botão esquerdo do mouse sobre ele, e depois selecione a opção GPIO_EXTI13.

O que falta agora é ativar as interrupções externas para esse pino e configurá-las para serem chamadas em bordas de descida (External Interrupt Mode with Falling Edge trigger detection), o que pode ser feito na aba `GPIO` dentro de `System Core`. (Consulte o STM32Guide para mais detalhes).

### ADC

Para a leitura do potenciômetro, utilizaremos o ADC1_IN1, que está disponível no pino PA0. Para configurá-lo, abra a aba `Analog` e clique em **ADC1**. Assim como anteriormente, uma nova aba chamada `ADC1 Mode and Configuration` aparecerá.

Em `Mode`, clique em `IN1` e selecione a opção **IN1 Single-ended**. Nesse momento, o pino PA0 deve ficar verde.

Em `Configuration`, precisamos mudar apenas algumas coisas em `Parameters Settings`:

* ADC_Settings

    * Clock Prescaler: **Synchronous clock mode divided by 2**
    * Continuou COnversion Mode: **Enable**
    * Overrun behavior: **Overrun data preserved**

* ADC_Regular_ConversionMode

    * Rank

        * Sampling Time: **61.5 Cycles**

Essas são apenas as coisas que precisam ser modificadas do padrão, as outras configurações devem ser mantidas no padrão do Cube.
