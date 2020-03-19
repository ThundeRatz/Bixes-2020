# Tarefa de Embarcados

Olá. Venho aqui dar uma pequena explicação sobre o que será esperado para a tarefa de embarcados do Processo Seletivo.

## Objetivo

O principal objetivo da tarefa é familiarizar você com a programação em C, o visual e a estrutura dos arquivos contidos na pasta dos programas da equipe e como fazer o uso de ADC, PWM e GPIO para interagir com elementos físicos.

Você deverá escrever um código que varie a intensidade de um led ao girar um potênciometro. Também deverá implementar um botão que inverta a lógica de variação da intensidade do led.

## Organização do repositório

O repositório dessa tarefinha foi baseado no [STM32ProjectTemplate](https://github.com/ThundeRatz/STM32ProjectTemplate). Acesse o link para saber mais.

## Configurando o Cube

Antes de sair programando, é necessário configurar o arquivo `tarefa_embarcados.ioc`. Para isso, siga as intruções contidas em [Configurando o Cube](cube_configuration.md).

## Funções principais

Seu programa deve utilizar algumas funções já pré estabelecidas, sejam elas já completamente impementadas por nós, ou apenas estruturadas para que vocês modifiquem da forma que bem entenderem.

### Funções prontas

1. **mcu_init():**
    Função essencial do programa. Nela serão inicializadados elementos como os pinos do botão e o clock do sistema. Essa função deve ser a primeira coisa a ser chamada no seu código.

2. **ADC_init():**
    Função que inicia o ADC do programa. Lembre-se de chamar essa função antes de tentar fazer leituras.

### Funções que devem ser implementadas

3. **PWM_init():**
    Função que inicia e configura a PWM do programa.

4. **Interrupção**

    A interrupção é uma pequena função que é chamada sempre que algum evento específico ocorre, como o aperto de um botão, exemplo utilizado nesse exercício. Para isso, você deve usar a função:

    ```C
        void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){}
    ```

    Assim, quando o botão é apertado tudo que você escrever dentro da função acontecerá. Essa ferramenta poderosa possibilita a operação de robôs de sumô a distância, mudando estratégias com o simples aperto de um botão (mais especificamente o botão de um app que envia o comanado por bluetooth).
