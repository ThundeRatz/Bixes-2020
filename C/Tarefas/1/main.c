#include <stdio.h>
#include <stdint.h>

#include header_file.h

#define ROZETA 1
#define ONIX 2

int main(){
    int robo;
    int strategia;

    printf("\nOlá!\nSeja bem vindo a estação de testes!!\n");
    printf("Qual robo você deseja testar?\n1. Rozeta  2. Onix\n");
    scanf("%d", &robo);

    switch (robo){
        case(ROZETA)
            printf("\nQual estratégia você deseja testar?\n");
            printf("1. Estrela \n2. Dibre \n3. Arco esquerda \n4. Defensiva \n5. Emergencia  \n6. Calibra estrela\n");
            scanf("%d", &strategia);
            test_preparation();
            rozeta_strategy_set(strategia);
            rozeta_strategy_run();
            break;

        case(ONIX):
            print("\nQual estratégia você deseja testar?\n");
            print("1. Estrelinha \n2. Toquinho \n3. Arco \n4. Statico defensivo \n5. Swing defensivo  \n6. Full bomba\n");
            scanf("%d", &strategia);
            test_preparation();
            onix_strategy_set(strategia);
            onix_strategy_run();
            break;

    }
    printf("Parabéns, seu robô passou em todos os testes!!");
    return 0
}
