# C
A segunda aula vai ter como tema a programação em C, porém, para não ser algo muito longo e cansativo, só vamos passar bem por cima pelos tópicos da aula. Por isso, é importante que, antes da aula, estudem por conta própria para saber pelo menos o básico de C, recomendo o seguinte:

* [Learn-C][learnc] - Um tutorial interativo de C, em inglês (Até a parte de Structures);
* [Material de MAC2166][mac2166] - Apostila de MAC2166 (Introdução à Computação da grande área Elétrica) sobre C.

Além disso, existe o maravilhoso Google e o Stack Overflow ~~melhor site~~.

Lembrando também que qualquer dúvida que vocês tiverem, podem perguntar, tanto para seus veteranos como no fórum, não precisam ter vergonha :)

---

## Organização de Código
Como já devem saber, essa é a estrutura básica de um programa em C:
```c
#include <stdio.h>       // Isso é um header
// Outros includes
#define CONSTANTE 1
// Outras diretivas de compilador

// Declaração de funções (protótipos)
int func(int a);

int main() {
	// código
}

// Definição de funções
int func(int a) {
	// código
}
```
Mas e se o código for muito grande e/ou tiver muitas funções?
Como organizá-lo e facilitar a manutenção e compreensão? Simples: dividimos o código em vários arquivos!
* Headers - Cabeçalhos, onde ficam as declarações de funções e definições de constantes;
* Sources - Fontes, onde ficam as definições das funções declaradas nos headers.

### Headers
Um header é um arquivo terminado em .h que contém a declaração de funções e,
se necessário, definições de constantes, por exemplo:
```c
//func.h
#pragma once // Essa é uma diretiva que impede esse arquivo de ser processado pelo
			 // compilador mais de uma vez, caso mais de um arquivo o inclua.
// Também se pode incluir outros headers aqui, se forem necesários
#define CONST 10

int func(int a);

```
### Source Files
Um arquivo fonte é um arquivo terminado em .c que contém as definições de funções e a função main, normalmente em seu próprio arquivo, por exemplo:
```c
//func.c
#include "func.h" // Temos que incluir

int func(int a) {
	int b = CONST * a; // Podemos usar CONST, que foi definido no header incluso
	return b;
}
```
---
## Compilação
Programas em C são compilados usando o gcc (GNU Compiler Collection), para usá-lo:
* Primeiro deve-se criar os objetos dos arquivos fonte que se deseja compilar:
```bash
$ gcc -c main.c helloworld.c # Gera main.o e helloworld.o
```
* Depois criamos o executável, o nome depois do `-o` é o nome do executável que queremos
(no Windows é bom colocar um .exe no final do nome), podemos omitir o `-o executavel`,
assim o nome será a.out (a.exe no Windows).
```bash
$ gcc -o executavel main.o helloworld.o
```
* É possível resumir esses dois passou em uma única linha fazendo:
```bash
$ gcc -o executavel main.c helloworld.c
```
Isso cria os objetos, usa-os e os apaga, mas isso só em bom quando temos um ou dois arquivos,
se tivermos vários, é melhor o primeiro método, já que se mudarmos algo só precisamos recriar os .o
dos arquivos modificados. Já se tivermos realmente *muitos* arquivos, o melhor é utilizar um Makefile,
como verão na aula de make!
* Também existem [várias flags][gcc-flags] que podem ser colocadas no final dessa linha, como `-Wall`,
que habilita vários warnings úteis. Para uso na guerra, sempre usem `-Wall -Wextra`, que ativam muitos
warnings. Eles geralmente te avisam de qualquer erro facilmente identificável por passagem de tipos errados
e expressões que podem ter efeitos indesejados.

#### Exemplo
Para compilar o programa na pasta Exemplo:
* Linux
```bash
# Na pasta que clonaram o seu fork do repositório dos bixos
~/Bixos-2019$ cd C/Exemplos
# Para compilar
~/Bixos-2019/C/Exemplos$ gcc -o exemplo *.c # Aqui diz "todos os arquivos terminados em .c na pasta atual"
# Para executar
~/Bixos-2019/C/Exemplos$ ./exemplo
```
* Windows
```
C:\Bixos-2019> cd C\Exemplos
C:\Bixos-2019\C\Exemplos> gcc -o exemplo.exe *.c
C:\Bixos-2019\C\Exemplos> exemplo.exe
```

[learnc]: http://www.learn-c.org/
[mac2166]: http://www.ime.usp.br/~hitoshi/introducao/
[codeblocks]: http://www.codeblocks.org/
[atom]: https://atom.io/
[vscode]: https://code.visualstudio.com/
[forum]: http://forum.thunderatz.org/
[mingw-down]: http://sourceforge.net/projects/mingw/files/
[mingw-wiki]: http://www.mingw.org/wiki/getting_started
[gcc-flags]: https://gcc.gnu.org/onlinedocs/gcc/Option-Summary.html
