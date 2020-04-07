# C
C é uma das linguagens de programação mais populares e uma das melhores para começar a programar. Por ser rápida e eficiente para lidar com camadas mais próximas do hardware, é a linguagem utilizada na programação de embarcados na equipe, dessa forma, é importante que qualquer membro da computação saiba o básico de C.

Como descrito no planejamento do PS, um material para estudo de C será disponibilizado no [forum] no dia 30/03. Porém, caso queira começar logo, existe uma infinidade de cursos e materiais sobre C na internet, aqui estão duas recomendações:

* [Learn-C][learnc] - Um tutorial interativo de C, em inglês (Até a parte de Structures);
* [Material de MAC2166][mac2166] - Apostila de MAC2166 (Introdução à Computação da grande área Elétrica) sobre C.

Além disso, têm sempre o maravilhoso Google e o Stack Overflow ~~melhor site~~.

Lembrando também que qualquer dúvida, pode perguntar tanto para seus veteranos como no fórum, não precisa ter vergonha :)

---
## Tarefas
A partir do momento que o material for disponibilizado, você terá até dia 08/04 para realizar as [Tarefas](./Tarefas/README.md). Siga o link para ler um pouco mais sobre elas.

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
dos arquivos modificados. Já se tivermos realmente *muitos* arquivos, o melhor é utilizar um Makefile.
* Também existem [várias flags][gcc-flags] que podem ser colocadas no final dessa linha, como `-Wall`,
que habilita vários warnings úteis. Para uso na guerra, **sempre** usem `-Wall -Wextra`, que ativam muitos
warnings. Eles geralmente te avisam de qualquer erro facilmente identificável por passagem de tipos errados
e expressões que podem ter efeitos indesejados. Além disso, é recomendável definir um nível de otimização 
para o compilador, para isso existem várias opções, recomendo deixar `-Og`, que otimiza o código para se 
utilizar ferramentas de depuração.

#### Exemplo
Para compilar o programa na pasta Exemplo:
* Linux
```bash
# Va para a pasta de Exemplos do repositório
~/Bixes-2020$ cd C/Exemplos
# Para compilar
~/Bixes-2020/C/Exemplos$ gcc -o exemplo *.c # Aqui diz "todos os arquivos terminados em .c na pasta atual"
# Para executar
~/Bixes-2020/C/Exemplos$ ./exemplo
```
* Windows
```
C:\Bixes-2020> cd C\Exemplos
C:\Bixes-2020\C\Exemplos> gcc -o exemplo.exe *.c
C:\Bixes-2020\C\Exemplos> exemplo.exe
```

[learnc]: http://www.learn-c.org/
[mac2166]: http://www.ime.usp.br/~hitoshi/introducao/
[forum]: http://forum.thunderatz.org/
[gcc-flags]: https://gcc.gnu.org/onlinedocs/gcc/Option-Summary.html
