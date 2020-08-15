# Instalando o necessário

# Índice

- [Índice](#Índice)
- [Introdução](#introdução)
- [Instruções para instalação](#instruções-para-instalação)
    - [Windows](#windows)
        - [GCC e Make](#gcc-e-make)
        - [Git](#git)
        - [ARM-GCC](#arm-gcc)
        - [STM32CubeMX e STM32CubeProgrammer](#stm32cubemx-e-stm32cubeprogrammer)
        - [VSCode](#vscode)
    - [Linux](#linux)
        - [GCC, Make e Git](#gcc,-make-e-git)
        - [ARM-GCC](#arm-gcc)
        - [STM32CubeMX e STM32CubeProgrammer](#stm32cubemx-e-stm32cubeprogrammer)
        - [VSCode](#vscode)
    - [MacOS](#macos)

---

# Introdução

Para realizar as tarefas do Processo Seletivo, é necessário instalar algumas coisas antes.
* GCC;
* Make;
* Git;
* ARM-GCC;
* STM32CubeMX;
* STM32CubeProgrammer.
* VSCode

Os seis primeiros itens são ferramentas essenciais para qualquer membro da computação da equipe. Já o VSCode é um editor de texto com várias ferramentas e extensões que auxiliam na hora de programar, não é essencial, mas muito recomendado.

# Instruções para instalação

## Windows

### GCC e Make

No Windows, instale o [MSYS2][msys2] e siga as instruções da página de download.
Após realizar todos os passos da página, podemos instalar pacotes pelo terminal do MSYS2, usando o comando:

`pacman -S <pacote1> <pacote2> ... <pacoteN>`

Assim, para instalar o que é necessário para o Processo Seletivo (que também é necessário para a equipe), basta utilizar o comando:

```
pacman -S gcc make
```

Após isso, é preciso colocar a pasta do msys2 nas variáveis de ambiente do Windows. Para isso, faça:

1. Pesquise variáveis de ambiente e clique em "Editar as variáveis de ambiente do sistema". Ou, vá para Painel de Controle > Sistema e Segurança > Sistema > Configurações avançadas do sistema.
2. Na janela que abriu, clique em "Variáveis de ambiente".
3. Na parte de "Variáveis do sistema", encontra a variável Path, selecione-a e clique em "Editar".
4. Clique em "Novo".
5. Digite o caminho para os executáveis MSYS2 (normalmente C:\msys64\usr\bin).
6. Clique em "OK".

Agora abra o Prompt de Comando (não o MSYS2, o prompt de comando normal do Windows) para testar se os pacotes foram instalados corretamente.
Para testar se gcc foi instalado corretamente, digite:

`gcc -v`

Algo parecido com isso deve aparecer no seu prompt de comando:

```
Using built-in specs.
COLLECT_GCC=gcc
COLLECT_LTO_WRAPPER=/usr/lib/gcc/x86_64-pc-msys/6.4.0/lto-wrapper.exe
Target: x86_64-pc-msys
Configured with: /msys_scripts/gcc/src/gcc-6.4.0/configure --build=x86_64-pc-msys --prefix=/usr --libexecdir=/usr/lib --enable-bootstrap --enable-shared --enable-shared-libgcc --enable-static --enable-version-specific-runtime-libs --with-arch=x86-64 --with-tune=generic --disable-multilib --enable-__cxa_atexit --with-dwarf2 --enable-languages=c,c++,fortran,lto --enable-graphite --enable-threads=posix --enable-libatomic --enable-libcilkrts --enable-libgomp --enable-libitm --enable-libquadmath --enable-libquadmath-support --enable-libssp --disable-win32-registry --disable-symvers --with-gnu-ld --with-gnu-as --disable-isl-version-check --enable-checking=release --without-libiconv-prefix --without-libintl-prefix --with-system-zlib --enable-linker-build-id --with-default-libstdcxx-abi=gcc4-compatible
Thread model: posix
gcc version 6.4.0 (GCC)
```

Para testar se make foi instalado corretamente, digite:

`make -v`

Algo parecido com isso deve aparecer no seu prompt de comando (não o MSYS2, o prompt de comando normal do Windows):

```
GNU Make 4.2.1
Built for x86_64-pc-msys
Copyright (C) 1988-2016 Free Software Foundation, Inc.
License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>
This is free software: you are free to change and redistribute it.
There is NO WARRANTY, to the extent permitted by law.
```
### Git

O mais recomendado é baixar o [git] pelo link indicado e rodar o programa de instalação normalmente.

Para testar se o git foi instalado corretamente, digite:

`git --version`

Algo parecido com isso deve aparecer no seu prompt de comando:

```
git version 2.15.0
```

### ARM-GCC

Faça o download do [ARM-GCC][arm-gcc-windows] e instale normalmente.

Após isso, é preciso colocar a pasta do arm-gcc nas variáveis de ambiente do Windows (o mesmo procedimento que foi feito para o MSYS2). Para isso, faça:
1. Pesquise variáveis de ambiente e clique em "Editar as variáveis de ambiente do sistema". Ou, vá para Painel de Controle > Sistema e Segurança > Sistema > Configurações avançadas do sistema.
2. Na janela que abriu, clique em "Variáveis de ambiente".
3. Na parte de "Variáveis do sistema", encontra a variável Path, selecione-a e clique em "Editar".
4. Clique em "Novo".
5. Digite o caminho para os executáveis do ARM-GCC (provavelmente C:\Program Files (x86)\GNU Tools ARM Embedded\8 2018-q4-major\bin).
6. Clique em "OK".


Para testar o arm-gcc, digite:

`arm-none-eabi-gcc --version`

Algo parecido com isso deve aparecer no seu prompt de comando:

```
arm-none-eabi-gcc.exe (GNU Tools for Arm Embedded Processors 8-2018-q4-major) 8.2.1 20181213 (release) [gcc-8-branch revision 267074]
Copyright (C) 2018 Free Software Foundation, Inc.
This is free software; see the source for copying conditions.  There is NO
warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
```

### STM32CubeMX e STM32CubeProgrammer

Para instalar o [STM32CubeMX][STM32CubeMX] e o [STM32CubeProgrammer][STM32CubeProgrammer], faça o download clicando em "Get Software" (será necessário criar uma conta no site da ST) e instale normalmente. Adicione o caminho do CubeProgrammer (normalmente C:\Program Files (x86)\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin) na variável de ambiente Path, como feito anteriormente para o msys2 e arm-gcc. Para o CubeMX, será necessário criar uma nova variável com o nome CUBE_PATH:
1. Na janela de Variáveis de Ambiente, clique em "Novo"
2. Digite CUBE_PATH como nome da variável
3. Coloque o caminho do CubeMX como valor da variável (normalmente C:\Program Files (x86)\STMicroelectronics\STM32Cube\STM32CubeMX)
4. Clique em Ok. A variável deverá aparecer na lista de variáveis de ambiente.

### VSCode

O [VS Code][vscode] é um editor bastante customizável, com extensões e um terminal integrado que facilitam muito a edição. Para baixá-lo, basta fazer o download do instalador para o seu sitema e seguir as instruções.

-----

## Linux

### GCC, Make e Git

Para instalar rode no terminal os comandos abaixo dependendo da sua distro.

#### Ubuntu
```bash
$ sudo apt update
$ sudo apt install gcc build-essential make git 
```

#### Fedora
```bash
$ sudo dnf upgrade
$ sudo dnf install gcc make git 
```

#### Arch Linux
```bash
$ sudo pacman -Syu
$ sudo pacman -S gcc make git 
```

Há outras distribuições do Linux com gerenciadores de pacotes diferentes. Com uma pesquisa rápida no Google você consegue encontrar a forma equivalente para instalar os pacotes na sua distribuição.

### ARM_GCC

Faça o download do [ARM-GCC][arm-gcc-linux] e extraia a pasta no local de sua preferência.

Após isso, é preciso colocar a pasta do arm-gcc nas variáveis de ambiente, isso depende da shell que você utiliza, mas vou deixar as instruções pro `bash` que é a shell padrão. Para isso, você irá precisar editar o arquivo `.bashrc`, localizado no seu diretório `home`, também simbolizado por `~`, ou seja o caminho absoluto, completo, até o arquivo é `~/.bashrc`. 

Então abra o arquivo com o seu editor de preferência (só falo sobre instalar o VSCode mais pra baixo, mas vou utilizar ele aqui pra editar o arquivo), fazendo:

```bash
code ~/.bashrc
```

Coloque isso no final do arquivo:
```bash
export PATH=$PATH:<local que você extraiu a pasta>/gcc-arm-none-eabi-9-2019-q4-major/bin
```
Salve o arquivo e pra aplicar as mudanças você pode fechar e abrir o terminal ou fazer:

```bash
source ~/.bashrc
```

Para testar se funcionou, feche o terminal, abra novamente e digite:

`arm-none-eabi-gcc --version`

Algo parecido com isso deve aparecer no seu terminal:

```
arm-none-eabi-gcc.exe (GNU Tools for Arm Embedded Processors 8-2018-q4-major) 8.2.1 20181213 (release) [gcc-8-branch revision 267074]
Copyright (C) 2018 Free Software Foundation, Inc.
This is free software; see the source for copying conditions.  There is NO
warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
```

### STM32CubeMX e STM32CubeProgrammer

Para instalar o [STM32CubeMX][STM32CubeMX] e o [STM32CubeProgrammer][STM32CubeProgrammer], faça o download clicando em "Get Software" (será necessário criar uma conta no site da ST) e extraia no lugar que desejar. 

Antes de continuar, é necessário instalar Java. Para isso, execute os seguintes comandos no terminal:

Para instalar o OpenJDK 8
```bash
$ sudo apt install openjdk-8-jre-headless
```

Para definir o OpenJDK 8 como o padrão do Java Runtime Engine
```bash
$ sudo update-alternatives --config java
```

Para instalar o OpenJFX
```bash
$ sudo apt purge openjfx
```
```bash
$ sudo apt install openjfx=8u161-b12-1ubuntu2 libopenjfx-jni=8u161-b12-1ubuntu2 libopenjfx-java=8u161-b12-1ubuntu2
```
```bash
$ sudo apt-mark hold openjfx libopenjfx-jni libopenjfx-java
```

Agora para instalar o CubeProgrammer, vá no direório onde você extraiu o programa e excute o arquivo de setup, dessa forma (os X são só pra substituir o número da versão):

```bash
$ sudo ./SetupSTM32CubeProgrammer-X.X.X.linux
```

Então diga as instruções na tela.

Além disso é necessário fazer mais duas coisinhas, para lidarmos com dispositivos USB.

Primeiramente é necessário instalar o libusb1.0, fazendo:
```bash
$ sudo apt install libusb-1.0.0-dev
```

Depois será necessário, colocar alguns arquivos de regras para lidar com dispositivos USB. Para isso, vá onde você instalou o CubeProgrammer pelo terminal, entre na pasta `Driver` e dentro dessa pasta, na pasta `rules`, então faça:
```bash
$ sudo cp *.* /etc/udev/rules.d
```

Para o CubeMX serão feitas algumas coisas semelhantes. Estando dentro da pasta que você extraiu, para poder executar o arquivo faça (substitua X.X.X pela versão baixada):

```bash
$ chmod 777 SetupSTM32CubeMX-X.X.X.linux
```

Depois disso, execute o arquivo fazendo:

```bash
$ sudo ./SetupSTM32CubeMX-X.X.X.linux
```

Siga as instruções na tela.

É possível que ocorra alguns erros porque o CubeMX depende de bibliotecas de
sistemas de 32 bits. Instale a biblioteca libc6-i386 para resolver o problema:

```bash
$ sudo apt install libc6-i386
```

Tente executar o arquivo novamente.

Após a instalação, crie uma variável chamada CUBE_PATH com o local de instalação do CubeMX nas configurações da shell que você utiliza. Na pasta deve conter o executável STM32CubeMX. O procedimento é similar a adicionar diretórios no PATH, mas no caso você vai criar uma variável, então você vai fazer:

Coloque isso no final do arquivo `~/.bashrc`:
```bash
export CUBE_PATH=<local de instalação>/STM32CubeMX
```

Adicione o caminho do CubeProgrammer  na variável de ambiente Path, como feito anteriormente para o arm-gcc, adicionando essa linha:

```bash
export PATH=$PATH:<local de instalação>/STM32CubeProgrammer/bin
```

### VSCode

O [VS Code][vscode] é um editor bastante customizável, com extensões e um terminal integrado que facilitam muito a edição.

#### Ubuntu
Baixe o arquivo `.deb` disponível no link a cima. Na pasta em que ele foi baixado, execute o seguinte comando:

```bash
$ sudo dpkg -i code_*.deb
```

#### Fedora
Baixe o arquivo `.rpm` disponível no link a cima. Na pasta em que ele foi baixado, execute o seguinte comando:

```bash
$ sudo rpm -Uhv code-*.rpm
```

#### Arch Linux
Execute os seguintes comandos

```bash
$ sudo pacman -Syu
$ sudo pacman -S code
```


----

## MacOS

Em breve...

Mande mensagem pra alguém que guiamos as instalações :)

----

Quaisquer dúvidas ou problemas, podem nos procurar.

[msys2]: http://www.msys2.org/
[git]: https://git-scm.com/download/win
[arm-gcc-windows]: https://developer.arm.com/-/media/Files/downloads/gnu-rm/8-2018q4/gcc-arm-none-eabi-8-2018-q4-major-win32-sha1.exe?revision=1cf82350-d608-4fdd-8b68-2e771baa13af?product=GNU%20Arm%20Embedded%20Toolchain,32-bit,,Windows,8-2018-q4-major
[arm-gcc-linux]: https://developer.arm.com/-/media/Files/downloads/gnu-rm/9-2019q4/gcc-arm-none-eabi-9-2019-q4-major-x86_64-linux.tar.bz2?revision=108bd959-44bd-4619-9c19-26187abf5225&la=en&hash=E788CE92E5DFD64B2A8C246BBA91A249CB8E2D2D
[STM32CubeMX]: https://www.st.com/b/en/development-tools/stm32cubemx.html
[STM32CubeProgrammer]: https://www.st.com/en/development-tools/stm32cubeprog.html
[vscode]: https://code.visualstudio.com/
