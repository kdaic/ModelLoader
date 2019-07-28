model\_loader library
===

Just Model Loader library in the hrplib of OpenHRP3.1 -- Not depended on CORBA, omniORB library.

# Dependency

## OS

Ubuntu 18.04

## gcc

newer version than 7.4.0


# Requirements

## LAPACK & BLAS

```
$ sudo apt-get install liblapack-dev libblas-dev
```

## Eigen3

```
$ sudo apt-get install libeigen3-dev
```

## Boost

```
$ sudo apt-get install libboost-all-dev
```

check apt-get pakcage libboost-dev version.

```
dpkg -s libboost-dev
```


## libjpeg, libpng

```
$ sudo apt-get install libjpeg-dev libpng-dev
```

## Google test

Please clone from https://github.com/google/googletest.git  
And compile & install.  

( Add the path of libgtest_main.{a,so} & libgtest.{a,so} into LD\_LIBRARY\_PATH.  
  Or fix linker option -L* of Makefile to link them )

# Compile

Make at the top of directory.

```
$ make
```

# Destination

## static library of ModelLoader 

generated in the lib/

- libmodel_loader.a

use HRP library 

## App binary

generated in the bin/

- model\_loader : from src/main.cpp
- unit\_test : from the sources test/*.cpp in the test/


# Make Clean

Clean option removes object files(*.o).

```
$ make clean
```

The CleanAll option removes not only \*.o but also lib/libmodel_loader.a and binaries(bin/*).

```
$ make cleanall
```


# Direcotry Map

I separate Include direcotry from the source (the originals are one-packed),
Because if library used at another place,
source files are not necessary, includes are just needed.

```
.
├── bin
├── lib
├── include
│   ├── hrpCollision
│   │   └── Opcode
│   │       └── Ice
│   ├── hrpModel
│   ├── hrpUtil
│   │   ├── Jpeg-6b
│   │   ├── lpng1232
│   │   └── zlib123
│   └── model_loader
├── src
│   ├── hrpCollision
│   │   └── Opcode
│   │       └── Ice
│   ├── hrpModel
│   ├── hrpUtil
│   │   ├── Jpeg-6b
│   │   ├── lpng1232
│   │   └── zlib123
│   └── model_loader
└── test
    └── model

```

<!-- ## tvmet -->

<!-- [Download & Install page](http://tvmet.sourceforge.net/build.html) -->

<!-- ``` -->
<!-- $ tar -xjvf tvmet-X.X.X.tar.bz2 -->
<!-- $ cd tvmet-X.X.X/ -->
<!-- $ ./configure -->
<!-- $ make -->
<!-- $ sudo make install -->
<!-- ``` -->




