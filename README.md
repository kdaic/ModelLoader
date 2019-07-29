model\_loader library
===

Just Model Loader library in the hrplib of OpenHRP3.1 -- Not depended on CORBA, omniORB library.

# Dependency

## OS

Ubuntu 18.04

## g++

version >= 7.4.0

## c++

version >= c++11

&nbsp;

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

clone from https://github.com/google/googletest.git  
and compile & install.  

( Add the Path of libgtest_main.{a/so} & libgtest.{a/so} into LD\_LIBRARY\_PATH.  
  Or fix linker option -L* of Makefile to link them )

&nbsp;

# Compile

Make at the top of directory.

```
$ make
```

&nbsp;

# Destination

## static library of ModelLoader 

generated in the lib/, using HRP library 

- libmodel_loader.a


## App binary

generated in the bin/

- model\_loader : from src/main.cpp
- unit\_test : from the sources test/*.cpp in the test/

&nbsp;

# Make Clean

Clean option removes object files(*.o).

```
$ make clean
```

The CleanAll option removes not only \*.o but also lib/libmodel_loader.a and binaries(bin/*).

```
$ make cleanall
```


# Directory Map

Include direcotry is separated from the source (the originals are one-package),
Because it is useful & portable when libraries are used for another project.
source files are not necessary, just only include files are needed.

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




