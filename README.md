model\_loader library
===

Just Model Loader library in the hrplib of OpenHRP3.1 -- Not depended on CORBA, omniORB library.

# Original

[openHRP3.1](https://github.com/fkanehiro/openhrp3.git)

&nbsp;

# 1. Dependency

## 1.1. OS

Ubuntu 18.04

## 1.2. Compiler

g++ version >= 7.4.0

## 1.3. Language

c++ version >= c++11

&nbsp;

# 2. Requirements

## 2.1. LAPACK & BLAS

Not needed?

```
$ sudo apt-get install liblapack-dev libblas-dev
```

## 2.2. Eigen3

```
$ sudo apt-get install libeigen3-dev
```

## 2.3. Boost

```
$ sudo apt-get install libboost-all-dev
```

check apt-get pakcage libboost-dev version.
version >= 1.65 in my environment.

```
dpkg -s libboost-dev
```


## 2.4. libjpeg, libpng


```
$ sudo apt-get install libjpeg-dev libpng-dev
```

## 2.5. Google test

clone from https://github.com/google/googletest.git  
and compile & install.  

( Add the Path of libgtest_main.{a/so} & libgtest.{a/so} into LD\_LIBRARY\_PATH.  
  Or fix linker option -L* of Makefile to link them )

&nbsp;

# 3. Compile

Make at the top of directory.

```
$ make
```

&nbsp;

# 4. Destination

## 4.1. static library of ModelLoader 

generated in the lib/, using HRP library 

- **libmodel_loader.a**


## 4.2. App binary

generated in the bin/

- **model\_loader** : from src/main.cpp
- **unit\_test** : from the sources test/*.cpp in the test/

&nbsp;

# 5. Make Clean

"clean" option is removing object files(*.o).

```
$ make clean
```

the "clanall" option removes not only \*.o but also lib/libmodel_loader.a and binaries(bin/*).

```
$ make cleanall
```


# 6. Directory Map

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
&nbsp;

# License

Copyright (c) 2008, kdaic.
All rights reserved. This program is made available under the terms of the
Eclipse Public License v1.0 which accompanies this distribution, and is
available at http://www.eclipse.org/legal/epl-v10.html

&nbsp;

<div align="right"> regards, <br> kdaic </div>


