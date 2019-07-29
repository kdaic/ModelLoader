model\_loader library
===

Just Model Loader library in the hrplib of OpenHRP3.1 -- Not depended on CORBA, omniORB library.

Original source is **openHRP3**.

- URL : https://github.com/fkanehiro/openhrp3.git
- tag : 3.1.9

&nbsp;

# 1. Requirements

## 1.1. OS

Linux
(kernel version >= 4.18.0-24-generic)

## 1.2. Compiler

g++ version >= 7.4.0

## 1.3. Language

c++ version >= c++11

&nbsp;

# 2. Dependency

## 2.1. LAPACK & BLAS

needless?

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
version >= 1.65 is OK as far as I can see.

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

# 3. Make Compile

Make at the top of directory.

```
$ make
```

&nbsp;

# 4. Destination

## 4.1. Static library of ModelLoader

generated in the lib/, using HRP library 

- **libmodel_loader.a**


## 4.2. App binary

generated in the bin/

- **model\_loader** : from src/main.cpp
- **unit\_test** : from the sources test/*.cpp with google test.

&nbsp;

# 5. Make Clean

"clean" option is removing object files(\*.o) and emacs backup(\*~).

```
$ make clean
```

"cleanall" option is removing not only \*.o but also lib/libmodel_loader.a and binaries(bin/*).

```
$ make cleanall
```


# 6. Directory Map

Include files are separated from the source (the original's are in one-package),  
Because it is useful & portable when libraries are used for another place (project),  
source files are not necessary, just only include files are needed.  

```
.
├── bin
├── lib
├── include
│   ├── hrpCollision : all the same as original oprnhrp3/hrplib/hrpCollision
│   │   └── Opcode
│   │       └── Ice
│   ├── hrpModel     : all the same as original oprnhrp3/hrplib/hrpModel
│   ├── hrpUtil      : all the same as original oprnhrp3/hrplib/hrpUtil
│   │   ├── Jpeg-6b
│   │   ├── lpng1232
│   │   └── zlib123
│   └── model_loader : derive from original oprnhrp3/server/ModelLoader
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

# 7. License

Copyright (c) 2019, kdaic.

Software and license text is available at the top of each file.

&nbsp;

<div align="right"> regards, <br> kdaic </div>


