# How to install Ipopt on arm environment  

## Inatall CPPAD & Fortran   
$ sudo apt-get install cppad gfortran  

## Get ipopt source code   
Download Ipopt-3.12.8.tgz from https://www.coin-or.org/download/source/Ipopt/  
Untar the package and cd into the "Ipopt-3.12.8" folder  

## Step by step   
$ cd CUSTOM_PATH/Ipopt-3.12.8/ThirdParty/Blas  
$ ./get.Blas    
$ cd ../Lapack  
$ ./get.Lapack  
$ cd ../Mumps  
$ ./get.Mumps  
$ cd ../Metis  
$ ./get.Metis  
   
$ cd CUSTOM_PATH/Ipopt-3.12.8  
$ mkdir build  
$ cd build  
$ ../configure --build=arm-linux  
$ make -j4  
$ make install  
    
## Copy install files into specific directory 
$ cd CUSTOM_PATH/Ipopt-3.12.8/build  
$ sudo cp -a include/* /usr/include/.  
$ sudo cp -a lib/* /usr/lib/.  
  
## Trouble Shooting
If something wrong like: "redefinition of ‘bool CppAD::is_pod()"  
Modify the file:  
$ sudo gedit /usr/include/cppad/configure.hpp   
Change line from:  
define CPPAD_SIZE_T_NOT_UNSIGNED_INT 1  
to 
define CPPAD_SIZE_T_NOT_UNSIGNED_INT 0  
