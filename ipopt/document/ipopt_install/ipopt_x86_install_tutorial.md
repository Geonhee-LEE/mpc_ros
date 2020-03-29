# How to install Ipopt on x86 environment  

## Install CPPAD & Fortran  
```
sudo apt-get install cppad gfortran  
```

## Get ipopt source code    
Download Ipopt-3.12.8.tgz from https://www.coin-or.org/download/source/Ipopt/  
Untar the package and cd into the "Ipopt-3.12.8" folder({$CUSTOM_PATH})  
  
## Step by step download the libraries
 
```
cd {$CUSTOM_PATH}/Ipopt-3.12.8/ThirdParty/Blas  
 ./get.Blas    
 cd ../Lapack  
 ./get.Lapack  
 cd ../Mumps  
 ./get.Mumps  
 cd ../Metis  
 ./get.Metis  
```

```
cd {$CUSTOM_PATH}/Ipopt-3.12.8  
mkdir build  && cd build  
../configure  
make -j4  
make install  
```

## Copy install files into specific directory 
```
cd CUSTOM_PATH/Ipopt-3.12.8/build  
sudo cp -a include/* /usr/include/.  
sudo cp -a lib/* /usr/lib/.  
```
