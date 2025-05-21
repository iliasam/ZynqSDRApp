# OpenZynqSDRApp  
Under development.
  
Linux Application of the OpenZynqSDR receiver.
See: https://habr.com/ru/articles/898490

**  Kernel module
This App is receiving data from FPGA throw kernel module "sdrdma".  
You can find its sources is the "kernel_dma_driver" folder.  
  
** Needed dependencies:  
```
sudo apt install install build-essential make cmake  
sudo apt install install pkgconf  
sudo apt install install fdk-aac-dev  
sudo apt install install libgps-dev  
sudo apt install install liblapack-dev libfftw3-dev  
sudo apt install libunwind-dev  
sudo apt install libsqlite3-dev  
sudo apt install libcurl4-openssl-dev  
sudo apt install libconfig++-dev  
sudo apt install -y libgpiod-dev gpiod  
```
  
** Building App:  
Execute from the application folder:  
```
mkdir build
cd build
cmake ..
cmake --build .
```

This code is forked from Web-888, which is forked from KiwiSDR project: https://github.com/RaspSDR/server  

