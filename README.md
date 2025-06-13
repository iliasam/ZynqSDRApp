# OpenZynqSDRApp  
Under development.
  
Linux Application of the OpenZynqSDR receiver - WEB SDR, based on "Antminer S9" board.  
See this article (Russian): https://habr.com/ru/articles/898490  
See also Zynq hardware description and FPGA sources: https://github.com/iliasam/OpenZynqSDR_HW  
This code is forked from Web-888 project, which is forked from KiwiSDR project: https://github.com/RaspSDR/server and https://github.com/jks-prv/KiwiSDR  

**Configuration files**  
Application is need to have configuration files. They are placed at the "config" folder of this repo.  
Path to the config folder at the target computer (where is the App is executed) is set at CMakeLists.txt:  
DIR_CFG="/home/ubuntu/sdr/config"  
DIR_SAMPLES="/home/ubuntu/sdr/config/samples"  
  
**Kernel module**  
This App is receiving data from FPGA throw kernel module "sdrdma".  
You can find its sources in the "kernel_dma_driver" folder.  
  
**Needed dependencies:**  
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
  
**Building App at the target computer:**  
Execute from the application folder:  
```
mkdir build
cd build
cmake ..
cmake --build .
```

SDR design:  
<img src="https://github.com/iliasam/OpenZynqSDRApp/blob/develop/SDR_Design.png">  

**Proxy**  
KiwiSDR and Web-888 are using frp (https://github.com/fatedier/frp) as a reverce proxy. I never tested my code with frp, so it is possible that there may be problems.  



