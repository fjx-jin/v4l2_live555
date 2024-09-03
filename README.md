# v4l2_live555
```shell
#使用v4l2打开设备，读取输出之后使用x264编码，编码之后输出到live555上，暂时局域网测试延时有点高，大概1-2s，后续再优化
#程序默认支持多播，点播暂时只支持一个设备连接，要改成多播请修改main.cpp下的multicast
#编译
mkdir build
cmake ..
make
#使用
./rtsp_server
```
