# Linux-RT on Raspberry Pi 3
by 杨俊杰
## Step-1：准备数据

下载要在树莓派上部署的Linux内核代码([地址](https://github.com/raspberrypi/linux/tree/rpi-4.9.y/kernel))

下载要使用的patch，需要注意与Linux内核代码的版本一致([地址](https://www.kernel.org/pub/linux/kernel/projects/rt/))

## Step-2：打patch

将上述代码的压缩包分别解压，将patch文件夹复制到Linux源码根目录，并在Linux源码根目录下执行下列命令进行打patch：
```
cat *.patch | patch p1
```

## Step-3：交叉编译Linux源码

在Ubuntu16.04下交叉编译树莓派源码，需要先安装工具：
```
git clone https://github.com/raspberrypi/tools ~/tools
```
接着配置PATH变量，注意32bit和64bit系统存在差异： 
32位系统：
```
echo PATH=\$PATH:~/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian/bin >> ~/.bashrc
source ~/.bashrc
```

64位系统：
```
echo PATH=\$PATH:~/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin >> ~/.bashrc
source ~/.bashrc
```

如果使用64位Ubuntu系统，则需要安装32位编译库支持：

```
sudo apt-get install libc6:i386
sudo apt-get install lib32stdc++6
sudo apt-get install lib32z1
```

进入LInux源码根目录，配置交叉编译设置： 
（对于树莓派3代）
```
KERNEL=kernel7
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- bcm2709_defconfig
```
然后就可以进行编译了：
```
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- zImage modules dtbs
```
如果电脑CPU拥有n个线程，为了使编译更加快速，可以使用 -jn 来设置执行交叉编译的线程数：

```
#-j4 表示使用4个线程，可以根据具体CPU配置进行设置
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- zImage modules dtbs -j4
```
注意：如果在编译时提示 warning: Clock skew detected. Your build may be imcomplete. ，需要将系统时间往后调1天，再尝试进行编译。

## Step-4：部署到SD卡

完成编译之后，可以将编译好的Linux内核部署到SD卡上。 
首先使用读卡器将SD卡连接到电脑，使用 lsblk 命令找到SD卡，得到类似下列结果：
```
sdb
    sdb1    #FAT格式（boot）
    sdb2    #EXT4格式（root）
```
然后将Linux代码部署到SD卡上：
```
sudo make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- INSTALL_MOD_PATH={$PATH_TO_ROOT} modules_install
```
然后将old kernels进行备份：
```
#{$KERNEL}取决与Ubuntu系统是32/64位，详见Step-3

sudo cp {$PATH_TO_BOOT}/{$KERNEL}.img  {$PATH_TO_BOOT}/{$KERNEL}-backup.img
sudo cp arch/arm/boot/zImage {$PATH_TO_BOOT}/{$KERNEL}.img
sudo cp arch/arm/boot/dts/*.dtb {$PATH_TO_BOOT}
sudo cp arch/arm/boot/dts/overlays/README {$PATH_TO_BOOT}
```
## Final Step

插卡，重启树莓派即可。