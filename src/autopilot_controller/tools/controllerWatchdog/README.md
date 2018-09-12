# controllerWatchdog

## 简介

controllerWatchdog可以作为控制器入口，监听端口。当收到开启控制器命令时动控制器，当收到关闭控制器命令时关闭控制器，可以编辑同目录下的"watchdog.ini"文件定义监听端口以及实例目录和可执行文件名。

更重要的是，controllerWatchdog是一个看门狗程序，可以检测控制器的运行状态，当控制器超过一定时间没有输出则循环重启控制器，直到监听到输出为止。

## TCP消息格式

### 启动控制器：start命令
``` 
start controllerRootName firstInstanceCode lastInstanceCode
```
其中，controllerRootName定义用于生成实例名称的根名称，firstInstanceCode定义该硬件上第一个冗余实例的编码，lastInstanceCode定义该硬件上最后冗余实例的编码，同下。
例如"start 3*2_HW1_SW 1 2"，表示启动根名称为3*2_HW1_SW的控制器， 起始冗余实例的编码为1，结束冗余实例的编码为2，即在本机启动2的软件冗余实例，编号为1,2，名称为3*2_HW1_SW1和3*2_HW1_SW2.

### 关闭控制器：stop命令
``` 
stop controllerRootName firstInstanceCode lastInstanceCode
```
作为一种校验，controllerRootName要与start命令对应，否则无法关闭。

### 重启控制器：restart命令
``` 
restart controllerRootName firstInstanceCode lastInstanceCode
```
controllerRootName也需要与start命令对应

### 重置controllerWatchdog：reset命令
``` 
reset ControllerWatchdog
```
将重置ControllerWatchdog，关闭所有冗余实例，并重置计时器和计数器等。

## 部署

当需要将控制器程序部署到嵌入式设备上时（如树莓派3），推荐使用该看门狗程序，主要优势有：

- 可以设置为联网启动，监听指定端口，使用TCP包就可以启动或停止控制器；
- 可以监控控制器状态，当控制器宕机时及时重启（十分重要）。

### 配置联网启动
这里以树莓派3为例，配置它联网启动controllerWatchdog，注意不能设置为开机启动，因为LCM需要联网才能工作。树莓派使用原始的Raspbian系统，已配置好环境并成功编译程序。

### 在work下新建runControllerWatchdog.sh
runControllerWatchdog.sh用于启动解析器和控制器。
将runControllerWatchdog.sh的权限设为777，它的内容为：
```
#!/bin/sh

case $1 in
        start)
                # echo "starting controllerWatchdog"
                su - root -c "/home/pi/work/autopilot_controller/tools/controllerWatchdog/controllerWatchdog &"
                ;;
        stop)
                # echo "killall controllerWatchdog"
                su - root -c "killall -9 controllerWatchdog && killall -9 AX7controller"
                ;;
        *)
                # echo "starting controllerWatchdog without echo"
                su - root -c "/home/pi/work/autopilot_controller/tools/controllerWatchdog/controllerWatchdog >/dev/null 2>&1 &"
                ;;
esac

exit 0

```

这样使用命令“./runControllerWatchdog.sh start”可以在后台启动控制器程序；
"./runControllerWatchdog.sh"可以后台启动程序并将输出重定向到/dev/null（丢弃所有输出）；
"./runControllerWatchdog.sh stop"可以停止所有后台运行的所有相关程序。

#### 修改/etc/network/interfaces
将以太网接口eth0配置为：
```
auto eth0
iface eth0 inet dhcp
    post-up /home/pi/work/autopilot_controller/tools/controllerWatchdog/runControllerWatchdog.sh
```

这样一旦连上网，就会自动运行"runControllerWatchdog.sh"。
