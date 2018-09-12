# plotter
控制层调试工具，可以实时画出控制输出曲线，方便调参。

"rtplot.py"启动后会监听30005端口，当收到UDP包时就会输出控制器状态，并画出折线图。要使用rtplot，必须在在控制器工程中的"controller.cpp"文件controller构造函数中，将controller成员变量enableLateralLog设为true，使能控制器向rtplot发送UDP包，需用户自行启动"rtplot.py"；反之，若不使用则设为false。

