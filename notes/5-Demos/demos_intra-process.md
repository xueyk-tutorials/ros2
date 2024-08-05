# 进程间通信

​		ROS2提供了intra-process机制，使得同一个进程中的不同node之间的消息通信实现0拷贝。对于图像、点云这类消息来说，大大提高了通信速率，避免了额外性能开销。

​		如果只有一个生产者，而存在多个消费者，那么消息只能通过拷贝方式传给前面的消费者，通过0拷贝的方式传给最后一个消费者。

参考：

[Setting up efficient intra-process communication — ROS 2 Documentation: Foxy documentation](https://docs.ros.org/en/foxy/Tutorials/Demos/Intra-Process-Communication.html)

