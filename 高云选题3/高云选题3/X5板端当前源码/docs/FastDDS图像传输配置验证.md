# Fast DDS 图像传输配置与验证

2026-10-11。配置适用于当前 X5 的 Fast DDS 2.6.10，目标是减少传输大图像时的小共享内存池压力，保持原始图像内容与 ROS QoS。

## 已完成的验证

在隔离 ROS 域 83 中运行独立发布者与订阅者，板上原有完整程序同时运行。使用 1280×1088 NV12，每帧 2088960 字节，发布目标 30 FPS；逐帧检查 CRC。隔离测试只传输测试图像，没有发布运动或喷水命令。

| 发布者配置 | 订阅者配置 | 发送帧数 | 收到帧数 | 接收率 | CRC 错误 |
|---|---|---:|---:|---:|---:|
| 默认 | 默认 | 450 | 348 | 77.33% | 0 |
| 64 MiB SHM 段 | 64 MiB SHM 段 | 450 | 450 | 100% | 0 |
| 64 MiB SHM 段 | 原默认配置 | 360 | 360 | 100% | 0 |

这些结果证明：当前板上真实解析器能够加载该 XML，在这些测试窗口和负载下接收更完整，且新发布者与默认接收者兼容。它们不等于“YOLO 已达到 30 FPS”，也不能证明无限时长、任何读者数量或任意网络条件均不丢帧。整机的识别 FPS、OCR 用时、图像帧龄、CPU 和内存需看完整服务的独立对照测量。

## 配置放在哪里

- `config/fastdds_large_images.xml`：共享内存段 64 MiB、SHM 最大消息 8 MiB，保留 UDPv4 传输。
- `tools/image_transport_env.sh`：启动前设置正确的 XML 环境变量。
- `tools/run_mipi_camera.sh`、`tools/run_mipi_vision.sh`：已经调用上述环境脚本。默认只给这两个大图像生产服务应用大池；其他仅收发小消息的任务节点仍用原来的配置。

环境脚本仅在 `RMW_IMPLEMENTATION` 未设置或为 `rmw_fastrtps_cpp`，并且用户没有自行指定 `FASTRTPS_DEFAULT_PROFILES_FILE`／`FASTDDS_DEFAULT_PROFILES_FILE` 时生效。它不会覆盖用户已有的传输文件，不切换 ROS 域或其他 RMW 实现。

节点启动后的实际环境可查 `FASTRTPS_DEFAULT_PROFILES_FILE`，目标应为本工作区的 `config/fastdds_large_images.xml`。配置在 DDS 参加者创建时读取，改环境变量不会自动改变已经运行的进程。

## 如何关闭这项优化

在启动上述脚本前设置：

```bash
export RESCUE_LARGE_IMAGE_SHM=0
```

然后重新启动对应的相机／视觉进程。该开关阻止环境脚本注入本配置；如果当前 shell 或服务本来已经显式设置了配置文件变量，脚本会继续尊重它们，需同时撤销自己设置的文件变量才能回到默认值。

这是给维护者切换测试用的开关，不要求正常使用时手动执行；避免另开重复的相机驱动或同名视觉节点。修改 systemd 服务时，应把变量加入对应服务环境，再由服务管理器重启该服务。

## 哪些内容保持一致

传输配置没有定义 publisher／subscriber QoS，不改订阅深度、可靠性或任务时效规则。源 NV12 仍是 1280×1088；OCR 复用通道仍为完整 BGR8 无损图像；预览 JPEG 质量与 YOLO 网络输入也不由此文件修改。没有启用 `RMW_FASTRTPS_USE_QOS_FROM_XML`。

64 MiB 是每个参加者的 SHM 段，不是所有程序共用的总额度。Fast DDS 会在初始化时实际触及这块内存；因此没有把所有小消息节点也改成大池，后续需继续观察 `/dev/shm` 使用量和进程内存。

## 为什么不能称为零拷贝或整帧无分片

2.6.10 源码中，`NetworkFactory::RegisterTransport()` 把所有注册传输的最大消息大小取最小值；`RTPSParticipantImpl::getMaxMessageSize()` 再用它限制发送大小。保留 UDPv4 时，UDP 默认约 65500 字节的限制仍会导致大图像分片，即使 SHM 自身配置 8 MiB。

因此这项优化主要扩大在途数据池，改善当前实测的传输完整率。SHM 发送路径仍执行 `memcpy`；ROS 序列化和 Python 接收也仍有成本。不能把这项改动描述为“整帧零拷贝”。

## 官方资料与 XML 校验

- [Fast DDS 2.6.10 SHM 文档](https://fast-dds.docs.eprosima.com/en/v2.6.10/fastdds/transport/shared_memory/shared_memory.html)：默认段容量、容量过小的数据丢失风险、同机消息机制。
- [官方传输 XML 参数](https://fast-dds.docs.eprosima.com/en/v2.6.10/fastdds/xml_configuration/transports.html)：`segment_size`、`maxMessageSize` 等字段。
- [官方环境变量](https://fast-dds.docs.eprosima.com/en/v2.6.10/fastdds/env_vars/env_vars.html)：本版本使用 `FASTRTPS_DEFAULT_PROFILES_FILE`。
- [NetworkFactory 源码](https://github.com/eProsima/Fast-DDS/blob/v2.6.10/src/cpp/rtps/network/NetworkFactory.cpp)和 [RTPSParticipantImpl 源码](https://github.com/eProsima/Fast-DDS/blob/v2.6.10/src/cpp/rtps/participant/RTPSParticipantImpl.cpp)：消息大小取最小值的实际实现。
- [SHM 实现](https://github.com/eProsima/Fast-DDS/blob/v2.6.10/src/cpp/rtps/transport/shared_mem/SharedMemTransport.cpp)：段初始化、内存触及和实际拷贝。

字段已核对该版本实际 XML 解析器，并已通过板上运行验证。官方 XSD 自身包含未定义 `int32/uint32/uint64` 类型，以及 `userTransports` 子元素与解析器不一致的缺陷；本地只修正验证用 XSD 来完成严格校验，没有修改板上 Fast DDS。原始和修正后的 XSD 位于 `reports/fastdds_v2610_schema/`；完整源码核对记录见 `reports/fastdds_large_images_review_20261011.md`。
