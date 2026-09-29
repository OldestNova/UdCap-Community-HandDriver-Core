# UdCap 宇叠动作捕捉手套社区驱动
## 当前功能支持状态

 - [x] 枚举 USB 设备
 - [x] 设备探测
 - [x] 接收器状态管理
 - [x] 连接状态管理
 - [x] 松散架构，允许混合手套
 - [x] 数据标定
 - [x] 手指骨骼四元数自动计算
 - [x] 控制器震动
 - [x] 控制器按钮(A/B/Menu/Joy/Power)支持和虚拟按钮(Trigger/Grip/Trackpad)支持
 - [x] 配对
 - [x] 调节信道
 - [x] 关节微调
 - [x] 存取手套用户设置
 - [x] 接收器热插拔检测
 - [x] 拔出并重新插入接收器后自动恢复连接
 - [x] 可注入原始串口数据并绑定其他串口 API（手动驱动构建）
 - [ ] OTA (暂时不打算支持)
 
## 介绍
第三方社区实现的宇叠动作捕捉手套驱动，理论上以原生方式支持 Windows macOS Linux 平台。以 C++ 17 实现，提供静态链接。

正常工作时，不会在探测后继续占用与其无关的串口。

核心内存占用约 `1.2M + 50k * N`，N 为连接的手套数量

## 编译和运行
### 依赖
 * CMake
 * Eigen
 * Boost
 * libusb
 * libusbp
 * hidapi

默认构建通过 [CPM](https://github.com/cpm-cmake/CPM.cmake) 下载上述库。Windows 使用 vcpkg toolchain 时会自动识别根目录的 `vcpkg.json`；Core manifest 当前没有额外包依赖。Linux 和 macOS 可直接使用 CPM 构建。`USE_MANUAL_SERIAL_DRIVER=ON` 时不编译 SerialReborn，不需要 Boost、libusb、libusbp 或 hidapi；Core 的 Eigen、nlohmann_json 和 AR1Linear 依赖仍在。

### 编译
Linux/macOS：

```bash
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build .
```

Windows 上的示例（首次配置就要传入 toolchain）：

```powershell
cmake -S . -B build-vcpkg -DCMAKE_TOOLCHAIN_FILE=D:/Program/vcpkg/scripts/buildsystems/vcpkg.cmake -DVCPKG_TARGET_TRIPLET=x64-windows
cmake --build build-vcpkg --config Release
```

如果需要编译测试程序，请在 `cmake` 时添加 `-DBUILD_TEST_TOOLS=ON`；需要复制 hidapi.dll 时还可以添加 `-DCOPY_HIDAPI_DLL=ON`。

如果想要输出更详细的原始协议数据包，请在 `-DBUILD_TEST_TOOLS=ON` 的同时添加 `-DTEST_TOOLS_PRINT_RAW=ON` 选项。

### 手动串口驱动

使用 `-DUSE_MANUAL_SERIAL_DRIVER=ON` 构建 Core。此模式不编译接收器探测，不枚举端口，也不要求 Core 知道有多少串口。宿主为每个接收器创建一个 `PortAccessor`，绑定发送回调，并把串口收到的**原始字节流**按顺序交给它；Core 自己处理分包、协议解析和命令编码。

```cpp
#include <UdCapV1Core.h> // 手动构建会公开轻量 PortAccessor.h

auto port = std::make_shared<PortAccessor>();
ManualSerialDriverCallbacks serial;
serial.write = [&](const uint8_t* bytes, std::size_t size) -> std::size_t {
    return mySerial.write(bytes, size);
};
serial.setBaudRate = [&](int baud) { mySerial.begin(baud); }; // 可选
port->bindSerialCallbacks(std::move(serial));

auto core = std::make_unique<UdCapV1Core>(port);
port->notifyConnectionState(PORT_ACCESSOR_CONNECTION_STATE_CONNECTED);

// 在串口接收任务中调用，可以按任何分段大小送入；不要直接从中断处理程序调用。
port->injectReceivedData(rxBytes, rxLength);

// 硬件断开时通知 Core；重连后再通知 CONNECTED。
port->notifyConnectionState(PORT_ACCESSOR_CONNECTION_STATE_DISCONNECTED);
```

`open`、`close`、`setBaudRate` 和 `waitForPendingWrites` 回调均可选，`write` 用于发送 Core 的命令，返回实际接受的字节数。若宿主异步排队发送，应实现 `waitForPendingWrites` 以保证关闭前命令发送完毕。`injectReceivedData` 会在调用它的任务线程上同步执行接收回调；宿主须保证注入期间 `core` 和 `port` 仍存活。此模式不提供 `UdCapProbe` API。单片机目标还需具备本项目其他 C++17 依赖及线程支持；目前只在桌面工具链验证，尚未在 ESP32 工具链编译。

### 手指归一化算法

`UdCapV1Core::setAlgorithm(UdCapV1Algorithm)` 可在运行时选择算法：

```cpp
core->setAlgorithm(UdCapV1Algorithm::COUMMUNITY_V1);
```

枚举值为 `DEFAULT = 0`、`ORIGINAL_V1 = 1`、`COUMMUNITY_V1 = 20`（按 API 约定保留此拼写）。`DEFAULT` 当前使用 `ORIGINAL_V1`；未调用 `setAlgorithm` 时，输出与原实现一致。`getAlgorithm()` 返回设置的枚举值。

`COUMMUNITY_V1` 在完成首次三姿势标定后，持续更新每路原始传感器的工作上下界。之后自然伸直、弯曲和张指即可提供校准依据，无需点击重新标定，也不要求动作顺序或每次在端点停留。工作上下界不被锁定在最初标定附近，可跟随长时间累积的双向零点漂移及量程变化。

- 端点分别跟踪：接近已有端点并稳定约 0.18 秒时即可更新；另一端超过 15 秒未被确认时，优先平移整个量程以补偿零点，不再要求 45 秒内两端都出现。
- 量程重新确认：持续的往返动作产生极值，两端分别经过至少两次一致的访问后更新量程；超过当前两倍的扩张需要三次。连续动作不需要停顿，短脉冲不能作为完整往返动作。
- 明显收缩采用更保守的条件：原量程至少 180 秒未被确认，新极值持续至少 30 秒且两端分别访问四次。移除了初始量程 25% 的固定下限；有效工作量程仍至少为 10 个原始单位。
- 5 帧中位数用于学习边界，工作边界按 0.25 秒时间常数平滑过渡；该滤波不替换原始动作数据。静止时不通过边界衰减推测新量程。非有限值或超过 2 秒的收包间隔会清空未确认观测，保留已应用边界，不积分离线时间。

切换算法或重新完成标定时重置在线状态；重复设置同一算法不会重置。社区模式不会改写保存的手动标定快照。

社区模式只更新 12 路原始传感器的工作上下界；之后与当前官方计算方式共用低/高端归一化、弱通道禁用规则及按设备型号选择的 AR1 手指角度计算。这样拇指和横向张指也沿用相同的角度模型。未识别型号时使用官方的 AA 默认模型。

自动测试连续输入 60 分钟的合成帧，覆盖零点偏移 ±1200（初始量程 800）、量程逐渐变为 0.3～2 倍、端点相隔两分钟、连续不停顿的动作、只回到伸直端、长时间静止、噪声及断线恢复。另检查 20 分钟静止后通过自然往返重新获取明显平移、扩大或缩小的范围，以及 20～200 FPS 下的时间一致性。

仅凭原始传感器不能严格区分温漂与极慢动作，也不能区分“长期只做半幅动作”和“传感器量程缩小”。短时间半幅动作不会触发明显的量程收缩；长期重复固定的小范围动作可能在上述保守条件满足后被当成新量程。日常偶尔充分伸直、握拳及张指有助于持续校正，无需特定顺序。一直保持未知中间姿势时无法保证自动恢复正确姿态；静止时优先保留已有映射。`f8`、`f11`、`f14` 的高端来自张指（`Protract`），仅握拳与伸手不能保证这些通道的量程准确。本实现补偿零点与范围，不构成完整的温度或迟滞物理模型；合成测试通过仍需真实设备长期使用验证。

### 测试
启用测试后会生成 `UdCapCommunityDriverCoreTest` 应用程序，该程序会自动枚举连接的接收器，并获取其序列号。连接手套后会自动开始标定过程，请跟随输出做出动作。
 
## 说明
宇叠为上海宇叠智能科技有限公司，UdCap 是其动作捕捉手套产品的名称。

其余库内容以 MIT 协议开源。
# Glove telemetry

The current receiver's `CMD_BATTERY` packet carries a big-endian raw battery
measurement followed by a signed RSSI byte. Core uses the battery thresholds
from the installed official v0.1.8.6 driver and reports both the five-step
level and the raw measurement through `UdCapV1MCUPacket` and `getBattery()`.
`getRssiDbm()` returns the latest signed RSSI. Both getters return
`std::nullopt` until telemetry arrives or after the glove disconnects.

## Response curves and bone offsets

`getHandResponse()` / `setHandResponse(settings)` expose 21 per-hand response
channels, indexed like `CMD_ANGLE.result[0..20]`. Start with
`defaultHandResponse()`. Each `HandResponseCurve` has `outputMin`, `outputMax`,
`balance` (0..1, default 0.5), and `offset` (degrees, -45..45).

The input ranges and two-segment interpolation match official v0.1.8.6,
including reversing balance for negative input ranges and extrapolating
outside the endpoints. Channel 11 (middle splay) only accepts a constant
offset. Settings are validated and replaced atomically under the Core lock.
They run after AR1/smoothing and before angle, skeleton and controller outputs,
with both ORIGINAL_V1 and COUMMUNITY_V1. Default settings preserve the pose.

```cpp
auto response = core->getHandResponse();
response[2].balance = 0.65; // Thumb proximal bend
response[2].offset = -5;    // Applied in angle space, before protocol axes
core->setHandResponse(response);
core->savePref();           // Also persists the full local XYZ bone offsets
```

`setHandOffset(HandRotation)` uses local Unity XYZ axes, with all 45 values
validated as finite degrees in [-45,45]. The existing scalar
`setHandOffset(HandBone,float)` still sets that bone's local X component.
`CMD_SKELETON_QUATERNION` already includes these offsets. `CMD_ANGLE` carries
the same frame's `angleBoneOffset` and `angleHand` snapshot, so angle-based
senders can apply their own axis mapping without querying Core in a callback.
`anglesWithHandOffset()` projects the measured bend/splay axes for scalar
protocols. Additional twist axes cannot be represented by VRChat OSC's avatar
parameters; full skeleton protocols can carry them. Do not apply offsets a
second time to the skeleton quaternion.

`savePref` / `loadPref` store response settings in `handResponse`. Files without
that field use identity curves. `UdCapHandResponseTest` checks endpoints,
midpoints, extrapolation, invalid settings and optional official DLL fixtures.
