# NekoSportsWorldTool

![Release](https://img.shields.io/github/v/release/enn777/run?include_prereleases)
![Build](https://github.com/enn777/run/actions/workflows/release.yml/badge.svg)
![Platform](https://img.shields.io/badge/platform-Windows%20%7C%20macOS%20%7C%20Linux%20%7C%20Android-blue)
![License](https://img.shields.io/badge/license-CC%20BY--NC%204.0-lightgrey)

运动世界校园版全协议自动化工具，纯 Rust 单文件，**GUI / CLI / Android** 三端。

本仓库是 [YanamiNeko/NekoSportsWorldTool](https://github.com/YanamiNeko/NekoSportsWorldTool) 的增强分支，
在原版 v0.2.5 的协议层之上，重点做了 **Android 移植** 与 **轨迹/记录的真人化**。

> ⚠️ 仅供学习研究，请勿用于违反学校规定的场景，后果自负；请勿倒卖。

## 本分支新增

### Android 移植
- ARM64 APK：竖屏排版、原生输入法桥、中文字体、屏幕常亮策略，与桌面端共用同一套协议层
- 应用内自更新：启动静默检查（可改询问/关闭），下载 APK 调起系统安装器覆盖安装
- GitHub Actions 推送 `v*-android.*` 标签自动构建并发布 Release

### 固定真实轨迹
- 轨迹不再由打卡点随机拟合，而是沿一条**内嵌的真实人工跑步路线**行进（闭环循环 / 开放路径往返）
- 每次任务自动刚体贴合（旋转 + 等比缩放 + 平移网格搜索）到当次打卡点，过远处插入平滑绕行弯
- CLI `route-convert` 支持 GPX / KML / TCX / JSON 及服务器历史记录导入，自动去重、平滑、重采样与漂移剔除

### 记录字段对齐真人
- 与真实 App 产生的记录逐字段比对校准：点类型分布（type0 为主、无漂移/异常点）、
  `state`/`bdG` 恒定、精度半径、GPS 瞬时速度特征、起终点哨兵字段
- 学校 App 详情页的绿色有效轨迹连续，观感与真人一致

### 打卡点 / 围栏 / 分段
- 五点数据的 `position` 采用服务器下发的通过顺序（0-4），详情页正常显示打卡点
- 电子围栏实时拉取（`getGeoFenceForRun`，借鉴上游 v0.3.0），失败回退内置围栏
- OBS 对象与真人键集一致（含 extension_json），segment_json 输出约 30s 分段（配速单位分/公里）

## 功能（承自上游）

- **跑步**：一键全链（策略 → 点位 → 轨迹 → 提交 → OBS → 验证），开始时间支持随机或指定前 3 天
- **AI 运动**：任务 / 自由练习双模式，批量补签（前 60 天 × 多项目）
- **数据**：学期完成度、违规自查、排行榜（个人/班级/院系 × 日/月、室内榜、历史榜）
- **登录**：人机验证自动通过；设备身份持久化，稳定不易触发风控
- **手动海拔**：固定值或区间映射，配合本地模板分析真实记录海拔（不上传）

## 界面

![GUI](docs/screenshot.png)

## 使用

桌面端双击 exe 进入 GUI；CLI：

```text
NekoSportsWorldTool login --user <手机号> --pass <密码> --remember
NekoSportsWorldTool run                                  # 一键跑步（--dist/--pace/--altitude 可选）
NekoSportsWorldTool route-convert --in 轨迹.gpx          # 导入真实轨迹为固定路线
NekoSportsWorldTool records                              # 历史记录
NekoSportsWorldTool template --file run.gpx              # 本地分析海拔（不上传）
NekoSportsWorldTool help                                 # 全部命令
```

青龙定时任务：首次手工 `login --remember` 一次，之后定时挂 `run` 即可。

## 常见问题

- **提示未登录 / 401**：先 `login --remember`（GUI 勾选记住密码等价），会话失效会自动重登
- **10121 设备风险**：设备身份持久化，删除 `identity.json` 等于换新设备，不要频繁删
- **10603 点位限流**：点位接口 5 分钟限 3 次，程序内置 300s 缓存
- **Android 更新**：应用内检查或到 [Release](https://github.com/enn777/run/releases/latest) 下载
  `NekoSportsWorldTool-android-arm64.apk` 直接覆盖安装（勿卸载旧版，会丢应用内数据）
- **杀软误报**：未签名编译产物可能被误报，自行判断后加白

## 构建

```bash
cargo build --release                        # 桌面 GUI + CLI
cargo build --release --no-default-features  # 仅 CLI
```

Android（ARM64）构建、签名与模拟器测试见 [android/README.md](android/README.md)。
推送 `v*` 标签自动构建发布；`v*-android.*` 额外触发 Android APK 工作流。

## 致谢与许可

- 上游项目：[YanamiNeko/NekoSportsWorldTool](https://github.com/YanamiNeko/NekoSportsWorldTool)
  ——协议层、桌面端与整体架构均出自原作，本分支所有增强同样遵循其理念与许可
- 围栏接口与路线规划思路参考上游 v0.3.0（route-planner）
- 许可证：[CC BY-NC 4.0](LICENSE) © YanamiNeko（禁止商用，修改分发需署名）
