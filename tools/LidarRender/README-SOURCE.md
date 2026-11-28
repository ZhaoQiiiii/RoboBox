# marsim_render 来源说明

本目录下的 `marsim_render/` 是从 SUPER 子模块原样复制的感知渲染核心，用于 LidarRender
的 PCD → 激光点云仿真。

- 源路径：`runtime/ZeroOS/external/SUPER/src/mars_uav_sim/marsim_render/`
- SUPER commit：`93ab88904ca21bd61eb629832b5676600889ee1a`
- 复制日期：2026-11-27
- 复制范围：`include/`、`src/`、`config/`

`marsim_render/` 下的文件未做任何修改；LidarRender 的新增代码只放在
`CMakeLists.txt`、`src/`、`runtime/`、`scripts/`、`docker/` 和 `tests/` 中。
如果 SUPER 上游更新了 `marsim_render`，需要重新复制并重新执行
`diff -rq` 对比。

## License

SUPER 以 GNU Lesser General Public License v3.0（LGPL-3.0）发布，复制过来的
`marsim_render/` 文件保留原版权与许可；LidarRender 中与该部分代码结合分发的部分按
LGPL-3.0 处理。仓库根目录的 MIT LICENSE 仅覆盖 RoboKit 自有代码，不改变
`marsim_render/` 的许可。
