# macOS / Linux GCC 构建 + OpenOCD (CMSIS-DAP) 烧录

与 Keil 工程并行存在，互不影响：Keil 仍用 `mdk/infantry.uvprojx` + `core/startup_stm32f427xx.s` + RVDS 端口。

## 依赖

```sh
brew install --cask gcc-arm-embedded   # arm-none-eabi-gcc (已验证 Arm GNU Toolchain 15.2.Rel1)
brew install cmake ninja open-ocd      # 已验证 OpenOCD 0.12.0
```

## 编译

```sh
cmake --preset default        # -O0，与 Keil 目标一致 (输出 build/)
cmake --build build           # -> build/infantry.elf / .hex / .bin / .map
cmake --preset release && cmake --build build-release   # -O2
```

源文件列表和 include 路径在 configure 时**直接从 `mdk/infantry.uvprojx` 读取**：
新增 `.c` 仍然只需加进 Keil 工程（CMake 会因 uvprojx 变化自动重新 configure）。

## 烧录 (OpenOCD + CMSIS-DAP, SWD)

```sh
cmake --build build -t flash  # program + verify + reset
cmake --build build -t reset
cmake --build build -t erase  # 整片擦除
# 等价手动命令:
openocd -f cmake/openocd-cmsis-dap.cfg -c "program build/infantry.elf verify reset exit"
```

多个探针时在 `-f` 前加 `-c "cmsis_dap_serial <序列号>"`；连不上可降低 `adapter speed`，
或在接了 NRST 时启用 cfg 中的 `reset_config` 行。

## 与 Keil 构建的差异 (GCC 专用文件)

| 文件 | 作用 |
|---|---|
| `cmake/startup_stm32f427xx_gcc.s` | 由 Keil 启动文件中断向量表生成，名字/顺序 1:1 |
| `cmake/STM32F427IITx_FLASH.ld` | 内存布局同 Keil (2M Flash / 192K RAM / 64K CCM 不默认使用)，栈 0x400 堆 0x1000 |
| `library/freertos/portable/GCC/ARM_CM4F/` | FreeRTOS **V9.0.0** 官方 GCC 端口（仓库内 RVDS 端口与官方 V9.0.0 一致） |
| `cmake/gcc_syscalls.c` | newlib `_write` → `usart.c` 里的 `fputc`，printf 仍输出到 USART6 |
| `library/atom/sys/sys.c` | `__asm` 嵌入汇编函数增加 `#else` GCC 版本 (ARMCC 分支未改) |

- 使用完整 newlib（Keil 未用 MicroLIB），`printf("%f")` 可用。
- `-fpermissive`：GCC ≥14 把隐式声明 / int-指针转换等改为错误，ARMCC 只是警告。现有警告未修。
- 枚举大小：arm-none-eabi-gcc 默认 short enums，与 ARMCC 一致（协议结构体布局不变）。
