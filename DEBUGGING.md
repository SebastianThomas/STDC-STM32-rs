# Debugging with cargo-embed and GDB

This project is configured to debug with `cargo embed` using `Embed.toml`.

## 1) Start probe-rs session

```bash
cargo embed
```

With the current `Embed.toml`, this will:

- build and flash the firmware,
- reset the target,
- halt after reset,
- open a GDB server on `127.0.0.1:1337`.

## 2) Connect GDB from a second terminal

```bash
arm-none-eabi-gdb target/thumbv7em-none-eabihf/debug/stdc_stm32_rs
```

Then in the GDB prompt:

```gdb
target extended-remote 127.0.0.1:1337
monitor reset halt
load
break main
continue
```

## Notes

- If `arm-none-eabi-gdb` is not installed, use `gdb-multiarch` if available.
- If no probe is found, check USB/probe connection and permissions first.
