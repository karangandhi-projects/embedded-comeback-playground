# 📟 STM32 FreeRTOS CLI Playground  
### **Advanced Embedded Firmware • NUCLEO-F446RE • FreeRTOS • UART • CLI • Logging System**

This project is a fully-featured embedded firmware playground built on the **STM32 NUCLEO-F446RE**.  
It demonstrates **real-world, production-style firmware patterns**, including:

- FreeRTOS multithreading  
- Interrupt-driven UART RX  
- Multi-word command line interface (CLI)  
- Log message queue (decouples logging from user input)  
- Thread-safe UART printing with mutex  
- Virtual temperature sensor module  
- Firmware versioning  
- ANSI terminal control (clear screen)  

This is designed as a **professional portfolio project** to demonstrate modern embedded-firmware architecture and RTOS design.

---

# 🚀 Features

### ✔ **FreeRTOS Tasks**
| Task | Purpose |
|------|---------|
| **LEDTask** | Heartbeat LED (500ms) |
| **SensorTask** | Reads a virtual sensor every 1s, pushes logs to queue |
| **CLITask** | Owns UART, handles CLI, prints logs |

---

### ✔ **Command Line Interface (Multi-Word)**
Supports both long and short commands:

| Command | Meaning |
|---------|---------|
| `help` | Show available commands |
| `status` | System health |
| `version` | Firmware version + build date/time |
| `clear` / `cls` | Clear terminal screen |
| `led toggle` / `t` | Toggle LED |
| `sensor read` / `r` | Read temperature sensor |
| `log start` / `l` | Start periodic logging |
| `log stop` / `x` | Stop periodic logging |

---

# ✔ **Logging System (Non-Blocking)**
A **dedicated log queue** ensures logging never interrupts typing.

**Producers:**  
- TIM2 interrupt callback  
- SensorTask  

**Consumer:**  
- CLITask (sole owner of UART TX)

This mirrors real RTOS firmware where logs flow from many contexts but only one task owns the UART/debug port.

---

# ✔ **Thread-Safe UART**
All UART writes go through:

- `uart_print()`  
- `uart_write_char()`  

Protected with a FreeRTOS mutex when scheduler is active.

---

# ✔ **Versioning Built In**
Version info displayed via `version` command:

```
Firmware v1.8.0
Built on Feb 2025 at HH:MM:SS
```

Version history embedded at top of `main.c`.

---

# ✔ **Virtual Sensor Module**
Located in:

```
Core/Src/sensor.c
Core/Inc/sensor.h
```

Simulates temperature readings (`float` °C) and demonstrates how a real I2C/SPI driver integrates.

Upgrade path: **BME280 real sensor via I2C** (planned v1.9.0).

---

# 🧩 System Architecture

```
+-----------------------+
|       TIM2 ISR        |
|  (periodic logging)   |
+----------+------------+
           |
           v (non-blocking enqueue)
   +------------------------+
   |      logQueue          |
   |   (LogMessage_t)       |
   +------------+-----------+
                |
                v
+---------------+------------------+
|             CLITask              |
|     - consumes logQueue          |
|     - prints logs                |
|     - processes CLI commands     |
|     - owns UART TX               |
+---------------+------------------+
                |
                v
        +---------------+
        |   UART2 TX    |
        +---------------+
```

---

# 🧵 FreeRTOS Task Overview

```
                 +------------------+
                 |     LEDTask      |
                 |  toggle LED/500ms|
                 +------------------+

+------------+          +----------------+
|  TIM2 ISR  |  --->    |  logQueue      |
|  produces  |          +----------------+
+------------+                  |
                                |
+-----------------+      +-----------------+
|   SensorTask    | ---> |     CLITask     |
| produces logs   |      | processes logs  |
+-----------------+      | & CLI commands  |
                          +----------------+
```

---

# 🖥 CLI Example Session

```
System boot OK

CLI Ready
> help
Commands:
  help                - show this help
  status              - show system status
  version             - show firmware version
  clear / cls         - clear terminal screen
  led toggle          - toggle LED
  sensor read         - read sensor once
  log start           - enable timer-based logging
  log stop            - disable timer-based logging

> version
Firmware v1.8.0
Built on Feb 2025 at 20:41:13

> log start
Logging started

LOG: 23.1 C
LOG: 23.2 C
LOG: 23.3 C

> clear
> 
```

---

# 📦 Project Structure

```
.
├── Core
│   ├── Inc
│   │   ├── main.h
│   │   └── sensor.h
│   └── Src
│       ├── main.c
│       └── sensor.c
├── Drivers
├── README.md
└── *.ioc (CubeMX configuration)
```

---

# 📚 Firmware Version History

| Version | Changes |
|--------|---------|
| **v1.0.0** | Basic LED timer |
| **v1.1.0** | UART support |
| **v1.2.0** | Simple single-char CLI |
| **v1.3.0** | Virtual sensor module |
| **v1.4.0** | FreeRTOS tasks added |
| **v1.5.0** | UART mutex + thread-safe prints |
| **v1.6.0** | Multi-word CLI |
| **v1.7.0** | Log queue (decoupled logging) |
| **v1.8.0** | Added `clear` command |
| **v1.8.1** | Remove SensorTask logging |

Next planned:

- **v1.9.0 — Real I2C BME280 sensor**
- **v2.0.0 — UART DMA circular buffer + log levels**

---

# 🛠 Build & Flash

1. Open in **STM32CubeIDE**
2. Build (`Ctrl + B`)
3. Flash (`Run → Debug`)

UART Settings:

- **115200 baud**
- 8-N-1
- No flow control

---

# 🤝 Contributing / Feature Ideas

- Add BME280 I2C sensor  
- Add ring buffer or UART DMA idle detection  
- Add unit tests (Unity + Ceedling)  
- Add configuration system + persistent settings  
- Add CLI history + tab completion  

---

# 📄 License

MIT License.
