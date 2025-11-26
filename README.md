

# NUCLEO-F446RE FreeRTOS CLI Playground

This project is a complete embedded "comeback" firmware written for the  
**STM32 NUCLEO-F446RE** board using **FreeRTOS**, **interrupt-driven UART**,  
and a **message-queue-based command line interface (CLI)**.

It is structured like a real production-quality firmware with clean layers,  
task separation, interrupt decoupling, and thread-safe UART logging.

---

## 🚀 Features

### 🧵 **FreeRTOS Tasks**
- **LEDTask**
  - Toggles LD2 (PA5) every 500 ms (heartbeat)
- **SensorTask**
  - Reads a virtual temperature sensor and logs values every 1 second
- **CLITask**
  - Consumes UART RX queue and processes commands

### 💬 **UART CLI (via USART2)**
Single-character commands:
| Command | Description |
|--------|-------------|
| `h` | Help menu |
| `t` | Toggle LED |
| `s` | Status |
| `r` | One-shot temperature read |
| `l` | Start periodic logging (TIM2 interrupt) |
| `x` | Stop periodic logging |

### 🔁 **Interrupt-Driven UART RX**
- Uses `HAL_UART_Receive_IT`
- `HAL_UART_RxCpltCallback()` pushes bytes into a FreeRTOS queue
- No heavy work inside ISR (best practice)

### 🔒 **Thread-Safe UART Logging**
- All UART prints wrapped in a mutex
- No garbled or interleaved output

### 🌡️ **Virtual Temperature Sensor Module**
- `sensor.c / sensor.h`
- Simulates temperature changes over time
- Clean API that can later be replaced with a real I²C sensor

### ⏱️ **Optional Timer-Based Logging**
- TIM2 interrupt can periodically print `LOG:` temperature values
- Activated using CLI (`l` to start, `x` to stop)

---

## 🧱 Architecture

### High-Level Flow
```
UART ISR → Queue → CLI Task → Command Handler
Sensor Task → uart_print() → Mutex → USART2
TIM2 ISR → Optional Logging → uart_print()
```

### File Structure (important parts)
```
Core/
  Src/
    main.c        ← FreeRTOS init, tasks, CLI, UART helpers
    sensor.c      ← virtual temperature sensor
  Inc/
    main.h
    sensor.h
Middlewares/
  Third_Party/
    FreeRTOS/     ← FreeRTOS sources
*.ioc             ← CubeMX configuration
```

### Tasks Overview
- **LEDTask** — simple LED heartbeat  
- **SensorTask** — periodic logging of virtual sensor  
- **CLITask** — handles UART commands (via message queue)

---

## 🛠️ Hardware Used

- STM32 NUCLEO-F446RE  
- On-board LED: **LD2 (PA5)**  
- UART: **USART2 (ST-Link Virtual COM Port)**  
- No external hardware required  

---

## 💻 Build & Run

### 1. Clone the repo
```bash
git clone https://github.com/<your-username>/<your-repo>.git
```

### 2. Import into STM32CubeIDE
- `File → Open Projects from File System`
- Select the cloned folder

### 3. Build
Click the **hammer** icon.

### 4. Flash
Click **Run/Debug** with your NUCLEO connected.

### 5. Open serial terminal
- Baud: **115200**
- 8-N-1
- ST-Link Virtual COM Port

You’ll see:

```
System boot OK
CLI Ready
```

Now type commands (`h`, `r`, `t`, etc.)

---

## 📄 CLI Example

```
h
Commands: h=help, t=toggle, s=status, r=read, l=start log, x=stop log

t
LED toggled

r
Sensor: 25.4 C

l
Logging started
LOG: 25.5 C
RTOS LOG: 25.6 C
LOG: 25.8 C
...
```

---

## 🔧 Future Extensions

- Replace virtual sensor with real I²C sensor (e.g., BME280)
- Add multi-word commands (`log start`, `log stop`, etc.)
- Add CSV logging mode
- Add SD-card logging
- Add unit tests for CLI parsing

---

## 📜 License

MIT License 

---

## 👤 Author

Developed as part of an embedded-systems comeback and practice project  
by **Karan Gandhi**.




