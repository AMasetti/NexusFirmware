# Optimus Biped Firmware - Makefile
# Target: ESP32 Super Mini C3  (native USB-CDC, no UART chip)
#
# First-time setup:
#   make setup          # install ESP32 core + all required libraries
#
# Daily workflow:
#   make compile        # build firmware
#   make upload         # upload to connected board
#   make compile-upload # build + upload in one step
#   make monitor        # open serial monitor (115200 baud)
#   make find-port      # list connected boards and auto-detected port
#   make clean          # remove build artefacts

# ─── Board ───────────────────────────────────────────────────────────────────
# USBMode=hwcdc  → native USB-CDC (no USB-to-UART chip on Super Mini C3)
# CDCOnBoot=cdc  → serial available immediately after boot
# huge_app       → larger app partition for FreeRTOS + WiFi + WebSockets
BOARD ?= esp32:esp32:esp32c3:CDCOnBoot=cdc,PartitionScheme=huge_app

# ─── Paths ───────────────────────────────────────────────────────────────────
ARDUINO_CLI  ?= arduino-cli
FIRMWARE_DIR := $(shell pwd)
SKETCH_DIR   := $(FIRMWARE_DIR)
SKETCH_FILE  := $(FIRMWARE_DIR)/firmware.ino
BUILD_DIR    := $(FIRMWARE_DIR)/build
BINARY_FILE  := firmware.ino.bin

# ─── Port detection ──────────────────────────────────────────────────────────
# macOS: ESP32-C3 hwcdc shows up as /dev/cu.usbmodem*
# Linux: typically /dev/ttyACM0  (see sketch.yaml)
# Override: make upload PORT=/dev/cu.usbmodemXXXX
ifndef PORT
PORT := $(shell $(ARDUINO_CLI) board list 2>/dev/null \
	| awk 'NR==1{next} /ESP32/{print $$1; exit}')
endif

# ─── Baud rate for serial monitor ────────────────────────────────────────────
BAUD ?= 115200

# ─── External libraries ──────────────────────────────────────────────────────
# These are the only libraries NOT bundled with the ESP32 core.
# Everything else (Wire, WiFi, FreeRTOS, math) is included with esp32:esp32.
#
#   WebSockets  by Links2004 (arduinoWebSockets)  → WebSocketsServer.h
#
# Install with:  make install-libs   (or: make setup)

LIBS := \
    "WebSockets"

# ─── Phony targets ───────────────────────────────────────────────────────────
.PHONY: help setup install-libs compile upload compile-upload monitor find-port clean info

# ─── Default ─────────────────────────────────────────────────────────────────
help:
	@echo "Optimus Biped Firmware"
	@echo "────────────────────────────────────────"
	@echo "  make setup          Install ESP32 core + all required libraries (first-time)"
	@echo "  make install-libs   Install only the Arduino libraries"
	@echo "  make compile        Build firmware"
	@echo "  make upload         Upload to board (auto-detects port)"
	@echo "  make compile-upload Build + upload"
	@echo "  make monitor        Serial monitor at $(BAUD) baud"
	@echo "  make find-port      List connected boards"
	@echo "  make clean          Remove build artefacts"
	@echo "  make info           Show current configuration"
	@echo ""
	@echo "Override port:  make upload PORT=/dev/cu.usbmodemXXXX"
	@echo "Override baud:  make monitor BAUD=115200"

# ─── First-time setup ────────────────────────────────────────────────────────
setup: install-libs
	@echo "[setup] Done. Run 'make compile' to build."

install-libs:
	@echo "[setup] Installing ESP32 Arduino core..."
	$(ARDUINO_CLI) core install esp32:esp32
	@echo "[setup] Installing required libraries..."
	$(ARDUINO_CLI) lib install $(LIBS)
	@echo "[setup] Libraries installed."

# ─── Info ────────────────────────────────────────────────────────────────────
info:
	@echo "Board:       $(BOARD)"
	@echo "Sketch:      $(SKETCH_FILE)"
	@echo "Build dir:   $(BUILD_DIR)"
	@echo "Binary:      $(BUILD_DIR)/$(BINARY_FILE)"
	@echo "Port:        $(if $(PORT),$(PORT),not detected)"

# ─── Find port ───────────────────────────────────────────────────────────────
find-port:
	@echo "Connected boards:"
	@$(ARDUINO_CLI) board list
	@echo ""
	@if [ -n "$(PORT)" ]; then \
		echo "Auto-detected port: $(PORT)"; \
	else \
		echo "ERROR: No ESP32 board detected. Connect the board or specify PORT=..."; \
		exit 1; \
	fi

# ─── Compile ─────────────────────────────────────────────────────────────────
compile: $(SKETCH_FILE)
	@echo "[compile] Board: $(BOARD)"
	@echo "[compile] Sketch: $(SKETCH_DIR)"
	@mkdir -p "$(BUILD_DIR)"
	$(ARDUINO_CLI) compile \
		--fqbn $(BOARD) \
		--output-dir "$(BUILD_DIR)" \
		"$(SKETCH_DIR)"
	@echo "[compile] Binary: $(BUILD_DIR)/$(BINARY_FILE)"

$(SKETCH_FILE):
	@echo "ERROR: $(SKETCH_FILE) not found."; exit 1

# ─── Upload ──────────────────────────────────────────────────────────────────
upload:
	@if [ -z "$(PORT)" ]; then \
		echo "ERROR: No ESP32 board detected."; \
		echo "Connect the board or run:  make upload PORT=/dev/cu.usbmodemXXXX"; \
		$(ARDUINO_CLI) board list; \
		exit 1; \
	fi
	@if [ ! -f "$(BUILD_DIR)/$(BINARY_FILE)" ]; then \
		echo "[upload] No binary found — compiling first..."; \
		$(MAKE) compile; \
	fi
	@echo "[upload] Port: $(PORT)"
	$(ARDUINO_CLI) upload \
		--fqbn $(BOARD) \
		--port "$(PORT)" \
		--input-dir "$(BUILD_DIR)" \
		"$(SKETCH_DIR)"
	@echo "[upload] Done."

# ─── Compile + upload ────────────────────────────────────────────────────────
compile-upload: compile upload

# ─── Serial monitor ──────────────────────────────────────────────────────────
# Streams Serial.println output from the robot (CPG state, IMU estimates,
# safety events).  Ctrl-C to exit.
monitor:
	@if [ -z "$(PORT)" ]; then \
		echo "ERROR: No ESP32 board detected. Run: make find-port"; \
		exit 1; \
	fi
	@echo "[monitor] $(PORT) @ $(BAUD) baud  (Ctrl-C to exit)"
	$(ARDUINO_CLI) monitor --port "$(PORT)" --config baudrate=$(BAUD)

# ─── Clean ───────────────────────────────────────────────────────────────────
clean:
	@if [ -d "$(BUILD_DIR)" ]; then \
		rm -rf "$(BUILD_DIR)"; \
		echo "[clean] Removed $(BUILD_DIR)"; \
	else \
		echo "[clean] Nothing to remove."; \
	fi
