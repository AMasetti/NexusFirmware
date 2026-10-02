# NexusFirmware — root Makefile
#
#   make setup           first-time: ESP32 core + libs, secrets.h, git hook
#   make compile         build firmware/            (main gait firmware)
#   make flash           build + upload firmware/
#   make rl-compile      build firmware-rl/         (WiFi RL inference firmware)
#   make rl-flash        build + upload firmware-rl/
#   make rl-runner MODEL=path/to/best_model.zip     stream policy actions to the robot
#
# Board and serial overrides (PORT, BAUD, BOARD) pass through to firmware/Makefile.

.PHONY: help setup hooks secrets \
        compile flash monitor flash-monitor clean find-port info \
        rl-compile rl-flash rl-monitor rl-clean rl-runner

ROOT    := $(shell pwd)
FW_DIR  := $(ROOT)/firmware
RL_DIR  := $(ROOT)/firmware-rl
PYTHON  ?= python3

# firmware-rl has no Makefile of its own: reuse firmware/Makefile with the RL sketch.
RL_MAKE := $(MAKE) -C $(RL_DIR) -f $(FW_DIR)/Makefile \
           SKETCH_FILE=$(RL_DIR)/firmware-rl.ino BINARY_FILE=firmware-rl.ino.bin

help:
	@echo ""
	@echo "  NexusFirmware"
	@echo "  ════════════════════════════════════════════════"
	@echo "  make setup            ESP32 core + libs, secrets.h, git hook (first time)"
	@echo "  make secrets          create secrets.h from the example (firmware + firmware-rl)"
	@echo "  make hooks            enable the commit-msg hook"
	@echo ""
	@echo "  Gait firmware (firmware/)"
	@echo "    make compile        build"
	@echo "    make flash          build + upload"
	@echo "    make monitor        serial monitor (Ctrl-C to exit)"
	@echo "    make flash-monitor  build + upload + monitor"
	@echo "    make clean          remove build artefacts"
	@echo "    make find-port      list connected boards"
	@echo ""
	@echo "  RL firmware (firmware-rl/)"
	@echo "    make rl-compile     build"
	@echo "    make rl-flash       build + upload"
	@echo "    make rl-monitor     serial monitor"
	@echo "    make rl-clean       remove build artefacts"
	@echo "    make rl-runner MODEL=<best_model.zip> [HOST=optimus-rl.local]"
	@echo ""
	@echo "  Override port:  make flash PORT=/dev/cu.usbmodemXXXX"
	@echo ""

# ─── Setup ───────────────────────────────────────────────────────────────────

setup: hooks secrets
	$(MAKE) -C $(FW_DIR) install-libs LIBS='"WebSockets" "ArduinoJson"'
	@echo "[setup] Done. Fill in secrets.h, then: make flash"

hooks:
	@git -C $(ROOT) config core.hooksPath .githooks && echo "[hooks] NexusFirmware → .githooks"

secrets:
	@for d in $(FW_DIR) $(RL_DIR); do \
		f=$$d/include/secrets.h; \
		if [ -f "$$f" ]; then \
			echo "[secrets] $$f already exists — edit it directly if needed."; \
		else \
			cp "$$f.example" "$$f" && echo "[secrets] Created $$f — fill in WIFI_SSID, WIFI_PASSWORD, WS_TOKEN."; \
		fi; \
	done

# ─── Gait firmware ───────────────────────────────────────────────────────────

compile:
	$(MAKE) -C $(FW_DIR) compile

flash:
	$(MAKE) -C $(FW_DIR) compile-upload

monitor:
	$(MAKE) -C $(FW_DIR) monitor

flash-monitor: flash monitor

clean:
	$(MAKE) -C $(FW_DIR) clean

find-port:
	$(MAKE) -C $(FW_DIR) find-port

info:
	$(MAKE) -C $(FW_DIR) info

# ─── RL firmware ─────────────────────────────────────────────────────────────

rl-compile:
	$(RL_MAKE) compile

rl-flash:
	$(RL_MAKE) compile-upload

rl-monitor:
	$(RL_MAKE) monitor

rl-clean:
	$(RL_MAKE) clean

HOST ?= optimus-rl.local

rl-runner:
	@if [ -z "$(MODEL)" ]; then echo "Usage: make rl-runner MODEL=path/to/best_model.zip [HOST=...]"; exit 1; fi
	$(PYTHON) $(ROOT)/tools/rl_runner.py --model $(MODEL) --host $(HOST)
