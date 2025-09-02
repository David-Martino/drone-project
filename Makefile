# ==== config (edit these if your paths/args differ) ====
COMM_DIR      := comms scripts
PY            ?= python3
BEACON_SCRIPT := Beacon/gs_beacon.py

BEACON_ARGS   ?=
GST           ?= gst-launch-1.0
GST_PIPELINE  := $(GST) -v udpsrc port=5004 \
  caps="application/x-rtp, media=video, encoding-name=H264, payload=96" \
  ! rtpjitterbuffer latency=60 \
  ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink sync=false

PID_DIR       := .pids
BEACON_PID    := $(PID_DIR)/beacon.pid

# Use bash for nicer traps
SHELL := /usr/bin/env bash

.PHONY: stream stop ps deps clean

# Check required tools exist
deps:
	@command -v $(PY) >/dev/null || { echo "Missing: $(PY)"; exit 1; }
	@command -v $(GST) >/dev/null || { echo "Missing: $(GST)"; exit 1; }

# Start beacon in background, then run GStreamer in foreground.
# When GStreamer exits, the beacon is stopped automatically.
stream: deps
	@set -euo pipefail; \
	mkdir -p "$(PID_DIR)"; \
	cd "$(COMM_DIR)"; \
	echo "▶ Starting beacon: $(PY) $(BEACON_SCRIPT) $(BEACON_ARGS)"; \
	$(PY) "$(BEACON_SCRIPT)" $(BEACON_ARGS) & \
	BEACON_PID=$$!; \
	echo $$BEACON_PID > "../$(BEACON_PID)"; \
	echo "✓ Beacon PID $$BEACON_PID"; \
	echo "▶ Launching GStreamer pipeline..."; \
	$(GST_PIPELINE); \
	STATUS=$$?; \
	echo "⏹ Stopping beacon $$BEACON_PID"; \
	kill $$BEACON_PID 2>/dev/null || true; \
	wait $$BEACON_PID 2>/dev/null || true; \
	rm -f "../$(BEACON_PID)"; \
	exit $$STATUS

# Manually stop background beacon and any gst-launch still running
stop:
	@set -e; \
	if [ -f "$(BEACON_PID)" ]; then \
	  PID=$$(cat "$(BEACON_PID)"); \
	  echo "⏹ Killing beacon $$PID"; \
	  kill $$PID 2>/dev/null || true; \
	  wait $$PID 2>/dev/null || true; \
	  rm -f "$(BEACON_PID)"; \
	else \
	  echo "No beacon PID file found."; \
	fi
	@pkill -f "^$(GST) " 2>/dev/null || true
	@echo "Done."

# Show PIDs if running
ps:
	@if [ -f "$(BEACON_PID)" ]; then \
	  echo "Beacon PID: $$(cat $(BEACON_PID))"; \
	else \
	  echo "Beacon not running."; \
	fi
	@pgrep -a $(notdir $(GST)) || echo "$(GST) not running."

clean: stop
	@rm -rf "$(PID_DIR)"
