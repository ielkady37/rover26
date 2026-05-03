# =============================================================================
#  rover26 – Makefile
#
#  Usage:
#      make install   – install system packages + Python dependencies
#      make build     – build all ROS 2 packages with colcon
#      make clean     – remove colcon artefacts (build/ install/ log/)
#      make all       – install then build
# =============================================================================

SHELL := /bin/bash
ROS_DISTRO ?= humble
ROS_SETUP   = /opt/ros/$(ROS_DISTRO)/setup.bash

.PHONY: all install build clean

# -----------------------------------------------------------------------------
all: install build

# -----------------------------------------------------------------------------
install:
	@echo ">>> [1/4] Installing system packages..."
	sudo apt-get update -qq
	sudo apt-get install -y redis-server python3-pip python3-colcon-common-extensions

	@echo ">>> [2/4] Installing Python dependencies..."
	pip install -r requirements.txt

	@echo ">>> [3/4] Setting up .env file..."
	@if [ ! -f .env.local ]; then \
		cp .env.example .env.local; \
		echo "    Created .env.local from .env.example — edit it before launching."; \
	else \
		echo "    .env.local already exists, skipping."; \
	fi

	@echo ">>> [4/4] Preparing log directories..."
	mkdir -p log/services log/nodes log/mocks log/helpers log/general
	@for d in . services nodes mocks helpers general; do \
		touch log/$$d/COLCON_IGNORE; \
		touch log/$$d/.gitkeep; \
	done
	chown -R $(USER):$(USER) log/
	chmod -R u+rwX log/

	@echo ">>> install done."

# -----------------------------------------------------------------------------
build:
	@echo ">>> [1/2] Sourcing ROS and building interfaces..."
	@bash -c "source $(ROS_SETUP) && \
	         colcon build --packages-select interfaces" \
	|| { \
		echo ""; \
		echo ">>> Build failed. Remove build/ and install/ for a clean rebuild? [y/N]"; \
		read -r ans; \
		if [ "$$ans" = "y" ] || [ "$$ans" = "Y" ]; then \
			echo ">>> Cleaning..."; \
			rm -rf build/ install/; \
			echo ">>> Retrying interfaces build..."; \
			bash -c "source $(ROS_SETUP) && colcon build --packages-select interfaces" || exit 1; \
		else \
			echo ">>> Aborted."; exit 1; \
		fi; \
	}

	@echo ">>> [2/2] Building utils and control..."
	@bash -c "source $(ROS_SETUP) && source install/setup.bash && \
	         colcon build --packages-select utils control --symlink-install" \
	|| { \
		echo ""; \
		echo ">>> Build failed. Remove build/ and install/ for a clean rebuild? [y/N]"; \
		read -r ans; \
		if [ "$$ans" = "y" ] || [ "$$ans" = "Y" ]; then \
			echo ">>> Cleaning..."; \
			rm -rf build/ install/; \
			echo ">>> Rebuilding all packages from scratch..."; \
			bash -c "source $(ROS_SETUP) && colcon build --packages-select interfaces" || exit 1; \
			bash -c "source $(ROS_SETUP) && source install/setup.bash && \
			         colcon build --packages-select utils control --symlink-install" || exit 1; \
		else \
			echo ">>> Aborted."; exit 1; \
		fi; \
	}

	@echo ""
	@echo ">>> Build complete. To use the workspace run:"
	@echo "        source install/setup.bash"

# -----------------------------------------------------------------------------
clean:
	@echo ">>> Removing build/, install/, and colcon log entries..."
	rm -rf build/ install/
	rm -rf log/build_* log/latest log/latest_build
	@echo ">>> clean done."
