# SPDX-FileCopyrightText: 2026 Mappalink
#
# SPDX-License-Identifier: MIT

# Reference: https://github.com/astral-sh/uv-docker-example/blob/main/multistage.Dockerfile

FROM ghcr.io/astral-sh/uv:python3.12-bookworm-slim AS builder

ENV UV_COMPILE_BYTECODE=1 UV_LINK_MODE=copy
ENV UV_PYTHON_DOWNLOADS=0

WORKDIR /app

COPY pyproject.toml README.md uv.lock ./
RUN --mount=type=cache,target=/root/.cache/uv \
    uv sync --locked --no-install-project --no-dev

COPY inorbit_omron_connector/ ./inorbit_omron_connector/
RUN --mount=type=cache,target=/root/.cache/uv \
    uv sync --locked --no-dev

FROM python:3.12-slim-bookworm

ENV PYTHONUNBUFFERED=1
ENV DEFAULT_CONFIG_FILE=/config/fleet.yaml

RUN mkdir -p /app/logs

COPY --from=builder /app /app

ENV PATH="/app/.venv/bin:$PATH"

WORKDIR /app

CMD ["sh", "-c", "inorbit-omron-connector -c ${CONFIG_FILE:-${DEFAULT_CONFIG_FILE}} -id $ROBOT_ID --log-level ${LOG_LEVEL:-INFO}"]
