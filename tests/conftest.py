# SPDX-FileCopyrightText: 2026 Mappalink
#
# SPDX-License-Identifier: MIT

"""
Test-wide fixtures.
"""

from __future__ import annotations

import asyncio

import pytest


@pytest.fixture(autouse=True)
def _fast_asyncio_sleep(monkeypatch):
    """Short-circuit asyncio.sleep to keep tests fast while still yielding the loop.
    """

    original_sleep = asyncio.sleep

    async def _sleep_stub(delay, *args, **kwargs):  # type: ignore[unused-argument]
        # Yield control once so background tasks can run, but don't actually delay.
        await original_sleep(0)

    monkeypatch.setattr(asyncio, "sleep", _sleep_stub)
    yield
