# Tests for src/streaming/streaming/constants.py
import sys
import importlib
import pytest


def _import_constants(monkeypatch, url_value):
    """Force a fresh import of constants with the given env var state."""
    sys.modules.pop("streaming.constants", None)
    if url_value is None:
        monkeypatch.delenv("WEBRTC_SIGNALING_URL", raising=False)
    else:
        monkeypatch.setenv("WEBRTC_SIGNALING_URL", url_value)
    return importlib.import_module("streaming.constants")


def test_valid_url(monkeypatch):
    mod = _import_constants(monkeypatch, "ws://example.com")
    assert mod.WEBRTC_SIGNALING_URL == "ws://example.com"


def test_empty_string_raises(monkeypatch):
    with pytest.raises(ValueError):
        _import_constants(monkeypatch, "")


def test_missing_env_var_raises(monkeypatch):
    with pytest.raises(ValueError):
        _import_constants(monkeypatch, None)
