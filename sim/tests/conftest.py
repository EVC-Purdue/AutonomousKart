"""Shared pytest fixtures for sim tests."""
import os
import sys

# Make `sim` importable as a top-level package by inserting the repo root.
HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
if REPO not in sys.path:
    sys.path.insert(0, REPO)
