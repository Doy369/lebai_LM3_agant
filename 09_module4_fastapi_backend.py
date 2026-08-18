"""Compatibility wrapper for the original FastAPI backend command."""

from apps.web.serve import app, main

__all__ = ["app", "main"]


if __name__ == "__main__":
    main()
