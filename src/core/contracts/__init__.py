"""Message contract helpers for dict-based module ports."""

from .messages import (
    ContractError,
    CURRENT_SCHEMA_VERSION,
    ValidationIssue,
    assert_valid_message,
    validate_message,
)

__all__ = [
    "CURRENT_SCHEMA_VERSION",
    "ContractError",
    "ValidationIssue",
    "assert_valid_message",
    "validate_message",
]
