import pytest

from core.contracts import ContractError, assert_valid_message, validate_message


def test_mission_status_contract_accepts_navigation_payload():
    assert_valid_message(
        "mission_status",
        {
            "state": "EXECUTING",
            "replan_count": 1,
            "wp_index": 2,
            "wp_total": 5,
            "speed_scale": 0.7,
            "degeneracy": "MILD",
            "ts": 10.0,
        },
    )


def test_contract_reports_missing_required_fields():
    issues = validate_message("localization_status", {"state": "TRACKING"})

    assert {issue.path for issue in issues} >= {"confidence", "degeneracy", "ts"}
    assert any(issue.code == "missing" for issue in issues)


def test_contract_reports_schema_version_mismatch():
    issues = validate_message(
        "fused_cost",
        {
            "schema_version": 999,
            "grid": [[0.0]],
            "resolution": 0.2,
            "origin": [0.0, 0.0],
            "ts": 1.0,
        },
    )

    assert any(issue.code == "version_mismatch" for issue in issues)


def test_contract_reports_illegal_values():
    issues = validate_message(
        "mission_status",
        {
            "state": "DRIVING",
            "replan_count": -1,
            "wp_index": 0,
            "wp_total": 1,
            "speed_scale": 1.5,
            "degeneracy": "NONE",
            "ts": 1.0,
        },
    )

    assert any(issue.path == "state" and issue.code == "invalid_value" for issue in issues)
    assert any(issue.path == "speed_scale" and issue.code == "out_of_range" for issue in issues)
    assert any(issue.path == "replan_count" and issue.code == "out_of_range" for issue in issues)


def test_scene_graph_contract_accepts_minimal_graph_dict():
    assert_valid_message(
        "scene_graph",
        {
            "objects": [],
            "relations": [],
            "regions": [],
            "frame_id": "map",
            "ts": 1.0,
        },
    )


def test_assert_valid_message_raises_with_issue_detail():
    with pytest.raises(ContractError, match="confidence"):
        assert_valid_message("localization_status", {"state": "LOST"})
