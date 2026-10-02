import json

from toolkit.policy.load import load_config


def test_unknown_config_fields_are_dropped_and_reported(tmp_path):
    (tmp_path / "config.json").write_text(json.dumps({"type": "act", "chunk_size": 48, "n_action_steps": 48, "use_peft": False}))
    config, dropped = load_config(tmp_path)
    assert dropped == ["use_peft"]
    assert config.chunk_size == 48
