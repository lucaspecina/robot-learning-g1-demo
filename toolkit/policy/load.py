"""Load a trained LeRobot policy together with its pre- and post-processor.

The processors are not optional: a LeRobot policy does not normalize inside the
network, so feeding it raw observations gives predictions on the wrong scale.

Checkpoints trained with a newer LeRobot can carry config fields our pinned
version does not know (for example `use_peft`). Those fields are dropped and
reported instead of editing the checkpoint on disk.
"""

from __future__ import annotations

import dataclasses
import json
import tempfile
from pathlib import Path

from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.factory import get_policy_class, make_pre_post_processors
from lerobot.policies.pretrained import PreTrainedPolicy
from lerobot.processor import PolicyProcessorPipeline

CONFIG_NAME = "config.json"


@dataclasses.dataclass
class LoadedPolicy:
    policy: PreTrainedPolicy
    preprocessor: PolicyProcessorPipeline
    postprocessor: PolicyProcessorPipeline
    dropped_config_fields: list[str]


def load_config(policy_dir: Path) -> tuple[PreTrainedConfig, list[str]]:
    raw = json.loads((policy_dir / CONFIG_NAME).read_text())
    config_class = PreTrainedConfig.get_choice_class(raw["type"])
    known = {f.name for f in dataclasses.fields(config_class)} | {"type"}
    dropped = sorted(set(raw) - known)
    with tempfile.TemporaryDirectory() as tmp:
        (Path(tmp) / CONFIG_NAME).write_text(json.dumps({k: v for k, v in raw.items() if k in known}))
        config = PreTrainedConfig.from_pretrained(tmp)
    return config, dropped


def load_policy(policy_dir: Path, device: str = "cpu") -> LoadedPolicy:
    policy_dir = Path(policy_dir)
    config, dropped = load_config(policy_dir)
    config.device = device
    policy = get_policy_class(config.type).from_pretrained(policy_dir, config=config)
    policy.eval()
    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=config,
        pretrained_path=str(policy_dir),
        preprocessor_overrides={"device_processor": {"device": device}},
        postprocessor_overrides={"device_processor": {"device": device}},
    )
    return LoadedPolicy(policy, preprocessor, postprocessor, dropped)
