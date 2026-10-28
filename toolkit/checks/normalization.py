"""Check that a dataset and a trained policy share normalization statistics.

A LeRobot policy does not normalize inside the network: its preprocessor scales
every input with the mean and standard deviation stored at training time. If
the dataset used for evaluation has different statistics, the comparison is
still computable but describes a different data distribution, so the gap must
be measured before trusting any metric.

Usage:
    python -m toolkit.checks.normalization \
        --dataset-root data/datasets/local/pick_mameluco \
        --policy-dir data/checkpoints/act_cheap_v1/pretrained_model
"""

from __future__ import annotations

import dataclasses
import json
from pathlib import Path

import numpy as np
import tyro
from safetensors.numpy import load_file

from toolkit.dataset.stats import subset_stats

POLICY_STATS_FILE = "policy_preprocessor_step_3_normalizer_processor.safetensors"
# Camera inputs are left out on purpose: by default LeRobot normalizes them with
# ImageNet statistics (use_imagenet_stats), not with the dataset's own.
COMPARED_FEATURES = ("observation.state", "action")


@dataclasses.dataclass(frozen=True)
class FeatureGap:
    feature: str
    max_mean_shift: float
    """Largest |dataset mean - policy mean| over dimensions, in policy standard deviations."""
    worst_dim: int
    min_std_ratio: float
    max_std_ratio: float
    """Range of dataset std / policy std over dimensions; 1.0 means identical spread."""


def load_policy_stats(policy_dir: Path) -> dict[str, dict[str, np.ndarray]]:
    flat = load_file(str(policy_dir / POLICY_STATS_FILE))
    stats: dict[str, dict[str, np.ndarray]] = {}
    for key, value in flat.items():
        feature, stat = key.rsplit(".", 1)
        stats.setdefault(feature, {})[stat] = value
    return stats


def load_dataset_stats(dataset_root: Path) -> dict[str, dict[str, np.ndarray]]:
    raw = json.loads((dataset_root / "meta" / "stats.json").read_text())
    return {feature: {k: np.asarray(v) for k, v in values.items()} for feature, values in raw.items()}


def feature_gap(feature: str, dataset: dict[str, np.ndarray], policy: dict[str, np.ndarray]) -> FeatureGap:
    # Dimensions with zero spread at training time (a joint that never moved)
    # would turn any difference into infinity; eps keeps the scale readable.
    policy_std = np.maximum(policy["std"].ravel(), 1e-8)
    shift = np.abs(dataset["mean"].ravel() - policy["mean"].ravel()) / policy_std
    ratio = dataset["std"].ravel() / policy_std
    return FeatureGap(
        feature=feature,
        max_mean_shift=float(shift.max()),
        worst_dim=int(shift.argmax()),
        min_std_ratio=float(ratio.min()),
        max_std_ratio=float(ratio.max()),
    )


def compare(
    dataset_stats: dict[str, dict[str, np.ndarray]],
    policy_stats: dict[str, dict[str, np.ndarray]],
    features: tuple[str, ...] = COMPARED_FEATURES,
) -> list[FeatureGap]:
    return [feature_gap(f, dataset_stats[f], policy_stats[f]) for f in features]


@dataclasses.dataclass
class CompareArgs:
    dataset_root: Path
    policy_dir: Path
    exclude_episodes: tuple[int, ...] = ()
    """Episode indices left out, to test which subset the policy was trained on."""


def main(args: CompareArgs) -> None:
    if args.exclude_episodes:
        dataset_stats = subset_stats(args.dataset_root, set(args.exclude_episodes))
    else:
        dataset_stats = load_dataset_stats(args.dataset_root)
    policy_stats = load_policy_stats(args.policy_dir)
    print(f"Cuadros: dataset {int(dataset_stats['index']['count'][0])}, entrenamiento {int(policy_stats['index']['count'][0])}")
    for gap in compare(dataset_stats, policy_stats):
        print(f"{gap.feature}: corrimiento máximo de la media {gap.max_mean_shift:.3f} desvíos "
              f"(dimensión {gap.worst_dim}); dispersión entre {gap.min_std_ratio:.2f} y {gap.max_std_ratio:.2f} veces")


if __name__ == "__main__":
    main(tyro.cli(CompareArgs))
