import numpy as np

from toolkit.validation import coherence, demonstrations, drift, signals
from toolkit.validation.criteria import Criteria
from toolkit.validation.episodes import Episode


def _episode(action: np.ndarray, state: np.ndarray | None = None, index: int = 0) -> Episode:
    return Episode(index=index, source=None, state=action.copy() if state is None else state, action=action)


def test_lag_is_recovered_from_a_delayed_state():
    t = np.linspace(0, 6, 300)
    action = np.sin(t)[:, None]
    state = np.concatenate([np.full(4, action[0, 0]), action[:-4, 0]])[:, None]
    assert coherence.best_lag(_episode(action, state), joint=0) == 4


def test_copied_action_is_detected():
    state = np.random.default_rng(0).normal(size=(50, 2))
    action = np.vstack([state[1:], state[-1:]])
    assert coherence.action_copies_state(_episode(action, state))


def test_only_the_spike_counts_as_a_jump():
    smooth = [_episode(np.sin(np.linspace(0, 3, 200))[:, None] * (1 + 0.1 * k), index=k) for k in range(20)]
    spiky_action = np.sin(np.linspace(0, 3, 200))[:, None]
    spiky_action[100] += 1.0
    episodes = smooth + [_episode(spiky_action, index=20)]
    counts = signals.jump_counts(episodes, "action", Criteria()).sum(axis=1)
    assert counts[:20].sum() == 0 and counts[20] > 0


def test_idle_ends_measure_the_still_head_and_tail():
    motion = np.concatenate([np.zeros(30), np.linspace(0, 1, 40), np.ones(20)])[:, None]
    ends = demonstrations.idle_ends(_episode(motion), arm_joints=[0], criteria=Criteria())
    assert 30 <= ends.head <= 33 and 20 <= ends.tail <= 23


def test_grasp_cycles_use_hysteresis():
    closure = np.array([0, 0.5, 0.25, 0.5, 0.1, 0.6, 0.0])
    assert demonstrations.grasp_cycles(closure, high=0.3, low=0.15) == 2


def test_change_point_finds_the_regime_switch():
    series = np.concatenate([np.full(5, 1.0), np.full(20, 0.0)]) + np.random.default_rng(1).normal(0, 0.05, 25)
    assert drift.change_point(series).split == 5
