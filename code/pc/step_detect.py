# -*- coding: utf-8 -*-
"""阶跃（刺穿跌落）检测器 —— 纯逻辑，无 IO，可离线回放与单测。

信号形态（mock_slave 与历史实验一致）：
    静息 -> 加载上升 -> 刺穿瞬间跌落 -> 回零

判据（三步）：
  1) 基线：进入钻孔后先采集 baseline_seconds 的静息样本，得均值 mu、标准差 sigma；
  2) 加载判定：滚动峰值需高出 mu 达到 load_k * sigma_eff，才认为"已经加载"；
  3) 触发：当前值比滚动峰值回落 >= max(sigma_k * sigma_eff, min_drop)，
     且回落发生在 peak_window_seconds 的回溯窗口内（防陈旧峰值误触发）。

参数全部可在构造时覆盖；阈值与真实单位无关（相对噪声带），
现场用真实刺穿 CSV 回放微调即可。

默认值即现场验证的小信号模式：sigma_k=2.5、load_k=1.5、min_noise=0.5、
min_drop=2.0、confirm_samples=1（首个满足样本立即触发）。
注意：sigma_eff = max(sigma, min_noise)，min_noise 默认 0.5，
避免整数输出（±1 量化噪声）被当成阶跃而误触发。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

__all__ = ['StepEvent', 'StepDetector', 'detect_step']


@dataclass
class StepEvent:
    """一次阶跃（刺穿跌落）触发记录。"""
    time: float            # 触发样本时间
    value: float           # 触发样本值
    peak: float            # 触发前的滚动峰值
    peak_time: float       # 峰值出现时间
    drop: float            # 峰值 - 当前值
    threshold: float       # 本次生效的回落阈值
    baseline_mean: float   # 静息基线均值
    baseline_std: float    # 静息基线标准差
    baseline_samples: int  # 基线样本数
    elapsed: float         # 从首个样本到触发的时间


class StepDetector:
    """在线阶跃检测器：逐样本 update(t, value)，触发时返回 StepEvent。"""

    def __init__(
        self,
        baseline_seconds=1.0,
        sigma_k=2.5,
        load_k=1.5,
        min_drop=2.0,
        min_noise=0.5,
        peak_window_seconds=3.0,
        min_baseline_samples=5,
        confirm_samples=1,
    ):
        if baseline_seconds < 0:
            raise ValueError('baseline_seconds 不能为负')
        if confirm_samples < 1:
            raise ValueError('confirm_samples 至少为 1')
        self.baseline_seconds = float(baseline_seconds)
        self.sigma_k = float(sigma_k)
        self.load_k = float(load_k)
        self.min_drop = float(min_drop)
        self.min_noise = float(min_noise)
        self.peak_window_seconds = float(peak_window_seconds)
        self.min_baseline_samples = int(min_baseline_samples)
        self.confirm_samples = int(confirm_samples)

        self._t0 = None
        self._baseline = []
        self._baseline_done = False
        self._mu = 0.0
        self._sigma = 0.0
        self._peak = None
        self._peak_time = None
        self._loaded = False
        self._pending_drop = 0
        self.triggered = False

    # ---------- 内部 ----------

    def _finish_baseline(self):
        values = self._baseline
        self._mu = sum(values) / len(values)
        variance = sum((v - self._mu) ** 2 for v in values) / len(values)
        self._sigma = math.sqrt(variance)
        self._baseline_done = True

    @property
    def baseline_mean(self):
        return self._mu

    @property
    def baseline_std(self):
        return self._sigma

    @property
    def baseline_ready(self):
        return self._baseline_done

    # ---------- 主接口 ----------

    def update(self, t, value):
        """喂入一个样本；触发返回 StepEvent，否则 None。触发后不再重复返回。"""
        if self.triggered:
            return None
        value = float(value)
        if self._t0 is None:
            self._t0 = t

        # 阶段 1：采集静息基线
        if not self._baseline_done:
            self._baseline.append(value)
            if (
                len(self._baseline) >= self.min_baseline_samples
                and t - self._t0 >= self.baseline_seconds
            ):
                self._finish_baseline()
            return None

        sigma_eff = max(self._sigma, self.min_noise)

        # 阶段 2：滚动峰值（峰值过旧则重置，避免拿很久以前的峰判回落）
        if self._peak is None or value > self._peak:
            self._peak = value
            self._peak_time = t
        elif t - self._peak_time > self.peak_window_seconds:
            self._peak = value
            self._peak_time = t

        # 加载判定：峰值必须明显高出基线，防止静息噪声直接触发
        if not self._loaded:
            if self._peak - self._mu >= self.load_k * sigma_eff:
                self._loaded = True

        # 阶段 3：回落触发（严格大于阈值；confirm_samples 连续满足才触发）
        threshold = max(self.sigma_k * sigma_eff, self.min_drop)
        drop = self._peak - value
        if (
            self._loaded
            and drop > threshold
            and t - self._peak_time <= self.peak_window_seconds
        ):
            self._pending_drop += 1
            if self._pending_drop < self.confirm_samples:
                return None
            self.triggered = True
            return StepEvent(
                time=t,
                value=value,
                peak=self._peak,
                peak_time=self._peak_time,
                drop=drop,
                threshold=threshold,
                baseline_mean=self._mu,
                baseline_std=self._sigma,
                baseline_samples=len(self._baseline),
                elapsed=t - self._t0,
            )
        self._pending_drop = 0
        return None


def detect_step(samples, **kwargs):
    """离线回放：samples 为 (t, value) 序列；返回 (StepEvent|None, StepDetector)。"""
    detector = StepDetector(**kwargs)
    for t, value in samples:
        event = detector.update(t, value)
        if event is not None:
            return event, detector
    return None, detector
