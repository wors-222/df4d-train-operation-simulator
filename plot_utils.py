# -*- coding: utf-8 -*-
"""绘图辅助：历史降采样、工况着色线段（减轻 Matplotlib 负担）。"""

from __future__ import annotations

from typing import List, Sequence, Tuple, TypeVar

T = TypeVar("T")


def downsample_history(
    s: Sequence[float],
    v: Sequence[float],
    modes: Sequence[T],
    max_points: int,
) -> Tuple[List[float], List[float], List[T]]:
    """
    将历史数据降采样到 max_points 以内，保留首尾与工况切换点。
    不改变仿真数据，仅用于显示。
    """
    n = len(s)
    if n <= max_points:
        return list(s), list(v), list(modes)

    keep = {0, n - 1}
    for i in range(1, n):
        if modes[i] != modes[i - 1]:
            keep.add(i)
            keep.add(i - 1)

    # 均匀补足采样点
    remaining = max_points - len(keep)
    if remaining > 0:
        step = max(1, (n - 1) // remaining)
        for i in range(0, n, step):
            keep.add(i)

    indices = sorted(keep)
    if len(indices) > max_points:
        stride = max(1, len(indices) // max_points)
        indices = indices[::stride]
        indices[0] = 0
        indices[-1] = n - 1

    return (
        [s[i] for i in indices],
        [v[i] for i in indices],
        [modes[i] for i in indices],
    )


def downsample_xy(
    x: Sequence[float],
    y: Sequence[float],
    max_points: int,
) -> Tuple[List[float], List[float]]:
    """简单均匀降采样（用于时间曲线等）。"""
    n = len(x)
    if n <= max_points:
        return list(x), list(y)
    step = max(1, (n - 1) // (max_points - 1))
    indices = list(range(0, n, step))
    if indices[-1] != n - 1:
        indices.append(n - 1)
    return [x[i] for i in indices], [y[i] for i in indices]


def build_colored_line_segments(
    s: Sequence[float],
    v: Sequence[float],
    modes: Sequence[T],
    color_map: dict,
) -> Tuple[List[List[Tuple[float, float]]], List[str]]:
    """生成 LineCollection 用的线段列表与颜色列表（合并同色相邻段）。"""
    if len(s) < 2:
        return [], []

    segments: List[List[Tuple[float, float]]] = []
    colors: List[str] = []

    cur_color = color_map.get(modes[1], "#64748b")
    cur_seg = [(float(s[0]), float(v[0])), (float(s[1]), float(v[1]))]

    for i in range(2, len(s)):
        c = color_map.get(modes[i], "#64748b")
        pt = (float(s[i]), float(v[i]))
        if c == cur_color:
            cur_seg.append(pt)
        else:
            segments.append(cur_seg)
            colors.append(cur_color)
            cur_color = c
            cur_seg = [cur_seg[-1], pt]

    segments.append(cur_seg)
    colors.append(cur_color)
    return segments, colors
