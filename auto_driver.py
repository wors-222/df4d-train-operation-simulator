# -*- coding: utf-8 -*-
"""
双策略自动驾驶控制及最优参考曲线演算模块。

核心改进：
1. 制动距离由车型实际联合制动率推导（60.7 N/kN 单位制动力），与物理引擎一致；
2. 增加前方限速区段预判，提前惰行/制动，不闯限速；
3. 终点对标停车采用"制动目标曲线"（v_target = sqrt(2*a*s)），平稳精确停车；
4. 预计算与实时控制共用同一控制器，参考线与实际轨迹一致；
5. 全程最大步数与坡道停滞保护，杜绝 UI 冻结。
"""
from __future__ import annotations

from typing import List, Optional, Tuple

from train_physics import TrainSimulator, OperatingMode

# 制动计算安全系数：实际按 85% 的可用减速度估算制动距离，留 15% 裕量
BRAKE_SAFETY = 0.85

MAX_SIM_STEPS = 40000  # 预计算防死循环上限（约 5.5 小时仿真时间）


def effective_brake_decel(sim: TrainSimulator) -> float:
    """车型联合制动可用减速度 (m/s^2)，含重载制动率折减。"""
    return sim.train_type.combined_brake_n_per_kn * sim.train_type.brake_ratio_modifier * 0.00981


def brake_distance_km(sim: TrainSimulator, v_kmh: float) -> float:
    """当前速度下按联合制动安全减速所需的制动距离 (km)。"""
    v_mps = v_kmh / 3.6
    a = effective_brake_decel(sim) * BRAKE_SAFETY
    return (v_mps ** 2) / (2.0 * a) / 1000.0


def apply_auto_control(sim: TrainSimulator, strategy_idx: int, cruise_v_set: float = 70.0) -> Tuple[OperatingMode, float]:
    """
    自动驾驶实时决策接口：
    - 终点制动目标曲线对标停车
    - 前方限速下降预判
    - 下坡/超速趋势联合制动
    """
    st = sim.state
    s, v = st.s_km, st.v_kmh
    L = sim.line.total_length_km

    limit_now = min(sim.line.get_speed_limit_at(s), sim.train_type.max_speed_kmh)

    # ---- 终点停车制动曲线 ----
    dist_to_end = max(0.0, L - s)
    stop_brake_km = brake_distance_km(sim, v)
    if dist_to_end <= stop_brake_km * 1.6:
        a = effective_brake_decel(sim) * BRAKE_SAFETY
        v_stop_target = 3.6 * (2.0 * a * dist_to_end * 1000.0) ** 0.5
        if v > v_stop_target + 1.0:
            return OperatingMode.COMBINED_BRAKE, 8.0
        if v < v_stop_target - 1.5 and dist_to_end > 0.03:
            return OperatingMode.COASTING, 0.0
        return OperatingMode.COMBINED_BRAKE, 5.0

    # ---- 前方限速预判 ----
    lookahead_km = max(0.6, stop_brake_km * 1.5)
    min_limit_ahead, _ = sim.line.get_limit_ahead(s, lookahead_km)
    tgt_limit = min(limit_now, min_limit_ahead)

    tgt_v = tgt_limit if strategy_idx == 1 else min(tgt_limit, cruise_v_set)

    # ---- 速度调节 ----
    if v > tgt_limit + 0.5:
        # 接近/越过实际限速：先惰行，仍偏高则联合制动压制（防止下坡惰行持续加速）
        if v > tgt_limit + 2.5:
            return OperatingMode.COMBINED_BRAKE, 4.0
        return OperatingMode.COASTING, 0.0

    if v < tgt_v - 2.5:
        return OperatingMode.TRACTION, 8.0

    if v > tgt_v + 0.5:
        return OperatingMode.COASTING, 0.0

    # 巡航带内：策略1低档位保持，策略2节能惰行
    if strategy_idx == 2:
        return OperatingMode.COASTING, 0.0
    return OperatingMode.TRACTION, 4.0


def _run_full_run(train_name: str, line_name: str, strategy_idx: int,
                  cruise_v: float, dt: float = 0.5) -> TrainSimulator:
    """从起点完整跑一遍全程（用于预演演算与二分测时）。"""
    sim = TrainSimulator(train_name, line_name)
    L = sim.line.total_length_km
    steps = 0
    while sim.state.s_km < L - 1e-9 and steps < MAX_SIM_STEPS:
        mode, handle = apply_auto_control(sim, strategy_idx, cruise_v)
        sim.step(dt, mode, handle)
        steps += 1
        if sim.is_stalled:
            break  # 坡道停滞保护
    return sim


def _test_run_time(train_name: str, line_name: str, test_cruise: float) -> float:
    """粗步长快速测算给定巡航速度下的全程走行时间 (秒)。"""
    return _run_full_run(train_name, line_name, 2, test_cruise, dt=1.0).state.t_sec


def precompute_reference_curve(
    train_name: str,
    line_name: str,
    strategy_idx: int,
    scheduled_time_min: Optional[float] = None,
    cruise_v_set: Optional[float] = None,
) -> Tuple[List[float], List[float], TrainSimulator]:
    """
    【策略演算预仿真】正式上道前模拟全线运行，生成灰色理论参考曲线。
    strategy_idx: 1 - 最大能力策略；2 - 准点运行策略（二分法寻找最优巡航速度）
    scheduled_time_min 为空时按线路长度自动给定（约 1.75 min/km，10km 线 ≈ 17.5 分钟）。
    cruise_v_set 为空时策略2自行寻优。
    """
    cruise_v = None
    if strategy_idx == 2:
        sim_probe = TrainSimulator(train_name, line_name)
        L = sim_probe.line.total_length_km
        sched = scheduled_time_min if scheduled_time_min else L * 1.75

        low_v, high_v = 25.0, sim_probe.train_type.max_speed_kmh
        cruise_v = high_v  # 若连最高速都赶不上点，则退化为最大能力
        for _ in range(5):
            mid_v = (low_v + high_v) / 2.0
            t_min = _test_run_time(train_name, line_name, mid_v) / 60.0
            if t_min > sched:
                low_v = mid_v
            else:
                high_v = mid_v
                cruise_v = mid_v
        if cruise_v_set:
            cruise_v = min(cruise_v, cruise_v_set)
    else:
        cruise_v = cruise_v_set if cruise_v_set else 1e9  # 策略1不受巡航限制

    sim = _run_full_run(train_name, line_name, strategy_idx, cruise_v if cruise_v else 1e9, dt=0.5)
    # 记录参考线使用的巡航速度，供策略对比面板展示
    sim.reference_cruise_v = cruise_v if strategy_idx == 2 else None
    return sim.state.history_s, sim.state.history_v, sim
