# -*- coding: utf-8 -*-
"""
自动化回归测试：验证物理引擎与自动驾驶的正确性。

运行方式：
    python tests/test_simulation.py

覆盖点（对应报告中 7.2 节的验收指标）：
1. 制动性能：常用空气制动从 80km/h 停车约需 1.5km（报告 7.2 第 3 条）。
2. 联合制动减速度应符合 60.7 N/kN 的单位制动力（报告 4.5 节表格）。
3. 自动驾驶全须运行：所有车型 x 线路组合均能到达终点、
   停车位置误差小、不超速、且在有限步数内结束（不冻结 UI）。
4. 重载列车在长大下坡道上能够保持限速不失控。
"""
from __future__ import annotations

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from train_physics import TrainSimulator, OperatingMode
from train_types import TRAIN_TYPES
from route_infra import BUILTIN_LINES
from auto_driver import precompute_reference_curve, apply_auto_control

PASS = "PASS"
FAIL = "FAIL"
results = []


def check(name: str, cond: bool, detail: str = ""):
    results.append((name, PASS if cond else FAIL, detail))
    tag = PASS if cond else FAIL
    print(f"[{tag}] {name}" + (f"  ({detail})" if detail else ""))


def test_braking_distance():
    """空气制动 80km/h -> 0 的制动距离，报告值约 1.5km。"""
    sim = TrainSimulator("DF4D普速货运", "起伏坡道线路")  # 前 3km 限速 120，可加速到 80
    # 先加速到 80
    for _ in range(6000):
        sim.step(0.5, OperatingMode.TRACTION, 8.0)
        if sim.state.v_kmh >= 80.0:
            break
    v0 = sim.state.v_kmh
    s0 = sim.state.s_km
    decels = []
    for _ in range(6000):
        _, _ = sim.step(0.5, OperatingMode.AIR_BRAKE, 8.0)
        decels.append(sim.state.v_kmh)
        if sim.state.v_kmh <= 0.01:
            break
    dist_km = sim.state.s_km - s0
    v_peak = max(decels)
    # 理论: 15 N/kN 常用制动 -> a ≈ 0.147 m/s^2 (外加基本阻力辅助)
    # 80km/h -> 0 约需 (22.2)^2 / (2*0.16) ≈ 1.54km
    check("空气制动 80km/h 停车距离约 1.1~2.2 km（报告≈1.5km）",
          1.1 <= dist_km <= 2.2, f"实测 {dist_km:.2f} km, 初速 {v0:.1f}, 峰值 {v_peak:.1f}")
    check("空气制动全程无中途停滞(v单调下降)",
          all(b <= a + 0.6 for a, b in zip(decels, decels[1:])), "")


def test_brake_deceleration_values():
    """单位制动力校验：空气 15、电阻 22、联合 60.7 N/kN（报告 4.5 节）。"""
    sim = TrainSimulator("DF4D普速货运", "甲站-乙站普速线路")
    for _ in range(1200):
        sim.step(0.5, OperatingMode.TRACTION, 8.0)
    v_ref = 60.0
    while sim.state.v_kmh < v_ref - 0.5:
        sim.step(0.5, OperatingMode.TRACTION, 8.0)

    def measured_decel(mode, handle=8.0):
        vs = []
        for _ in range(8):  # 4 秒短窗口，避免低速段被 0 截断
            st, _ = sim.step(0.5, mode, handle)
            vs.append(st.v_kmh)
        dv = (vs[0] - vs[-1]) / 3.6
        return dv / (7 * 0.5)

    a_air = measured_decel(OperatingMode.AIR_BRAKE)
    a_dyn = measured_decel(OperatingMode.DYNAMIC_BRAKE)
    a_comb = measured_decel(OperatingMode.COMBINED_BRAKE)
    # 15 N/kN -> 0.147 m/s^2（+基本阻力）;  22 -> 0.216;  60.7 -> 0.595
    check("空气制动减速度≈0.14~0.20 m/s^2 (15 N/kN)", 0.12 <= a_air <= 0.25, f"{a_air:.3f}")
    check("电阻制动减速度≈0.21~0.28 m/s^2 (22 N/kN)", 0.19 <= a_dyn <= 0.32, f"{a_dyn:.3f}")
    check("联合制动减速度≈0.55~0.70 m/s^2 (60.7 N/kN)", 0.50 <= a_comb <= 0.75, f"{a_comb:.3f}")
    check("空气<电阻<联合 制动强度排序", a_air < a_dyn < a_comb, "")


def test_auto_driving_all_combos():
    """所有车型 x 线路组合的自动驾驶全程运行。"""
    for tname in TRAIN_TYPES:
        for lname in BUILTIN_LINES:
            sim = TrainSimulator(tname, lname)
            L = sim.line.total_length_km
            limit_exceed = 0
            max_steps = 20000  # 10000 秒仿真上限，防死循环
            steps = 0
            while sim.state.s_km < L - 1e-9 and steps < max_steps:
                mode, handle = apply_auto_control(sim, 1, 80)
                st, eff = sim.step(0.5, mode, handle)
                if st.v_kmh > eff + 0.5:
                    limit_exceed += 1
                steps += 1
            s_end = sim.state.s_km
            t_min = sim.state.t_sec / 60.0
            v_end = sim.state.v_kmh
            check(f"自动驾驶可达终点: {tname} x {lname}",
                  s_end >= L - 0.001, f"终点 {s_end:.3f}/{L} km, 用时 {t_min:.1f} min, 步数 {steps}")
            check(f"  停车平稳(终点速度≈0): {tname} x {lname}",
                  v_end <= 2.0, f"终点速度 {v_end:.2f} km/h")
            check(f"  全程几乎不超速: {tname} x {lname}",
                  limit_exceed <= 5, f"超速步数 {limit_exceed}")


def test_precompute_no_hang():
    """预计算参考线在所有组合下应在有限时间内完成（重载+大坡道曾死循环）。"""
    for tname in TRAIN_TYPES:
        for lname in BUILTIN_LINES:
            import time
            t0 = time.time()
            s_arr, v_arr, st = precompute_reference_curve(tname, lname, 1)
            dt = time.time() - t0
            reached = s_arr[-1] if s_arr else 0
            check(f"预计算参考线(策略1)不卡死: {tname} x {lname}",
                  reached >= BUILTIN_LINES[lname].total_length_km - 1e-6,
                  f"终点 {reached:.2f} km, 耗时 {dt:.2f}s")


def test_heavy_haul_downhill_control():
    """重载列车在 -12‰ 长大下坡道联合制动可保持限速。"""
    sim = TrainSimulator("重载货运列车", "长大下坡线路")
    # 直接放到下坡段起点并给到限速
    sim.state.s_km = 2.5
    sim.state.v_kmh = 64.0
    overspeed = False
    for _ in range(1200):  # 600 秒，足够跑完 10km 下坡
        mode, handle = apply_auto_control(sim, 1, 80)
        st, eff = sim.step(0.5, mode, handle)
        if st.v_kmh > eff + 1.0:
            overspeed = True
            break
    check("重载列车长大下坡不失控超速", not overspeed,
          f"失控前位置 {sim.state.s_km:.2f} km, 速度 {sim.state.v_kmh:.1f}")


def test_emu_and_stall_guard():
    """高速动车组在起伏坡道（限速110-120）也能全程运行。"""
    s_arr, v_arr, st = precompute_reference_curve("高速动车组", "起伏坡道线路", 1)
    check("高速动车组 x 起伏坡道 参考线到达终点",
          s_arr[-1] >= BUILTIN_LINES["起伏坡道线路"].total_length_km - 1e-6,
          f"终点 {s_arr[-1]:.2f} km")


def main():
    test_braking_distance()
    test_brake_deceleration_values()
    test_precompute_no_hang()
    test_auto_driving_all_combos()
    test_heavy_haul_downhill_control()
    test_emu_and_stall_guard()

    n_fail = sum(1 for _, r, _ in results if r == FAIL)
    print(f"\n===== 汇总: {len(results) - n_fail} 通过 / {n_fail} 失败 =====")
    sys.exit(1 if n_fail else 0)


if __name__ == "__main__":
    main()
