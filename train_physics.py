# -*- coding: utf-8 -*-
"""
列车牵引动力学核心计算引擎
严格整合多车型配置、车长阻力平滑滤波以及自动化超速监控保护(ATP)。
制动力依据《列车牵引计算规程》单位制动力：空气(常用)15、电阻22、联合60.7 N/kN。
"""
from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Tuple

from train_types import TrainType, TRAIN_TYPES, MAX_HANDLE
from route_infra import Line, BUILTIN_LINES

MAX_TRACTION_HANDLE = MAX_HANDLE  # 兼容旧命名

class OperatingMode(Enum):
    TRACTION = auto()
    COASTING = auto()
    AIR_BRAKE = auto()
    DYNAMIC_BRAKE = auto()
    COMBINED_BRAKE = auto()

    @property
    def is_braking(self) -> bool:
        return self in (OperatingMode.AIR_BRAKE, OperatingMode.DYNAMIC_BRAKE, OperatingMode.COMBINED_BRAKE)

    @property
    def chinese_name(self) -> str:
        return {
            OperatingMode.TRACTION: "牵引",
            OperatingMode.COASTING: "惰行",
            OperatingMode.AIR_BRAKE: "空气制动",
            OperatingMode.DYNAMIC_BRAKE: "电阻制动",
            OperatingMode.COMBINED_BRAKE: "联合制动",
        }[self]

MODE_COLORS = {
    OperatingMode.TRACTION: "#e74c3c",      # 牵引红
    OperatingMode.COASTING: "#2ecc71",      # 惰行绿
    OperatingMode.AIR_BRAKE: "#3498db",     # 空气蓝
    OperatingMode.DYNAMIC_BRAKE: "#f1c40f", # 电阻黄
    OperatingMode.COMBINED_BRAKE: "#95a5a6" # 联合灰
}

@dataclass
class TrainState:
    s_km: float = 0.0
    v_kmh: float = 0.0
    t_sec: float = 0.0
    mode: OperatingMode = OperatingMode.COASTING
    handle: float = 0.0
    force_kn: float = 0.0        # 当前实际轮周力 (牵引为正 / 制动为负)
    net_force_kn: float = 0.0    # 当前总合力 (kN)

    # 扩展能耗指标
    total_energy_kwh: float = 0.0
    total_fuel_kg: float = 0.0

    # 采样序列
    history_s: List[float] = field(default_factory=list)
    history_v: List[float] = field(default_factory=list)
    history_t_min: List[float] = field(default_factory=list)
    history_mode: List[OperatingMode] = field(default_factory=list)
    history_handle: List[float] = field(default_factory=list)
    history_force: List[float] = field(default_factory=list)


class TrainSimulator:
    def __init__(self, train_name: str = "DF4D普速货运", line_name: str = "甲站-乙站普速线路"):
        self.train_type: TrainType = TRAIN_TYPES[train_name]
        self.line: Line = BUILTIN_LINES[line_name]
        self.stall_steps = 0   # 连续停滞计数（低速+牵引力不足）
        self.state = TrainState()
        self.reset()

    def reset(self):
        self.state = TrainState()
        self.stall_steps = 0
        self.state.history_s = [0.0]
        self.state.history_v = [0.0]
        self.state.history_t_min = [0.0]
        self.state.history_mode = [OperatingMode.COASTING]
        self.state.history_handle = [0.0]
        self.state.history_force = [0.0]

    def change_config(self, train_name: str, line_name: str):
        self.train_type = TRAIN_TYPES[train_name]
        self.line = BUILTIN_LINES[line_name]
        self.reset()

    @property
    def is_stalled(self) -> bool:
        """列车在坡道上停滞（牵引力不足以克服坡道阻力）持续超过约 20 秒仿真时间。"""
        return self.stall_steps >= 40

    def get_max_traction_force_kn(self, v_kmh: float) -> float:
        """根据牵引特性曲线计算当前速度下最大轮周牵引力"""
        return self.train_type.get_traction_force_kn(v_kmh, MAX_HANDLE)

    def step(self, dt: float, target_mode: OperatingMode, target_handle: float) -> Tuple[TrainState, float]:
        """
        步进仿真核心积分器 (嵌入安全超速防护逻辑)
        """
        st = self.state
        v = st.v_kmh
        s = st.s_km

        # 1. 动态获取当前公里标综合速度屏障 (线路限速与车型限速交集)
        raw_line_limit = self.line.get_speed_limit_at(s)
        effective_limit = min(raw_line_limit, self.train_type.max_speed_kmh)

        mode = target_mode
        handle = max(0.0, min(MAX_HANDLE, target_handle))

        # 超速防护逻辑 (ATP) 强制触发与阻断
        if v >= effective_limit:
            mode = OperatingMode.COMBINED_BRAKE  # 强制联合惩罚制动（下坡也能压得住）
            handle = MAX_HANDLE
        elif v >= 0.95 * effective_limit:
            if mode == OperatingMode.TRACTION:
                mode = OperatingMode.COASTING  # 切断牵引强制惰行
                handle = 0.0

        # 2. 精确阻力计算（基本阻力 + 车长解耦加权坡阻/弯阻）
        w0 = self.train_type.get_basic_resistance_n_per_kn(v)
        wi = self.line.get_averaged_gradient(s, self.train_type.length_m)
        wr = self.line.get_averaged_curve_resistance(s, self.train_type.length_m)

        w_total = w0 + wi + wr
        F_res_kn = w_total * self.train_type.total_mass_t * 0.00981 # 转换至千牛力

        # 3. 各动力工况出力计算
        F_prop_kn = 0.0
        power_kw = 0.0

        if mode == OperatingMode.TRACTION:
            F_prop_kn = self.train_type.get_traction_force_kn(v, handle)
            power_kw = (F_prop_kn * max(1.0, v)) / 3.6

        elif mode == OperatingMode.AIR_BRAKE:
            F_prop_kn = -self.train_type.brake_force_kn("air", handle)

        elif mode == OperatingMode.DYNAMIC_BRAKE:
            F_prop_kn = -self.train_type.brake_force_kn("dynamic", handle)

        elif mode == OperatingMode.COMBINED_BRAKE:
            F_prop_kn = -self.train_type.brake_force_kn("combined", handle)

        # 4. 合力及加速度积分
        F_net_kn = F_prop_kn - F_res_kn
        mass_effective = self.train_type.total_mass_t * self.train_type.rotating_mass_factor
        a = F_net_kn / mass_effective  # kN/t 等价于 m/s^2

        # 速度与位移更新 (前向欧拉法)
        v_mps = v / 3.6
        v_mps_next = v_mps + a * dt
        if v_mps_next < 0:
            v_mps_next = 0.0
            a = 0.0

        v_next = v_mps_next * 3.6
        ds_km = ((v_mps + v_mps_next) / 2.0 * dt) / 1000.0
        s_next = s + ds_km

        if s_next >= self.line.total_length_km:
            s_next = self.line.total_length_km
            v_next = 0.0

        # 5. 能耗计算
        if mode == OperatingMode.TRACTION and power_kw > 0:
            energy_step = power_kw * (dt / 3600.0)
            st.total_energy_kwh += energy_step
            if "DF4D" in self.train_type.name or "重载" in self.train_type.name:
                st.total_fuel_kg += energy_step * 0.225  # 柴油机燃油消耗率 (kg/kWh)

        # 6. 坡道停滞监测：低速且仍指令牵引却无法前进
        if v_next < 0.2 and mode == OperatingMode.TRACTION and handle > 2.0:
            self.stall_steps += 1
        else:
            self.stall_steps = 0

        # 更新状态结构体
        st.s_km = s_next
        st.v_kmh = v_next
        st.t_sec += dt
        st.mode = mode
        st.handle = handle
        st.force_kn = F_prop_kn
        st.net_force_kn = F_net_kn

        st.history_s.append(s_next)
        st.history_v.append(v_next)
        st.history_t_min.append(st.t_sec / 60.0)
        st.history_mode.append(mode)
        st.history_handle.append(handle)
        st.history_force.append(F_prop_kn)

        return st, effective_limit
