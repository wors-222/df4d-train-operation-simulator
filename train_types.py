# -*- coding: utf-8 -*-
"""
车型参数与阻力计算模块
严格遵循《列车牵引计算规程》(TB/T 1407) 标准，支持普速货运、高速动车组和重载列车。
"""
from __future__ import annotations
from dataclasses import dataclass


@dataclass
class TrainType:
    name: str
    loco_mass_t: float          # 机车质量 (t)
    wagon_mass_t: float         # 货车/客车总重 (t)
    total_mass_t: float         # 全列总质量 (t)
    max_power_kw: float         # 最大功率 (kW)
    max_traction_kn: float      # 最大牵引力 (kN)
    continuous_speed_kmh: float    # 持续速度 (km/h)
    max_speed_kmh: float        # 车型最高安全限速 (km/h)
    rotating_mass_factor: float # 回转质量系数 (1 + gamma)
    length_m: float             # 列车总长度 (m)
    brake_ratio_modifier: float # 制动率修正系数 (重载列车降低20%即0.8，其余为1.0)
    # 单位制动力 (N/kN)，取值依据《列车牵引计算规程》常用制动/电阻制动典型值：
    # 空气制动(常用) 15、电阻制动 22、联合制动 60.7
    air_brake_n_per_kn: float = 15.0
    dynamic_brake_n_per_kn: float = 22.0
    combined_brake_n_per_kn: float = 60.7
    # 有功电流显示用的满级电流 (A)，教学简化模型
    max_active_current_a: float = 5000.0

    def get_basic_resistance_n_per_kn(self, v_kmh: float) -> float:
        """
        根据《列车牵引计算规程》计算基本单位阻力 (N/kN)
        （教学简化：机车与车辆阻力按质量加权后的综合公式）
        """
        v = v_kmh
        if "高速动车组" in self.name:
            # 动车组标准阻力公式示例
            return 1.2 + 0.0035 * v + 0.00016 * (v ** 2)
        elif "重载" in self.name:
            # 重载组合列车标准货车阻力公式
            return 0.92 + 0.004 * v + 0.0001 * (v ** 2)
        else:
            # DF4D 普速货运经典机客/机货联合阻力公式
            return 1.3 + 0.005 * v + 0.0002 * (v ** 2)

    def get_traction_force_kn(self, v_kmh: float, handle: float) -> float:
        """
        指定速度与手柄级位下的实际轮周牵引力 (kN)。
        f_max(v) = min(起动/持续牵引力限制, 功率限制 3.6*P/v)，低速恒力矩、高速恒功率，
        曲线连续无跳变。
        """
        handle = max(0.0, min(handle, MAX_HANDLE))
        v = max(1.0, v_kmh)
        f_power_limit = (3.6 * self.max_power_kw) / v
        f_max = min(self.max_traction_kn, f_power_limit)
        return f_max * (handle / MAX_HANDLE)

    def brake_force_kn(self, brake_mode: str, handle: float) -> float:
        """
        指定制动方式与制动级位下的制动力 (kN)。
        brake_mode: "air" / "dynamic" / "combined"
        """
        handle = max(0.0, min(handle, MAX_HANDLE))
        w_brake = {
            "air": self.air_brake_n_per_kn,
            "dynamic": self.dynamic_brake_n_per_kn,
            "combined": self.combined_brake_n_per_kn,
        }[brake_mode]
        w_brake *= self.brake_ratio_modifier
        # 单位制动力(N/kN) x 全列重量(kN) -> 制动力(kN)
        return w_brake * self.total_mass_t * 0.00981 * (handle / MAX_HANDLE)


MAX_HANDLE = 8.0

# 内置3种标准车型定义
TRAIN_TYPES = {
    "DF4D普速货运": TrainType(
        name="DF4D普速货运",
        loco_mass_t=138.0,
        wagon_mass_t=3000.0,
        total_mass_t=3138.0,
        max_power_kw=2320.0,
        max_traction_kn=480.0,
        continuous_speed_kmh=32.0,
        max_speed_kmh=100.0,
        rotating_mass_factor=1.06,
        length_m=650.0,
        brake_ratio_modifier=1.0,
        air_brake_n_per_kn=15.0,
        dynamic_brake_n_per_kn=22.0,
        combined_brake_n_per_kn=60.7,
        max_active_current_a=5000.0,
    ),
    "高速动车组": TrainType(
        name="高速动车组",
        loco_mass_t=500.0,
        wagon_mass_t=0.0,
        total_mass_t=500.0,
        max_power_kw=8800.0,
        max_traction_kn=300.0,
        continuous_speed_kmh=145.0,
        max_speed_kmh=350.0,
        rotating_mass_factor=1.10,
        length_m=200.0,
        brake_ratio_modifier=1.0,
        air_brake_n_per_kn=15.0,
        dynamic_brake_n_per_kn=22.0,
        combined_brake_n_per_kn=60.7,
        max_active_current_a=1200.0,
    ),
    "重载货运列车": TrainType(
        name="重载货运列车",
        loco_mass_t=276.0,  # 双机DF4D重联
        wagon_mass_t=10000.0,
        total_mass_t=10276.0,
        max_power_kw=4640.0, # 双机总功率
        max_traction_kn=960.0,
        continuous_speed_kmh=28.0,
        max_speed_kmh=80.0,
        rotating_mass_factor=1.05,
        length_m=800.0,
        brake_ratio_modifier=0.8,  # 规程要求：重载列车制动率降低20%
        air_brake_n_per_kn=15.0,
        dynamic_brake_n_per_kn=22.0,
        combined_brake_n_per_kn=60.7,
        max_active_current_a=10000.0,
    )
}
