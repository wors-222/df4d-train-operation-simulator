# -*- coding: utf-8 -*-
"""
线路基础设施模块
重构后的 Line 线路类，全面支持多线路选择、全程限速区段及整列车长范围阻力均衡计算。
"""
from __future__ import annotations
from dataclasses import dataclass, field
from typing import List, Tuple

@dataclass(frozen=True)
class GradeSegment:
    s_start_km: float
    s_end_km: float
    gradient_per_mille: float  # 坡度千分率 (上坡为正，下坡为负)

@dataclass(frozen=True)
class CurveSegment:
    s_start_km: float
    s_end_km: float
    radius_m: float

@dataclass(frozen=True)
class LimitSegment:
    s_start_km: float
    s_end_km: float
    limit_kmh: float

@dataclass(frozen=True)
class StationInfo:
    s_km: float
    name: str

@dataclass(frozen=True)
class SignalInfo:
    s_km: float
    name: str
    is_open: bool = True

@dataclass
class Line:
    name: str
    total_length_km: float
    grades: List[GradeSegment]
    curves: List[CurveSegment]
    limits: List[LimitSegment]
    stations: List[StationInfo]
    signals: List[SignalInfo]
    km_marker_base: int = 221

    def get_gradient_at(self, s_km: float) -> float:
        """单点获取当前坡度"""
        for g in self.grades:
            if g.s_start_km <= s_km <= g.s_end_km:
                return g.gradient_per_mille
        return 0.0

    def get_curve_radius_at(self, s_km: float) -> float:
        """单点获取曲线半径"""
        for c in self.curves:
            if c.s_start_km <= s_km <= c.s_end_km:
                return c.radius_m
        return float('inf')

    def get_speed_limit_at(self, s_km: float) -> float:
        """单点获取线路限速"""
        for lim in self.limits:
            if lim.s_start_km <= s_km <= lim.s_end_km:
                return lim.limit_kmh
        return 80.0

    def get_limit_ahead(self, s_km: float, dist_km: float) -> Tuple[float, float]:
        """
        预判：查询从 s_km 起前方 dist_km 范围内的最低限速及该限速区段起点。
        返回 (最低限速, 区段起点公里标)。用于自动驾驶提前减速。
        """
        s_end = s_km + dist_km
        min_limit = self.get_speed_limit_at(s_km)
        min_limit_start = s_km
        for lim in self.limits:
            if lim.s_start_km > s_end or lim.s_end_km < s_km:
                continue
            if lim.limit_kmh < min_limit:
                min_limit = lim.limit_kmh
                min_limit_start = max(lim.s_start_km, s_km)
        return min_limit, min_limit_start

    def get_averaged_gradient(self, s_head_km: float, length_m: float) -> float:
        """
        【规程专项高级功能】考虑整列车长度(L)范围内的加权平均坡度附加阻力
        """
        length_km = length_m / 1000.0
        s_tail_km = max(0.0, s_head_km - length_km)
        if s_head_km <= s_tail_km or length_km <= 0:
            return self.get_gradient_at(s_head_km)

        total_work = 0.0
        for g in self.grades:
            overlap_start = max(s_tail_km, g.s_start_km)
            overlap_end = min(s_head_km, g.s_end_km)
            if overlap_start < overlap_end:
                total_work += g.gradient_per_mille * (overlap_end - overlap_start)
        
        actual_len = s_head_km - s_tail_km
        return total_work / actual_len if actual_len > 0 else 0.0

    def get_averaged_curve_resistance(self, s_head_km: float, length_m: float) -> float:
        """
        【规程专项高级功能】计算整列车通过曲线区段全过程的平均曲线附加阻力 (w_r = 600/R)
        """
        length_km = length_m / 1000.0
        s_tail_km = max(0.0, s_head_km - length_km)
        if s_head_km <= s_tail_km or length_km <= 0:
            r = self.get_curve_radius_at(s_head_km)
            return 600.0 / r if r > 0 and r != float('inf') else 0.0

        total_res_work = 0.0
        for c in self.curves:
            overlap_start = max(s_tail_km, c.s_start_km)
            overlap_end = min(s_head_km, c.s_end_km)
            if overlap_start < overlap_end and c.radius_m > 0:
                wr = 600.0 / c.radius_m
                total_res_work += wr * (overlap_end - overlap_start)

        actual_len = s_head_km - s_tail_km
        return total_res_work / actual_len if actual_len > 0 else 0.0

# 内置3条标准教学演练线路
BUILTIN_LINES = {
    "甲站-乙站普速线路": Line(
        name="甲站-乙站普速线路",
        total_length_km=10.0,
        grades=[
            GradeSegment(0.0, 1.5, 2.5),
            GradeSegment(1.5, 3.5, -4.0),
            GradeSegment(3.5, 6.0, 6.0),
            GradeSegment(6.0, 8.0, 0.0),
            GradeSegment(8.0, 10.0, -1.5)
        ],
        curves=[
            CurveSegment(1.0, 2.5, 800.0),
            CurveSegment(4.5, 7.0, 1200.0)
        ],
        limits=[
            LimitSegment(0.0, 3.0, 80.0),
            LimitSegment(3.0, 7.0, 95.0),
            LimitSegment(7.0, 10.0, 75.0)
        ],
        stations=[StationInfo(0.0, "甲站"), StationInfo(10.0, "乙站")],
        # 末位信号机为乙站进站信号（关闭/红灯），其余开放，便于教学展示红绿信号
        signals=[SignalInfo(float(i), f"X{i}", i < 10) for i in range(1, 11)],
        km_marker_base=221
    ),
    "长大下坡线路": Line(
        name="长大下坡线路",
        total_length_km=15.0,
        grades=[
            GradeSegment(0.0, 2.0, 0.0),
            GradeSegment(2.0, 12.0, -12.0),  # 连续12‰长大下坡
            GradeSegment(12.0, 15.0, 1.5)
        ],
        curves=[
            CurveSegment(3.0, 6.0, 600.0),
            CurveSegment(8.0, 11.0, 700.0)
        ],
        limits=[
            LimitSegment(0.0, 2.0, 90.0),
            LimitSegment(2.0, 12.0, 65.0),  # 下坡段严苛限速
            LimitSegment(12.0, 15.0, 80.0)
        ],
        stations=[StationInfo(0.0, "丙站"), StationInfo(15.0, "丁站")],
        signals=[SignalInfo(float(i), f"Y{i}", i < 15) for i in range(1, 16)],
        km_marker_base=501
    ),
    "起伏坡道线路": Line(
        name="起伏坡道线路",
        total_length_km=20.0,
        grades=[
            GradeSegment(0.0, 3.0, 8.0),
            GradeSegment(3.0, 7.0, -6.0),
            GradeSegment(7.0, 11.0, 9.0),
            GradeSegment(11.0, 16.0, -7.0),
            GradeSegment(16.0, 20.0, 2.0)
        ],
        curves=[
            CurveSegment(2.0, 5.0, 1000.0),
            CurveSegment(9.0, 13.0, 900.0),
            CurveSegment(15.0, 18.0, 1100.0)
        ],
        limits=[
            LimitSegment(0.0, 5.0, 120.0),
            LimitSegment(5.0, 12.0, 100.0),
            LimitSegment(12.0, 20.0, 110.0)
        ],
        stations=[StationInfo(0.0, "戊站"), StationInfo(20.0, "己站")],
        signals=[SignalInfo(float(i), f"Z{i}", i < 20) for i in range(1, 21)],
        km_marker_base=101
    )
}

def km_marker_ticks(total_len: float, step: float = 1.0) -> List[float]:
    ticks = []
    curr = 0.0
    while curr <= total_len:
        ticks.append(curr)
        curr += step
    return ticks