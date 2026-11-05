# -*- coding: utf-8 -*-
"""
DF4D 列车运行操纵模拟仿真系统 — 主界面（完善版）

布局（对照课程报告第五章）：
    上部: 主运行曲线图（速度-距离工况着色 + 时间-距离双Y轴 + 限速线 + 参考线 + 信号机 + 档位标注）
    中部: 线路纵断面图（坡道/曲线/车站/公里标 + 列车实时位置）
    下图: 牵引力特性曲线 | 有功电流特性曲线（8档曲线族 + 实时工作点）
    下面板: 运行配置 | 操纵台（手动/自动） | 仿真控制 | 12项实时参数仪表盘
"""
from __future__ import annotations

import os
import sys
import csv
from datetime import datetime

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.collections import LineCollection
from matplotlib.figure import Figure

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
    QFileDialog,
    QDialog,
    QTableWidget,
    QTableWidgetItem,
    QHeaderView,
    QButtonGroup,
)

from train_physics import TrainSimulator, OperatingMode, MODE_COLORS
from auto_driver import apply_auto_control, precompute_reference_curve
from route_infra import BUILTIN_LINES, km_marker_ticks
from train_types import TRAIN_TYPES
from font_config import get_qfont, setup_matplotlib_chinese
from plot_utils import build_colored_line_segments, downsample_history, downsample_xy

MAX_PLOT_POINTS = 600       # 主曲线绘制降采样上限
ALL_MODES = list(OperatingMode)
MODE_BUTTON_TEXT = {
    OperatingMode.TRACTION: "牵引",
    OperatingMode.COASTING: "惰行",
    OperatingMode.AIR_BRAKE: "空气制动",
    OperatingMode.DYNAMIC_BRAKE: "电阻制动",
    OperatingMode.COMBINED_BRAKE: "联合制动",
}


class StrategyComparisonDialog(QDialog):
    """双策略运行能耗考核对比弹窗"""

    def __init__(self, sim1, sim2, parent=None):
        super().__init__(parent)
        self.setWindowTitle("自动驾驶策略运行结果对比面板")
        self.resize(760, 540)
        self.setFont(get_qfont(10))

        layout = QVBoxLayout(self)

        table = QTableWidget(6, 3)
        table.setHorizontalHeaderLabels(["考核评价指标", "最大能力策略 (策略1)", "准点运行策略 (策略2)"])
        table.verticalHeader().setVisible(False)

        def metrics_of(sim):
            st = sim.state
            t_min = st.t_sec / 60.0
            s_max = st.history_s[-1] if st.history_s else 1.0
            v_tech = s_max / (st.t_sec / 3600.0) if st.t_sec > 0 else 0
            v_peak = max(st.history_v) if st.history_v else 0
            cruise = getattr(sim, "reference_cruise_v", None)
            return t_min, st.total_energy_kwh, st.total_fuel_kg, v_tech, v_peak, cruise

        t1, e1, f1, vt1, vp1, c1 = metrics_of(sim1)
        t2, e2, f2, vt2, vp2, c2 = metrics_of(sim2)

        rows = [
            ("区间运行总时间 (分)", t1, t2, "time"),
            ("总有功电能消耗 (kWh)", e1, e2, "energy"),
            ("总燃油消耗质量 (kg)", f1, f2, "energy"),
            ("平均技术速度 (km/h)", vt1, vt2, ""),
            ("全线最高车速 (km/h)", vp1, vp2, ""),
            ("准点策略寻优巡航速度 (km/h)", c1 if c1 else 0, c2 if c2 else 0, ""),
        ]

        for row, (lbl, v1, v2, m_type) in enumerate(rows):
            table.setItem(row, 0, QTableWidgetItem(lbl))
            table.setItem(row, 1, QTableWidgetItem(f"{v1:.2f}"))
            table.setItem(row, 2, QTableWidgetItem(f"{v2:.2f}"))

            # 动态渲染最优颜色背景
            item1 = table.item(row, 1)
            item2 = table.item(row, 2)
            if m_type == "time":
                if v1 < v2:
                    item1.setBackground(Qt.GlobalColor.green)
                else:
                    item2.setBackground(Qt.GlobalColor.green)
            elif m_type == "energy":
                if v1 < v2:
                    item1.setBackground(Qt.GlobalColor.cyan)
                else:
                    item2.setBackground(Qt.GlobalColor.cyan)

        table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        layout.addWidget(QLabel("<b>牵引计算规程考核指标对比：</b>"))
        layout.addWidget(table)

        # 曲线快速对比视窗
        fig = Figure(figsize=(6, 3), dpi=100)
        canvas = FigureCanvasQTAgg(fig)
        ax = fig.add_subplot(111)
        st1, st2 = sim1.state, sim2.state
        ax.plot(st1.history_s, st1.history_v, color="#e74c3c", linestyle="-", label="最大能力策略")
        ax.plot(st2.history_s, st2.history_v, color="#3498db", linestyle="--", label="准点运行策略")
        ax.set_xlabel("位置 (km)")
        ax.set_ylabel("速度 (km/h)")
        ax.grid(True, linestyle=":")
        ax.legend(fontsize=9)
        layout.addWidget(canvas)

        btn = QPushButton("确定并返回主界面")
        btn.clicked.connect(self.accept)
        layout.addWidget(btn)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("DF4D 列车运行操纵模拟仿真系统")
        self.resize(1400, 920)

        # 初始化动力学核心引擎
        self.sim = TrainSimulator("DF4D普速货运", "甲站-乙站普速线路")

        # 定时器配置 (每 50ms 一帧，每帧步进 dt=0.5s 物理时间 x 倍速)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._on_simulation_step)
        self.sim_dt = 0.5
        self.is_paused = False
        self.is_finished = False

        # 手动操纵状态
        self.manual_mode = OperatingMode.COASTING
        self.manual_handle = 0.0
        self._handle_texts = []
        self._annot_scan_idx = 0
        self._region_t0_sec = 0.0

        # 区间时分计算（经过中间车站时重置）
        self._station_ptr = 1

        # 预计算参考线数据
        self.ref_s = []
        self.ref_v = []

        self._init_ui_layout()
        self._on_drive_mode_changed()  # 初始为自动模式：禁用手动操纵台
        self._rebuild_static_plots()

    # ------------------------------------------------------------------
    # UI 构建
    # ------------------------------------------------------------------
    def _init_ui_layout(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(8)

        # ---------------- 上/中/下: Matplotlib 画布 ----------------
        self.fig = Figure(figsize=(12, 7))
        self.canvas = FigureCanvasQTAgg(self.fig)
        main_layout.addWidget(self.canvas, stretch=62)

        gs = self.fig.add_gridspec(3, 2, height_ratios=[4.0, 2.0, 2.4], hspace=0.55, wspace=0.18)
        self.ax_main = self.fig.add_subplot(gs[0, :])
        self.ax_time = self.ax_main.twinx()          # 时间-距离双Y轴
        self.ax_profile = self.fig.add_subplot(gs[1, :], sharex=self.ax_main)
        self.ax_traction = self.fig.add_subplot(gs[2, 0])
        self.ax_current = self.fig.add_subplot(gs[2, 1])

        # ---------------- 下部: 交互与监控面板 ----------------
        bottom_layout = QHBoxLayout()
        main_layout.addLayout(bottom_layout, stretch=38)

        # 1. 运行配置区
        config_group = QGroupBox("运行配置（车型 / 线路 / 模式）")
        config_group.setFont(get_qfont(9, bold=True))
        config_grid = QGridLayout(config_group)

        config_grid.addWidget(QLabel("运行车型:"), 0, 0)
        self.combo_train = QComboBox()
        self.combo_train.addItems(list(TRAIN_TYPES.keys()))
        self.combo_train.currentTextChanged.connect(self._on_config_changed)
        config_grid.addWidget(self.combo_train, 0, 1)

        config_grid.addWidget(QLabel("仿真线路:"), 1, 0)
        self.combo_line = QComboBox()
        self.combo_line.addItems(list(BUILTIN_LINES.keys()))
        self.combo_line.currentTextChanged.connect(self._on_config_changed)
        config_grid.addWidget(self.combo_line, 1, 1)

        config_grid.addWidget(QLabel("操纵模式:"), 2, 0)
        self.combo_mode = QComboBox()
        self.combo_mode.addItem("自动操纵 (自动驾驶仪)", "auto")
        self.combo_mode.addItem("手动操纵 (司机操纵台)", "manual")
        self.combo_mode.currentIndexChanged.connect(self._on_drive_mode_changed)
        config_grid.addWidget(self.combo_mode, 2, 1)

        config_grid.addWidget(QLabel("自动策略:"), 3, 0)
        self.combo_strategy = QComboBox()
        self.combo_strategy.addItem("策略1: 最大能力策略", 1)
        self.combo_strategy.addItem("策略2: 准点节能策略 (二分寻优)", 2)
        self.combo_strategy.currentIndexChanged.connect(self._on_strategy_changed)
        config_grid.addWidget(self.combo_strategy, 3, 1)

        config_grid.addWidget(QLabel("巡航限速 (km/h):"), 4, 0)
        self.slider_cruise = QSlider(Qt.Orientation.Horizontal)
        self.slider_cruise.setRange(30, 100)
        self.slider_cruise.setValue(80)
        self.slider_cruise.valueChanged.connect(self._on_cruise_slider_moved)
        self.slider_cruise.sliderReleased.connect(self._on_cruise_slider_released)
        self.lbl_cruise_val = QLabel("80 km/h")
        config_grid.addWidget(self.slider_cruise, 4, 1)
        config_grid.addWidget(self.lbl_cruise_val, 4, 2)

        config_grid.addWidget(QLabel("仿真倍速:"), 5, 0)
        self.combo_speed = QComboBox()
        self.combo_speed.addItem("1× (实时)", 1)
        self.combo_speed.addItem("10×", 10)
        self.combo_speed.addItem("20×", 20)
        config_grid.addWidget(self.combo_speed, 5, 1)

        bottom_layout.addWidget(config_group, stretch=34)

        # 2. 手动操纵台
        manual_group = QGroupBox("司机操纵台（手动模式）")
        manual_group.setFont(get_qfont(9, bold=True))
        manual_vbox = QVBoxLayout(manual_group)

        mode_btn_grid = QGridLayout()
        self.mode_buttons = {}
        self.mode_btn_group = QButtonGroup(self)
        self.mode_btn_group.setExclusive(True)
        for i, mode in enumerate(ALL_MODES):
            btn = QPushButton(MODE_BUTTON_TEXT[mode])
            btn.setCheckable(True)
            btn.setStyleSheet(
                f"QPushButton:checked {{ background-color: {MODE_COLORS[mode]}; color: white; font-weight: bold; }}"
            )
            btn.clicked.connect(lambda _=False, m=mode: self._on_manual_mode_selected(m))
            self.mode_btn_group.addButton(btn)
            self.mode_buttons[mode] = btn
            mode_btn_grid.addWidget(btn, i // 2, i % 2)
        self.mode_buttons[OperatingMode.COASTING].setChecked(True)
        manual_vbox.addLayout(mode_btn_grid)

        handle_hbox = QHBoxLayout()
        handle_hbox.addWidget(QLabel("手柄级位:"))
        self.slider_handle = QSlider(Qt.Orientation.Horizontal)
        self.slider_handle.setRange(0, 80)  # 0.1 精度
        self.slider_handle.setValue(0)
        self.slider_handle.valueChanged.connect(self._on_manual_handle_moved)
        handle_hbox.addWidget(self.slider_handle)
        self.lbl_handle_val = QLabel("0.0 档")
        handle_hbox.addWidget(self.lbl_handle_val)
        manual_vbox.addLayout(handle_hbox)

        self.lbl_manual_hint = QLabel("提示：切到手动模式后，运行中可实时调整工况与级位")
        self.lbl_manual_hint.setWordWrap(True)
        manual_vbox.addWidget(self.lbl_manual_hint)

        bottom_layout.addWidget(manual_group, stretch=30)

        # 3. 仿真控制区
        control_group = QGroupBox("仿真运行控制")
        control_group.setFont(get_qfont(9, bold=True))
        control_vbox = QVBoxLayout(control_group)

        btn_hbox = QGridLayout()
        self.btn_start = QPushButton("开始仿真发车")
        self.btn_start.setStyleSheet("background-color: #2ecc71; color: white; font-weight: bold;")
        self.btn_start.clicked.connect(self._on_start_clicked)
        btn_hbox.addWidget(self.btn_start, 0, 0)

        self.btn_pause = QPushButton("暂停")
        self.btn_pause.setEnabled(False)
        self.btn_pause.clicked.connect(self._on_pause_clicked)
        btn_hbox.addWidget(self.btn_pause, 0, 1)

        self.btn_reset = QPushButton("复位重置")
        self.btn_reset.clicked.connect(self._on_reset_clicked)
        btn_hbox.addWidget(self.btn_reset, 1, 0)

        self.btn_compare = QPushButton("一键策略能耗对比")
        self.btn_compare.setStyleSheet("background-color: #9b59b6; color: white;")
        self.btn_compare.clicked.connect(self._on_compare_clicked)
        btn_hbox.addWidget(self.btn_compare, 1, 1)

        self.btn_export = QPushButton("导出高清运行图与数据报表")
        self.btn_export.setStyleSheet("background-color: #e67e22; color: white;")
        self.btn_export.clicked.connect(self.export_simulation_report)
        btn_hbox.addWidget(self.btn_export, 2, 0, 1, 2)

        control_vbox.addLayout(btn_hbox)
        control_vbox.addStretch(1)
        bottom_layout.addWidget(control_group, stretch=26)

        # 4. 规程数据监控数字墙（报告要求 12 项实时参数）
        telemetry_group = QGroupBox("列车数字化动态仪表盘")
        telemetry_group.setFont(get_qfont(9, bold=True))
        telemetry_grid = QGridLayout(telemetry_group)
        telemetry_grid.setSpacing(4)

        self.telemetry_labels = {}
        items = [
            "机车工况", "手柄档位", "牵引力(kN)", "总合力(kN)",
            "运行速度(km/h)", "累计距离(km)", "区间时分(min)", "累计时分(min)",
            "公里标(DK)", "当前限速(km/h)", "累计能耗(kWh)", "累计燃油(kg)",
        ]
        for idx, name in enumerate(items):
            lbl = QLabel(f"{name}: --")
            self.telemetry_labels[name] = lbl
            telemetry_grid.addWidget(lbl, idx // 2, idx % 2)

        self.lbl_atp = QLabel("ATP 防护状态: 正常监视")
        self.lbl_atp.setStyleSheet("color: #2ecc71; font-weight: bold;")
        self.lbl_atp.setWordWrap(True)
        telemetry_grid.addWidget(self.lbl_atp, (len(items) + 1) // 2, 0, 1, 2)

        bottom_layout.addWidget(telemetry_group, stretch=40)

    # ------------------------------------------------------------------
    # 静态图重建（配置 / 策略变化时）
    # ------------------------------------------------------------------
    def _rebuild_static_plots(self):
        """全面刷新：换配置、预计算参考线、重绘静态图层。"""
        t_name = self.combo_train.currentText()
        l_name = self.combo_line.currentText()
        strat_idx = self.combo_strategy.currentData()
        cruise_cap = float(self.slider_cruise.value())

        self.sim.change_config(t_name, l_name)

        # 同步更新巡航滑块边界
        self.slider_cruise.setRange(30, int(self.sim.train_type.max_speed_kmh))
        if self.slider_cruise.value() > self.sim.train_type.max_speed_kmh:
            self.slider_cruise.setValue(int(self.sim.train_type.max_speed_kmh))
            cruise_cap = float(self.slider_cruise.value())

        # 预演演算灰色理论参考线（策略2做二分寻优，结果存 reference_cruise_v）
        self.ref_s, self.ref_v, ref_sim = precompute_reference_curve(
            t_name, l_name, strat_idx, cruise_v_set=cruise_cap if strat_idx == 2 else None
        )
        self.ref_cruise_v = getattr(ref_sim, "reference_cruise_v", None)

        self._draw_static_plots()

    def _draw_static_plots(self):
        """依据当前线路/车型绘制全部静态图层。"""
        line = self.sim.line
        base_marker = line.km_marker_base
        max_len = line.total_length_km
        v_axis_max = max(self.sim.train_type.max_speed_kmh, line_max_v(line)) * 1.15

        self.ax_main.clear()
        self.ax_profile.clear()
        self.ax_traction.clear()
        self.ax_current.clear()
        self.ax_time.clear()
        self._handle_texts = []
        self._annot_scan_idx = 0

        # ---- 主图: 线路限速红线 (ATP防护底线) ----
        for i, lim in enumerate(line.limits):
            self.ax_main.plot(
                [lim.s_start_km + base_marker, lim.s_end_km + base_marker],
                [lim.limit_kmh, lim.limit_kmh], 'r-', lw=2,
                label="线路限速 (ATP防护)" if i == 0 else "",
            )
        # 车型最高构造速度
        self.ax_main.axhline(self.sim.train_type.max_speed_kmh,
                             color='brown', linestyle=':', lw=1.5, label="车辆构造最高速度")
        # 预计算参考线
        self.ref_line, = self.ax_main.plot(
            [x + base_marker for x in self.ref_s], self.ref_v,
            color='gray', linestyle='-.', alpha=0.75, lw=1.2, label="预计算参考线",
        )

        # 实际运行轨迹（工况着色）与时间-距离曲线
        self.lc_segments = LineCollection([], linewidths=2.5)
        self.ax_main.add_collection(self.lc_segments)
        self.trace_time_line, = self.ax_time.plot([], [], 'b-', lw=1.8, label="累计时间")

        # 信号机（绿=开放 / 红=关闭）
        for sig in line.signals:
            color = '#2ecc71' if sig.is_open else '#e74c3c'
            self.ax_main.plot(sig.s_km + base_marker, v_axis_max * 0.93,
                              marker='o', color=color, markersize=9,
                              markeredgecolor='black', markeredgewidth=0.6, zorder=5)
            self.ax_main.text(sig.s_km + base_marker, v_axis_max * 0.96, sig.name,
                              ha='center', va='bottom', fontsize=7, color='#333333')

        self.ax_main.set_xlim(base_marker, base_marker + max_len)
        self.ax_main.set_ylim(0, v_axis_max)
        self.ax_main.set_xticks([base_marker + t for t in km_marker_ticks(max_len, 1.0)])
        self.ax_main.set_ylabel("列车速度 (km/h)")
        self.ax_main.grid(True, linestyle=":")
        self.ax_main.legend(loc="upper left", fontsize=8, ncol=2, framealpha=0.85)

        self.ax_time.set_ylim(0, 1.0)
        self.ax_time.set_ylabel("累计时间 (min)", color='#2980b9')
        self.ax_time.yaxis.set_label_position("right")
        self.ax_time.tick_params(axis='y', labelcolor='#2980b9')

        # ---- 纵断面: 坡道 / 曲线 / 车站 ----
        grade_values = [g.gradient_per_mille for g in line.grades] or [0]
        y_min, y_max = min(min(grade_values) * 1.3, -2), max(max(grade_values) * 1.3, 2)
        for g in line.grades:
            g_start, g_end = g.s_start_km + base_marker, g.s_end_km + base_marker
            color = '#fab1a0' if g.gradient_per_mille >= 0 else '#74b9ff'
            self.ax_profile.fill_between(
                [g_start, g_end], [0, 0],
                [g.gradient_per_mille, g.gradient_per_mille], color=color, alpha=0.65)
            length_m = (g.s_end_km - g.s_start_km) * 1000
            self.ax_profile.text(
                (g_start + g_end) / 2, g.gradient_per_mille * 0.5,
                f"{g.gradient_per_mille:+.1f}‰\n{length_m:.0f}m",
                ha='center', va='center', fontsize=7)

        # 曲线半径标注：紫色横线放在最深坡道色块之下的带内，文字置于线上方，避免重叠
        y_text_base = y_min * 0.88
        for c in line.curves:
            c_mid = (c.s_start_km + c.s_end_km) / 2 + base_marker
            self.ax_profile.plot(
                [c.s_start_km + base_marker, c.s_end_km + base_marker],
                [y_text_base, y_text_base], color='#8e44ad', lw=2.5, alpha=0.9)
            self.ax_profile.text(c_mid, y_text_base, f"R={c.radius_m:.0f}",
                                 ha='center', va='bottom', fontsize=7, color='#8e44ad')

        for st in line.stations:
            st_pos = st.s_km + base_marker
            self.ax_profile.axvline(st_pos, color='blue', linestyle='--', alpha=0.5)
            self.ax_profile.text(st_pos, y_max * 0.86, st.name, rotation=90,
                                 va='top', ha='right', color='blue', fontsize=9)

        # 列车当前位置标记（红色方块 + 绿色位置线）
        self.train_marker, = self.ax_profile.plot(
            [base_marker], [0], marker='s', color='#e74c3c', markersize=9, zorder=6)
        self.train_pos_line, = self.ax_profile.plot(
            [base_marker, base_marker], [y_min, y_max], color='#2ecc71', lw=1.2, alpha=0.7, zorder=5)

        self.ax_profile.set_ylim(y_min, y_max)
        self.ax_profile.set_ylabel("坡度 (‰)")
        self.ax_profile.grid(True, linestyle=":")
        self.ax_profile.set_xlabel("铁路公里标位置 (DK / km)")

        # ---- 特性曲线族 ----
        self._draw_characteristic_curves()

        self.fig.tight_layout()
        self.canvas.draw_idle()

    def _draw_characteristic_curves(self):
        """牵引力 / 有功电流 8 档特性曲线族。"""
        tt = self.sim.train_type
        v_max = tt.max_speed_kmh
        vs = [v_max * i / 200.0 for i in range(1, 201)]

        # 牵引力特性
        for h in range(1, 9):
            fs = [tt.get_traction_force_kn(v, h) for v in vs]
            alpha = 1.0 if h == 8 else 0.55
            lw = 1.8 if h == 8 else 1.0
            self.ax_traction.plot(vs, fs, color='#c0392b', alpha=alpha, lw=lw)
        self.ax_traction.set_xlim(0, v_max)
        self.ax_traction.set_ylim(0, tt.max_traction_kn * 1.08)
        self.ax_traction.set_title("牵引力特性 (1~8 档)", fontsize=9)
        self.ax_traction.set_xlabel("速度 (km/h)", fontsize=8)
        self.ax_traction.set_ylabel("牵引力 (kN)", fontsize=8)
        self.ax_traction.grid(True, linestyle=":")
        self.work_point_traction, = self.ax_traction.plot(
            [], [], 'o', color='#e74c3c', markersize=8, markeredgecolor='black',
            markeredgewidth=0.8, zorder=6)

        # 有功电流特性（教学简化：电流与轮周牵引力成正比）
        for h in range(1, 9):
            iis = [tt.max_active_current_a * tt.get_traction_force_kn(v, h) / tt.max_traction_kn for v in vs]
            alpha = 1.0 if h == 8 else 0.55
            lw = 1.8 if h == 8 else 1.0
            self.ax_current.plot(vs, iis, color='#2980b9', alpha=alpha, lw=lw)
        self.ax_current.set_xlim(0, v_max)
        self.ax_current.set_ylim(0, tt.max_active_current_a * 1.08)
        self.ax_current.set_title("有功电流特性 (1~8 档)", fontsize=9)
        self.ax_current.set_xlabel("速度 (km/h)", fontsize=8)
        self.ax_current.set_ylabel("有功电流 (A)", fontsize=8)
        self.ax_current.grid(True, linestyle=":")
        self.work_point_current, = self.ax_current.plot(
            [], [], 'o', color='#e74c3c', markersize=8, markeredgecolor='black',
            markeredgewidth=0.8, zorder=6)

    # ------------------------------------------------------------------
    # 仿真步进
    # ------------------------------------------------------------------
    def _on_simulation_step(self):
        st = self.sim.state
        L = self.sim.line.total_length_km
        if st.s_km >= L:
            self._finish_run("仿真结束：列车已对标平稳到达终点站停车。")
            return

        # 1. 获取操纵指令（自动 / 手动）
        if self.combo_mode.currentData() == "auto":
            strat_idx = self.combo_strategy.currentData()
            cruise_v = float(self.slider_cruise.value())
            tgt_mode, tgt_handle = apply_auto_control(self.sim, strat_idx, cruise_v)
        else:
            tgt_mode, tgt_handle = self.manual_mode, self.manual_handle

        # 2. 物理引擎步进（倍速 = 每帧多步）
        effective_limit = 80.0
        steps_per_frame = self.combo_speed.currentData()
        for _ in range(steps_per_frame):
            st, effective_limit = self.sim.step(self.sim_dt, tgt_mode, tgt_handle)
            if st.s_km >= L:
                break

        # 区间时分：经过车站后重置
        while self._station_ptr < len(self.sim.line.stations) - 1 and \
                st.s_km >= self.sim.line.stations[self._station_ptr].s_km:
            self._station_ptr += 1
            self._region_t0_sec = st.t_sec
        region_t_min = (st.t_sec - getattr(self, "_region_t0_sec", 0.0)) / 60.0

        # 3. 刷新仪表盘
        self._update_telemetry(st, effective_limit, region_t_min)
        self._update_dynamic_plots(st)
        self.canvas.draw_idle()

        if st.s_km >= L:
            self._finish_run("仿真结束：列车已对标平稳到达终点站停车。")

    def _update_telemetry(self, st, effective_limit, region_t_min):
        base_marker = self.sim.line.km_marker_base
        handle_disp = f"{st.handle:.1f} 档" if st.mode == OperatingMode.TRACTION else "–"
        self.telemetry_labels["机车工况"].setText(f"机车工况: {st.mode.chinese_name}")
        self.telemetry_labels["手柄档位"].setText(f"手柄档位: {handle_disp}")
        self.telemetry_labels["牵引力(kN)"].setText(f"牵引力(kN): {st.force_kn:.1f}")
        self.telemetry_labels["总合力(kN)"].setText(f"总合力(kN): {st.net_force_kn:.1f}")
        self.telemetry_labels["运行速度(km/h)"].setText(f"运行速度: {st.v_kmh:.1f} km/h")
        self.telemetry_labels["累计距离(km)"].setText(f"累计距离: {st.s_km:.3f} km")
        self.telemetry_labels["区间时分(min)"].setText(f"区间时分: {region_t_min:.2f} min")
        self.telemetry_labels["累计时分(min)"].setText(f"累计时分: {st.t_sec / 60.0:.2f} min")
        self.telemetry_labels["公里标(DK)"].setText(f"公里标: DK{st.s_km + base_marker:.3f}")
        self.telemetry_labels["当前限速(km/h)"].setText(f"当前限速: {effective_limit:.0f} km/h")
        self.telemetry_labels["累计能耗(kWh)"].setText(f"累计能耗: {st.total_energy_kwh:.2f} kWh")
        self.telemetry_labels["累计燃油(kg)"].setText(f"累计燃油: {st.total_fuel_kg:.2f} kg")

        # ATP / 停滞 状态
        if self.sim.is_stalled and st.v_kmh < 0.5:
            self.lbl_atp.setText("警告：列车坡道停滞！牵引力不足以克服当前坡道阻力，请复位后更换车型/线路组合。")
            self.lbl_atp.setStyleSheet("color: #c0392b; font-weight: bold;")
        elif st.v_kmh >= effective_limit:
            self.lbl_atp.setText("ATP 强制介入：列车超速！已触发最大联合惩罚制动。")
            self.lbl_atp.setStyleSheet("color: #e74c3c; font-weight: bold;")
        elif st.v_kmh >= 0.95 * effective_limit:
            self.lbl_atp.setText("ATP 安全预警：逼近限速屏障，已自动断电惰行。")
            self.lbl_atp.setStyleSheet("color: #f39c12; font-weight: bold;")
        else:
            self.lbl_atp.setText("ATP 防护状态: 正常监控中")
            self.lbl_atp.setStyleSheet("color: #2ecc71; font-weight: bold;")

    def _update_dynamic_plots(self, st):
        base_marker = self.sim.line.km_marker_base

        # 主图轨迹（降采样后按工况着色）
        n = len(st.history_s)
        if n >= 2:
            ds, dv, dm = downsample_history(st.history_s, st.history_v, st.history_mode, MAX_PLOT_POINTS)
            segments, colors = build_colored_line_segments(
                [x + base_marker for x in ds], dv, dm, MODE_COLORS)
            self.lc_segments.set_segments(segments)
            self.lc_segments.set_colors(colors)

            # 档位标注：仅在牵引工况且级位变化点标注一次
            self._annotate_handles(st)

            # 时间-距离曲线
            dts = downsample_xy(st.history_s, st.history_t_min, MAX_PLOT_POINTS)
            self.trace_time_line.set_data([x + base_marker for x in dts[0]], dts[1])
            self.ax_time.set_ylim(0, max(st.history_t_min) * 1.2 + 0.5)

        # 特性曲线工作点
        v_now = st.v_kmh
        f_disp = max(0.0, st.force_kn)
        i_now = self.sim.train_type.max_active_current_a * f_disp / self.sim.train_type.max_traction_kn
        self.work_point_traction.set_data([v_now], [f_disp])
        self.work_point_current.set_data([v_now], [i_now])

        # 纵断面列车位置
        pos = st.s_km + base_marker
        self.train_marker.set_data([pos], [0])
        self.train_pos_line.set_data([pos, pos], self.train_pos_line.get_ydata())

    def _annotate_handles(self, st):
        """在主图牵引工况级位变化点标注档位数字（增量扫描）。"""
        hist = st.history_handle
        hist_mode = st.history_mode
        hist_s = st.history_s
        hist_v = st.history_v
        base_marker = self.sim.line.km_marker_base
        i = max(1, self._annot_scan_idx)
        while i < len(hist):
            if hist_mode[i] == OperatingMode.TRACTION and hist_mode[i - 1] != OperatingMode.TRACTION:
                txt = self.ax_main.annotate(
                    f"{hist[i]:.1f}",
                    (hist_s[i] + base_marker, hist_v[i]),
                    textcoords="offset points", xytext=(0, 6),
                    fontsize=8, color='#c0392b', fontweight='bold', ha='center')
                self._handle_texts.append(txt)
            i += 1
        self._annot_scan_idx = len(hist)

    def _finish_run(self, message: str):
        self.timer.stop()
        self.is_paused = False
        self.is_finished = True
        self.btn_start.setEnabled(True)
        self.btn_start.setText("重新开始 (自动复位)")
        self.btn_pause.setEnabled(False)
        self.btn_pause.setText("暂停")
        self._set_config_widgets_enabled(True)
        self.lbl_atp.setText(message)
        self.lbl_atp.setStyleSheet("color: #3498db; font-weight: bold;")
        self.canvas.draw_idle()

    def _set_config_widgets_enabled(self, enabled: bool):
        """运行/空闲状态切换：空闲时才允许修改车型、线路、模式与策略。"""
        self.combo_train.setEnabled(enabled)
        self.combo_line.setEnabled(enabled)
        self.combo_strategy.setEnabled(enabled)
        self.combo_mode.setEnabled(enabled)
        self._on_drive_mode_changed()

    def _on_drive_mode_changed(self):
        """手动 / 自动模式切换时同步操纵台与巡航滑块可用性。"""
        manual = self.combo_mode.currentData() == "manual"
        for btn in self.mode_buttons.values():
            btn.setEnabled(manual)
        self.slider_handle.setEnabled(manual)
        # 巡航限速仅自动操纵相关；自动模式下运行中也可实时调节
        self.slider_cruise.setEnabled(not manual)

    # ------------------------------------------------------------------
    # 交互事件
    # ------------------------------------------------------------------
    def _on_start_clicked(self):
        if self.is_finished:
            self._reset_runtime()  # 完成后再次开始 → 先复位再发车
        self.is_finished = False
        self._station_ptr = 1
        self._region_t0_sec = 0.0
        self.timer.start(50)
        self.btn_start.setEnabled(False)
        self.btn_start.setText("开始仿真发车")
        self.btn_pause.setEnabled(True)
        self._set_config_widgets_enabled(False)

    def _on_pause_clicked(self):
        if not self.is_paused:
            self.timer.stop()
            self.is_paused = True
            self.btn_pause.setText("恢复")
        else:
            self.timer.start(50)
            self.is_paused = False
            self.btn_pause.setText("暂停")

    def _on_reset_clicked(self):
        self.timer.stop()
        self._reset_runtime()

    def _reset_runtime(self):
        """复位运行状态：清空动态轨迹与仪表，保留静态线路图与参考线。"""
        self.timer.stop()
        self.is_paused = False
        self.is_finished = False
        self.sim.reset()
        self._station_ptr = 1
        self._region_t0_sec = 0.0
        self._annot_scan_idx = 0

        for txt in self._handle_texts:
            try:
                txt.remove()
            except Exception:
                pass
        self._handle_texts = []
        self.lc_segments.set_segments([])
        self.trace_time_line.set_data([], [])
        self.ax_time.set_ylim(0, 1.0)
        self.work_point_traction.set_data([], [])
        self.work_point_current.set_data([], [])
        base_marker = self.sim.line.km_marker_base
        self.train_marker.set_data([base_marker], [0])
        self.train_pos_line.set_data([base_marker, base_marker], self.train_pos_line.get_ydata())

        self.btn_start.setEnabled(True)
        self.btn_start.setText("开始仿真发车")
        self.btn_pause.setEnabled(False)
        self.btn_pause.setText("暂停")
        self._set_config_widgets_enabled(True)

        for name in self.telemetry_labels:
            self.telemetry_labels[name].setText(f"{name}: --")
        self.lbl_atp.setText("ATP 防护状态: 正常监视")
        self.lbl_atp.setStyleSheet("color: #2ecc71; font-weight: bold;")
        self.canvas.draw_idle()

    def _on_config_changed(self):
        self._reset_runtime()
        self._rebuild_static_plots()

    def _on_strategy_changed(self):
        self._reset_runtime()
        self._rebuild_static_plots()

    def _on_manual_mode_selected(self, mode: OperatingMode):
        self.manual_mode = mode
        # 同步按钮选中态（兼容程序化调用；用户点击时本就处于选中）
        btn = self.mode_buttons.get(mode)
        if btn is not None and not btn.isChecked():
            btn.setChecked(True)

    def _on_manual_handle_moved(self, val):
        self.manual_handle = val / 10.0
        self.lbl_handle_val.setText(f"{self.manual_handle:.1f} 档")

    def _on_cruise_slider_moved(self, val):
        self.lbl_cruise_val.setText(f"{val} km/h")

    def _on_cruise_slider_released(self):
        # 松开后重算参考线（拖动过程中不重算，避免卡顿；运行/暂停中不重算）
        if not self.timer.isActive() and not self.is_paused and not self.is_finished:
            strat_idx = self.combo_strategy.currentData()
            if strat_idx == 2:
                self._rebuild_static_plots()

    # ------------------------------------------------------------------
    # 导出与对比
    # ------------------------------------------------------------------
    def export_simulation_report(self, file_path: str | None = None):
        """保存 300DPI 高清运行图 + 全量历史离散点 CSV 明细表。"""
        now_str = datetime.now().strftime("%Y%m%d_%H%M")
        default_name = f"运行图_{self.sim.train_type.name}_{self.sim.line.name}_{now_str}.png"

        if not file_path:
            file_path, _ = QFileDialog.getSaveFileName(
                self, "选择报告与运行图存储路径", default_name, "PNG图像 (*.png)")
            if not file_path:
                return

        root, ext = os.path.splitext(file_path)
        if ext.lower() not in (".png",):
            root, ext = file_path, ".png"
        png_path = root + ext
        csv_path = root + ".csv"

        try:
            # 图顶注入运行元数据印章
            st = self.sim.state
            summary = (
                f"车型: {self.sim.train_type.name} | 线路: {self.sim.line.name}\n"
                f"全程耗时: {st.t_sec / 60.0:.2f} min | 电能: {st.total_energy_kwh:.2f} kWh | "
                f"燃油: {st.total_fuel_kg:.2f} kg | 最高速度: {max(st.history_v) if st.history_v else 0:.1f} km/h"
            )
            watermark = self.ax_main.text(
                0.5, 0.96, summary, transform=self.ax_main.transAxes,
                ha='center', va='top', fontsize=9, fontweight='bold',
                bbox=dict(boxstyle='round,pad=0.4', facecolor='#f1c40f', alpha=0.7))
            try:
                self.canvas.figure.savefig(png_path, dpi=300, bbox_inches='tight')
            finally:
                watermark.remove()
                self.canvas.draw_idle()

            # 全离散点 CSV 明细
            with open(csv_path, 'w', newline='', encoding='utf-8-sig') as f:
                writer = csv.writer(f)
                writer.writerow(["公里标位置(km)", "列车速度(km/h)", "耗时(min)",
                                 "操纵工况", "手柄级位", "轮周力(kN)", "能耗(kWh)", "燃油(kg)"])
                for i in range(len(st.history_s)):
                    writer.writerow([
                        f"{st.history_s[i]:.3f}",
                        f"{st.history_v[i]:.2f}",
                        f"{st.history_t_min[i]:.3f}",
                        st.history_mode[i].chinese_name,
                        f"{st.history_handle[i]:.1f}",
                        f"{st.history_force[i]:.1f}",
                        f"{st.total_energy_kwh:.3f}",
                        f"{st.total_fuel_kg:.3f}",
                    ])
            self.lbl_atp.setText(f"导出成功：{os.path.basename(png_path)} 与同名 CSV 已保存。")
            self.lbl_atp.setStyleSheet("color: #3498db; font-weight: bold;")
        except Exception as e:
            self.lbl_atp.setText(f"导出失败：{e}")
            self.lbl_atp.setStyleSheet("color: #e74c3c; font-weight: bold;")

    def _on_compare_clicked(self):
        """后台演算双策略全程运行，弹出可视化能耗差异对比对话框。"""
        t_name = self.combo_train.currentText()
        l_name = self.combo_line.currentText()

        _, _, sim1 = precompute_reference_curve(t_name, l_name, 1)
        _, _, sim2 = precompute_reference_curve(t_name, l_name, 2)

        dialog = StrategyComparisonDialog(sim1, sim2, self)
        dialog.exec()


def line_max_v(line) -> float:
    """线路全程最高限速。"""
    return max((lim.limit_kmh for lim in line.limits), default=80.0)


def run_app():
    app = QApplication(sys.argv)
    setup_matplotlib_chinese()
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    run_app()
