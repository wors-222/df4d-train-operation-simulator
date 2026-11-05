# -*- coding: utf-8 -*-
"""
列车运行操纵模拟软件 — 程序入口
Python + PySide6 + Matplotlib
"""

# 须在导入 Matplotlib 绘图模块之前配置中文字体
from font_config import setup_matplotlib_chinese

setup_matplotlib_chinese()

from main_window import run_app

if __name__ == "__main__":
    run_app()
