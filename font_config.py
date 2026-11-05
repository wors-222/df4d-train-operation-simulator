# -*- coding: utf-8 -*-
"""
Windows / Qt / Matplotlib 中文字体配置。
在创建界面与图表前调用，避免中文显示为方框。
"""

from __future__ import annotations

import os
from functools import lru_cache
from typing import List, Optional, Tuple

# 按优先级尝试的字体（显示名与常见字体文件）
_FONT_CANDIDATES: List[Tuple[str, str]] = [
    ("Microsoft YaHei UI", "msyuh.ttc"),
    ("Microsoft YaHei", "msyh.ttc"),
    ("Microsoft YaHei", "msyhbd.ttc"),
    ("SimHei", "simhei.ttf"),
    ("SimSun", "simsun.ttc"),
    ("SimSun", "simsunb.ttf"),
    ("KaiTi", "simkai.ttf"),
    ("FangSong", "simfang.ttf"),
]

_WINDOWS_FONTS_DIR = os.path.join(os.environ.get("WINDIR", r"C:\Windows"), "Fonts")

# 解析成功后供样式表使用的字体族名
CHINESE_FONT_FAMILY: str = "Microsoft YaHei"


def _qt_application_ready() -> bool:
    try:
        from PySide6.QtWidgets import QApplication

        return QApplication.instance() is not None
    except Exception:
        return False


@lru_cache(maxsize=1)
def resolve_chinese_font() -> Tuple[str, Optional[str]]:
    """
    解析本机可用的中文字体。
    返回 (Qt/Matplotlib 字体族名, 字体文件绝对路径或 None)。
    """
    # 1) Windows 字体目录（不依赖 Qt，可在 QApplication 创建前使用）
    for family, filename in _FONT_CANDIDATES:
        path = _find_font_file(filename)
        if path:
            return family, path

    # 2) Qt 字体数据库（须在 QApplication 创建之后调用）
    if _qt_application_ready():
        try:
            from PySide6.QtGui import QFontDatabase

            qt_families = set(QFontDatabase.families())
            for family, filename in _FONT_CANDIDATES:
                if family in qt_families:
                    path = _find_font_file(filename)
                    return family, path or None
        except Exception:
            pass

    # 3) matplotlib 已安装字体中模糊匹配
    try:
        from matplotlib import font_manager

        for f in font_manager.fontManager.ttflist:
            name = f.name or ""
            if any(k in name for k in ("YaHei", "SimHei", "SimSun", "Microsoft")):
                return name, f.fname
    except Exception:
        pass

    return "Microsoft YaHei", _find_font_file("msyh.ttc")


def _find_font_file(filename: str) -> Optional[str]:
    path = os.path.join(_WINDOWS_FONTS_DIR, filename)
    return path if os.path.isfile(path) else None


def setup_matplotlib_chinese() -> str:
    """配置 Matplotlib 使用中文字体，返回实际族名。"""
    global CHINESE_FONT_FAMILY

    import matplotlib
    from matplotlib import font_manager

    family, font_path = resolve_chinese_font()
    mpl_family = family

    if font_path and os.path.isfile(font_path):
        try:
            font_manager.fontManager.addfont(font_path)
            prop = font_manager.FontProperties(fname=font_path)
            mpl_family = prop.get_name()
        except Exception:
            mpl_family = family

    matplotlib.rcParams["font.sans-serif"] = [mpl_family, family, "SimHei", "SimSun"]
    matplotlib.rcParams["font.family"] = "sans-serif"
    matplotlib.rcParams["axes.unicode_minus"] = False

    CHINESE_FONT_FAMILY = family
    return mpl_family


def setup_qt_chinese(app) -> str:
    """设置 QApplication 全局字体，返回字体族名。"""
    global CHINESE_FONT_FAMILY

    from PySide6.QtGui import QFont, QFontDatabase

    # QApplication 已存在，可重新解析（清除仅文件路径的缓存无必要，路径已足够）
    family, font_path = resolve_chinese_font()

    # 优先用字体文件创建 QFont（避免族名与系统登记不一致）
    font = QFont()
    if font_path and os.path.isfile(font_path):
        font_id = QFontDatabase.addApplicationFont(font_path)
        if font_id >= 0:
            families = QFontDatabase.applicationFontFamilies(font_id)
            if families:
                family = families[0]
                font = QFont(family, 10)

    if not font.family() or font.family() == "MS Shell Dlg 2":
        font = QFont(family, 10)

    font.setStyleHint(QFont.StyleHint.SansSerif)
    app.setFont(font)
    CHINESE_FONT_FAMILY = family
    return family


def get_qfont(point_size: int = 10, bold: bool = False) -> "QFont":
    """获取已配置的中文字体 QFont。"""
    from PySide6.QtGui import QFont

    f = QFont(CHINESE_FONT_FAMILY, point_size)
    if bold:
        f.setWeight(QFont.Weight.Bold)
    return f


def qt_stylesheet_font_rule() -> str:
    """供 Qt 样式表使用的 font-family 规则（勿回退到无中文的 sans-serif）。"""
    family = CHINESE_FONT_FAMILY.replace('"', "")
    return f'font-family: "{family}";'


def get_matplotlib_legend_prop() -> "font_manager.FontProperties":
    """Matplotlib 图例/文字用的 FontProperties。"""
    import matplotlib
    from matplotlib import font_manager

    name = matplotlib.rcParams["font.sans-serif"][0]
    return font_manager.FontProperties(family=name)
