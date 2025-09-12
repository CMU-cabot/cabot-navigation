#!/usr/bin/env python3

from importlib.metadata import entry_points
from typing import List, Type
from cabot_ui.plugin import NavigationPlugin


def load_plugins(group: str = "cabot_ui.plugins") -> List[NavigationPlugin]:
    eps = entry_points(group=group)
    plugins: List[NavigationPlugin] = []
    for ep in eps:
        print(ep)
        #cls: Type[NavigationPlugin] = ep.load()  # エントリポイントが指すクラスをロード
        #plugins.append(cls())
    return plugins

if __name__ == "__main__":
    for p in load_plugins():
        print(f"[{p.name}] -> {p.run(message='hello')}")
