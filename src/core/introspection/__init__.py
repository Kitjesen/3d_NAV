"""core.introspection — Blueprint and SystemHandle visualization.

Quick usage::

    from core.introspection import render_text, render_dot, render_svg

    print(render_text(system))           # ANSI terminal tree (no deps)
    print(render_dot(system))            # Graphviz DOT string
    render_svg(system, "graph.svg")      # SVG file (needs graphviz CLI)
    render_png(system, "graph.png")      # PNG file (needs graphviz CLI)
"""

from .dot import render as render_dot, render_svg, render_png
from .text import render_text, render_connections

__all__ = [
    "render_dot",
    "render_svg",
    "render_png",
    "render_text",
    "render_connections",
]
