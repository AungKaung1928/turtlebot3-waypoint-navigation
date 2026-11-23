#!/usr/bin/env python3
"""
Draw the waypoint loop hard-coded in the navigator node as SVG.

plot_route.py [node.py] [out.svg]: self.waypoints = [(x, y, yaw), ...] as points in visiting
order (map frame, metres), numbered, loop-back edge dashed. The configured route, not a
recorded run. Standard library only.
"""
import ast
import sys

src = sys.argv[1] if len(sys.argv) > 1 else 'autonomous_navigator/waypoint_navigator.py'
out = sys.argv[2] if len(sys.argv) > 2 else 'docs/route.svg'
wps = next(ast.literal_eval(n.value) for n in ast.walk(ast.parse(open(src).read()))
           if isinstance(n, ast.Assign) and isinstance(n.targets[0], ast.Attribute)
           and n.targets[0].attr == 'waypoints')
pts = [(f'wp{i}', float(x), float(y)) for i, (x, y, _) in enumerate(wps, 1)]
loop = True  # send_next_goal wraps the index: the node loops forever

x0, x1 = min(p[1] for p in pts) - 1.0, max(p[1] for p in pts) + 1.0
y0, y1 = min(p[2] for p in pts) - 1.0, max(p[2] for p in pts) + 1.0
W = 560
s = W / (x1 - x0)
H = int((y1 - y0) * s) + 56


def X(x):
    return (x - x0) * s


def Y(y):
    return (y1 - y) * s


el = [f'<rect width="{W}" height="{H}" fill="#ffffff"/>']
for v in range(int(x0) + (x0 > 0), int(x1) + 1):
    if 12 < X(v) < W - 12:
        el.append(f'<line x1="{X(v):.1f}" y1="0" x2="{X(v):.1f}" y2="{Y(y0):.1f}" '
                  'stroke="#e6eaed"/>'
                  f'<text x="{X(v):.1f}" y="{Y(y0) - 4:.1f}" text-anchor="middle" '
                  f'fill="#6b7785">{v}</text>')
for v in range(int(y0) + (y0 > 0), int(y1) + 1):
    if 12 < Y(v) < Y(y0) - 12:
        el.append(f'<line x1="0" y1="{Y(v):.1f}" x2="{W}" y2="{Y(v):.1f}" stroke="#e6eaed"/>'
                  f'<text x="4" y="{Y(v) + 14:.1f}" fill="#6b7785">{v}</text>')
el.append('<defs><marker id="a" viewBox="0 0 10 10" refX="9" refY="5" '
          'markerWidth="7" markerHeight="7" '
          'orient="auto-start-reverse"><path d="M0 0L10 5L0 10z" fill="#10161e"/></marker></defs>')
edges = list(zip(pts, pts[1:])) + ([(pts[-1], pts[0])] if loop else [])
for i, ((_, ax, ay), (_, bx, by)) in enumerate(edges):
    dx, dy = X(bx) - X(ax), Y(by) - Y(ay)
    d = (dx * dx + dy * dy) ** 0.5 or 1.0
    sx, sy = X(ax) + dx / d * 12, Y(ay) + dy / d * 12
    ex, ey = X(bx) - dx / d * 14, Y(by) - dy / d * 14
    dash = ' stroke-dasharray="5 4"' if i == len(pts) - 1 else ''
    el.append(f'<line x1="{sx:.1f}" y1="{sy:.1f}" x2="{ex:.1f}" y2="{ey:.1f}" stroke="#10161e" '
              f'stroke-width="1.6" marker-end="url(#a)"{dash}/>')


def label(px, py, text):
    """Right of the point, or left when it would run past the edge (~6.6 px per char)."""
    if px + 14 + 6.6 * len(text) > W - 4:
        return (f'<text x="{px - 14:.1f}" y="{py - 12:.1f}" text-anchor="end" '
                f'fill="#10161e">{text}</text>')
    return f'<text x="{px + 14:.1f}" y="{py - 12:.1f}" fill="#10161e">{text}</text>'


for i, (name, x, y) in enumerate(pts, 1):
    el.append(f'<circle cx="{X(x):.1f}" cy="{Y(y):.1f}" r="10" fill="#005f73"/>'
              f'<text x="{X(x):.1f}" y="{Y(y) + 4.5:.1f}" text-anchor="middle" '
              f'fill="#ffffff" font-weight="700">{i}</text>'
              + label(X(x), Y(y), f'{name} ({x:g}, {y:g})'))
dashed = 'Dashed = loop back to 1. ' if loop else ''
el.append(f'<text x="8" y="{H - 28}" fill="#3f4d58">'
          f'Waypoint loop from {src}, map frame, metres.</text>'
          f'<text x="8" y="{H - 10}" fill="#3f4d58">{dashed}Not a recorded run.</text>')
svg = (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" '
       f'font-family="Helvetica, Arial, sans-serif" font-size="12">\n'
       + '\n'.join(el) + '\n</svg>\n')
open(out, 'w').write(svg)
print(f'{out}: {len(pts)} points')
