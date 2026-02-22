"""Generate a racing-line CSV from data (ONLY FOR TESTING SIM): s_m,x_m,y_m,nx,ny,vx_mps"""
import argparse, math, os


def oval(straight=60.0, radius=25.0, n=500, v_str=12.0, v_crv=7.0):
    perim = 2 * straight + 2 * math.pi * radius
    pts, s = [], 0.0
    for i in range(n):
        d = (i / n) * perim
        if d < straight:
            x, y, v = d, 0.0, v_str
        elif d < straight + math.pi * radius:
            a = (d - straight) / radius
            x, y, v = straight + radius * math.sin(a), radius * (1 - math.cos(a)), v_crv
        elif d < 2 * straight + math.pi * radius:
            x, y, v = straight - (d - straight - math.pi * radius), 2 * radius, v_str
        else:
            a = (d - 2 * straight - math.pi * radius) / radius
            x, y, v = -radius * math.sin(a), radius * (1 + math.cos(a)), v_crv
        if pts:
            s += math.hypot(x - pts[-1][1], y - pts[-1][2])
        pts.append((s, x, y, 0.0, 0.0, v))
    return pts


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--output", default="/ws/data/racing_line/line.csv")
    args = p.parse_args()
    pts = oval()
    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    with open(args.output, "w") as f:
        for r in pts:
            f.write(",".join(f"{v:.6f}" for v in r) + "\n")
    print(f"Wrote {len(pts)} points into {args.output}")


if __name__ == "__main__":
    main()