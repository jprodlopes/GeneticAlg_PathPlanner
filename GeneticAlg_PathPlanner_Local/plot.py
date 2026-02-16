import matplotlib.pyplot as plt
import math
import numpy as np
import matplotlib.collections as mcoll
import matplotlib as mpl 
from matplotlib import cm
from config import B_MIN, B_MAX
from matplotlib.collections import PolyCollection

def plot_circles(axs, map_size, bestIndv):
    obs = bestIndv.obs

    for i in range(1, bestIndv.n_obs-1):

        # main obstacle
        circle_obj = plt.Circle(
            (obs[i]['x'], obs[i]['y']),
            obs[i]['radius'],
            fill=True,
 
            color='lightgray',
            alpha=0.5
        )
        axs.add_patch(circle_obj)
    
        circle_obj = plt.Circle(
            (obs[i]['x'], obs[i]['y']),
            obs[i]['radius'],
            fill=False,
            color='black',
            linewidth=0.5,
            alpha=0.5
        )

        axs.add_patch(circle_obj)
        # wingspan margin
        circle_obj = plt.Circle(
            (obs[i]['x'], obs[i]['y']),
            bestIndv.r_min[2*i],
            fill=False,
            color='purple',
            linestyle='--',
            alpha=0.3
        )
        axs.add_patch(circle_obj)

        # write index over the circle center
        axs.text(
            obs[i]['x'],
            obs[i]['y'],
            str(i),
            ha='center',
            va='center',
            fontsize=8,
            color='black',
            weight='bold',
            zorder=10   # keeps text above circles
        )

    # start circle
    circle_obj = plt.Circle(
        (bestIndv.obs[0]['x'], bestIndv.obs[0]['y']),
        bestIndv.obs[0]['radius'],
        fill=False,
        color='black',
        linestyle='dotted'
    )
    axs.add_patch(circle_obj)

    # crossed obstacles in red
    for j in bestIndv.obsCrossed:
        circle_obj = plt.Circle(
            (bestIndv.obs[j]['x'], bestIndv.obs[j]['y']),
            bestIndv.obs[j]['radius'],
            fill=False,
            color='red'
        )
        axs.add_patch(circle_obj)
    
    print('Obstacles crossed ' + str(len(bestIndv.obsCrossed)) + f": {bestIndv.obsCrossed}") 
    axs.add_patch(circle_obj)


def smooth_path(path, window_size=70):
    smoothed_path = []

    for i in range(len(path)):
        start_index = max(0, i - window_size // 2)
        end_index = min(len(path), i + window_size // 2 + 1)
        neighbors = path[start_index:end_index]

        smoothed_value = sum(neighbors) / len(neighbors)
        smoothed_path.append(smoothed_value)

    return smoothed_path



def draw_path(axs, path, colorScale, label):
    span_norm = mpl.colors.Normalize(vmin=B_MIN, vmax=B_MAX)
    #cmap = plt.cm.summer_r
    cmap = plt.cm.summer

    sm = mpl.cm.ScalarMappable(norm=span_norm, cmap=cmap)
    sm.set_array([])

    x = np.asarray(path[0], dtype=float)
    y = np.asarray(path[1], dtype=float)

    # centerline
    axs.plot(x, y, color="steelblue", linewidth=1.2, zorder=3)

    try:
        import inspect
        frame = inspect.currentframe()
        while frame:
            if "bestIndv" in frame.f_locals:
                bestIndv = frame.f_locals["bestIndv"]
                break
            frame = frame.f_back
        else:
            bestIndv = None

        if bestIndv is not None and hasattr(bestIndv, "path_wingspan"):
            wings = np.asarray(bestIndv.path_wingspan, dtype=float)

            # --- make wings length match points ---
            if len(wings) < len(x):
                wings = np.pad(wings, (0, len(x) - len(wings)), mode="edge")
            else:
                wings = wings[:len(x)]

            # --- detect closed loop (helps circles) ---
            closed = (len(x) > 3) and (np.hypot(x[0] - x[-1], y[0] - y[-1]) < 1e-9)

            # --- per-point tangents (periodic if closed) ---
            if closed:
                dx = np.roll(x, -1) - np.roll(x, 1)
                dy = np.roll(y, -1) - np.roll(y, 1)
            else:
                dx = np.gradient(x)
                dy = np.gradient(y)

            tlen = np.hypot(dx, dy)
            tlen[tlen == 0] = 1.0
            tx = dx / tlen
            ty = dy / tlen

            # normals
            nx = -ty
            ny =  tx

            off = 0.5 * wings
            xL = x + nx * off
            yL = y + ny * off
            xR = x - nx * off
            yR = y - ny * off

            # --- build ribbon quads with smooth boundaries ---
            polys = []
            facecolors = []

            n = len(x)
            last = n if closed else n - 1

            for i in range(last):
                j = (i + 1) % n

                # skip degenerate segments
                if x[i] == x[j] and y[i] == y[j]:
                    continue

                # quad: L_i -> L_j -> R_j -> R_i
                polys.append([(xL[i], yL[i]), (xL[j], yL[j]), (xR[j], yR[j]), (xR[i], yR[i])])

                wseg = 0.5 * (wings[i] + wings[j])   # wings per point -> segment color
                facecolors.append(cmap(span_norm(wseg)))

            if polys:
                pc = PolyCollection(
                    polys,
                    facecolors=facecolors,
                    edgecolors="none",
                    alpha=0.35,
                    zorder=1,
                )
                axs.add_collection(pc)

                # optional subtle edges (turn on if you want)
                # axs.plot(xL, yL, color="k", alpha=0.12, linewidth=0.6, zorder=2)
                # axs.plot(xR, yR, color="k", alpha=0.12, linewidth=0.6, zorder=2)

                # keep centerline on top
                axs.plot(x, y, color="steelblue", linewidth=1.2, zorder=3)

        cbar = plt.colorbar(sm, ax=axs, pad=0.02)
        cbar.set_label("Wingspan")

    except Exception as e:
        print(f"Wingspan visualization error: {e}")


def draw_map(ax, x_init, x_goal, map_size, bestIndv, colorScale, label):
    plot_circles(ax, map_size, bestIndv)
    ax.set_title("Speed: " + str(round(bestIndv.v[0],2)), fontsize=10)
    ax.scatter(x_init[0], x_init[1], color='orange',  edgecolors='black', marker='o')
    ax.scatter(x_goal[0], x_goal[1], color='green',  edgecolors='black',  marker='o')
    draw_path(ax, bestIndv.path, None, None)

def draw_everything( map_size, circles, x_init, x_goal, bestIndv, perf):
    fig, ax = plt.subplots(1, 2, figsize=(8, 4), gridspec_kw={'width_ratios': [1, 1]})

    ax_info = ax[1]
    plt.suptitle('Obstacles crossed ' + str(len(bestIndv.obsCrossed)), fontsize=10)
    x_values = [item[0] for item in perf]
    y_values = [(item[1] - bestIndv.score) / 1000 for item in perf]

    # Plotting the bar graph
    ax_info.bar(x_values, y_values)
    ax_info.set_xlabel('Index')
    ax_info.set_title('Performance')
    draw_map(ax[0], x_init, x_goal, map_size, bestIndv, None, None)

    # Displaying the plot
    plt.show()