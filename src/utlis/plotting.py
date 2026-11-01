import numpy as np
import matplotlib.pyplot as plt


def draw_arm_2d(ax, points, color="tab:blue", label=None):
    pts = np.asarray(points)
    ax.plot(pts[:, 0], pts[:, 1], "-o", color=color, lw=3, ms=7, label=label)


def draw_arm_3d(ax, points, color="tab:blue", label=None):
    pts = np.asarray(points)
    ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], "-o", color=color, lw=3, ms=6, label=label)


def tidy_2d(ax, title, xlabel="x (cm)", ylabel="y (cm)"):
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)


def save(fig, path):
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    print("saved", path)
    plt.close(fig)
