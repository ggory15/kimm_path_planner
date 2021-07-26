#!/usr/bin/env python3

import fileinput
import numpy as np
import matplotlib.pyplot as plt


def main():
    data = []
    for line in fileinput.input():
        subs = line.split(' ')
        if len(subs) < 3:
            continue
        data.append([float(subs[0]), float(subs[1]), float(subs[2])])
    data = np.array(data)
    plt.plot(data[:, 0], data[:, 1])
    plt.quiver(data[:, 0], data[:, 1], np.cos(data[:, 2]), np.sin(data[:, 2]), angles='xy')
    plt.grid()
    plt.axis("equal")
    plt.show()


if __name__ == "__main__":
    main()
