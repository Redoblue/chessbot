import numpy as np
import scipy.linalg
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class DepthPlaneFitter:

    def __init__(self):
        self.C = None

    def fit(self, depth, cbtransformer, nrow=10, ncol=9, show=False):
        assert cbtransformer.is_available()

        x, y, f = [], [], []
        for i in range(nrow):
            for j in range(ncol):
                z = cbtransformer.transform_rc2xy(i, j)
                z = list(map(int, z))
                d = depth[z[1], z[0]] / 1000.
                # print(d)
                if not 0.5 < d < 1.5:
                    continue

                x.append(i)
                y.append(j)
                f.append(d)
        
        self.fit_plane(x, y, f, show=show)

    def get_depth(self, row, col):
        d = self.C[0]*row + self.C[1]*col + self.C[2]
        return d

    def fit_plane(self, xdata, ydata, f, show=False):
        data = np.c_[xdata, ydata, f]

        # best-fit linear plane
        A = np.c_[data[:, 0], data[:, 1], np.ones(data.shape[0])]
        self.C, _, _, _ = scipy.linalg.lstsq(A, data[:, 2])

        if show:
            # regular grid covering the domain of the data
            X, Y = np.meshgrid(np.arange(-2, 12, 0.5), np.arange(-2, 11, 0.5))
            XX = X.flatten()
            YY = Y.flatten()

            # evaluate it on grid
            Z = self.C[0]*X + self.C[1]*Y + self.C[2]
            Z = Z

            # plot points and fitted surface
            fig = plt.figure()
            ax = fig.gca(projection='3d')
            ax.plot_surface(X, Y, Z, rstride=1, cstride=1, alpha=0.2)
            ax.scatter(data[:, 0], data[:, 1], data[:, 2], c='r', s=50)
            # ax.set_zlim(0.8, 1.2)
            plt.xlabel('X')
            plt.ylabel('Y')
            ax.set_zlabel('Z')
            ax.axis('equal')
            ax.axis('tight')
            plt.show()

    def is_available(self):
        return self.C is not None