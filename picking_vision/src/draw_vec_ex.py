import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
sns.set_style('whitegrid')

rng = np.random.RandomState(13)

# (2, 2)행렬, (2, 200) 정규분포 행렬의 곱
X = np.dot(rng.rand(2, 2), rng.randn(2, 200)).T
X.shape

plt.scatter(X[:, 0], X[:, 1])
plt.axis('equal')
plt.show()

from sklearn.decomposition import PCA
# n_components : 표현할 주성분의 수
pca = PCA(n_components=2, random_state = 13)
pca.fit(X)

print(pca.components_)

print(pca.explained_variance_)

print(pca.mean_)

def draw_vector(v0, v1, ax=None):
    ax = ax or plt.gca()
    arrowprops = dict(arrowstyle='->',
                      linewidth=2, color='black',
                      shrinkA=0, shrinkB=0)
    ax.annotate('', v1, v0, arrowprops=arrowprops)
plt.scatter(X[:, 0], X[:,1], alpha=0.4)
for length, vector in zip(pca.explained_variance_, pca.components_):
    print(length)
    print(vector)
    v = vector * 3 * np.sqrt(length)
    print(v)
    draw_vector(pca.mean_, pca.mean_ + v)
plt.axis('equal')
plt.show()
