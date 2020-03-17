import numpy as np
from matplotlib import pyplot as plt

h = np.linspace(-np.pi, np.pi, 16)
c, s = np.cos(h), np.sin(h)
rax_x, rax_y = 2.0 * c, 2.0 * s
ctr_x, ctr_y = rax_x-s, rax_y+c

plt.plot(rax_x, rax_y)
plt.plot(ctr_x, ctr_y)
for x1,y1,x2,y2 in zip(rax_x,rax_y,ctr_x,ctr_y):
    plt.plot([x1,x2],[y1,y2])
plt.axis('equal')
plt.gca().set_aspect('equal')
plt.show()
