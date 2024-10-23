import matplotlib.pyplot as plt
import numpy as np

# Data for plotting
x = np.linspace(0, 10, 100)
y1 = np.sin(x)
y2 = np.cos(x)
y3 = x ** 2

# Create the first figure
plt.figure(1)
plt.plot(x, y1, label='sin(x)')
plt.title('Figure 1: Sine Wave')
plt.xlabel('x')
plt.ylabel('sin(x)')
plt.legend()

# Create the second figure
plt.figure(2)
plt.plot(x, y2, 'r', label='cos(x)')
plt.title('Figure 2: Cosine Wave')
plt.xlabel('x')
plt.ylabel('cos(x)')
plt.legend()

# Create the third figure
plt.figure(3)
plt.plot(x, y3, 'g', label='x^2')
plt.title('Figure 3: Parabola')
plt.xlabel('x')
plt.ylabel('x^2')
plt.legend()

# Show all figures
plt.show()
