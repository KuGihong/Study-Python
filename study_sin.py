import matplotlib.pyplot as plt
import numpy as np

def draw_sin(t, A, f, b):   #defination draw_sin
    #t = x(domain)
    y= A * np.sin(np.pi * f * t) + b

    plt.figure(figsize=(12,6))
    plt.plot(t,y)
    plt.grid()
    plt.show()


t= np.arange(0, 6, 0.01)
draw_sin(t, 1, 1, 0)