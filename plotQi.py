import numpy as np
import matplotlib.pyplot as plt
from math import pi

if __name__ == '__main__':
    plt.close(0)
    print("Recuperation des données")
    # Read File
    t = []
    q1 = []
    q2 = []
    q3 = []
    Q = [q1, q2, q3]

    with open("logQi.csv", "r") as file:
        lines = file.readlines()

    lines.pop(0)
    for line in lines:
        words = line.split()

        for i in range(1, 4):
            if words[i] == '-0.0':
                words[i] = '0.0'
        t.append(float(words[0]))
        q1.append(float(words[1]))
        q2.append(float(words[2]))
        q3.append(float(words[3]))

    # Affichage des courbes
    print("Affichage des données")

    plt.figure(0)
    for i in range(3):
        plt.subplot(3, 1, i + 1)
        plt.title("q" + str(i + 1))
        plt.ylim(-pi, pi)
        plt.yticks([round(-pi, 2), round(pi, 2)])
        plt.xticks([round(min(t), 2), round(max(t), 2)])
        plt.ylabel("Position")
        plt.plot(t, Q[i])

        plt.xlabel("Time (s)")

    plt.show()
