import matplotlib.pyplot as plt

x = [1, 2, 3, 4]
y = [2, 5, 3, 7]

plt.scatter(x, y)
plt.xlabel("X")
plt.ylabel("Y")
plt.title("Matplotlib WSL2 Test")
plt.grid(True)
plt.show()