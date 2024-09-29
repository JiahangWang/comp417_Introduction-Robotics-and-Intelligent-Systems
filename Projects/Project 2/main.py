import matplotlib.pyplot as plt

# 数据
gravity = [10, 12, 14, 16, 18, 20]
average_settling_time = [2000, 2000, 1630, 679, 417, 292]

# 创建折线图
plt.figure(figsize=(10, 6))
plt.plot(gravity, average_settling_time, marker='o')  # 折线图，添加点标记

# 标题和轴标签
plt.title('Average Settling Time at Different Gravity Levels')
plt.xlabel('Gravity')
plt.ylabel('Average Settling Time')

# 网格线
plt.grid(True)

# 显示图表
plt.show()