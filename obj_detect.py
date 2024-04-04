import numpy as np
import matplotlib.pyplot as plt
file_path = 'obj.txt'

# OBJ 파일 읽기
vertices = []
with open(file_path, 'r') as file:
    for line in file:
        if line.startswith('v '):  # Vertex 데이터 확인
            _, x, y, z = line.split()  # 공백으로 구분하여 데이터 추출
            vertices.append([float(x), float(y), float(z)])

# NumPy 배열로 변환
vertices = np.array(vertices)

# 꺾이는 지점(변곡점)을 찾기 위해 gradient와 second derivative를 계산합니다.
# 변곡점은 second derivative가 0을 지나는 지점입니다.
z = vertices[:, 2]
grad_z = np.gradient(z)
second_deriv_z = np.gradient(grad_z)

# 변곡점의 인덱스를 찾습니다.
# np.sign()를 사용하여 second derivative의 부호 변경 지점을 찾습니다.
# 이 지점들은 대략적으로 변곡점에 해당합니다.
inflexion_points = np.where(np.diff(np.sign(second_deriv_z)))[0]

# 첫 번째와 마지막 변곡점을 포함하여 group을 나눕니다.
# 첫 번째 그룹은 첫 번째 변곡점까지, 마지막 그룹은 마지막 변곡점 이후로 설정합니다.
group_indices = [0] + list(inflexion_points) + [len(z) - 1]

# group 별로 나누어 색상을 지정하여 그립니다.
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

colors = ['red', 'green', 'blue', 'yellow']
for i in range(len(group_indices) - 1):
    start_idx = group_indices[i]
    end_idx = group_indices[i + 1]
    ax.scatter(vertices[start_idx:end_idx, 0],
               vertices[start_idx:end_idx, 1],
               vertices[start_idx:end_idx, 2],
               color=colors[i % len(colors)],
               s=1, label=f'Group {i+1}')

# 그래프 레이블, 범례, 타이틀 설정
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Grouped OBJ Visualization')
ax.legend()

plt.show()