import matplotlib.pyplot as plt

# 데이터 파일 읽기 함수
def read_data1(file_path):
    time = []
    x = []
    y = []
    with open(file_path, 'r') as file:
        lines = file.readlines()
        for line in lines:
            t, x_val, y_val = map(float, line.split())
            time.append(t)
            x.append(x_val)
            y.append(y_val)
    return time, x, y


# 두 파일에서 데이터 읽기
time_exp, x_exp, y_exp = read_data1('experiment.txt')

max_val=max(max(x_exp),max(y_exp))
min_val=min(min(x_exp),min(y_exp))

# 그래프 그리기
# X vs Y
plt.figure(figsize=(8, 8))
plt.plot(x_exp, y_exp, marker='o', color='red',  label='Experiment')
plt.title('X vs Y')
plt.xlabel('X')
plt.ylabel('Y')
plt.xlim([-1.5, 1.5])      # X축의 범위: [xmin, xmax]
plt.ylim([-1.5, 1.5])     # Y축의 범위: [ymin, ymax]
plt.legend()
plt.savefig('graph_ex1.png')
plt.close()

# Time vs X
plt.figure(figsize=(8, 8))
plt.plot(time_exp, x_exp, marker='o', color='red', label='Experiment')
plt.title('Time vs X')
plt.xlabel('Time')
plt.ylabel('X')   # X축의 범위: [xmin, xmax]
plt.ylim([-1.5, 1.5])  
plt.legend()
plt.savefig('graph_ex2.png')
plt.close()

# Time vs Y
plt.figure(figsize=(8, 8))
plt.plot(time_exp, y_exp, marker='o', color='red', label='Experiment')
plt.title('Time vs Y')# X축의 범위: [xmin, xmax]
plt.ylim([-1.5, 1.5])  
plt.xlabel('Time')
plt.ylabel('Y')
plt.legend()
plt.savefig('graph_ex3.png')
plt.close()


time_exp, dis_exp, po_exp = read_data1('test.txt')

plt.figure(figsize=(8, 8))
plt.plot(time_exp, dis_exp, marker='o', color='red', label='Experiment')
plt.title('test')
plt.xlabel('Time')
plt.ylabel('distance')
plt.legend()
plt.savefig('graph_ex4.png')
plt.close()
