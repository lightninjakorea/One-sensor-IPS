
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit


def read_data(file_path):
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

time_exp, dis_exp, po_exp = read_data('test.txt')


x_data = np.array(time_exp[:-100])

y_data = np.array(dis_exp[:-100])

def custom_function(x, a, b, c):
    return np.sqrt(a + b * np.sin((x * 27) + c))

# 3. 초기 추정값 설정
initial_guess = [0.1, 0.1, 0.1]

# 4. 비선형 최소제곱법을 사용하여 최적화
popt, pcov = curve_fit(custom_function, x_data, y_data, p0=initial_guess)

# 5. 피팅된 결과 시각화
plt.figure(figsize=(10, 6))
plt.scatter(x_data, y_data, label='Data', color='blue')
plt.plot(x_data, custom_function(x_data, *popt), label='Fitted function', color='red')
plt.legend()
plt.xlabel('x')
plt.ylabel('y')
plt.title('Custom Function Fitting')
plt.show()

# 피팅된 파라미터 출력
print('Fitted parameters:', popt)