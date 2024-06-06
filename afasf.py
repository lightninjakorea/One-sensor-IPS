import numpy as np
import matplotlib.pyplot as plt
import time


def pred(a,b,c,x_data, y_data, check) :
    
    function_pred = lambda x: np.sqrt(a+b*np.sin(w*x+c))
    
    if not check : return function_pred, x_data[y_data.index(min(y_data))], (min(y_data) + 0.36)
    
    else :return function_pred, x_data[y_data.index(max(y_data))], (max(y_data) - 0.36)
