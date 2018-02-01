from sklearn.neural_network import MLPRegressor
import pandas as pd
import numpy as np
input_file = pd.read_csv('results-consolidated.csv')
test_file = pd.read_csv('test_data.csv')
out, test_data = input_file.as_matrix(), test_file.as_matrix()
row, col = np.shape(out)[0], np.shape(out)[1]
for c_ind in range(0, col-2):
    out[:, c_ind] = out[:, c_ind]/np.max(out[:, c_ind])
train_data, output = out[:,:-2], out[:,-2:]
travel_time_max, wait_time_max = np.max(output[:,0]), np.max(output[:,1])
output[:, 0] = output[:, 0]/travel_time_max
output[:, 1] = output[:, 1]/wait_time_max
nn = MLPRegressor(hidden_layer_sizes = (10,10), alpha = 0.01,
                  max_iter = 1000, verbose = True, activation = 'logistic')
nn.fit(train_data, output)
r, c = np.shape(test_data)[0], np.shape(test_data)[1]
for x in range(0, c):
    test_data[:, x] = test_data[:, x]/np.max(test_data[:, x])
y_out = nn.predict(test_data)
print(y_out)
print(y_out[0,0]*travel_time_max)
