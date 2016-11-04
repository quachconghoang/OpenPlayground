# import tensorflow as tf
import numpy as np

x = 1000000000;

for iter in xrange(1000000):
    x+= 1e-6

x-= 1000000000;

print(x);


#   https://github.com/tensorflow/tensorflow/blob/master/tensorflow/examples/udacity/1_notmnist.ipynb